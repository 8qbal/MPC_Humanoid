# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
NMPC on the rigid-contact model with a gait plan (design D in docs/stage1.md, stepping in docs/stage2.md), acados
SQP-RTI.

x = [q (14), qd (14), u (9)], control v = du/dt; the output is u, the servo targets of the model joints. The caller
passes a plan over the horizon: per node the contact modes (flat and air; edge modes are never planned), contact
references, DCM, CoM velocity and the sole-centre position of a swinging foot.

The contact algebra (Jc, (Jc M^-1 Jc^T)^-1, Jdot qd, contact rows) is evaluated numerically at each node of the
previous plan and passed as parameters, so inside a node interval the contact dynamics are linear in the state while
servo torques and gravity stay exact. Eliminating the contact wrench symbolically instead took ~245 ms per solve
(integrator sensitivities); this takes a few ms.

- per-node references as parameters: the cost residuals subtract them, so yref is constant;
- per-node gates: the leg-length cost only for feet in contact, the swing-position cost only for feet in the
  air, the minimum normal force only for feet in contact; all constraint bounds are then constant (a foot in
  the air has zero wrench, so its CoP and friction rows read 0 >= 0);
- all node parameters are written with one set_flat call; the contact algebra along the previous plan is a
  compiled CasADi function and the (Jc M^-1 Jc^T)^-1 blocks are inverted as one batch;
- the RTI iteration can be split: prepare() linearises for the next tick after this tick's targets are out,
  feedback() then only solves the QP with the new state.

scripts/build_controller.py (no Isaac) builds the solver into outputs/acados/robonion_nmpc_<hash>/, the hash taken
from the OCP expressions and numbers; at runtime it is only loaded.
"""

from __future__ import annotations

import hashlib
import math
import os
import subprocess

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from MPC_Humanoid.robots.robonion_params import URDF_LIMITS, XH540_DAMPING, XH540_STIFFNESS

from .model import SOLE_HALF_LENGTH, SOLE_HALF_WIDTH, RigidContactModel, contact_config, rot_axis

BUILD_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../outputs/acados"))
BUILD_HINT = "run `uv run python scripts/build_controller.py` first (no Isaac needed)"

# 16 nodes of 31.25 ms over the 0.5 s horizon behave like 25 nodes of 20 ms while stepping and cost ~40 % less per
# tick; non-uniform grids (20 ms nodes first, longer ones after) raised the torso torque above the servo limit.
N_NODES = 16
HORIZON = 0.5  # s
OMEGA = math.sqrt(9.81 / 0.48)  # LIPM natural frequency at the standing CoM height
F_MIN = 5.0  # N, minimum normal force of a sole in contact
MU = 0.5  # friction coefficient for the controller; the env gives ~0.75 (ground 1.0, feet PhysX default 0.5)
COP_MARGIN = 0.01  # m inside the sole edges
EFFORT_FRACTION = 0.8
# Servo-target excursion from the standing pose [deg]: hip roll, front thigh, ankle pitch, ankle roll
# (left, right), torso pitch. Without bounds the NMPC swung the torso by over 100 deg after pushes.
U_RANGE_DEG = np.array([15, 20, 20, 15, 15, 20, 20, 15, 15], dtype=float)
U_RATE_MAX = URDF_LIMITS["legs"][1]

# Cost weights. W_U = 0.1 lets the CoM move to the sole centre: better against backward pushes
# (the weakest direction) than W_U = 10, which holds the pose and is better against forward pushes.
W_DCM, W_COMD, W_TORSO, W_U, W_RATE = 100.0, 5.0, 10.0, 0.1, 0.5
W_SQUEEZE, W_TILT, W_TERM = 1e-4, 1e3, 1e3
# The leg-length and pelvis terms keep the NMPC from folding a knee or rolling the pelvis: at the
# straight-leg pose a knee bend is nearly invisible to the linearised contact model.
W_LEG, W_PELVIS = 1e4, 100.0
# At 1e4 the swing foot reached 30 % of its planned height (the servo-rate cost dominated), at 1e6 94 %.
W_SWING = 1e6


def _ocp_key(functions: list[ca.Function], arrays: list, options: dict) -> str:
    h = hashlib.sha1("".join(f.serialize() for f in functions).encode())
    for a in arrays:
        h.update(np.asarray(a, dtype=float).tobytes())
    h.update(repr(sorted(options.items())).encode())
    return h.hexdigest()[:12]


class RobonionNMPC:
    def __init__(self, model: RigidContactModel, build: bool = False):
        """build: generate and compile the solver (scripts/build_controller.py); otherwise load the existing build."""
        N = self.N = N_NODES
        self.t_nodes = np.linspace(0.0, HORIZON, N + 1)  # node times from the current tick
        M = self.m = model
        nq = self.nq = M.nq
        q, qd, u = M.q_sym, M.qd_sym, M.u_sym
        v = ca.SX.sym("v", 9)
        flags, cref, pc = M.flags_sym, M.cref_sym, M.pc_sym
        isflat = ca.SX.sym("isflat", 2)
        Jk = ca.SX.sym("Jk", 10, nq)
        Aik = ca.SX.sym("Aik", 10, 10)
        gk = ca.SX.sym("gk", 10)
        ck = ca.SX.sym("ck", 10)
        qk = ca.SX.sym("qk", nq)
        contact = ca.SX.sym("contact", 2)
        dcm_ref = ca.SX.sym("dcm_ref", 2)
        comd_ref = ca.SX.sym("comd_ref", 2)
        sole_ref = ca.SX.sym("sole_ref", 6)
        p = ca.vertcat(
            flags, cref, pc, isflat, ca.vec(Jk), ca.vec(Aik), gk, ck, qk, contact, dcm_ref, comd_ref, sole_ref
        )

        tau = XH540_STIFFNESS * (u - q[5:]) - XH540_DAMPING * qd[5:]
        b = M.Gj.T @ tau - M.gravity_expr
        gamma = gk + 2 * M.alpha * (Jk @ qd) + M.alpha**2 * (ck + Jk @ (q - qk) - cref)
        lam = -Aik @ (flags * (Jk @ (M.Minv @ b) + gamma))
        qdd = M.Minv @ (b + Jk.T @ lam)
        x = ca.vertcat(q, qd, u)
        xdot = ca.SX.sym("xdot", x.shape[0])
        f = ca.vertcat(qd, qdd, v)
        am = AcadosModel()
        am.x, am.u, am.xdot, am.p = x, v, xdot, p
        am.f_expl_expr, am.f_impl_expr = f, xdot - f

        wrench = []
        for i, (ex, ey) in enumerate(M.axes_expr):
            l_ = lam[5 * i : 5 * i + 5]
            wrench.append(ca.vertcat(l_[:3], l_[3] * ey[1] + l_[4] * ex[1], -(l_[3] * ey[0] + l_[4] * ex[0])))
        W = ca.horzcat(*wrench)
        C, tilt, corners = M.points_expr, M.tilt_expr, M.corners_expr
        fz, mx, my = W[2, :].T, W[3, :].T, W[4, :].T
        hw, hl = SOLE_HALF_WIDTH - COP_MARGIN, SOLE_HALF_LENGTH - COP_MARGIN
        x_bar = (isflat[0] * C[0, 0] + isflat[1] * C[0, 1]) / (isflat[0] + isflat[1] + 1e-9)
        fz_flat = isflat[0] * fz[0] + isflat[1] * fz[1]
        m_flat = sum(isflat[i] * ((C[0, i] - x_bar) * fz[i] - my[i]) for i in (0, 1))
        fric = [MU * fz[i] + s * W[j, i] for i in (0, 1) for j in (0, 1) for s in (-1, 1)]
        am.con_h_expr = ca.vertcat(
            fz - F_MIN * contact,  # 0-1
            hw * fz[0] - mx[0],
            hw * fz[0] + mx[0],
            hw * fz[1] - mx[1],
            hw * fz[1] + mx[1],  # 2-5 CoP-y
            hl * fz_flat - m_flat,
            hl * fz_flat + m_flat,  # 6-7 combined CoP-x of the flat soles
            *fric,  # 8-15
            XH540_STIFFNESS * (u - q[5:]),  # 16-24
        )
        self.nh = am.con_h_expr.shape[0]
        self.effort = M.effort * EFFORT_FRACTION

        com, comd = M.com_expr, M.comd_expr
        dcm = com[:2] + comd[:2] / OMEGA
        squeeze = W[:2, 0] - W[:2, 1]
        soles = [ca.sum2(corners[:, 4 * i : 4 * i + 4]) / 4 for i in (0, 1)]
        Rb = rot_axis((0, 1, 0), q[4]) @ rot_axis((1, 0, 0), q[3])
        leg = ca.vertcat(*[(Rb.T @ (soles[i] - q[:3]))[2] for i in (0, 1)])
        self.u_nom = M.q0_joints.copy()
        q_init, _ = M.initial_state(self.u_nom)
        leg0 = np.array(ca.Function("leg", [q], [leg])(q_init)).ravel()
        swing = ca.vertcat(*[(1 - contact[i]) * (soles[i] - sole_ref[3 * i : 3 * i + 3]) for i in (0, 1)])
        leg_err = ca.vertcat(*[contact[i] * (leg[i] - leg0[i]) for i in (0, 1)])
        track = ca.vertcat(dcm - dcm_ref, comd[:2] - comd_ref)
        am.cost_y_expr = ca.vertcat(track, q[13], u, v, squeeze, ca.vec(tilt), swing, leg_err, q[3:5])
        am.cost_y_expr_e = ca.vertcat(track, ca.vec(tilt), swing, leg_err, q[3:5])
        self.f_sole = ca.Function("soles", [q], [ca.horzcat(*soles)])

        qs, qds, pcs = ca.SX.sym("qs", nq), ca.SX.sym("qds", nq), ca.SX.sym("pcs", 6)
        raw = M.raw_sx
        f_map = ca.Function(
            "alg", [qs, qds, pcs], [raw["Jc"](qs, pcs), raw["crow"](qs, pcs), raw["Jdqd"](qs, qds, pcs)]
        )
        f_map = f_map.map(N + 1)
        Qs, QDs, PCs = ca.SX.sym("Q", nq, N + 1), ca.SX.sym("QD", nq, N + 1), ca.SX.sym("PC", 6, N + 1)
        f_alg = ca.Function("alg_all", [Qs, QDs, PCs], f_map(Qs, QDs, PCs))

        yref = np.zeros(4 + 1 + 9 + 9 + 2 + 4 + 6 + 2 + 2)
        yref[4] = self.u_nom[8]
        yref[5:14] = self.u_nom

        ocp = AcadosOcp()
        ocp.model = am
        ocp.solver_options.N_horizon = N
        ocp.solver_options.tf = HORIZON
        ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.diag(
            [W_DCM] * 2
            + [W_COMD] * 2
            + [W_TORSO]
            + [W_U] * 9
            + [W_RATE] * 9
            + [W_SQUEEZE] * 2
            + [W_TILT] * 4
            + [W_SWING] * 6
            + [W_LEG] * 2
            + [W_PELVIS] * 2
        )
        ocp.cost.W_e = np.diag(
            [W_TERM] * 2 + [W_COMD] * 2 + [W_TILT] * 4 + [W_SWING] * 6 + [W_LEG] * 2 + [W_PELVIS] * 2
        )
        ocp.cost.yref, ocp.cost.yref_e = yref, np.zeros(18)
        ocp.parameter_values = np.zeros(p.shape[0])
        lh, uh = np.zeros(self.nh), np.full(self.nh, 1e4)
        lh[16:], uh[16:] = -self.effort, self.effort
        ocp.constraints.lh, ocp.constraints.uh = lh, uh
        ocp.constraints.idxsh = np.arange(self.nh)
        # slack penalties (L1, L2) per row group: forces [N], moments [N m], servo torque [N m]
        z1 = np.r_[[10.0] * 2, [1e3] * 6, [10.0] * 8, [1e3] * 9]
        z2 = np.r_[[10.0] * 2, [1e4] * 6, [10.0] * 8, [1e2] * 9]
        ocp.cost.zl = ocp.cost.zu = z1
        ocp.cost.Zl = ocp.cost.Zu = z2
        ocp.constraints.x0 = np.r_[q_init, np.zeros(nq), self.u_nom]
        ocp.constraints.idxbx = 2 * nq + np.arange(9)
        ocp.constraints.lbx = self.u_nom - np.radians(U_RANGE_DEG)
        ocp.constraints.ubx = self.u_nom + np.radians(U_RANGE_DEG)
        ocp.constraints.idxbu = np.arange(9)
        ocp.constraints.lbu, ocp.constraints.ubu = np.full(9, -U_RATE_MAX), np.full(9, U_RATE_MAX)
        options = {
            "qp_solver": "PARTIAL_CONDENSING_HPIPM",
            # partial condensing to 5 stages, HPIPM speed mode and a warm start: QP 2.9 -> 2.1 ms per tick
            "qp_solver_cond_N": 5,
            "hpipm_mode": "SPEED",
            "qp_solver_warm_start": 1,
            "qp_solver_iter_max": 100,
            "hessian_approx": "GAUSS_NEWTON",
            # Radau IIA, 2 stages, one step per node: matches RK4 at 1 ms within 0.005 mm of CoM over the
            # horizon and stays stable for the -600 1/s mode of a foot in the air
            "integrator_type": "IRK",
            "collocation_type": "GAUSS_RADAU_IIA",
            "sim_method_num_stages": 2,
            "sim_method_num_steps": 1,
            "sim_method_newton_iter": 1,
            "sim_method_jac_reuse": 1,
            "nlp_solver_type": "SQP_RTI",
        }
        for key, value in options.items():
            setattr(ocp.solver_options, key, value)

        f_ocp = ca.Function("ocp", [x, v, xdot, p], [am.f_impl_expr, am.cost_y_expr, am.cost_y_expr_e, am.con_h_expr])
        numbers = [[N, HORIZON], ocp.cost.W, ocp.cost.W_e, yref, lh, uh, z1, z2, ocp.constraints.x0]
        numbers += [ocp.constraints.idxbx, ocp.constraints.lbx, ocp.constraints.ubx]
        numbers += [ocp.constraints.idxbu, ocp.constraints.lbu, ocp.constraints.ubu]
        name = f"robonion_nmpc_{_ocp_key([f_ocp, f_alg], numbers, options)}"
        am.name = ocp.name = name
        ocp.code_export_directory = os.path.join(BUILD_DIR, name)
        json_file = os.path.join(ocp.code_export_directory, f"{name}.json")
        lib_file = os.path.join(ocp.code_export_directory, f"libacados_ocp_solver_{name}.so")
        # the contact algebra in the CasADi VM took ~2 ms per tick; compiled it is a small fraction of that
        so_file = os.path.join(ocp.code_export_directory, "alg.so")
        if build:
            self.solver = AcadosOcpSolver(ocp, json_file=json_file, verbose=False)
            cg = ca.CodeGenerator("alg.c")
            cg.add(f_alg)
            cg.generate(ocp.code_export_directory + os.sep)
            c_file = os.path.join(ocp.code_export_directory, "alg.c")
            subprocess.run(["gcc", "-O3", "-march=native", "-shared", "-fPIC", c_file, "-o", so_file], check=True)
        else:
            missing = [f for f in (json_file, lib_file, so_file) if not os.path.isfile(f)]
            if missing:
                raise FileNotFoundError(f"NMPC build not found ({', '.join(missing)}): {BUILD_HINT}")
            self.solver = AcadosOcpSolver(
                ocp, json_file=json_file, generate=False, build=False, check_reuse_possible=False, verbose=False
            )
        self.f_alg = ca.external("alg_all", so_file)
        self.nx = 2 * nq + 9
        self._diag = np.arange(10)
        self._cfg_cache = {}
        self.nan_resets = 0
        self.X: np.ndarray | None = None
        self.reset(np.r_[q_init, np.zeros(nq), self.u_nom])

    def reset(self, x0: np.ndarray):
        """Initial guess x0 at every node; forget the previous plan."""
        for k in range(self.N + 1):
            self.solver.set(k, "x", x0)
        self.X = None

    def _config(self, modes):
        if modes not in self._cfg_cache:
            flags, pc = contact_config(*modes)
            self._cfg_cache[modes] = (
                flags,
                pc,
                np.array([float(m == "flat") for m in modes]),
                np.array([float(m != "air") for m in modes]),
            )
        return self._cfg_cache[modes]

    def _set_params(self, q, qd, plan):
        n, nq = self.N + 1, self.nq
        if self.X is None:
            Q, QD = np.tile(q, (n, 1)), np.tile(qd, (n, 1))
        else:
            Q, QD = self.X[:, :nq].copy(), self.X[:, nq : 2 * nq].copy()
            Q[0], QD[0] = q, qd
        cfgs = [self._config(tuple(node["modes"])) for node in plan]
        flags = np.array([cf[0] for cf in cfgs])
        pcs = np.array([cf[1] for cf in cfgs])
        J, c, g = (np.array(a) for a in self.f_alg(Q.T, QD.T, pcs.T))
        Jb = J.reshape(10, n, nq).transpose(1, 0, 2)
        A = flags[:, :, None] * (Jb @ self.m.Minv @ Jb.transpose(0, 2, 1))
        A[:, self._diag, self._diag] += 1.0 - flags
        P = np.concatenate(
            [
                flags,
                np.array([node["cref"] for node in plan]),
                pcs,
                np.array([cf[2] for cf in cfgs]),
                Jb.transpose(0, 2, 1).reshape(n, -1),
                np.linalg.inv(A).transpose(0, 2, 1).reshape(n, -1),
                g.T,
                c.T,
                Q,
                np.array([cf[3] for cf in cfgs]),
                np.array([node["dcm"] for node in plan]),
                np.array([node["comd"] for node in plan]),
                np.array([np.asarray(node["sole"]).T.ravel() for node in plan]),
            ],
            axis=1,
        )
        self.solver.set_flat("p", P.ravel())

    def _feedback(self, q, qd, u_prev, dt, phase):
        x0 = np.r_[q, qd, u_prev]
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        self.solver.options_set("rti_phase", phase)
        status = self.solver.solve()
        self.X = self.solver.get_flat("x").reshape(self.N + 1, self.nx)
        v = self.solver.get(0, "u")
        time_tot = float(self.solver.get_stats("time_tot"))
        if not (np.all(np.isfinite(self.X)) and np.all(np.isfinite(v))):
            # a NaN iterate stays NaN in every later RTI step: restart from the measured state, hold the targets
            self.nan_resets += 1
            self.reset(x0)
            return u_prev, status, time_tot
        return u_prev + v * dt, status, time_tot

    def solve(self, q, qd, u_prev, plan, dt):
        """One full SQP-RTI iteration from (q, qd) and the current servo targets u_prev. plan: N + 1 dicts with
        keys modes, cref, dcm, comd, sole (3 x 2 sole-centre targets, used for feet in the air). Returns the
        servo targets for the next tick, the QP status and the solver time [s]."""
        self._set_params(q, qd, plan)
        return self._feedback(q, qd, u_prev, dt, 0)

    def prepare(self, q, qd, plan):
        """RTI preparation for the next tick: linearisation with the next tick's plan around the current iterate."""
        self._set_params(q, qd, plan)
        self.solver.options_set("rti_phase", 1)
        self.solver.solve()

    def feedback(self, q, qd, u_prev, dt):
        """RTI feedback on a prepared linearisation: only the QP with the new initial state."""
        return self._feedback(q, qd, u_prev, dt, 2)
