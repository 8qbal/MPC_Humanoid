# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
NMPC on the rigid-contact model with a gait plan, acados SQP-RTI.

x = [q, qd, u_s], controls [v, u_a]: the servo targets of the leg joints and torso (with hip yaw) are states u_s
driven by their rates v; the arm targets u_a are controls (a target the servo follows through K/D needs no rate
state, and the arm rate states cost too much of the tick). The output is the servo targets of the model joints. The
caller passes a plan over the horizon: per node the contact modes (flat and air; edge modes are never planned),
contact references, DCM, CoM velocity, the sole-centre position of a swinging foot and the pelvis yaw.

The contact algebra is evaluated numerically at each node of the previous plan, and with the frozen mass matrix the
contact elimination is then affine in (b, q, qd): its matrices are passed per node as parameters, so inside a node
interval the contact dynamics are linear in the state while servo torques and gravity stay exact. Eliminating the
contact wrench symbolically instead took ~245 ms per solve (integrator sensitivities); this takes a few ms.

- per-node references as parameters: the cost residuals subtract them, so yref is constant;
- per-node gates: the leg-length cost only for feet in contact, the swing-position and sole-yaw costs only for feet
  in the air, the minimum normal force only for feet in contact; all constraint bounds are then constant (a foot in
  the air has zero wrench, so its CoP and friction rows read 0 >= 0);
- all node parameters are written with one set_flat call; the contact algebra and the node matrices along the
  previous plan are one compiled CasADi function (in numpy they took ~0.8 ms per preparation);
- the RTI iteration can be split: prepare() linearises for the next tick after this tick's targets are out,
  feedback() then only solves the QP with the new state;
- lock(): arm and hip yaw targets held at the default pose by their bounds, at runtime on the same build.

scripts/build_controller.py (no Isaac) builds the solver into outputs/acados/robonion_nmpc_<hash>/, the hash taken
from the OCP expressions and numbers; at runtime it is only loaded.
"""

from __future__ import annotations

import hashlib
import math
import os
import subprocess
from typing import NamedTuple

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from .model import (
    BUILD_DIR,
    BUILD_HINT,
    GRAVITY,
    SOLE_HALF_LENGTH,
    SOLE_HALF_WIDTH,
    RigidContactModel,
    rot_axis,
)

# 16 nodes of 31.25 ms over the 0.5 s horizon behave like 25 nodes of 20 ms while stepping and cost ~40 % less per
# tick; non-uniform grids (20 ms nodes first, longer ones after) raised the torso torque above the servo limit.
N_NODES = 16
HORIZON = 0.5  # s
OMEGA = math.sqrt(GRAVITY / 0.48)  # LIPM natural frequency at the standing CoM height
F_MIN = 5.0  # N, minimum normal force of a sole in contact
MU = 0.5  # friction coefficient for the controller; the env gives ~0.75 (ground 1.0, feet PhysX default 0.5)
COP_MARGIN = 0.01  # m inside the sole edges
EFFORT_FRACTION = 0.8
# Servo-target excursion from the standing pose [deg]: hip roll, front thigh, ankle pitch, ankle roll
# (left, right), torso pitch. Without bounds the NMPC swung the torso by over 100 deg after pushes.
U_RANGE_DEG = np.array([15, 20, 20, 15, 15, 20, 20, 15, 15], dtype=float)
HIP_YAW_RANGE_DEG = 15.0
ARM_RANGE_DEG = np.array([30, 20, 30, 30] * 2, dtype=float)  # shoulder pitch, roll, elbow yaw, pitch (left, right)

# Cost weights. W_U = 0.1 lets the CoM move to the sole centre: better against backward pushes
# (the weakest direction) than W_U = 10, which holds the pose and is better against forward pushes.
# At W_DCM = 100 the servo-rate cost kept the planned DCM 19 mm (RMS) behind the walking reference at 0.1 m/s;
# 1000 halves it and the walking holds, at the cost of longer holds after in-place backward pushes.
W_DCM, W_COMD, W_TORSO, W_U, W_RATE = 1000.0, 5.0, 10.0, 0.1, 0.5
W_SQUEEZE, W_TILT, W_TERM = 1e-4, 1e3, 1e3
# The leg-length and pelvis terms keep the NMPC from folding a knee or rolling the pelvis: at the
# straight-leg pose a knee bend is nearly invisible to the linearised contact model.
W_LEG, W_PELVIS = 1e4, 100.0
# At 1e4 the swing foot reached 30 % of its planned height (the servo-rate cost dominated), at 1e6 94 %.
W_SWING = 1e6
# Arm targets pull to the default pose with W_ARM. Summed ground yaw moment: the centroidal yaw momentum changes only
# through it, so this damps yaw without the feet cancelling an external torque (a cost on the momentum itself did).
W_ARM, W_MZ = 1.0, 10.0


def _ocp_key(functions: list[ca.Function], arrays: list, options: dict) -> str:
    h = hashlib.sha1("".join(f.serialize() for f in functions).encode())
    for a in arrays:
        h.update(np.asarray(a, dtype=float).tobytes())
    h.update(repr(sorted(options.items())).encode())
    return h.hexdigest()[:12]


class NodeConfig(NamedTuple):
    flags: np.ndarray
    pc: np.ndarray
    isflat: np.ndarray
    contact: np.ndarray


class RobonionNMPC:
    def __init__(self, model: RigidContactModel, build: bool = False):
        """build: generate and compile the solver (scripts/build_controller.py); otherwise load the existing build."""
        N = self.N = N_NODES
        self.t_nodes = np.linspace(0.0, HORIZON, N + 1)  # node times from the current tick
        M = self.m = model
        nq, nj, nb, nr, nl = M.nq, M.nj, M.nb, M.nr, M.n_legs
        self.nq, self.nj, self.nr = nq, nj, nr
        na = self.na = nj - nl
        ns = self.ns = nl
        q, qd = M.q_sym, M.qd_sym
        us, ua = ca.SX.sym("us", ns), ca.SX.sym("ua", na)
        u = ca.vertcat(us, ua)
        v = ca.SX.sym("v", ns)
        ctrl = ca.vertcat(v, ua)
        pc = M.pc_sym
        isflat = ca.SX.sym("isflat", 2)
        Pq, GJ, cq = ca.SX.sym("Pq", nq, nq), ca.SX.sym("GJ", nq, nq), ca.SX.sym("cq", nq)
        Lb, LJ, cl = ca.SX.sym("Lb", 2 * nr, nq), ca.SX.sym("LJ", 2 * nr, nq), ca.SX.sym("cl", 2 * nr)
        contact = ca.SX.sym("contact", 2)
        dcm_ref = ca.SX.sym("dcm_ref", 2)
        comd_ref = ca.SX.sym("comd_ref", 2)
        sole_ref = ca.SX.sym("sole_ref", 6)
        yaw_ref = ca.SX.sym("yaw_ref", 1)
        p = ca.vertcat(
            pc,
            isflat,
            ca.vec(Pq),
            ca.vec(GJ),
            cq,
            ca.vec(Lb),
            ca.vec(LJ),
            cl,
            contact,
            dcm_ref,
            comd_ref,
            sole_ref,
            yaw_ref,
        )

        K, D = ca.DM(M.stiffness), ca.DM(M.damping)
        tau = K * (u - q[nb:]) - D * qd[nb:]
        b = M.Gj.T @ tau - M.gravity_expr
        a1, a2 = 2 * M.alpha, M.alpha**2
        qdd = Pq @ b - GJ @ (a1 * qd + a2 * q) + cq
        lam = Lb @ b - LJ @ (a1 * qd + a2 * q) + cl

        x = ca.vertcat(q, qd, us)
        xdot = ca.SX.sym("xdot", x.shape[0])
        f = ca.vertcat(qd, qdd, v)
        am = AcadosModel()
        am.x, am.u, am.xdot, am.p = x, ctrl, xdot, p
        am.f_expl_expr, am.f_impl_expr = f, xdot - f

        W = ca.horzcat(*[M.wrench_of(lam[nr * i : nr * (i + 1)], M.axes_expr[i]) for i in (0, 1)])

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
            K * (u - q[nb:]),  # 16-, servo torque
        )
        self.nh = am.con_h_expr.shape[0]
        effort = M.effort * EFFORT_FRACTION

        com, comd = M.com_expr, M.comd_expr
        dcm = com[:2] + comd[:2] / OMEGA
        squeeze = W[:2, 0] - W[:2, 1]
        soles = [ca.sum2(corners[:, 4 * i : 4 * i + 4]) / 4 for i in (0, 1)]
        Rb = rot_axis((0, 1, 0), q[4]) @ rot_axis((1, 0, 0), q[3])
        if M.yaw:
            Rb = rot_axis((0, 0, 1), q[5]) @ Rb
        leg = ca.vertcat(*[(Rb.T @ (soles[i] - q[:3]))[2] for i in (0, 1)])
        self.u_nom = M.q0_joints.copy()
        q_init, _ = M.initial_state(self.u_nom)
        leg0 = np.array(ca.Function("leg", [q], [leg])(q_init)).ravel()
        swing = ca.vertcat(*[(1 - contact[i]) * (soles[i] - sole_ref[3 * i : 3 * i + 3]) for i in (0, 1)])
        leg_err = ca.vertcat(*[contact[i] * (leg[i] - leg0[i]) for i in (0, 1)])
        track = ca.vertcat(dcm - dcm_ref, comd[:2] - comd_ref)
        y = [track, q[nb + 8], u, v, squeeze, ca.vec(tilt), swing, leg_err, q[3:5]]
        y_e = [track, ca.vec(tilt), swing, leg_err, q[3:5]]
        w = [W_DCM] * 2 + [W_COMD] * 2 + [W_TORSO] + [W_U] * nl + [W_ARM] * na + [W_RATE] * ns + [W_SQUEEZE] * 2
        w += [W_TILT] * 4 + [W_SWING] * 6 + [W_LEG] * 2 + [W_PELVIS] * 2
        w_e = [W_TERM] * 2 + [W_COMD] * 2 + [W_TILT] * 4 + [W_SWING] * 6 + [W_LEG] * 2 + [W_PELVIS] * 2
        if na:
            # an overdamped servo moves at ~ K/D (u - q): this stands in for the rate cost the arm targets lack
            y.append(ua - q[nb + ns :])
            w += list(W_RATE * (M.stiffness[ns:] / M.damping[ns:]) ** 2)
        if M.hip_yaw:
            # with hip yaw the pelvis and the swing sole can turn: both follow the planned yaw
            yaw_rows = [(1 - contact[i]) * (M.sole_yaw_expr[i] - yaw_ref) for i in (0, 1)] + [q[5] - yaw_ref]
            y += yaw_rows
            y_e += yaw_rows
            w += [W_TILT] * 2 + [W_PELVIS]
            w_e += [W_TILT] * 2 + [W_PELVIS]
        if M.yaw:
            y.append(W[5, 0] + W[5, 1])
            w.append(W_MZ)
        am.cost_y_expr, am.cost_y_expr_e = ca.vertcat(*y), ca.vertcat(*y_e)

        self.f_sole = ca.Function("soles", [q], [ca.horzcat(*soles)])

        # lambda = L (g0 - J M^-1 b - 2 alpha J qd - alpha^2 J q) with L = A^-1 diag(flags), g0 the contact terms
        # that do not depend on (b, q, qd) at this node
        # the matrix algebra in SX on plain matrix inputs (J with the sparsity of the contact Jacobian), called on the
        # contact algebra through MX: in SX on q directly it generated ~30 MB of C, in MX (QR solve, generic
        # products) it ran slower than numpy
        raw = M.raw_sx
        Jm = ca.SX.sym("J", raw["Jc"].sparsity_out(0))
        cm, gm = ca.SX.sym("c", 2 * nr), ca.SX.sym("g", 2 * nr)
        qm, fl, cr = ca.SX.sym("q", nq), ca.SX.sym("flags", 2 * nr), ca.SX.sym("cref", 2 * nr)
        Minv = ca.DM(M.Minv)
        K = Jm @ Minv
        L = ca.solve(ca.diag(fl) @ (K @ Jm.T) + ca.diag(1 - fl), ca.diag(fl))
        Lg0 = L @ (gm + a2 * (cm - Jm @ qm - cr))
        LJ, LK = L @ Jm, L @ K
        GJ = K.T @ LJ
        node = ca.vertcat(ca.vec(Minv - K.T @ LK), ca.vec(GJ), -K.T @ Lg0, ca.vec(-LK), ca.vec(LJ), -Lg0)
        f_mat = ca.Function("mat", [Jm, cm, gm, qm, fl, cr], [node])
        qs, qds, pcs = ca.MX.sym("qs", nq), ca.MX.sym("qds", nq), ca.MX.sym("pcs", 6)
        fls, crs = ca.MX.sym("flags", 2 * nr), ca.MX.sym("cref", 2 * nr)
        alg = f_mat(raw["Jc"](qs, pcs), raw["crow"](qs, pcs), raw["Jdqd"](qs, qds, pcs), qs, fls, crs)
        f_node = ca.Function("alg", [qs, qds, pcs, fls, crs], [alg]).map(N + 1)
        args = [ca.MX.sym(a, k, N + 1) for a, k in (("Q", nq), ("QD", nq), ("PC", 6), ("FL", 2 * nr), ("CR", 2 * nr))]
        f_alg = ca.Function("alg_all", args, [ca.densify(f_node(*args))])

        yref = np.zeros(am.cost_y_expr.shape[0])
        yref[4] = self.u_nom[8]
        yref[5 : 5 + nj] = self.u_nom

        ocp = AcadosOcp()
        ocp.model = am
        ocp.solver_options.N_horizon = N
        ocp.solver_options.tf = HORIZON
        ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W, ocp.cost.W_e = np.diag(w), np.diag(w_e)
        ocp.cost.yref, ocp.cost.yref_e = yref, np.zeros(am.cost_y_expr_e.shape[0])
        ocp.parameter_values = np.zeros(p.shape[0])

        lh, uh = np.zeros(self.nh), np.full(self.nh, 1e4)
        lh[16:], uh[16:] = -effort, effort
        ocp.constraints.lh, ocp.constraints.uh = lh, uh
        ocp.constraints.idxsh = np.arange(self.nh)
        # slack penalties (L1, L2) per row group: forces [N], moments [N m], servo torque [N m]
        z1 = np.r_[[10.0] * 2, [1e3] * 6, [10.0] * 8, [1e3] * nj]
        z2 = np.r_[[10.0] * 2, [1e4] * 6, [10.0] * 8, [1e2] * nj]
        ocp.cost.zl = ocp.cost.zu = z1
        ocp.cost.Zl = ocp.cost.Zu = z2
        ocp.constraints.x0 = np.r_[q_init, np.zeros(nq), self.u_nom[:ns]]
        span = np.radians(np.r_[U_RANGE_DEG, [HIP_YAW_RANGE_DEG] * (nl - 9), ARM_RANGE_DEG[:na]])
        rate = M.rate_limit[:nl]
        self.u_lo, self.u_hi = self.u_nom - span, self.u_nom + span
        ocp.constraints.idxbx = 2 * nq + np.arange(ns)
        ocp.constraints.lbx, ocp.constraints.ubx = self.u_lo[:ns], self.u_hi[:ns]
        ocp.constraints.idxbu = np.arange(ns + na)
        ocp.constraints.lbu, ocp.constraints.ubu = np.r_[-rate, self.u_lo[ns:]], np.r_[rate, self.u_hi[ns:]]

        options = {
            "qp_solver": "PARTIAL_CONDENSING_HPIPM",
            # partial condensing to 5 stages, HPIPM speed mode and a warm start: QP 2.9 -> 2.1 ms per tick
            # (stage 2); SPEED_ABS saved another ~0.5 ms per feedback with the whole body
            "qp_solver_cond_N": 5,
            "hpipm_mode": "SPEED_ABS",
            "qp_solver_warm_start": 1,
            "qp_solver_iter_max": 100,
            # 1e-5 instead of the defaults: 4.7 instead of 6.1 HPIPM iterations per feedback; 1e-4 (3.9) started holds
            # after -x 1.0 pushes in Isaac (dual warm start, qp_solver_warm_start 2, took more iterations)
            "qp_solver_tol_stat": 1e-5,
            "qp_solver_tol_eq": 1e-5,
            "qp_solver_tol_ineq": 1e-5,
            "qp_solver_tol_comp": 1e-5,
            "hessian_approx": "GAUSS_NEWTON",
            # Radau IIA, one step per node: 2 stages matched RK4 at 1 ms within 0.005 mm of CoM over the horizon
            # (stage 2). 1 stage everywhere fit the whole-body tick but rolled the soles onto edges in Isaac; 2 on the
            # first 4 intervals still did with the deferred switch, on the first 8 it steps as cleanly as stage 2
            "integrator_type": "IRK",
            "collocation_type": "GAUSS_RADAU_IIA",
            "sim_method_num_stages": np.r_[[2] * 8, [1] * 8],
            "sim_method_num_steps": 1,
            "sim_method_newton_iter": 1,
            "sim_method_jac_reuse": 1,
            "nlp_solver_type": "SQP_RTI",
        }
        for key, value in options.items():
            setattr(ocp.solver_options, key, value)

        f_ocp = ca.Function(
            "ocp", [x, ctrl, xdot, p], [am.f_impl_expr, am.cost_y_expr, am.cost_y_expr_e, am.con_h_expr]
        )
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

        self.ocp = ocp
        self.f_alg = ca.external("alg_all", so_file)
        # evaluated in place on numpy arrays: converting the ~32k outputs from a DM took ~1.3 ms
        self._alg_buf, self._alg_eval = self.f_alg.buffer()
        self._alg_in = [np.zeros((N + 1, k)) for k in (nq, nq, 6, 2 * nr, 2 * nr)]
        self._alg_out = np.zeros((N + 1, self.f_alg.size1_out(0)))
        for i, a in enumerate(self._alg_in):
            self._alg_buf.set_arg(i, memoryview(a))
        self._alg_buf.set_res(0, memoryview(self._alg_out))
        self.nx = 2 * nq + ns
        self._cfg_cache = {}
        self.nan_resets = 0
        self.X: np.ndarray | None = None
        self.reset(np.r_[q_init, np.zeros(nq), self.u_nom[:ns]])

    def lock(self, arms: bool = False, hip_yaw: bool = False):
        """Hold the arm and / or hip yaw targets at the default pose through their bounds (False: free again). Hip
        yaw targets are states: only their rates are bounded to 0 (also bounding the states would make the QP
        degenerate), so they stay where the caller's targets are, the default pose from the start."""
        ns = self.ns
        lbu, ubu = self.ocp.constraints.lbu.copy(), self.ocp.constraints.ubu.copy()
        if hip_yaw and self.m.hip_yaw:
            lbu[9:ns] = ubu[9:ns] = 0.0
        if arms and self.na:
            lbu[ns:] = ubu[ns:] = self.u_nom[ns:]
        for k in range(self.N):
            self.solver.constraints_set(k, "lbu", lbu)
            self.solver.constraints_set(k, "ubu", ubu)

    def reset(self, x0: np.ndarray):
        """Initial guess x0 at every node; forget the previous plan."""
        for k in range(self.N + 1):
            self.solver.set(k, "x", x0)
        for k in range(self.N):
            self.solver.set(k, "u", np.r_[np.zeros(self.ns), self.u_nom[self.ns :]])
        self.X = None

    def _config(self, modes) -> NodeConfig:
        if modes not in self._cfg_cache:
            flags, pc = self.m.contact_config(*modes)
            self._cfg_cache[modes] = NodeConfig(
                flags,
                pc,
                isflat=np.array([float(m == "flat") for m in modes]),
                contact=np.array([float(m != "air") for m in modes]),
            )
        return self._cfg_cache[modes]

    def _set_params(self, q, qd, plan):
        nq = self.nq
        Q, QD, pcs, flags, cref = self._alg_in
        if self.X is None:
            Q[:], QD[:] = q, qd
        else:
            Q[:], QD[:] = self.X[:, :nq], self.X[:, nq : 2 * nq]
            Q[0], QD[0] = q, qd

        cfgs = [self._config(tuple(node["modes"])) for node in plan]
        flags[:] = [cf.flags for cf in cfgs]
        pcs[:] = [cf.pc for cf in cfgs]
        cref[:] = [node["cref"] for node in plan]
        self._alg_eval()
        P = np.concatenate(
            [
                pcs,
                np.array([cf.isflat for cf in cfgs]),
                self._alg_out,
                np.array([cf.contact for cf in cfgs]),
                np.array([node["dcm"] for node in plan]),
                np.array([node["comd"] for node in plan]),
                np.array([np.asarray(node["sole"]).T.ravel() for node in plan]),
                np.array([[node.get("yaw", 0.0)] for node in plan]),
            ],
            axis=1,
        )
        self.solver.set_flat("p", P.ravel())

    def _feedback(self, q, qd, u_prev, dt, phase):
        ns = self.ns
        x0 = np.r_[q, qd, u_prev[:ns]]
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        self.solver.options_set("rti_phase", phase)
        status = self.solver.solve()
        self.X = self.solver.get_flat("x").reshape(self.N + 1, self.nx)
        c = self.solver.get(0, "u")
        time_tot = float(self.solver.get_stats("time_tot"))

        v = np.r_[c[:ns], np.zeros(self.na)]
        if not (np.all(np.isfinite(self.X)) and np.all(np.isfinite(c))):
            # a NaN iterate stays NaN in every later RTI step: restart from the measured state, hold the targets
            self.nan_resets += 1
            self.reset(x0)
            return u_prev, v * 0.0, status, time_tot
        return np.r_[u_prev[:ns] + c[:ns] * dt, c[ns:]], v, status, time_tot

    def solve(self, q, qd, u_prev, plan, dt):
        """One full SQP-RTI iteration from (q, qd) and the current servo targets u_prev. plan: N + 1 dicts with
        keys modes, cref, dcm, comd, sole (3 x 2 sole-centre targets, used for feet in the air) and optionally yaw
        (pelvis yaw, default 0). Returns the servo targets for the next tick, their rate for the ticks until the
        next NMPC run (0 for the arm targets), the QP status and the solver time [s]."""
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
