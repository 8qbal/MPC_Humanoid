# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
NMPC on the rigid-contact model (design D in docs/stage1.md), acados SQP-RTI.

x = [q (14), qd (14), u (9)], control v = du/dt; the output is u, the servo targets of the model joints.
The contact mode of each foot is a per-node parameter, so the caller passes a contact plan over the
horizon (flat and air; edge modes are not planned, the robot recovers from them passively).

The contact algebra (Jc, (Jc M^-1 Jc^T)^-1, Jdot qd, contact rows) is evaluated numerically at each node
of the previous plan and passed as parameters, so inside a node interval the contact dynamics are
linear in the state while servo torques and gravity stay exact. Eliminating the contact wrench
symbolically instead took ~245 ms per solve (integrator sensitivities); this takes a few ms.
"""

from __future__ import annotations

import math
import os

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from MPC_Humanoid.robots.servo_params import URDF_LIMITS, XH540_DAMPING, XH540_STIFFNESS

from .model import SOLE_HALF_LENGTH, SOLE_HALF_WIDTH, RigidContactModel, contact_config, rot_axis

BUILD_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../outputs/acados"))

N_NODES = 25
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
W_SQUEEZE, W_TILT, W_SOLE_Z, W_TERM = 1e-4, 1e3, 1e3, 1e3
# The leg-length and pelvis terms keep the NMPC from folding a knee or rolling the pelvis: at the
# straight-leg pose a knee bend is nearly invisible to the linearised contact model.
W_LEG, W_PELVIS = 1e4, 100.0


class ContactNMPC:
    def __init__(self, model: RigidContactModel, build: bool | None = None, name: str = "robinion_nmpc"):
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
        p = ca.vertcat(flags, cref, pc, isflat, ca.vec(Jk), ca.vec(Aik), gk, ck, qk)

        tau = XH540_STIFFNESS * (u - q[5:]) - XH540_DAMPING * qd[5:]
        b = M.Gj.T @ tau - M.gravity_expr
        gamma = gk + 2 * M.alpha * (Jk @ qd) + M.alpha**2 * (ck + Jk @ (q - qk) - cref)
        lam = -Aik @ (flags * (Jk @ (M.Minv @ b) + gamma))
        qdd = M.Minv @ (b + Jk.T @ lam)
        x = ca.vertcat(q, qd, u)
        xdot = ca.SX.sym("xdot", x.shape[0])
        f = ca.vertcat(qd, qdd, v)
        am = AcadosModel()
        am.name, am.x, am.u, am.xdot, am.p = name, x, v, xdot, p
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
            fz,  # 0-1
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
        sole_z = ca.vertcat(ca.sum2(corners[2, :4]), ca.sum2(corners[2, 4:])) / 4
        # sole centre height below the pelvis, in the pelvis frame
        Rb = rot_axis((0, 1, 0), q[4]) @ rot_axis((1, 0, 0), q[3])
        leg = ca.vertcat(*[(Rb.T @ (ca.sum2(corners[:, 4 * i : 4 * i + 4]) / 4 - q[:3]))[2] for i in (0, 1)])
        am.cost_y_expr = ca.vertcat(dcm, comd[:2], q[13], u, v, squeeze, ca.vec(tilt), sole_z, leg, q[3:5])
        am.cost_y_expr_e = ca.vertcat(dcm, comd[:2], ca.vec(tilt), sole_z, leg, q[3:5])

        qs, qds, pcs = ca.SX.sym("qs", nq), ca.SX.sym("qds", nq), ca.SX.sym("pcs", 6)
        self.f_alg = ca.Function(
            "alg", [qs, qds, pcs], [M.f_Jc(qs, pcs), M.f_crow(qs, pcs), M.f_Jdqd(qs, qds, pcs)]
        ).map(N_NODES + 1)

        self.u_nom = M.q0_joints.copy()
        q_init, cref0 = M.initial_state(self.u_nom)
        self.q_init = q_init
        leg0 = np.array(ca.Function("leg", [q], [leg])(q_init)).ravel()
        self.yref = np.r_[np.zeros(4), self.u_nom[8], self.u_nom, np.zeros(9 + 2 + 4 + 2), leg0, 0.0, 0.0]
        self.yref_e = np.r_[np.zeros(2 + 2 + 4 + 2), leg0, 0.0, 0.0]

        ocp = AcadosOcp()
        ocp.model = am
        ocp.solver_options.N_horizon = N_NODES
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
            + [W_SOLE_Z] * 2
            + [W_LEG] * 2
            + [W_PELVIS] * 2
        )
        ocp.cost.W_e = np.diag(
            [W_TERM] * 2 + [W_COMD] * 2 + [W_TILT] * 4 + [W_SOLE_Z] * 2 + [W_LEG] * 2 + [W_PELVIS] * 2
        )
        ocp.cost.yref, ocp.cost.yref_e = self.yref.copy(), self.yref_e.copy()
        flat_plan = [(["flat", "flat"], cref0)] * (N_NODES + 1)
        Q0, QD0 = np.tile(q_init, (N_NODES + 1, 1)), np.zeros((N_NODES + 1, nq))
        ocp.parameter_values = self._params(flat_plan, Q0, QD0)[0]
        ocp.constraints.lh, ocp.constraints.uh = self._bounds(("flat", "flat"))
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
        so = ocp.solver_options
        so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        so.hessian_approx = "GAUSS_NEWTON"
        # Radau IIA, 2 stages, one step per node: matches RK4 at 1 ms within 0.005 mm of CoM over the
        # horizon and stays stable for the -600 1/s mode of a foot in the air
        so.integrator_type = "IRK"
        so.collocation_type = "GAUSS_RADAU_IIA"
        so.sim_method_num_stages = 2
        so.sim_method_num_steps = 1
        so.sim_method_newton_iter = 1
        so.nlp_solver_type = "SQP_RTI"
        so.qp_solver_iter_max = 100
        ocp.code_export_directory = os.path.join(BUILD_DIR, name)
        json_file = os.path.join(ocp.code_export_directory, f"{name}.json")
        if build is None:
            build = not os.path.isfile(json_file)
        self.solver = AcadosOcpSolver(ocp, json_file=json_file, build=build, generate=build, verbose=False)
        self.X: np.ndarray | None = None
        self.reset(np.r_[q_init, np.zeros(nq), self.u_nom])

    def reset(self, x0: np.ndarray):
        """Initial guess x0 at every node; forget the previous plan."""
        for k in range(N_NODES + 1):
            self.solver.set(k, "x", x0)
        self.X = None

    def _params(self, plan, Q: np.ndarray, QD: np.ndarray) -> list[np.ndarray]:
        cfgs = [contact_config(*modes) for modes, _ in plan]
        J, c, g = (np.array(a) for a in self.f_alg(Q.T, QD.T, np.array([cf[1] for cf in cfgs]).T))
        params = []
        for k, (modes, cref) in enumerate(plan):
            flags, pc = cfgs[k]
            Jk = J[:, self.nq * k : self.nq * (k + 1)]
            A = flags[:, None] * (Jk @ self.m.Minv @ Jk.T) + np.diag(1.0 - flags)
            params.append(
                np.r_[
                    flags,
                    cref,
                    pc,
                    [float(m == "flat") for m in modes],
                    Jk.ravel(order="F"),
                    np.linalg.inv(A).ravel(order="F"),
                    g[:, k],
                    c[:, k],
                    Q[k],
                ]
            )
        return params

    def _bounds(self, modes) -> tuple[np.ndarray, np.ndarray]:
        lh, uh = np.full(self.nh, -1e4), np.full(self.nh, 1e4)
        for i, mode in enumerate(modes):
            if mode == "air":
                continue
            if mode != "flat":
                raise ValueError(f"edge mode {mode!r} cannot be planned")
            lh[i] = F_MIN
            lh[2 + 2 * i : 4 + 2 * i] = 0.0
            lh[6:8] = 0.0
            lh[8 + 4 * i : 12 + 4 * i] = 0.0
        lh[16:], uh[16:] = -self.effort, self.effort
        return lh, uh

    def solve(
        self, q: np.ndarray, qd: np.ndarray, u_prev: np.ndarray, plan, c_ref: np.ndarray, dt: float
    ) -> tuple[np.ndarray, int, float]:
        """One SQP-RTI iteration from state (q, qd) and the current servo targets u_prev.

        plan: N_NODES + 1 pairs (modes, contact references) per node. Returns the servo targets for the
        next tick, the QP status and the solver time [s].
        """
        x0 = np.r_[q, qd, u_prev]
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        if self.X is None:
            Q, QD = np.tile(q, (N_NODES + 1, 1)), np.tile(qd, (N_NODES + 1, 1))
        else:
            Q, QD = self.X[:, : self.nq].copy(), self.X[:, self.nq : 2 * self.nq].copy()
            Q[0], QD[0] = q, qd
        y, y_e = self.yref.copy(), self.yref_e.copy()
        y[:2] = y_e[:2] = c_ref
        for k, (p, (modes, _)) in enumerate(zip(self._params(plan, Q, QD), plan)):
            self.solver.set(k, "p", p)
            if k < N_NODES:
                self.solver.set(k, "yref", y)
                if k > 0:
                    lh, uh = self._bounds(modes)
                    self.solver.constraints_set(k, "lh", lh)
                    self.solver.constraints_set(k, "uh", uh)
        self.solver.set(N_NODES, "yref", y_e)
        status = self.solver.solve()
        self.X = np.array([self.solver.get(k, "x") for k in range(N_NODES + 1)])
        return u_prev + self.solver.get(0, "u") * dt, status, float(self.solver.get_stats("time_tot"))
