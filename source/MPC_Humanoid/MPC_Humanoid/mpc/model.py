# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Rigid-contact model of Robonion for the NMPC.

q = [base x, y, z, roll, pitch, (yaw), MODEL_JOINTS, (HIP_YAW_JOINTS), (ARM_JOINTS)], base rotation
Rz(yaw) Ry(pitch) Rx(roll). With yaw, hip_yaw and arms off this is the stage-2 model:
q = [base x, y, z, roll, pitch, L hip_roll, L front_thigh, L ankle_pitch, L ankle_roll,
     R hip_roll, R front_thigh, R ankle_pitch, R ankle_roll, torso_pitch]   (14)

Joints not in the model (head, and hip yaw / arms when off) stay at their defaults; the passive parallelogram joints
follow PASSIVE_COUPLING. The mass matrix is frozen at the standing pose; gravity, contact kinematics and servo
torques are exact. Each sole has 5 or 6 contact rows: position of a contact point (3), z of the sole y axis (roll),
z of the sole x axis (pitch) and, with yaw, y of the sole x axis (yaw). The contact point and a flag per row select
the contact mode of each foot (flat, rolling on an edge, in the air): an active row is a Baumgarte-stabilised
acceleration constraint, an inactive row has lambda_i = 0. The position-mode servos leave the contact wrench no
freedom, so it is eliminated in closed form, lambda(q, qd, u).
"""

from __future__ import annotations

import hashlib
import os
import subprocess

import casadi as ca
import numpy as np
import pinocchio as pin

from MPC_Humanoid.robots.robonion_params import servo_of

CONTROLLER_URDF = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../../assets/robonionv2_controller.urdf")
)
BUILD_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../outputs/acados"))
BUILD_HINT = "run `uv run python scripts/build_controller.py` first (no Isaac needed)"

PASSIVE_COUPLING = {  # parallelogram (4-bar) passive joint suffix: (actuated joint suffix, gain)
    "knee_pitch_joint": ("front_thigh_pitch_joint", -1.0),
    "back_thigh_pitch_joint": ("front_thigh_pitch_joint", 1.0),
    "front_shin_pitch_joint": ("ankle_pitch_joint", -1.0),
    "back_shin_pitch_joint": ("ankle_pitch_joint", -1.0),
}

GRAVITY = 9.81

JOINT_AXES = {"JointModelRX": (1, 0, 0), "JointModelRY": (0, 1, 0), "JointModelRZ": (0, 0, 1)}

MODEL_JOINTS = [
    f"{side}_{joint}"
    for side in ("left", "right")
    for joint in ("hip_roll_joint", "front_thigh_pitch_joint", "ankle_pitch_joint", "ankle_roll_joint")
] + ["torso_pitch_joint"]
HIP_YAW_JOINTS = ["left_hip_yaw_joint", "right_hip_yaw_joint"]
ARM_JOINTS = [
    f"{side}_{joint}"
    for side in ("left", "right")
    for joint in ("shoulder_pitch_joint", "shoulder_roll_joint", "elbow_yaw_joint", "elbow_pitch_joint")
]

# Sole corners in *_foot_roll_link, from the foot mesh *_foot_visual.stl: +0.068 / -0.107 m along x
# from the ankle-roll axis, +-0.0335 m along y, 0.056 m below the axis (flat rectangular sole).
SOLE_CORNERS = np.array([[x, y, -0.056] for x in (0.068, -0.107) for y in (0.0335, -0.0335)])
SOLE_CENTRE = SOLE_CORNERS.mean(0)
SOLE_HALF_LENGTH = 0.0875
SOLE_HALF_WIDTH = 0.0335

# per foot: contact point in *_foot_roll_link, flags for [x, y, z, roll row, pitch row, yaw row]; a sole rolling
# on an edge still resists yaw
CONTACT_MODES = {
    "flat": (SOLE_CENTRE, (1, 1, 1, 1, 1, 1)),
    "toe": (np.r_[SOLE_CORNERS[:, 0].max(), 0.0, SOLE_CENTRE[2]], (1, 1, 1, 1, 0, 1)),
    "heel": (np.r_[SOLE_CORNERS[:, 0].min(), 0.0, SOLE_CENTRE[2]], (1, 1, 1, 1, 0, 1)),
    "left_edge": (SOLE_CENTRE + [0, SOLE_HALF_WIDTH, 0], (1, 1, 1, 0, 1, 1)),
    "right_edge": (SOLE_CENTRE - [0, SOLE_HALF_WIDTH, 0], (1, 1, 1, 0, 1, 1)),
    "air": (SOLE_CENTRE, (0, 0, 0, 0, 0, 0)),
}
EDGE_MODES = ("toe", "heel", "left_edge", "right_edge")


def rot_axis(axis, angle):
    """Rotation matrix about a unit axis (CasADi)."""
    c, s = ca.cos(angle), ca.sin(angle)
    x, y, z = axis
    C = 1 - c
    return ca.vertcat(
        ca.horzcat(c + x * x * C, x * y * C - z * s, x * z * C + y * s),
        ca.horzcat(y * x * C + z * s, c + y * y * C, y * z * C - x * s),
        ca.horzcat(z * x * C - y * s, z * y * C + x * s, c + z * z * C),
    )


def landing_mode(roll: float, pitch: float, tilt_on: float) -> str:
    """Mode of a foot that touches down with the given sole tilt: level -> flat, else the lower edge."""
    if max(abs(roll), abs(pitch)) < tilt_on:
        return "flat"
    if abs(roll) >= abs(pitch):
        return "left_edge" if roll < 0 else "right_edge"
    return "toe" if pitch > 0 else "heel"


def leaves_edge(mode: str, roll: float, pitch: float) -> bool:
    """True once a foot rolling on an edge is level again (its tilt crossed zero)."""
    return (
        (mode == "toe" and pitch <= 0)
        or (mode == "heel" and pitch >= 0)
        or (mode == "left_edge" and roll >= 0)
        or (mode == "right_edge" and roll <= 0)
    )


def compile_functions(functions: list[ca.Function], prefix: str, build: bool = False) -> dict[str, ca.Function]:
    """CasADi functions as generated C in one shared library under outputs/acados/. The file name is a hash of
    the serialized functions, so a changed model never loads a stale build. build: compile the library if it is
    missing (scripts/build_controller.py); otherwise a missing library is an error."""
    key = hashlib.sha1("".join(f.serialize() for f in functions).encode()).hexdigest()[:12]
    name = f"{prefix}_{key}"
    so = os.path.join(BUILD_DIR, f"{name}.so")

    if not os.path.isfile(so):
        if not build:
            raise FileNotFoundError(f"{so} not found: {BUILD_HINT}")
        os.makedirs(BUILD_DIR, exist_ok=True)

        cg = ca.CodeGenerator(f"{name}.c")
        for f in functions:
            cg.add(f)
        cg.generate(BUILD_DIR + os.sep)
        c_file = os.path.join(BUILD_DIR, f"{name}.c")
        subprocess.run(["gcc", "-O3", "-march=native", "-shared", "-fPIC", c_file, "-o", so + ".tmp"], check=True)
        os.replace(so + ".tmp", so)

    return {f.name(): ca.external(f.name(), so) for f in functions}


class RigidContactModel:
    """Floating base + model joints with rigid sole contacts; see the module docstring."""

    def __init__(
        self,
        act_names: list[str],
        q0_act: np.ndarray,
        alpha: float = 20.0,
        compiled: bool = False,
        build: bool = False,
        yaw: bool = True,
        hip_yaw: bool = True,
        arms: bool = True,
    ):
        """compiled: the f_* are generated C (one shared library) instead of the CasADi VM; symbolic users
        (the NMPC) take the CasADi functions from raw_sx. build: see compile_functions. yaw, hip_yaw, arms: pelvis
        yaw (with a yaw row per sole), the hip yaw joints and the 8 arm joints as model coordinates."""
        self.yaw, self.hip_yaw, self.arms = yaw, hip_yaw, arms
        nb = self.nb = 6 if yaw else 5
        nr = self.nr = 6 if yaw else 5
        joints = self.joints = MODEL_JOINTS + (HIP_YAW_JOINTS if hip_yaw else []) + (ARM_JOINTS if arms else [])
        self.n_legs = len(MODEL_JOINTS) + (len(HIP_YAW_JOINTS) if hip_yaw else 0)
        nj = self.nj = len(joints)
        servos = [servo_of(n) for n in joints]
        self.effort, self.rate_limit = np.array([s.effort for s in servos]), np.array([s.velocity for s in servos])
        self.stiffness, self.damping = np.array([s.stiffness for s in servos]), np.array([s.damping for s in servos])
        self.armature = np.array([s.armature for s in servos])
        nq = self.nq = nb + nj

        m = pin.buildModelFromUrdf(CONTROLLER_URDF, pin.JointModelFreeFlyer())
        act_names = list(act_names)
        qn = pin.neutral(m)
        for n, v in zip(act_names, q0_act):
            qn[m.joints[m.getJointId(n)].idx_q] = v
        for side in ("left", "right"):
            for passive, (a, g) in PASSIVE_COUPLING.items():
                qn[m.joints[m.getJointId(f"{side}_{passive}")].idx_q] = (
                    g * qn[m.joints[m.getJointId(f"{side}_{a}")].idx_q]
                )

        jmap = {n: (nb + i, 1.0) for i, n in enumerate(joints)}
        for side in ("left", "right"):
            for passive, (a, g) in PASSIVE_COUPLING.items():
                jmap[f"{side}_{passive}"] = (jmap[f"{side}_{a}"][0], g)

        q = ca.SX.sym("q", nq)
        qd = ca.SX.sym("qd", nq)
        u = ca.SX.sym("u", nj)
        flags = ca.SX.sym("flags", 2 * nr)
        cref = ca.SX.sym("cref", 2 * nr)
        pc = ca.SX.sym("pc", 6)
        f_ext = ca.SX.sym("f_ext", 3)  # at the upper-body CoM (push tests)

        Rb = rot_axis((0, 1, 0), q[4]) @ rot_axis((1, 0, 0), q[3])
        if yaw:
            Rb = rot_axis((0, 0, 1), q[5]) @ Rb
        R, p = {1: Rb}, {1: q[:3]}
        for j in range(2, m.njoints):
            P, par, n = m.jointPlacements[j], m.parents[j], m.names[j]
            sn = m.joints[j].shortname()
            axis = (
                tuple(float(a) for a in m.joints[j].extract().axis)
                if sn == "JointModelRevoluteUnaligned"
                else JOINT_AXES[sn]
            )
            qj = jmap[n][1] * q[jmap[n][0]] if n in jmap else float(qn[m.joints[j].idx_q])
            p[j] = p[par] + R[par] @ ca.DM(P.translation)
            R[j] = R[par] @ ca.DM(P.rotation) @ rot_axis(axis, qj)

        self.mass = sum(inertia.mass for inertia in m.inertias)
        com = sum(m.inertias[j].mass * (p[j] + R[j] @ ca.DM(m.inertias[j].lever)) for j in range(1, m.njoints))
        com = com / self.mass

        rows, points, axes, tilts, corners, sole_yaw = [], [], [], [], [], []
        for i, side in enumerate(("left", "right")):
            j = m.getJointId(f"{side}_ankle_roll_joint")
            Rf = R[j]
            c = p[j] + Rf @ pc[3 * i : 3 * i + 3]
            rows += [c, Rf[2, 1], Rf[2, 0]] + ([Rf[1, 0]] if yaw else [])
            points.append(c)
            axes.append((Rf[:, 0], Rf[:, 1]))
            tilts.append(ca.vertcat(ca.atan2(Rf[2, 1], Rf[2, 2]), -ca.asin(Rf[2, 0])))
            sole_yaw.append(ca.atan2(Rf[1, 0], Rf[0, 0]))
            corners += [p[j] + Rf @ ca.DM(k) for k in SOLE_CORNERS]
        crow = ca.vertcat(*rows)
        Jc = ca.jacobian(crow, q)
        cdot = Jc @ qd
        gamma = ca.jacobian(cdot, q) @ qd + 2 * alpha * cdot + alpha**2 * (crow - cref)

        Gj = np.zeros((nj, nq))
        Gj[:, nb:] = np.eye(nj)
        tau = ca.DM(self.stiffness) * (u - Gj @ q) - ca.DM(self.damping) * (Gj @ qd)

        data = m.createData()
        Mfull = pin.crba(m, data, qn)
        Mfull = np.triu(Mfull) + np.triu(Mfull, 1).T

        T = np.zeros((m.nv, nq))
        T[0:3, 0:3] = np.eye(3)
        T[3, 3] = T[4, 4] = 1.0
        if yaw:
            T[5, 5] = 1.0
        for n, (iq, g) in jmap.items():
            T[m.joints[m.getJointId(n)].idx_v, iq] = g
        M0 = T.T @ Mfull @ T + Gj.T @ np.diag(self.armature) @ Gj
        Minv = np.linalg.inv(M0)

        tj = m.getJointId("torso_pitch_joint")
        c_torso = p[tj] + R[tj] @ ca.DM(m.inertias[tj].lever)
        gravity = ca.jacobian(self.mass * GRAVITY * com[2], q).T
        b = Gj.T @ tau - gravity + ca.jacobian(c_torso, q).T @ f_ext

        A = ca.diag(flags) @ (Jc @ Minv @ Jc.T) + ca.diag(1 - flags)
        lam = ca.solve(A, -flags * (Jc @ (Minv @ b) + gamma))
        qdd = Minv @ (b + Jc.T @ lam)

        W = ca.horzcat(*[self.wrench_of(lam[nr * i : nr * (i + 1)], axes[i]) for i in (0, 1)])
        C = ca.horzcat(*points)

        self.alpha, self.Gj, self.M0, self.Minv = alpha, Gj, M0, Minv
        self.q_sym, self.qd_sym, self.u_sym = q, qd, u
        self.flags_sym, self.cref_sym, self.pc_sym, self.fext_sym = flags, cref, pc, f_ext
        self.com_expr, self.comd_expr = com, ca.jacobian(com, q) @ qd
        self.gravity_expr = gravity
        self.points_expr, self.axes_expr = C, axes  # contact points; per foot (sole x axis, sole y axis)
        self.tilt_expr = ca.horzcat(*tilts)  # (roll, pitch) x (left, right)
        self.sole_yaw_expr = ca.vertcat(*sole_yaw)
        self.corners_expr = ca.horzcat(*corners)  # 3 x 8, left then right

        args = [q, qd, u, flags, cref, pc, f_ext]
        tilt_rate = ca.reshape(ca.jacobian(ca.vec(self.tilt_expr), q) @ qd, 2, 2)
        functions = [
            ca.Function("qdd", args, [ca.cse(qdd)]),
            ca.Function("wrench", args, [W]),
            ca.Function("crow", [q, pc], [crow]),
            ca.Function("Jc", [q, pc], [Jc]),
            ca.Function("Jdqd", [q, qd, pc], [ca.jacobian(cdot, q) @ qd]),
            ca.Function("com", [q], [com]),
            ca.Function("comd", [q, qd], [self.comd_expr]),
            ca.Function("tilt", [q], [self.tilt_expr]),
            ca.Function("tilt_rate", [q, qd], [tilt_rate]),
            ca.Function("sole_yaw", [q], [self.sole_yaw_expr]),
            ca.Function("corners", [q], [self.corners_expr]),
        ]

        self.raw_sx = {f.name(): f for f in functions}
        if compiled:
            functions = [f for f in functions if f.name() != "qdd"]  # qdd is only used symbolically / offline
            functions = list(compile_functions(functions, "robonion_model", build).values()) + [self.raw_sx["qdd"]]
        for f in functions:
            setattr(self, f"f_{f.name()}", f)

        self.q0_joints = np.array([qn[m.joints[m.getJointId(n)].idx_q] for n in joints])

    def wrench_of(self, lam_i, axes_i):
        """Fx Fy Fz Mx My (Mz) about the contact point from one foot's row multipliers: the row e_a^T R e_b
        carries the moment lam (R e_b) x e_a."""
        ex, ey = axes_i
        w = ca.vertcat(lam_i[:3], lam_i[3] * ey[1] + lam_i[4] * ex[1], -(lam_i[3] * ey[0] + lam_i[4] * ex[0]))
        if self.yaw:
            w[3] -= lam_i[5] * ex[2]
            w = ca.vertcat(w, lam_i[5] * ex[0])
        return w

    def contact_config(self, mode_left: str, mode_right: str) -> tuple[np.ndarray, np.ndarray]:
        """Row flags (2 nr) and contact points (6) for a pair of foot modes.

        Sole pitch equals pelvis pitch through the leg parallelograms, so two active pitch rows are identical; the
        right one is then switched off and the left one carries the summed pitch moment. Without hip yaw the same
        holds for the yaw rows."""
        nr = self.nr
        (pl, fl), (pr, fr) = CONTACT_MODES[mode_left], CONTACT_MODES[mode_right]
        flags = np.r_[fl[:nr], fr[:nr]].astype(float)
        if flags[4] and flags[nr + 4]:
            flags[nr + 4] = 0.0
        if self.yaw and not self.hip_yaw and flags[5] and flags[nr + 5]:
            flags[nr + 5] = 0.0
        return flags, np.r_[pl, pr]

    def contact_refs(self, q: np.ndarray, pc: np.ndarray) -> np.ndarray:
        """Contact references for a configuration: contact points and sole yaw where they are, on the ground
        (z = 0), soles level about the constrained axes."""
        cref = np.array(self.f_crow(q, pc)).ravel()
        for i in (0, 1):
            cref[self.nr * i + 2 : self.nr * i + 5] = 0.0
        return cref

    def initial_state(self, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Base placed so both flat soles touch z = 0; returns q and the contact references."""
        _, pc = self.contact_config("flat", "flat")
        q = np.zeros(self.nq)
        q[self.nb :] = joints
        c = np.array(self.f_crow(q, pc)).ravel()
        q[2] = -0.5 * (c[2] + c[self.nr + 2])
        return q, self.contact_refs(q, pc)

    def update_refs(self, cref: np.ndarray, q: np.ndarray, pc: np.ndarray, changed: list[bool]) -> np.ndarray:
        """Contact references after a mode change: feet that changed mode take their contact point
        where it is now, the others keep their references."""
        fresh = self.contact_refs(q, pc)
        cref = np.array(cref, dtype=float)
        nr = self.nr
        for i in (0, 1):
            if changed[i]:
                cref[nr * i : nr * (i + 1)] = fresh[nr * i : nr * (i + 1)]
        return cref
