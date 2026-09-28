# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
State and contact-mode estimation for the rigid-contact NMPC.

Uses only what the real robot measures: joint encoders, the AHRS orientation and the gyro. There is
no contact sensing: the contact mode of each foot is inferred from sole tilt, sole tilt rate, sole
height and the normal force the model predicts, and the floating-base state follows from the
kinematics under the assumption that the feet in contact do not move.
"""

from __future__ import annotations

import math

import casadi as ca
import numpy as np
import pinocchio as pin

from .model import (
    CONTROLLER_URDF,
    MODEL_JOINTS,
    PASSIVE_COUPLING,
    RigidContactModel,
    compile_functions,
    landing_mode,
    leaves_edge,
    rot_axis,
)


class AhrsGyro:
    """Pelvis roll/pitch (model convention R = Ry(pitch) Rx(roll), yaw removed) and their rates from the
    AHRS quaternion, the gyro and the encoders. The IMU sits on upper_body_link, behind torso_pitch.

    compiled: the same computation as one generated C function (Pinocchio took ~0.2 ms per tick)."""

    def __init__(self, act_names: list[str], compiled: bool = False):
        self.model = pin.buildModelFromUrdf(CONTROLLER_URDF, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        act = [self.model.joints[self.model.getJointId(n)] for n in act_names]
        self._act_idx_q = np.array([j.idx_q for j in act])
        self._act_idx_v = np.array([j.idx_v for j in act])
        self._passive = []  # (idx_q, idx_v, index into the actuated vector, gain)
        for side in ("left", "right"):
            for passive, (active, gain) in PASSIVE_COUPLING.items():
                j = self.model.joints[self.model.getJointId(f"{side}_{passive}")]
                self._passive.append((j.idx_q, j.idx_v, list(act_names).index(f"{side}_{active}"), gain))
        self._imu = self.model.getFrameId("imu_link")
        self._f = self._compile(list(act_names)) if compiled else None

    def _compile(self, act_names: list[str]) -> ca.Function:
        m, n = self.model, len(act_names)
        q_act, qd_act = ca.SX.sym("q_act", n), ca.SX.sym("qd_act", n)
        axes = {"JointModelRX": (1, 0, 0), "JointModelRY": (0, 1, 0), "JointModelRZ": (0, 0, 1)}
        frame = m.frames[self._imu]
        R, j = ca.DM(frame.placement.rotation), frame.parentJoint
        while j > 1:  # the IMU in the base frame; joint 1 is the free flyer
            name = m.names[j]
            if name in act_names:
                qj = q_act[act_names.index(name)]
            else:
                side, rest = name.split("_", 1)
                active, gain = PASSIVE_COUPLING[rest]
                qj = gain * q_act[act_names.index(f"{side}_{active}")]
            R = ca.DM(m.jointPlacements[j].rotation) @ rot_axis(axes[m.joints[j].shortname()], qj) @ R
            j = m.parents[j]
        quat, gyro = ca.SX.sym("quat", 4), ca.SX.sym("gyro", 3)
        x, y, z, w = ca.vertsplit(quat / ca.norm_2(quat))
        Rq = ca.vertcat(
            ca.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            ca.horzcat(2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            ca.horzcat(2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )
        Rb = Rq @ R.T
        roll, pitch = ca.atan2(Rb[2, 1], Rb[2, 2]), -ca.asin(Rb[2, 0])
        S = R.T @ ca.reshape(ca.jacobian(ca.vec(R), q_act) @ qd_act, 3, 3)  # IMU rate relative to the base
        w_world = (
            rot_axis((0, 1, 0), pitch) @ rot_axis((1, 0, 0), roll) @ R @ (gyro - ca.vertcat(S[2, 1], S[0, 2], S[1, 0]))
        )
        roll_d = ca.cos(pitch) * w_world[0] - ca.sin(pitch) * w_world[2]
        f = ca.Function("ahrs", [q_act, qd_act, quat, gyro], [ca.vertcat(roll, pitch, roll_d, w_world[1])])
        return compile_functions([f], "robinion_ahrs")["ahrs"]

    def _configuration(self, q_act: np.ndarray, base_rotation: np.ndarray = np.eye(3)) -> np.ndarray:
        q = pin.neutral(self.model)
        q[3:7] = pin.Quaternion(base_rotation).coeffs()
        q[self._act_idx_q] = q_act
        for idx_q, _, src, gain in self._passive:
            q[idx_q] = gain * q_act[src]
        return q

    def __call__(
        self, q_act: np.ndarray, qd_act: np.ndarray, imu_quat_xyzw: np.ndarray, gyro: np.ndarray
    ) -> tuple[float, float, float, float]:
        """(roll, pitch, roll rate, pitch rate) of the pelvis."""
        if self._f is not None:
            r = np.array(self._f(q_act, qd_act, imu_quat_xyzw, gyro)).ravel()
            return float(r[0]), float(r[1]), float(r[2]), float(r[3])
        pin.framesForwardKinematics(self.model, self.data, self._configuration(q_act))
        R_base_imu = self.data.oMf[self._imu].rotation
        x, y, z, w = (float(v) for v in imu_quat_xyzw / np.linalg.norm(imu_quat_xyzw))
        R = pin.Quaternion(w, x, y, z).matrix() @ R_base_imu.T
        R = pin.utils.rotate("z", -math.atan2(R[1, 0], R[0, 0])) @ R
        roll, pitch = math.atan2(R[2, 1], R[2, 2]), -math.asin(float(np.clip(R[2, 0], -1.0, 1.0)))

        v = np.zeros(self.model.nv)
        v[self._act_idx_v] = qd_act
        for _, idx_v, src, gain in self._passive:
            v[idx_v] = gain * qd_act[src]
        pin.computeJointJacobians(self.model, self.data, self._configuration(q_act, R))
        J = pin.getFrameJacobian(self.model, self.data, self._imu, pin.LOCAL)[3:]
        w_world = R @ np.linalg.solve(J[:, 3:6], gyro - J[:, 6:] @ v[6:])
        # w = pitch_rate e_y + roll_rate Ry(pitch) e_x (+ yaw rate, ignored)
        A = np.c_[[math.cos(pitch), 0.0, -math.sin(pitch)], [0.0, 1.0, 0.0]]
        roll_d, pitch_d = np.linalg.lstsq(A, w_world, rcond=None)[0]
        return roll, pitch, float(roll_d), float(pitch_d)


def _edge(roll: float, pitch: float, on: float) -> str | None:
    if roll < -on:
        return "left_edge"
    if roll > on:
        return "right_edge"
    if pitch > on:
        return "toe"
    if pitch < -on:
        return "heel"
    return None


class ContactModeMonitor:
    """Contact mode per foot without contact sensing.

    From the AHRS + FK sole tilt, the sole tilt rate (zero while a sole is flat on the ground, so a
    rotation shows up ~15 ms before the tilt angle does), the sole heights by FK (a foot whose lowest
    corner is h_air above the other foot's is in the air; one in the air touches down within h_land)
    and the normal force the model predicts for the current modes (release): a foot can be unloaded
    while it hovers a millimetre above the ground, which kinematics cannot see.

    lead (predicted tilt, used while stepping): instead of the tilt rate beyond rate_on, a sole is flagged
    once tilt + lead * rate is beyond tilt_on. For a sole starting to roll at any constant rate that is lead
    earlier than the angle test, and a rate that brings a slightly tilted sole back to level never triggers.
    While stepping, the plain rate test flagged soles levelling out after a weight shift (3-5 deg/s at
    0.1 deg) and touchdown rocking (8 deg/s for 10 ms at 0.03 deg).

    rate_ticks: the rate (or predicted-tilt) test must name the same edge on this many consecutive ticks;
    the tilt-angle test stays immediate. A touchdown impact reads 12-19 deg/s on a single tick at zero
    tilt, flagging a flat sole.
    """

    def __init__(
        self,
        model: RigidContactModel,
        tilt_on: float = math.radians(0.2),
        rate_on: float = math.radians(3.0),
        h_air: float = 0.003,
        h_land: float = 0.0005,
        f_off: float = 10.0,
        f_on: float = 20.0,
        d_off: float = 0.003,
        lead: float | None = None,
        rate_ticks: int = 1,
    ):
        self.m = model
        self.tilt_on, self.rate_on, self.h_air, self.h_land = tilt_on, rate_on, h_air, h_land
        self.lead, self.rate_ticks = lead, rate_ticks
        self.f_off, self.f_on, self.d_off = f_off, f_on, d_off
        self.reset()

    def reset(self):
        self.tilt0 = np.zeros((2, 2))
        self.modes = ["flat", "flat"]
        self.landed = [False, False]
        self._rate_edge: list[tuple[str | None, int]] = [(None, 0), (None, 0)]

    def calibrate(self, q_meas: np.ndarray):
        """Store the sole tilt of the standing robot (both soles flat) as zero."""
        self.tilt0 = np.array(self.m.f_tilt(q_meas))

    def update(self, q_meas: np.ndarray, qd_meas: np.ndarray) -> tuple[list[str], list[bool]]:
        """Kinematic update. q_meas / qd_meas: [0, 0, 0, pelvis roll, pitch, model joints] and their
        rates. Returns the modes and which feet changed mode."""
        tilt = np.array(self.m.f_tilt(q_meas)) - self.tilt0
        rate = np.array(self.m.f_tilt_rate(q_meas, qd_meas))
        P = np.array(self.m.f_corners(q_meas))
        low = np.array([P[2, :4].min(), P[2, 4:].min()])
        new = list(self.modes)
        for i in (0, 1):
            (roll, pitch), (rr, pr), mode = tilt[:, i], rate[:, i], self.modes[i]
            if low[i] - low[1 - i] > self.h_air:
                new[i] = "air"
            elif mode == "air":
                if low[i] - low[1 - i] <= self.h_land:
                    new[i] = landing_mode(roll, pitch, self.tilt_on)
            elif mode == "flat":
                if self.lead is None:
                    ri, pi, on = rr, pr, self.rate_on
                else:
                    ri, pi, on = roll + self.lead * rr, pitch + self.lead * pr, self.tilt_on
                edge = _edge(ri, pi, on)
                last, n = self._rate_edge[i]
                n = n + 1 if edge is not None and edge == last else int(edge is not None)
                self._rate_edge[i] = (edge, n)
                if n < self.rate_ticks:
                    ri = pi = 0.0
                if roll < -self.tilt_on or ri < -on:
                    new[i] = "left_edge"
                elif roll > self.tilt_on or ri > on:
                    new[i] = "right_edge"
                elif pitch > self.tilt_on or pi > on:
                    new[i] = "toe"
                elif pitch < -self.tilt_on or pi < -on:
                    new[i] = "heel"
            elif leaves_edge(mode, roll, pitch):
                new[i] = "flat"
            if new[i] != "flat":
                self._rate_edge[i] = (None, 0)
        changed = [a != b for a, b in zip(new, self.modes)]
        self.landed = [a == "air" and b != "air" for a, b in zip(self.modes, new)]
        self.modes = new
        return new, changed

    def release(
        self, fz: np.ndarray, q_meas: np.ndarray, cref: np.ndarray, pc: np.ndarray
    ) -> tuple[list[str], list[bool]]:
        """Force update: a foot in contact whose predicted normal force is below f_off (f_on for a foot
        that touched down this tick, so a hovering foot does not chatter) goes to the air, as does the
        less loaded foot once the contact points measured by encoders + AHRS have moved more than d_off
        relative to each other (both feet on the ground keep that distance). One foot always stays on
        the ground."""
        new = list(self.modes)
        contact = [i for i in (0, 1) if new[i] != "air"]
        off = [i for i in contact if fz[i] < (self.f_on if self.landed[i] else self.f_off)]
        if len(contact) == 2 and not off:
            c = np.array(self.m.f_crow(q_meas, pc)).ravel()
            drift = (c[5:7] - c[0:2]) - (np.asarray(cref)[5:7] - np.asarray(cref)[0:2])
            if np.linalg.norm(drift) > self.d_off:
                off = [int(np.argmin(fz))]
        if len(off) == len(contact):
            off = []
        for i in off:
            new[i] = "air"
        changed = [a != b for a, b in zip(new, self.modes)]
        self.modes = new
        return new, changed


class ContactProjectionEstimator:
    """Model state (q, qd) from encoders + AHRS roll/pitch (+ rates) under given contact modes.

    Weighted least squares onto the contact manifold, so the state always satisfies the active contact
    rows. The base position is set only by the contacts; the AHRS roll/pitch are a weak prior that
    matters only for rotations the active contacts leave free (e.g. pitch while the soles roll on their
    toes).

    compiled: the whole update as one generated C function; the KKT systems then keep all 10 contact rows
    and an inactive row reads lambda_i = 0, which gives the same solution as dropping it.
    """

    def __init__(self, model: RigidContactModel, w_tilt: float = 0.01, iters: int = 2, compiled: bool = False):
        self.m, self.iters = model, iters
        self.w = np.r_[np.full(3, 1e-8), np.full(2, w_tilt), np.ones(len(MODEL_JOINTS))]
        self.q: np.ndarray | None = None
        self._f = self._compile() if compiled else None

    def _compile(self) -> ca.Function:
        m, n = self.m, self.m.nq
        crow, Jc = m.raw_sx["crow"], m.raw_sx["Jc"]
        w = ca.DM(self.w)
        base, meas, meas_d = ca.SX.sym("base", 3), ca.SX.sym("meas", n - 3), ca.SX.sym("meas_d", n - 3)
        flags, cref, pc = ca.SX.sym("flags", 10), ca.SX.sym("cref", 10), ca.SX.sym("pc", 6)

        def kkt(J, top, bot):
            K = ca.vertcat(ca.horzcat(ca.diag(w), (J * flags).T), ca.horzcat(J * flags, ca.diag(1 - flags)))
            return ca.solve(K, ca.vertcat(top, flags * bot))[:n]

        qm = ca.vertcat(base, meas)
        q = qm
        for _ in range(self.iters):
            q = q + kkt(Jc(q, pc), w * (qm - q), -(crow(q, pc) - cref))
        qd = kkt(Jc(q, pc), w * ca.vertcat(ca.DM.zeros(3), meas_d), ca.DM.zeros(10))
        f = ca.Function("project", [base, meas, meas_d, flags, cref, pc], [q, qd])
        return compile_functions([f], "robinion_projection")["project"]

    def reset(self):
        self.q = None

    def _kkt(self, J: np.ndarray, rhs_top: np.ndarray, rhs_bot: np.ndarray) -> np.ndarray:
        n, k = self.m.nq, J.shape[0]
        K = np.zeros((n + k, n + k))
        K[:n, :n] = np.diag(self.w)
        K[:n, n:] = J.T
        K[n:, :n] = J
        return np.linalg.solve(K, np.r_[rhs_top, rhs_bot])[:n]

    def update(
        self,
        joints: np.ndarray,
        roll: float,
        pitch: float,
        joints_d: np.ndarray,
        roll_d: float,
        pitch_d: float,
        flags: np.ndarray,
        cref: np.ndarray,
        pc: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        a = np.asarray(flags) > 0.5
        qm = np.r_[0.0, 0.0, 0.0, roll, pitch, joints]
        if self.q is None:
            self.q = qm.copy()
            self.q[2] = -np.array(self.m.f_crow(qm, pc)).ravel()[[2, 7]].mean()
        if self._f is not None:
            q, qd = self._f(self.q[:3], qm[3:], np.r_[roll_d, pitch_d, joints_d], flags, cref, pc)
            self.q = np.array(q).ravel()
            return self.q, np.array(qd).ravel()
        q = np.r_[self.q[:3], qm[3:]]
        qm[:3] = self.q[:3]
        for _ in range(self.iters):
            c = np.array(self.m.f_crow(q, pc)).ravel()[a] - np.asarray(cref)[a]
            J = np.array(self.m.f_Jc(q, pc))[a]
            q = q + self._kkt(J, self.w * (qm - q), -c)
        J = np.array(self.m.f_Jc(q, pc))[a]
        qd = self._kkt(J, self.w * np.r_[0.0, 0.0, 0.0, roll_d, pitch_d, joints_d], np.zeros(a.sum()))
        self.q = q
        return q, qd
