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

from MPC_Humanoid.robots.robonion_params import PASSIVE_JOINTS

from .model import (
    CONTROLLER_URDF,
    JOINT_AXES,
    RigidContactModel,
    compile_functions,
    rot_axis,
)


class AhrsGyro:
    """Pelvis roll/pitch (model convention R = Rz(yaw) Ry(pitch) Rx(roll), yaw removed), their rates and the yaw rate
    from the AHRS quaternion, the gyro and the encoders. The IMU sits on upper_body_link, behind torso_pitch.

    compiled: the same computation as one generated C function (Pinocchio took ~0.2 ms per tick). Its inputs are in
    sorted joint-name order, so the build does not depend on the order the caller delivers the joints in."""

    def __init__(self, act_names: list[str], build: bool = False):
        act_names = list(act_names)
        self.model = pin.buildModelFromUrdf(CONTROLLER_URDF, pin.JointModelFreeFlyer())
        self._imu = self.model.getFrameId("imu_link")
        order = sorted(act_names)
        self._perm = np.array([act_names.index(n) for n in order])
        self._f = self._compile(order, build)

    def _compile(self, act_names: list[str], build: bool) -> ca.Function:
        m, n = self.model, len(act_names)
        q_act, qd_act = ca.SX.sym("q_act", n), ca.SX.sym("qd_act", n)
        frame = m.frames[self._imu]
        R, j = ca.DM(frame.placement.rotation), frame.parentJoint
        while j > 1:  # the IMU in the base frame; joint 1 is the free flyer
            name = m.names[j]
            if name in act_names:
                qj = q_act[act_names.index(name)]
            else:
                active, gain = PASSIVE_JOINTS[name]
                qj = gain * q_act[act_names.index(active)]
            R = ca.DM(m.jointPlacements[j].rotation) @ rot_axis(JOINT_AXES[m.joints[j].shortname()], qj) @ R
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
        yaw_d = w_world[2] + ca.sin(pitch) * roll_d

        f = ca.Function("ahrs", [q_act, qd_act, quat, gyro], [ca.vertcat(roll, pitch, roll_d, w_world[1], yaw_d)])
        return compile_functions([f], "robonion_ahrs", build)["ahrs"]

    def __call__(
        self, q_act: np.ndarray, qd_act: np.ndarray, imu_quat_xyzw: np.ndarray, gyro: np.ndarray
    ) -> tuple[float, float, float, float, float]:
        """(roll, pitch, roll rate, pitch rate, yaw rate) of the pelvis."""
        r = np.array(self._f(q_act[self._perm], qd_act[self._perm], imu_quat_xyzw, gyro)).ravel()
        return float(r[0]), float(r[1]), float(r[2]), float(r[3]), float(r[4])


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

    settle_ticks: for this many ticks from the touchdown of one foot (that tick included), the other is not
    released by force or drift: the predicted force uses servo targets solved before the touchdown.
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
        settle_ticks: int = 0,
    ):
        self.m = model
        self.tilt_on, self.rate_on, self.lead, self.rate_ticks = tilt_on, rate_on, lead, rate_ticks
        self.settle_ticks = settle_ticks
        self.h_air, self.h_land = h_air, h_land
        self.f_off, self.f_on, self.d_off = f_off, f_on, d_off

        self.reset()

    def reset(self):
        self.tilt0 = np.zeros((2, 2))
        self.modes = ["flat", "flat"]
        self.landed = [False, False]
        self._since_landed = [self.settle_ticks, self.settle_ticks]
        self._rate_edge: list[tuple[str | None, int]] = [(None, 0), (None, 0)]

    def calibrate(self, q_meas: np.ndarray):
        """Store the sole tilt of the standing robot (both soles flat) as zero."""
        self.tilt0 = np.array(self.m.f_tilt(q_meas))

    def update(self, q_meas: np.ndarray, qd_meas: np.ndarray) -> tuple[list[str], list[bool]]:
        """Kinematic update. q_meas / qd_meas: [0, 0, 0, pelvis roll, pitch, (yaw), model joints] and their
        rates. Returns the modes and which feet changed mode."""
        tilt = np.array(self.m.f_tilt(q_meas)) - self.tilt0
        rate = np.array(self.m.f_tilt_rate(q_meas, qd_meas))
        P = np.array(self.m.f_corners(q_meas))
        low = np.array([P[2, :4].min(), P[2, 4:].min()])

        new = list(self.modes)
        for i in (0, 1):
            (roll, pitch), (roll_rate, pitch_rate), mode = tilt[:, i], rate[:, i], self.modes[i]
            if low[i] - low[1 - i] > self.h_air:
                new[i] = "air"
            elif mode == "air":
                if low[i] - low[1 - i] <= self.h_land:
                    new[i] = landing_mode(roll, pitch, self.tilt_on)
            elif mode == "flat":
                if self.lead is None:
                    roll_test, pitch_test, on = roll_rate, pitch_rate, self.rate_on
                else:
                    roll_test = roll + self.lead * roll_rate
                    pitch_test = pitch + self.lead * pitch_rate
                    on = self.tilt_on

                edge = _edge(roll_test, pitch_test, on)
                last, n = self._rate_edge[i]
                n = n + 1 if edge is not None and edge == last else int(edge is not None)
                self._rate_edge[i] = (edge, n)
                if n < self.rate_ticks:
                    roll_test = pitch_test = 0.0

                if roll < -self.tilt_on or roll_test < -on:
                    new[i] = "left_edge"
                elif roll > self.tilt_on or roll_test > on:
                    new[i] = "right_edge"
                elif pitch > self.tilt_on or pitch_test > on:
                    new[i] = "toe"
                elif pitch < -self.tilt_on or pitch_test < -on:
                    new[i] = "heel"
            elif leaves_edge(mode, roll, pitch):
                new[i] = "flat"
            if new[i] != "flat":
                self._rate_edge[i] = (None, 0)

        changed = [a != b for a, b in zip(new, self.modes)]
        self.landed = [a == "air" and b != "air" for a, b in zip(self.modes, new)]
        self._since_landed = [0 if lnd else n + 1 for lnd, n in zip(self.landed, self._since_landed)]
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
            cref = np.asarray(cref)
            nr = self.m.nr
            drift = (c[nr : nr + 2] - c[0:2]) - (cref[nr : nr + 2] - cref[0:2])
            if np.linalg.norm(drift) > self.d_off:
                off = [int(np.argmin(fz))]

        off = [i for i in off if self._since_landed[1 - i] >= self.settle_ticks]
        if len(off) == len(contact):
            off = []
        for i in off:
            new[i] = "air"

        changed = [a != b for a, b in zip(new, self.modes)]
        self.modes = new
        return new, changed


class ContactProjectionEstimator:
    """Model state (q, qd) from encoders + base roll/pitch/yaw (+ rates) under given contact modes.

    Weighted least squares onto the contact manifold, so the state always satisfies the active contact
    rows. The base position is set only by the contacts; the measured base orientation is a weak prior that
    matters only for rotations the active contacts leave free (e.g. pitch while the soles roll on their
    toes).

    compiled: the whole update as one generated C function; the KKT systems then keep all contact rows
    and an inactive row reads lambda_i = 0, which gives the same solution as dropping it.
    """

    def __init__(
        self,
        model: RigidContactModel,
        w_tilt: float = 0.01,
        iters: int = 2,
        build: bool = False,
    ):
        self.m, self.iters = model, iters
        self.w = np.r_[np.full(3, 1e-8), np.full(model.nb - 3, w_tilt), np.ones(model.nj)]
        self.q: np.ndarray | None = None
        self._f = self._compile(build)

    def _compile(self, build: bool) -> ca.Function:
        m, n = self.m, self.m.nq
        crow, Jc = m.raw_sx["crow"], m.raw_sx["Jc"]
        w = ca.DM(self.w)
        base, meas, meas_d = ca.SX.sym("base", 3), ca.SX.sym("meas", n - 3), ca.SX.sym("meas_d", n - 3)
        nr2 = 2 * m.nr
        flags, cref, pc = ca.SX.sym("flags", nr2), ca.SX.sym("cref", nr2), ca.SX.sym("pc", 6)

        def kkt(J, top, bot):
            K = ca.vertcat(ca.horzcat(ca.diag(w), (J * flags).T), ca.horzcat(J * flags, ca.diag(1 - flags)))
            return ca.solve(K, ca.vertcat(top, flags * bot))[:n]

        qm = ca.vertcat(base, meas)
        q = qm
        for _ in range(self.iters):
            q = q + kkt(Jc(q, pc), w * (qm - q), -(crow(q, pc) - cref))
        qd = kkt(Jc(q, pc), w * ca.vertcat(ca.DM.zeros(3), meas_d), ca.DM.zeros(nr2))

        f = ca.Function("project", [base, meas, meas_d, flags, cref, pc], [q, qd])
        return compile_functions([f], "robonion_projection", build)["project"]

    def reset(self):
        self.q = None

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
        yaw: float = 0.0,
        yaw_d: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """yaw, yaw_d: base yaw and its rate, used by a model with yaw."""
        m = self.m
        ori, ori_d = ([roll, pitch, yaw], [roll_d, pitch_d, yaw_d]) if m.yaw else ([roll, pitch], [roll_d, pitch_d])
        qm = np.r_[0.0, 0.0, 0.0, ori, joints]
        if self.q is None:
            self.q = qm.copy()
            self.q[2] = -np.array(m.f_crow(qm, pc)).ravel()[[2, m.nr + 2]].mean()

        q, qd = self._f(self.q[:3], qm[3:], np.r_[ori_d, joints_d], flags, cref, pc)
        self.q = np.array(q).ravel()
        return self.q, np.array(qd).ravel()
