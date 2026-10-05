# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Robonion controller

Timeline: standing at the default pose, crouch ramp (straight legs are singular and too slow for the swing),
AHRS bias calibration with both soles flat, then the gait plan (gait.py) runs through RobonionNMPC every tick.

Every tick: AHRS/gyro -> contact modes (ContactModeMonitor) -> base position and velocity (LegKinematicsEkf) ->
joints and tilt on the contact manifold (ContactProjectionEstimator) -> gait logic -> RobonionNMPC -> servo targets
of the 9 model joints; the other actuated joints hold their defaults. Node 0 of the contact plan is the measured
mode, later nodes follow the gait plan. The gait logic is a state machine (State), each rule justified by an Isaac
failure in docs/stage2.md; within STEP:

- a foot scheduled to swing that tips onto an edge while the other sole is flat is peeling off: it is planned
  in the air and the NMPC keeps running;
- a swing foot that touches down early ends single support there; one still in the air at the planned end
  extends it tick by tick (up to MAX_STRETCH) while its reference keeps descending.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from .ekf import LegKinematicsEkf
from .estimator import AhrsGyro, ContactModeMonitor, ContactProjectionEstimator
from .gait import GaitParams, GaitPlan
from .model import EDGE_MODES, MODEL_JOINTS, RigidContactModel, contact_config
from .nmpc import U_RATE_MAX, RobonionNMPC

CROUCH = 0.2  # rad on front thigh and ankle pitch (negative, the LEG_CROUCH convention of robots/robonion_params.py)
CROUCH_START, CROUCH_TIME = 0.5, 0.8  # s
CALIBRATION_START, GAIT_START = 1.5, 2.0  # s; AHRS bias averaged in between, both soles flat
# Edge detection by predicted tilt (ContactModeMonitor lead): 15 ms ahead, as early as the stage-1 rate trigger
# (docs/stage1.md), which flags soles levelling out after a weight shift and touchdown rocking while stepping.
MONITOR_LEAD = 0.015  # s
# The predicted tilt must flag the same edge on 2 consecutive ticks: a one-tick touchdown impact (12-14 deg/s at zero
# tilt) otherwise held the targets 0.4-0.8 s over 20 steps at 3 cm.
MONITOR_TICKS = 2
COM_RAMP = 1.0  # s, DCM target from the CoM at the start to the middle of the soles
MAX_STRETCH = 0.15  # s a single-support phase may run beyond its planned end waiting for touchdown
# A sole resting on an edge: its tilt angle stays within STILL_SPAN for STILL_TIME; the targets then move to the
# standing posture at RESET_RATE [rad/s]. Resuming the NMPC with the tilted sole planned flat instead rolled it
# again within ~50 ms.
STILL_SPAN, STILL_TIME, RESET_RATE = math.radians(0.3), 0.2, 0.5
# Holding with both feet down keeps RECOVER_TIME of double support ahead, from PAUSE_AFTER into the hold (a foot
# about to lift often tips onto an edge for a few ticks, pausing then keeps it waiting there) up to MAX_PAUSE.
RECOVER_TIME, PAUSE_AFTER, MAX_PAUSE = 0.6, 0.1, 3.0
# After a hold the NMPC stands until the DCM is within CALM_DCM of the middle of the soles and the CoM speed
# below CALM_SPEED for CALM_TIME.
CALM_DCM, CALM_SPEED, CALM_TIME = 0.01, 0.03, 0.2


class State(Enum):
    """
    STEP     NMPC along the gait plan.
    HOLD     a sole on an edge, both feet down: targets held, plan paused.
    RESET    as HOLD, the sole resting still on its edge: targets move to the standing posture.
    CATCH    the stance sole on an edge, the other foot in the air: stance leg held, swing leg back to its
             targets at lift-off, step ended (holding everything, or landing the swing foot with the NMPC, fell
             more often in Isaac).
    RECOVER  after HOLD / RESET / CATCH with both feet down: NMPC standing, plan paused, until calm.
    """

    STEP = "step"
    HOLD = "hold"
    RESET = "reset"
    CATCH = "catch"
    RECOVER = "recover"


HELD = (State.HOLD, State.RESET, State.CATCH)


def nominal_pose(act_names: list[str], q_default_act: np.ndarray) -> np.ndarray:
    """Crouched pose of the actuated joints, rounded to 6 decimals [rad]: a float32 default pose (-0.20000000298
    instead of -0.2) then builds the same model and solver as scripts/build_controller.py."""
    q = np.asarray(q_default_act, dtype=float).copy()
    for i, n in enumerate(act_names):
        if n.endswith(("front_thigh_pitch_joint", "ankle_pitch_joint")):
            q[i] -= CROUCH
    return np.round(q, 6)


@dataclass
class ControllerStatus:
    modes: list[str]  # contact mode per foot (left, right)
    q: np.ndarray | None  # estimated model state; None before the gait starts
    qd: np.ndarray | None
    plan_time: float | None  # time in the gait plan (retiming and pauses move the plan, not this clock)
    holding: bool  # servo targets held or moved without the NMPC (edge rules)
    solve_time: float | None  # NMPC solver time of this tick [s]
    qp_status: int | None
    state: State


class RobonionController:
    def __init__(
        self,
        act_names: list[str],
        q_default_act: np.ndarray,
        dt: float,
        params: GaitParams | None = None,
        build: bool = False,
    ):
        """build: generate and compile the C functions and the NMPC solver (scripts/build_controller.py); otherwise
        they are loaded and a missing build is an error."""
        self.act_names = list(act_names)
        self.q_default = np.asarray(q_default_act, dtype=float).copy()
        self.q_crouch = nominal_pose(self.act_names, self.q_default)
        self.dt = dt
        self.params = params or GaitParams()
        self.ind = [self.act_names.index(n) for n in MODEL_JOINTS]

        self.model = RigidContactModel(self.act_names, self.q_crouch, compiled=True, build=build)
        self.ahrs = AhrsGyro(self.act_names, compiled=True, build=build)
        self.monitor = ContactModeMonitor(self.model, lead=MONITOR_LEAD, rate_ticks=MONITOR_TICKS)
        self.estimator = ContactProjectionEstimator(self.model, compiled=True, build=build)
        self.nmpc = RobonionNMPC(self.model, build=build)
        self.ekf = LegKinematicsEkf(self.act_names)

        self.reset()

    def reset(self):
        """Call on every env reset."""
        self.t = 0.0
        self.q_des = self.q_default.copy()
        self.u = self.q_crouch[self.ind].copy()

        self.monitor.reset()
        self.estimator.reset()
        self.ekf.reset()
        self._ekf_frame = None  # (rotation, offset) from the EKF world to the model world
        self._ekf_base = None

        self.plan: GaitPlan | None = None
        self._calib: list[np.ndarray] = []
        self._bias = self._cref = None
        self._cfg = contact_config("flat", "flat")

        self._ctrl_modes = ["flat", "flat"]
        self._lift_xy = self._u_lift = None
        self._airborne: set[int] = set()
        self.state = State.STEP
        self._hold_since = None
        self._calm = 0.0
        self._tilt_hist = []
        self._prepared = None
        self.status = ControllerStatus(["flat", "flat"], None, None, None, False, None, None, self.state)

    def step(
        self,
        q_act: np.ndarray,
        qd_act: np.ndarray,
        imu_quat_xyzw: np.ndarray,
        gyro: np.ndarray,
        accel: np.ndarray,
    ) -> np.ndarray:
        """One control tick. gyro and accel (with gravity, as the accelerometer reads it) in the IMU frame. Returns
        joint position targets for the actuated joints."""
        roll, pitch, roll_d, pitch_d = self.ahrs(q_act, qd_act, imu_quat_xyzw, gyro)
        joints, joints_d = q_act[self.ind], qd_act[self.ind]
        self.t += self.dt
        ekf_modes = list(self.status.modes) if self.t >= GAIT_START else ["flat", "flat"]
        self._ekf_base = self.ekf.update(q_act, qd_act, accel, gyro, imu_quat_xyzw, ekf_modes, self.dt)

        if self.t < GAIT_START:
            s = min(max((self.t - CROUCH_START) / CROUCH_TIME, 0.0), 1.0)
            self.q_des = self.q_default + 0.5 * (1 - math.cos(math.pi * s)) * (self.q_crouch - self.q_default)
            if self.t >= CALIBRATION_START:
                self._calib.append(np.r_[roll, pitch, joints])
            return self.q_des

        if self._bias is None:
            self._start()

        r, p = roll - self._bias[0], pitch - self._bias[1]
        q_meas = np.r_[0.0, 0.0, 0.0, r, p, joints]
        meas = (joints, r, p, joints_d, roll_d, pitch_d)

        modes, changed = self.monitor.update(q_meas, np.r_[0.0, 0.0, 0.0, roll_d, pitch_d, joints_d])
        if any(changed):
            self._cfg = contact_config(*modes)
            self._cref = self.model.update_refs(self._cref, self.estimator.q, self._cfg[1], changed)
        q, qd = self._estimate(meas)

        wrench = np.array(self.model.f_wrench(q, qd, self.u, self._cfg[0], self._cref, self._cfg[1], np.zeros(3)))
        modes, changed = self.monitor.release(wrench[2], q_meas, self._cref, self._cfg[1])
        if any(changed):
            self._cfg = contact_config(*modes)
            q, qd = self._estimate(meas)

        tp = self.t - GAIT_START
        solve_time = qp_status = None
        u = self._gait(tp, q, qd, list(modes))
        if isinstance(u, tuple):
            u, qp_status, solve_time = u

        self.u = u
        self.q_des[self.ind] = self.u
        self.status = ControllerStatus(list(modes), q, qd, tp, self.state in HELD, solve_time, qp_status, self.state)
        return self.q_des

    def _estimate(self, meas):
        # contact points of the feet in contact where the EKF has them (they may slip), so the joints and tilt
        # projected onto the contact manifold agree with the EKF base
        Rz, offset = self._ekf_frame
        for i in (0, 1):
            if self.ekf.modes[i] != "air":
                self._cref[5 * i : 5 * i + 3] = Rz @ self.ekf.p[i] + offset
        q, qd = self.estimator.update(*meas, self._cfg[0], self._cref, self._cfg[1])
        pos, vel, _ = self._ekf_base
        q, qd = q.copy(), qd.copy()
        q[:3], qd[:3] = Rz @ pos + offset, Rz @ vel
        return q, qd

    def _align_ekf(self, q_stand: np.ndarray):
        """EKF world -> model world: the model has no yaw, and its origin is set by the standing pose."""
        pos, _, R_base = self._ekf_base
        yaw = math.atan2(R_base[1, 0], R_base[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        self._ekf_frame = (Rz, q_stand[:3] - Rz @ pos)

    def _start(self):
        """End of calibration: AHRS bias, contact references, gait plan and NMPC initial guess."""
        mean = np.mean(self._calib, axis=0)
        q_stand, self._cref = self.model.initial_state(mean[2:])
        self._bias = mean[:2] - q_stand[3:5]
        self.monitor.calibrate(np.r_[0.0, 0.0, 0.0, mean[:2] - self._bias, mean[2:]])
        self._align_ekf(q_stand)

        self._home = self._cref.copy()
        feet_xy = np.array([self._cref[0:2], self._cref[5:7]])
        self._mid = feet_xy.mean(0)
        self._lift_xy = feet_xy.copy()
        self._u_lift = [None, None]

        com = np.array(self.model.f_com(q_stand)).ravel()
        self._com0 = com[:2]
        self.plan = GaitPlan(self.params, feet_xy, com[2])
        self.nmpc.reset(np.r_[q_stand, np.zeros(self.model.nq), self.u])

    # gait logic -------------------------------------------------------------------------------------------------

    def _gait(self, t, q, qd, modes):
        """Servo targets (or (targets, QP status, solve time) when the NMPC ran) for plan time t."""
        u = self.u
        for i in (0, 1):
            if self._ctrl_modes[i] != "air" and modes[i] == "air":
                self._lift_xy[i] = np.array(self.nmpc.f_sole(q))[:2, i]
                self._u_lift[i] = u.copy()
        self._ctrl_modes = list(modes)

        self._retime(t, modes)
        planned = self.plan.modes(t)
        now = tuple(
            "air" if m in EDGE_MODES and planned[i] == "air" and modes[1 - i] == "flat" else m
            for i, m in enumerate(modes)
        )
        self._transition(t, q, qd, now)

        if self.state == State.RESET:
            return u + np.clip(self.nmpc.u_nom - u, -RESET_RATE * self.dt, RESET_RATE * self.dt)
        if self.state == State.HOLD:
            return u
        if self.state == State.CATCH:
            return self._catch(t, now)
        return self._run_nmpc(t, q, qd, now)

    def _transition(self, t, q, qd, now):
        """Next state from the contact modes of this tick (see State)."""
        edge, air = any(m in EDGE_MODES for m in now), "air" in now
        if edge and not air:
            self._tilt_hist.append((t, np.array(self.model.f_tilt(q))))
            while self._tilt_hist[0][0] < t - STILL_TIME:
                self._tilt_hist.pop(0)
        else:
            self._tilt_hist = []

        if edge:
            if air:
                self.state = State.CATCH
            elif self._resting(t):
                self.state = State.RESET
            else:
                self.state = State.HOLD
            self._hold(t)
            return

        self._hold_since = None
        if self.state in HELD:
            self.nmpc.reset(np.r_[q, qd, self.u])
            self._prepared = None
            self.state = State.STEP if air else State.RECOVER
            self._calm = 0.0

        if self.state == State.RECOVER:
            com, comd = np.array(self.model.f_com(q)).ravel(), np.array(self.model.f_comd(q, qd)).ravel()
            dcm = com[:2] + comd[:2] / self.plan.omega
            calm = np.linalg.norm(dcm - self._mid) < CALM_DCM and np.linalg.norm(comd[:2]) < CALM_SPEED
            self._calm = self._calm + self.dt if calm else 0.0
            if air or self._calm >= CALM_TIME:
                self.state, self._prepared = State.STEP, None
            else:
                self._pause(t, force=True)

    def _resting(self, t):
        """A sole on an edge whose tilt stayed within STILL_SPAN for STILL_TIME."""
        span = np.ptp(np.array([h for _, h in self._tilt_hist]), axis=0)
        return t - self._tilt_hist[0][0] >= STILL_TIME - 1.5 * self.dt and span.max() < STILL_SPAN

    def _catch(self, t, now):
        """Stance-leg targets held, the swing leg back to its targets at lift-off at the servo rate limit, and the
        single-support phase ended."""
        u, plan = self.u, self.plan
        i = now.index("air")
        if self._u_lift[i] is None:
            return u

        ph_i = plan.index(t)
        if plan.phases[ph_i].swing == i and t < plan.phases[ph_i].t1:
            plan.stretch(ph_i, t - plan.phases[ph_i].t1)

        leg = slice(4 * i, 4 * i + 4)
        u = u.copy()
        u[leg] += np.clip(self._u_lift[i][leg] - u[leg], -U_RATE_MAX * self.dt, U_RATE_MAX * self.dt)
        return u

    def _run_nmpc(self, t, q, qd, now):
        u = self.u
        if self._prepared == now:
            u, status, solve_time = self.nmpc.feedback(q, qd, u, self.dt)
        else:
            u, status, solve_time = self.nmpc.solve(q, qd, u, self._nodes(t, now), self.dt)
        # linearise for the next tick now that this tick's targets are known
        self.nmpc.prepare(q, qd, self._nodes(t + self.dt, now))
        self._prepared = now
        return u, status, solve_time

    def _hold(self, t):
        if self._hold_since is None:
            self._hold_since = t
        self._pause(t)

    def _pause(self, t, force=False):
        """Double support until t + RECOVER_TIME; later phases move back. A single-support phase whose foot has
        not lifted yet is postponed by lengthening the double support before it."""
        if not force and not PAUSE_AFTER <= t - self._hold_since <= MAX_PAUSE:
            return

        i = self.plan.index(t)
        if self.plan.phases[i].swing is not None:
            if i in self._airborne or i == 0:
                return
            i -= 1

        d = t + RECOVER_TIME - self.plan.phases[i].t1
        if d > 0:
            self.plan.stretch(i, d)
            self._prepared = None

    def _retime(self, t, modes):
        i = self.plan.index(t)
        ph = self.plan.phases[i]
        if ph.swing is None:
            return

        if modes[ph.swing] == "air":
            self._airborne.add(i)
            if ph.t1 <= t + self.dt and ph.t1 - ph.t0 < ph.t_nom + MAX_STRETCH:
                self.plan.stretch(i, t + 1.5 * self.dt - ph.t1)
                self._prepared = None
        elif i in self._airborne and t < ph.t1:
            self.plan.stretch(i, t - ph.t1)
            self._prepared = None

    def _nodes(self, t, now):
        """NMPC plan: per node the modes, contact references, DCM / CoM velocity and swing-sole targets."""
        nodes = []
        for tk in t + self.nmpc.t_nodes:
            r = self.plan.reference(tk)
            cref = np.array(self._cref, dtype=float)
            for i in (0, 1):
                if now[i] == "air":
                    cref[5 * i : 5 * i + 5] = self._home[5 * i : 5 * i + 5]

            foot, z, _ = self.plan.swing(tk)
            sole = np.zeros((3, 2))
            sole[:2] = self._lift_xy.T
            if foot is not None:
                sole[2, foot] = z

            if self.state == State.RECOVER:
                modes, dcm, comd = ("flat", "flat"), self._mid.copy(), np.zeros(2)
            else:
                s = min(tk / COM_RAMP, 1.0)
                modes, dcm, comd = self.plan.modes(tk), r["dcm"] + (1 - s) * (self._com0 - self._mid), r["com_vel"]
            nodes.append({"modes": modes, "cref": cref, "dcm": dcm, "comd": comd, "sole": sole})

        nodes[0]["modes"] = now
        return nodes
