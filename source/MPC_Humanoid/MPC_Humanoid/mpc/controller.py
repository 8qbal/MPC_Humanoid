# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Robonion controller

Timeline: standing at the default pose, crouch ramp (straight legs are singular and too slow for the swing),
AHRS bias calibration with both soles flat, then the gait plan (gait.py) runs through RobonionNMPC.

Every tick: AHRS/gyro -> contact modes (ContactModeMonitor) -> base position, velocity and yaw (LegKinematicsEkf)
-> joints and tilt on the contact manifold (ContactProjectionEstimator) -> gait logic -> servo targets of the model
joints (legs, torso, hip yaw, arms); the head holds its default. The model's yaw 0 is the heading at the end of the
calibration. The NMPC runs every NMPC_EVERY ticks; on the ticks between, the targets continue at the rate of its
first interval and the tick before the next NMPC run does the RTI preparation, so preparation and feedback never
share a tick. When the contact plan of node 0 changes (or the plan is retimed, or the NMPC restarts after a hold),
that tick only prepares with the new plan and the feedback follows on the next one. Node 0 of the contact plan is the
measured mode, later nodes follow the gait plan. The gait logic is a state machine (State), each rule justified by an
Isaac failure in docs/stage2.md; within STEP:

- a foot scheduled to swing that tips onto an edge while the other sole is flat is peeling off (unless held): it is
  planned in the air and the NMPC keeps running;
- a swing foot that touches down early ends single support there; one still in the air at the planned end
  extends it tick by tick (up to MAX_STRETCH) while its reference keeps descending.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from dataclasses import dataclass
from enum import Enum

import numpy as np
import pinocchio as pin

from . import alip
from .ekf import LegKinematicsEkf, roll_pitch
from .estimator import AhrsGyro, ContactModeMonitor, ContactProjectionEstimator
from .gait import GaitParams, GaitPlan, swing_xy
from .model import EDGE_MODES, GRAVITY, RigidContactModel
from .nmpc import RobonionNMPC

CROUCH = 0.2  # rad on front thigh and ankle pitch (negative, the LEG_CROUCH convention of robots/robonion_params.py)
CROUCH_START, CROUCH_TIME = 0.5, 0.8  # s
CALIBRATION_START, GAIT_START = 1.5, 2.0  # s; AHRS bias averaged in between, both soles flat
# Edge detection by predicted tilt (ContactModeMonitor lead): 15 ms ahead, as early as the stage-1 rate trigger
# (docs/stage1.md), which flags soles levelling out after a weight shift and touchdown rocking while stepping.
MONITOR_LEAD = 0.015  # s
# The predicted tilt must flag the same edge on 3 consecutive ticks: with 1, a one-tick touchdown impact (12-14 deg/s
# at zero tilt) held the targets 0.4-0.8 s over 20 steps at 3 cm; with 2 and the 0.038 armature, the 0.1-0.15 deg
# sole wobble after touchdown held them for seconds (2026-10-15).
MONITOR_TICKS = 3
# A foot released by force while it still stood (lift-off) and taken back the next tick predicted -15 N on the other
# foot with the targets of the solve before, which released that one (fell walking at 0.1 m/s, 2026-10-17)
MONITOR_SETTLE = 3
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
# NMPC at 100 Hz under the 200 Hz estimation and gait logic, so the edge and touchdown tuning stays at 5 ms
# (docs/stage3.md, decision of 2026-10-07)
NMPC_EVERY = 2
# Walking: the ALIP foothold of a step is set once, at swing progress ALIP_AT. The offset of du is learned at
# ALIP_RATE per step, ALIP_GAIN and ALIP_MAX [m] apply to what is left. At 0.1 m/s in Isaac du at progress 0.5 stayed
# within +-5 mm of its offset from step to step: lateral +-31-37 mm per swing foot; sagittal ALIP_V_GAIN x the plan's
# CoM velocity, 1.5-1.6 s at 0.05 and 0.1 m/s through the speed ramps (a constant offset lagged over the ramp, and a
# 33 mm longer step put the stance sole on its toe). Learned only while the plan moves faster than ALIP_V_MIN [m/s].
CATCH_UNTIL = 0.85  # swing progress; later the swing foot comes down vertically (gait.XY_DONE)
RESTART_DS = 0.6  # s, first double support after a hold while walking
RESTART_STAGGER = 0.01  # m between the soles along x from which the leading foot steps back first
ALIP_AT, ALIP_RATE, ALIP_GAIN, ALIP_MAX, ALIP_V_MIN = 0.5, 0.7, 0.5, 0.05, 0.01
# The first two steps start from standing (lateral du -21 mm against -33 mm later): not learned
ALIP_LEARN_FROM = 2
# du offsets per swing progress, for a du away from ALIP_AT: the sagittal ratio fell from 1.2 s at 0.5 to 0.5 s at 0.7
# and the lateral offset from +-30 to +-25 mm; step-to-step spread <= 5 mm (steady) and 13 mm (ramps) from 0.4 to 0.9,
# 30-50 mm before 0.3 (0.05 and 0.1 m/s, docs/stage3.md). A du further than ALIP_OUTLIER [m] from its offset (a push)
# is not learned.
ALIP_BINS, ALIP_OUTLIER = (0.4, 0.5, 0.6, 0.7, 0.8, 0.9), 0.03


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class State(Enum):
    """
    STEP     NMPC along the gait plan.
    HOLD     a sole on an edge, both feet down: targets held, plan paused.
    RESET    as HOLD, the sole resting still on its edge: targets move to the standing posture.
    CATCH    the stance sole on an edge, the other foot in the air: stance leg and arms held, swing leg (with its
             hip yaw) back to its targets at lift-off, step ended (holding everything, or landing the swing foot
             with the NMPC, fell more often in Isaac).
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
        arms: bool = True,
        hip_yaw: bool = True,
    ):
        """build: generate and compile the C functions and the NMPC solver (scripts/build_controller.py); otherwise
        they are loaded and a missing build is an error. arms, hip_yaw: False holds those targets at the default
        pose (same build)."""
        self.act_names = list(act_names)
        self.q_default = np.asarray(q_default_act, dtype=float).copy()
        self.q_crouch = nominal_pose(self.act_names, self.q_default)
        self.dt = dt
        self.params = params or GaitParams()

        self.model = RigidContactModel(self.act_names, self.q_crouch, compiled=True, build=build)
        self.ind = [self.act_names.index(n) for n in self.model.joints]
        self.ahrs = AhrsGyro(self.act_names, compiled=True, build=build)
        self.monitor = ContactModeMonitor(
            self.model, lead=MONITOR_LEAD, rate_ticks=MONITOR_TICKS, settle_ticks=MONITOR_SETTLE
        )
        self.estimator = ContactProjectionEstimator(self.model, compiled=True, build=build)
        self.nmpc = RobonionNMPC(self.model, build=build)
        self.nmpc.lock(arms=not arms, hip_yaw=not hip_yaw)
        self.ekf = LegKinematicsEkf(self.act_names, compiled=True, build=build)

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
        self._yaw0 = 0.0  # EKF yaw of the model world
        self._ekf_base = None

        self.plan: GaitPlan | None = None
        self._calib: list[np.ndarray] = []
        self._bias = self._cref = None
        self._cfg = self.model.contact_config("flat", "flat")

        self._ctrl_modes = ["flat", "flat"]
        self._lift_xy = self._u_lift = None
        self._lift_step = [None, None]
        self._soles = None
        self._alip_ref = None  # (step, plan before its ALIP foothold)
        # learned du per progress bin: lateral per swing foot, sagittal per plan velocity
        self._alip_table = {s: [None, None, None] for s in ALIP_BINS}
        self._alip_offset = self._alip_table[ALIP_AT]
        self._alip_learned = (None, set())  # (step, bins learned in it)
        self._alip_held = False  # held or recovering since the last touchdown
        self._airborne: set[int] = set()
        self.state = State.STEP
        self._hold_since = None
        self._calm = 0.0
        self._tilt_hist = []
        self._prepared = None
        self._since_nmpc = 0  # ticks since the NMPC last ran
        self._deferred = False  # the last NMPC tick only prepared after a contact change
        self._v = np.zeros(len(self.ind))  # target rate of the first NMPC interval
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
        roll, pitch, roll_d, pitch_d, yaw_d = self.ahrs(q_act, qd_act, imu_quat_xyzw, gyro)
        joints, joints_d = q_act[self.ind], qd_act[self.ind]
        self.t += self.dt
        # the monitor needs the AHRS bias, so the EKF sees flat/flat up to and including the tick that sets it
        starting = self._bias is None
        if starting:
            self._ekf_base = self.ekf.update(q_act, qd_act, accel, gyro, imu_quat_xyzw, ["flat", "flat"], self.dt)

        if self.t < GAIT_START:
            s = min(max((self.t - CROUCH_START) / CROUCH_TIME, 0.0), 1.0)
            self.q_des = self.q_default + 0.5 * (1 - math.cos(math.pi * s)) * (self.q_crouch - self.q_default)
            if self.t >= CALIBRATION_START:
                self._calib.append(np.r_[roll, pitch, joints])
            return self.q_des

        if starting:
            self._start(q_act, qd_act, imu_quat_xyzw, roll, pitch)

        r, p = roll - self._bias[0], pitch - self._bias[1]
        # sole tilt and heights do not depend on yaw, the foot drift test in release() does: yaw is set below
        q_meas = np.r_[0.0, 0.0, 0.0, r, p, 0.0, joints]

        modes, changed = self.monitor.update(q_meas, np.r_[0.0, 0.0, 0.0, roll_d, pitch_d, 0.0, joints_d])
        if not starting:
            self._ekf_base = self.ekf.update(
                q_act, qd_act, accel, gyro, imu_quat_xyzw, modes, self.dt, np.array([r, p])
            )
        # the monitor keeps the AHRS tilt its thresholds were tuned on; the projection starts from the EKF tilt
        R_base = self._ekf_base[2]
        yaw = q_meas[5] = _wrap(math.atan2(R_base[1, 0], R_base[0, 0]) - self._yaw0)
        meas = (joints, *roll_pitch(R_base), joints_d, roll_d, pitch_d, yaw, yaw_d)
        if any(changed):
            self._cfg = self.model.contact_config(*modes)
            # before the first estimate the references from _start stand
            if self.estimator.q is not None:
                self._cref = self.model.update_refs(self._cref, self.estimator.q, self._cfg[1], changed)
        q, qd = self._estimate(meas)

        wrench = np.array(self.model.f_wrench(q, qd, self.u, self._cfg[0], self._cref, self._cfg[1], np.zeros(3)))
        modes, changed = self.monitor.release(wrench[2], q_meas, self._cref, self._cfg[1])
        if any(changed):
            self._cfg = self.model.contact_config(*modes)
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
        # contact points of the feet in contact where the EKF has them and sole yaw as the EKF base yaw and the leg
        # kinematics give it (they may slip), so the joints and tilt projected onto the contact manifold agree with
        # the EKF base
        Rz, offset = self._ekf_frame
        flags, pc = self._cfg
        joints, roll, pitch, _, _, _, yaw, _ = meas
        crow = np.array(self.model.f_crow(np.r_[0.0, 0.0, 0.0, roll, pitch, yaw, joints], pc)).ravel()
        nr = self.model.nr
        for i in (0, 1):
            if self.ekf.modes[i] != "air":
                self._cref[nr * i : nr * i + 3] = Rz @ self.ekf.p[i] + offset
                self._cref[nr * i + 5] = crow[nr * i + 5]
        q, qd = self.estimator.update(*meas[:6], flags, self._cref, pc, *meas[6:])
        pos, vel, _ = self._ekf_base
        q, qd = q.copy(), qd.copy()
        q[:3], qd[:3] = Rz @ pos + offset, Rz @ vel
        return q, qd

    def _align_ekf(self, q_stand: np.ndarray):
        """EKF world -> model world: the model's yaw 0 is the current heading, its origin is set by the standing
        pose."""
        pos, _, R_base = self._ekf_base
        yaw = self._yaw0 = math.atan2(R_base[1, 0], R_base[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        self._ekf_frame = (Rz, q_stand[:3] - Rz @ pos)

    def _start(self, q_act, qd_act, imu_quat_xyzw, roll, pitch):
        """End of calibration: AHRS bias, EKF restart with the bias-corrected AHRS, contact references, gait plan
        and NMPC initial guess."""
        mean = np.mean(self._calib, axis=0)
        q_stand, self._cref = self.model.initial_state(mean[2:])
        self._bias = mean[:2] - q_stand[3:5]
        self.monitor.calibrate(np.r_[0.0, 0.0, 0.0, mean[:2] - self._bias, 0.0, mean[2:]])

        x, y, z, w = np.asarray(imu_quat_xyzw, dtype=float) / np.linalg.norm(imu_quat_xyzw)
        R_ib = self.ekf.R.T @ self._ekf_base[2]
        R_base = pin.Quaternion(w, x, y, z).matrix() @ R_ib
        yaw = math.atan2(R_base[1, 0], R_base[0, 0])
        R_base = pin.rpy.rpyToMatrix(roll - self._bias[0], pitch - self._bias[1], yaw)
        self._ekf_base = self.ekf.reinit(q_act, qd_act, R_base @ R_ib.T)
        self._align_ekf(q_stand)

        self._home = self._cref.copy()
        nr = self.model.nr
        feet_xy = np.array([self._cref[0:2], self._cref[nr : nr + 2]])
        self._mid = feet_xy.mean(0)
        self._lift_xy = feet_xy.copy()
        self._u_lift = [None, None]
        self._walking = self.params.vx != 0.0

        com = np.array(self.model.f_com(q_stand)).ravel()
        self._com0 = com[:2]
        self.plan = GaitPlan(self.params, feet_xy, com[2])
        self.nmpc.reset(np.r_[q_stand, np.zeros(self.model.nq), self.u[: self.nmpc.ns]])

    # gait logic -------------------------------------------------------------------------------------------------

    def _gait(self, t, q, qd, modes):
        """Servo targets (or (targets, QP status, solve time) when the NMPC ran) for plan time t."""
        u = self.u
        self._soles = np.array(self.nmpc.f_sole(q))
        j = self.plan.index(t)
        ph, nxt = self.plan.phases[j], self.plan.phases[min(j + 1, len(self.plan.phases) - 1)]
        for i in (0, 1):
            if self._ctrl_modes[i] != "air" and modes[i] == "air":
                self._lift_xy[i] = self._soles[:2, i]
                self._u_lift[i] = u.copy()
                # a foot unloaded at the end of the double support lifts off before its swing phase
                self._lift_step[i] = ph.step if ph.swing == i else nxt.step if nxt.swing == i else None
            elif self._walking and self._ctrl_modes[i] == "air" and modes[i] != "air":
                k = self._lift_step[i]
                # lifted and set down again within a (paused) double support: not the landing of step k, which
                # moved every later foothold 0.1 m back (0.1 m/s, 2026-10-17)
                if k is not None and t >= next(p.t0 for p in self.plan.phases if p.step == k):
                    self.plan.replan(k, self._soles[:2, i])
                self._lift_step[i] = None
                self._alip_held = False
        self._ctrl_modes = list(modes)
        if self._walking and self.state == State.STEP:
            self._alip(t, q, qd)
            self._learn_offsets(t, q, qd)

        self._retime(t, modes)
        planned = self.plan.modes(t)
        # not while held: a hold from touchdown reaches the next single support before PAUSE_AFTER, and peel-off
        # then lifted the loaded edge foot (+y 1.0 N·s fell, docs/stage3.md)
        peel = self.state not in HELD
        now = tuple(
            "air" if peel and m in EDGE_MODES and planned[i] == "air" and modes[1 - i] == "flat" else m
            for i, m in enumerate(modes)
        )
        self._transition(t, q, qd, now)
        self._alip_held |= self.state != State.STEP

        if self.state == State.RESET:
            return u + np.clip(self.nmpc.u_nom - u, -RESET_RATE * self.dt, RESET_RATE * self.dt)
        if self.state == State.HOLD:
            return u
        if self.state == State.CATCH:
            return self._catch(t, now)
        return self._run_nmpc(t, q, qd, now)

    def _restart(self, t):
        """Walking, recovering from a hold: a new plan from the soles where they stand, standing first (paused while
        recovering), then the steps left with the speed ramp from zero. The paused plan expected the CoM to move
        towards the next stance foot; resumed from rest, the stance sole rolled inward, caught and held again on
        every step (0.1 m/s, 2026-10-17)."""
        plan = self.plan
        k = next(
            (
                ph.step
                for i, ph in enumerate(plan.phases)
                if ph.step is not None and ph.t1 > t and i not in self._airborne
            ),
            None,
        )
        feet = self._soles[:2].T.copy()
        self._mid = feet.mean(0)
        if k is None:
            return
        # from rest the NMPC did not shift the weight onto a sole ahead of the other within the double support (the
        # CoM went back, the sole rolled inward and held again, 0.1 m/s): the leading foot steps back level first
        lead = int(np.argmax(feet[:, 0]))
        first = lead if abs(feet[0, 0] - feet[1, 0]) > RESTART_STAGGER else plan.swings[k]
        p = dataclasses.replace(self.params, n_steps=self.params.n_steps - k, first_swing=first, t_stand=RECOVER_TIME)
        self.plan = GaitPlan(p, feet, GRAVITY / plan.omega**2, spacing=plan.spacing)
        self.plan.stretch(len(self.plan.phases) - 1, self.params.t_stand - RECOVER_TIME)
        self.plan.stretch(1, RESTART_DS - self.params.t_ds)
        self.plan.shift(t)
        self._airborne = set()
        self._lift_step = [None, None]
        self._lift_xy = feet.copy()
        self._alip_ref = None

    def _transition(self, t, q, qd, now):
        """Next state from the contact modes of this tick (see State)."""
        edge, air = any(m in EDGE_MODES for m in now), "air" in now
        if edge and not air:
            self._tilt_hist.append((t, np.array(self.model.f_tilt(q))))
            while self._tilt_hist[0][0] < t - STILL_TIME:
                self._tilt_hist.pop(0)
        else:
            self._tilt_hist = []

        ph = self.plan.phase(t)
        # walking: the stance sole read on its toe at the touchdown tick, and the catch pulled the swing foot back
        # to its lift-off point 0.1 m behind (2026-10-17)
        landing = self._walking and ph.swing is not None and (t - ph.t0) / ph.t_nom >= CATCH_UNTIL
        if edge and not (air and landing):
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
            self.nmpc.reset(np.r_[q, qd, self.u[: self.nmpc.ns]])
            self._v = np.zeros_like(self._v)
            self._prepared = None
            self.state = State.STEP if air else State.RECOVER
            self._calm = 0.0
            if self._walking and not air:
                self._restart(t)

        if self.state == State.RECOVER:
            com, comd = np.array(self.model.f_com(q)).ravel(), np.array(self.model.f_comd(q, qd)).ravel()
            dcm = com[:2] + comd[:2] / self.plan.omega
            calm = np.linalg.norm(dcm - self._mid) < CALM_DCM and np.linalg.norm(comd[:2]) < CALM_SPEED
            self._calm = self._calm + self.dt if calm else 0.0
            # walking: the foot opposite the lean read air by force within 40 ms, and the gait went on leaning on the
            # other sole, which rolled inward and held again on every step (0.1 m/s, 2026-10-17)
            if (air and not self._walking) or self._calm >= CALM_TIME:
                self.state, self._prepared = State.STEP, None
            else:
                self._pause(t, force=True)

    def _alip(self, t, q, qd):
        """Foothold of the swinging foot: nominal plus the ALIP feedback (alip.py) about the learned orbit. The last
        step too: without it the CoM ran 30% faster than the plan sideways into the stop and fell (0.1 m/s)."""
        plan = self.plan
        ph = plan.phase(t)
        k = ph.step
        if k is None or (t - ph.t0) / ph.t_nom < ALIP_AT:
            return
        if self._alip_ref is not None and self._alip_ref[0] == k:
            return
        self._alip_ref = (k, copy.deepcopy(plan))
        if self._alip_held:
            # the plan was paused: its reference CoM velocity is not the robot's, du read -0.1 m for a step that
            # then landed behind the stance foot (0.1 m/s, 2026-10-17)
            return
        du, v = self._alip_du(t, q, qd, plan, ph)
        i = ph.swing
        fb = np.zeros(2)
        if k >= ALIP_LEARN_FROM:
            off = self._alip_offset
            offset = du[1] if off[i] is None else off[i]
            fb[1] = np.clip(ALIP_GAIN * (du[1] - offset), -ALIP_MAX, ALIP_MAX)
            if off[2] is not None:
                fb[0] = np.clip(ALIP_GAIN * (du[0] - off[2] * v), -ALIP_MAX, ALIP_MAX)
            off[i] = offset + ALIP_RATE * (du[1] - offset)
            if abs(v) > ALIP_V_MIN:
                off[2] = du[0] / v if off[2] is None else off[2] + ALIP_RATE * (du[0] / v - off[2])
        self._move_foothold(t, ph, plan, fb)

    def _learn_offsets(self, t, q, qd):
        """du offsets of the bins other than ALIP_AT (learned in _alip), once per step as the swing passes them."""
        ph = self.plan.phase(t)
        k = ph.step
        if k is None or k < ALIP_LEARN_FROM or self._alip_held:
            return
        if self._alip_learned[0] != k:
            self._alip_learned = (k, set())
        prog = (t - ph.t0) / ph.t_nom
        due = [b for b in ALIP_BINS if b != ALIP_AT and b <= prog < b + 0.1 and b not in self._alip_learned[1]]
        if not due:
            return
        self._alip_learned[1].add(due[0])
        ref = self._alip_ref[1] if self._alip_ref is not None and self._alip_ref[0] == k else self.plan
        du, v = self._alip_du(t, q, qd, ref, ph)
        off, i = self._alip_table[due[0]], ph.swing
        if off[i] is None or abs(du[1] - off[i]) < ALIP_OUTLIER:
            off[i] = du[1] if off[i] is None else off[i] + ALIP_RATE * (du[1] - off[i])
        if abs(v) > ALIP_V_MIN and (off[2] is None or abs(du[0] - off[2] * v) < ALIP_OUTLIER):
            off[2] = du[0] / v if off[2] is None else off[2] + ALIP_RATE * (du[0] / v - off[2])

    def _alip_du(self, t, q, qd, plan, ph):
        """ALIP step change du about the reference of plan, and the plan's sagittal CoM velocity."""
        r = plan.reference(t)
        stance = self._soles[:2, 1 - ph.swing]
        com, comd = np.array(self.model.f_com(q)).ravel(), np.array(self.model.f_comd(q, qd)).ravel()
        h = self.model.Ag @ qd
        lm = comd[:2] + np.array([h[1], -h[0]]) * plan.omega**2 / (GRAVITY * self.model.mass)
        du = alip.step_change(
            com[:2] - stance - (r["com"] - plan.stance_xy(ph.step)),
            lm - r["com_vel"],
            ph.t1 - t + 0.5 * plan.p.t_ds,
            plan.p.t_ss + plan.p.t_ds,
            plan.omega,
        )
        return du, r["com_vel"][0]

    def _move_foothold(self, t, ph, ref, fb):
        """Foothold of the current step to the stance sole + (nominal step of plan ref) + fb, within the step limits."""
        k, i, plan = ph.step, ph.swing, self.plan
        stance = self._soles[:2, 1 - i]
        old = plan.footholds[k].copy()
        plan.replan(k, stance + alip.clamp_step(ref.footholds[k] - ref.stance_xy(k) + fb, i == 0))
        # the swing reference stays continuous: swing_xy is linear in its lift and target points
        f = swing_xy(np.zeros(2), np.ones(2), (t - ph.t0) / ph.t_nom)[0]
        if f < 1.0:
            self._lift_xy[i] -= (plan.footholds[k] - old) * f / (1.0 - f)

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

        leg = [*range(4 * i, 4 * i + 4)]
        if self.model.hip_yaw:
            leg.append(self.model.joints.index(("left_hip_yaw_joint", "right_hip_yaw_joint")[i]))
        u = u.copy()
        step = self.model.rate_limit[leg] * self.dt
        u[leg] += np.clip(self._u_lift[i][leg] - u[leg], -step, step)
        return u

    def _run_nmpc(self, t, q, qd, now):
        if self._since_nmpc < NMPC_EVERY - 1 and self._prepared is not None and self._prepared[0] == now:
            self._since_nmpc += 1
            if self._since_nmpc == NMPC_EVERY - 1:
                # linearise for the next NMPC tick now that this tick's targets are known
                self.nmpc.prepare(q, qd, self._nodes(t + self.dt, now))
                self._prepared = (now, True)
            return self.u + self._v * self.dt

        deferred, self._deferred = self._deferred, False
        if self._prepared != (now, True) and not deferred:
            # a full solve (prepare + feedback) of the whole-body NMPC does not fit one tick with the estimation:
            # prepare with the new plan now, feedback on the next tick (also the cold restart after a hold, ~8 ms)
            self.nmpc.prepare(q, qd, self._nodes(t + self.dt, now))
            self._prepared = (now, True)
            self._since_nmpc = NMPC_EVERY - 1
            self._deferred = True
            return self.u + self._v * self.dt

        # after a deferral the feedback runs even if the modes changed again (contact flicker): deferring again
        # left the targets open loop for up to 25 ticks offline, a full solve instead went over the tick
        u, self._v, status, solve_time = self.nmpc.feedback(q, qd, self.u, self.dt)
        self._since_nmpc = 0
        self._prepared = (now, False)
        if NMPC_EVERY == 1:
            self.nmpc.prepare(q, qd, self._nodes(t + self.dt, now))
            self._prepared = (now, True)
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
        nodes, nr, plan = [], self.model.nr, self.plan
        cur = plan.index(t)
        # the DCM reference stays the one of the plan at the start of the step: following the ALIP footholds, it
        # pulled the CoM after the moved foothold and the foothold moved further (fell backwards in Isaac)
        ref_plan = (
            self._alip_ref[1] if self._alip_ref is not None and self._alip_ref[0] == plan.phases[cur].step else plan
        )
        for tk in t + self.nmpc.t_nodes:
            r = ref_plan.reference(tk)
            j = plan.index(tk)
            cref = np.array(self._cref, dtype=float)
            for i in (0, 1):
                if now[i] == "air":
                    cref[nr * i : nr * (i + 1)] = self._home[nr * i : nr * (i + 1)]
                    if self._walking:
                        cref[nr * i : nr * i + 2] = self._lift_xy[i]
                if self._walking and plan.phases[j].swing != i:
                    # a foot that lands within the horizon: in contact at its foothold
                    k = next(
                        (plan.phases[p].step for p in range(j - 1, cur - 1, -1) if plan.phases[p].swing == i), None
                    )
                    pre_swing = (
                        plan.phases[cur].swing is None and plan.phases[min(cur + 1, len(plan.phases) - 1)].swing == i
                    )
                    if k is None and now[i] == "air" and not pre_swing:
                        k = self._lift_step[i]
                    if k is not None:
                        cref[nr * i : nr * i + 2] = plan.footholds[k]

            foot, z, prog = plan.swing(tk)
            sole = np.zeros((3, 2))
            sole[:2] = self._lift_xy.T
            if foot is not None:
                sole[2, foot] = z
                if self._walking:
                    lift = self._lift_xy[foot] if now[foot] == "air" else self._soles[:2, foot]
                    sole[:2, foot] = swing_xy(lift, plan.footholds[plan.phases[j].step], prog)

            if self.state == State.RECOVER:
                modes, dcm, comd = ("flat", "flat"), self._mid.copy(), np.zeros(2)
            else:
                s = min(tk / COM_RAMP, 1.0)
                modes, dcm, comd = self.plan.modes(tk), r["dcm"] + (1 - s) * (self._com0 - self._mid), r["com_vel"]
            nodes.append({"modes": modes, "cref": cref, "dcm": dcm, "comd": comd, "sole": sole})

        nodes[0]["modes"] = now
        return nodes
