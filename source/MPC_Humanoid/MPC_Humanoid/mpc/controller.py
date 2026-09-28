# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Stage-1 standing controller (design D in docs/stage1.md).

Every tick: AHRS/gyro -> contact modes (ContactModeMonitor) -> state (ContactProjectionEstimator) ->
NMPC -> servo targets of the 9 model joints; the other actuated joints hold their defaults. The NMPC
runs only while both soles are flat. When a sole rolls on an edge or a foot leaves the ground the servo
targets are held: the robot then recovers better on its own than with the NMPC acting on a model that
predicts those phases poorly.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .estimator import AhrsGyro, ContactModeMonitor, ContactProjectionEstimator
from .model import MODEL_JOINTS, RigidContactModel, contact_config
from .nmpc import N_NODES, ContactNMPC

CALIBRATION_TIME = 0.5  # s standing still at the default pose to measure the AHRS bias
RAMP_TIME = 1.0  # s to move the CoM target from the standing CoM to the sole centre


@dataclass
class ControllerStatus:
    modes: list[str]
    # contact mode per foot (left, right)
    q: np.ndarray | None
    qd: np.ndarray | None
    # estimated model state; None during calibration
    holding: bool
    # servo targets held because a sole is not flat
    solve_time: float | None
    # NMPC solver time of this tick [s]
    qp_status: int | None


class StandingController:
    def __init__(self, act_names: list[str], q_default_act: np.ndarray, dt: float, build: bool | None = None):
        self.act_names = list(act_names)
        self.q_default = np.asarray(q_default_act, dtype=float).copy()
        self.dt = dt
        self.ind = [self.act_names.index(n) for n in MODEL_JOINTS]
        self.model = RigidContactModel(self.act_names, self.q_default)
        self.ahrs = AhrsGyro(self.act_names)
        self.monitor = ContactModeMonitor(self.model)
        self.estimator = ContactProjectionEstimator(self.model)
        self.nmpc = ContactNMPC(self.model, build=build)
        self.reset()

    def reset(self):
        """Call on every env reset."""
        self.t = 0.0
        self.q_des = self.q_default.copy()
        self.u = self.q_default[self.ind].copy()
        self.monitor.reset()
        self.estimator.reset()
        self._calib: list[np.ndarray] = []
        self._bias: np.ndarray | None = None
        self._cfg = contact_config("flat", "flat")
        self._cref: np.ndarray | None = None
        self._c_start = self._c_goal = None
        self._holding = False
        self.status = ControllerStatus(["flat", "flat"], None, None, False, None, None)

    def step(self, q_act: np.ndarray, qd_act: np.ndarray, imu_quat_xyzw: np.ndarray, gyro: np.ndarray) -> np.ndarray:
        """One control tick. Returns joint position targets for the actuated joints."""
        roll, pitch, roll_d, pitch_d = self.ahrs(q_act, qd_act, imu_quat_xyzw, gyro)
        joints, joints_d = q_act[self.ind], qd_act[self.ind]
        self.t += self.dt
        if self.t <= CALIBRATION_TIME:
            self._calib.append(np.r_[roll, pitch, joints])
            return self.q_des
        if self._bias is None:
            self._start()

        r, p = roll - self._bias[0], pitch - self._bias[1]
        q_meas = np.r_[0.0, 0.0, 0.0, r, p, joints]
        modes, changed = self.monitor.update(q_meas, np.r_[0.0, 0.0, 0.0, roll_d, pitch_d, joints_d])
        if any(changed):
            self._cfg = contact_config(*modes)
            self._cref = self.model.update_refs(self._cref, self.estimator.q, self._cfg[1], changed)
        q, qd = self.estimator.update(joints, r, p, joints_d, roll_d, pitch_d, self._cfg[0], self._cref, self._cfg[1])
        wrench = np.array(self.model.f_wrench(q, qd, self.u, self._cfg[0], self._cref, self._cfg[1], np.zeros(3)))
        modes, changed = self.monitor.release(wrench[2], q_meas, self._cref, self._cfg[1])
        if any(changed):
            self._cfg = contact_config(*modes)
            q, qd = self.estimator.update(
                joints, r, p, joints_d, roll_d, pitch_d, self._cfg[0], self._cref, self._cfg[1]
            )

        solve_time = qp_status = None
        if modes == ["flat", "flat"]:
            if self._holding:
                self.nmpc.reset(np.r_[q, qd, self.u])
                self._holding = False
            s = min((self.t - CALIBRATION_TIME) / RAMP_TIME, 1.0)
            c_ref = self._c_start + s * (self._c_goal - self._c_start)
            plan = [(modes, self._cref)] * (N_NODES + 1)
            self.u, qp_status, solve_time = self.nmpc.solve(q, qd, self.u, plan, c_ref, self.dt)
            self.q_des[self.ind] = self.u
        else:
            self._holding = True
        self.status = ControllerStatus(modes, q, qd, self._holding, solve_time, qp_status)
        return self.q_des

    def _start(self):
        """End of calibration: AHRS bias (the soles are flat, so the joints fix the true pelvis tilt),
        contact references, CoM target and NMPC initial guess."""
        mean = np.mean(self._calib, axis=0)
        q_stand, self._cref = self.model.initial_state(mean[2:])
        self._bias = mean[:2] - q_stand[3:5]
        self.monitor.calibrate(np.r_[0.0, 0.0, 0.0, mean[:2] - self._bias, mean[2:]])
        self._c_start = np.array(self.model.f_com(q_stand)).ravel()[:2]
        self._c_goal = 0.5 * (self._cref[0:2] + self._cref[5:7])
        self.nmpc.reset(np.r_[q_stand, np.zeros(self.model.nq), self.u])
