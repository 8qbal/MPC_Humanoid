# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Stage-1 stabilizer: double-support standing that recovers from pushes without
stepping. Each tick runs estimator -> DCM MPC -> LIPM step -> IK -> safety and
returns joint position targets for the actuated joints (see docs/stabilizer.md).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .dcm_mpc import DcmMpc, DcmMpcSolution
from .estimator import GRAVITY, EstimatorState, StabilizerEstimator
from .ik import StabilizerIK

# Hull of both soles in S at the standing pose, from the foot mesh *_foot_visual.stl and FK.
ZMP_MIN = np.array([-0.0875, -0.0885])
ZMP_MAX = np.array([0.0875, 0.0885])


@dataclass
class StabilizerOutput:
    q_des: np.ndarray
    # Joint position targets for the actuated joints [rad].
    est: EstimatorState
    mpc: DcmMpcSolution | None
    c_ref: np.ndarray
    cdot_ref: np.ndarray
    mode: str
    # "stand" or "safe_stop".
    recoverable: bool
    # False when the measured DCM is outside the support polygon: needs a step.


class StabilizerController:
    def __init__(
        self,
        actuated_joint_names: list[str],
        q_default_act: np.ndarray,
        dt: float,
        tilt_limit_deg: float = 20.0,
        max_mpc_failures: int = 10,
        soft_limit_factor: float = 0.9,
    ):
        self.dt = dt
        self.ik = StabilizerIK(actuated_joint_names, q_default_act, dt, soft_limit_factor=soft_limit_factor)
        self.omega = math.sqrt(GRAVITY / self.ik.com_height)
        self.estimator = StabilizerEstimator(actuated_joint_names, dt, self.omega)
        self.mpc = DcmMpc(self.omega)
        self.tilt_limit = math.radians(tilt_limit_deg)
        self.max_mpc_failures = max_mpc_failures

        joints = [self.ik.model.joints[self.ik.model.getJointId(n)] for n in actuated_joint_names]
        idx_q = [j.idx_q for j in joints]
        idx_v = [j.idx_v for j in joints]
        lo, hi = self.ik.model.lowerPositionLimit[idx_q], self.ik.model.upperPositionLimit[idx_q]
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo) * soft_limit_factor
        self._q_min, self._q_max = mid - half, mid + half
        self._qd_max = self.ik.model.velocityLimit[idx_v]
        self.reset(q_default_act)

    def reset(self, q_act: np.ndarray):
        self.ik.reset(q_act)
        self.estimator.reset()
        self.c_ref = self.ik.com[:2].copy()
        self.cdot_ref = np.zeros(2)
        self.mpc.reset(self.c_ref)
        self.q_des = np.asarray(q_act, dtype=float).copy()
        self.mode = "stand"
        self._mpc_failures = 0

    def lipm_step(self, p0: np.ndarray):
        """Advance the CoM reference one tick under the ZMP p0 (exact LIPM solution)."""
        ch, sh = math.cosh(self.omega * self.dt), math.sinh(self.omega * self.dt)
        dc = self.c_ref - p0
        self.c_ref, self.cdot_ref = p0 + dc * ch + self.cdot_ref / self.omega * sh, dc * self.omega * sh + self.cdot_ref * ch

    def step(
        self, q_act: np.ndarray, qd_act: np.ndarray, imu_quat_xyzw: np.ndarray, gyro: np.ndarray
    ) -> StabilizerOutput:
        est = self.estimator.update(q_act, qd_act, imu_quat_xyzw, gyro)
        recoverable = bool(np.all(est.xi >= ZMP_MIN) and np.all(est.xi <= ZMP_MAX))
        R = self.estimator.base_rotation(q_act, imu_quat_xyzw)
        tilted = math.acos(np.clip(R[2, 2], -1.0, 1.0)) > self.tilt_limit

        sol = None
        if self.mode == "stand":
            sol = self.mpc.solve(est.xi, ZMP_MIN, ZMP_MAX)
            self._mpc_failures = 0 if sol.ok else self._mpc_failures + 1
            if tilted or est.fallen or self._mpc_failures > self.max_mpc_failures:
                self.mode = "safe_stop"
        if self.mode == "stand":
            self.lipm_step(sol.p0)
            q_des = self.ik.step(self.c_ref, self.cdot_ref)
            step_max = self._qd_max * self.dt
            q_des = np.clip(q_des, self.q_des - step_max, self.q_des + step_max)
            self.q_des = np.clip(q_des, self._q_min, self._q_max)

        return StabilizerOutput(
            q_des=self.q_des.copy(),
            est=est,
            mpc=sol,
            c_ref=self.c_ref.copy(),
            cdot_ref=self.cdot_ref.copy(),
            mode=self.mode,
            recoverable=recoverable,
        )
