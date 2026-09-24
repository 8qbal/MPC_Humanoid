# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
CoM / DCM estimator for the stabilizer.
Uses only what the real robot measures: actuated joint encoders and the AHRS
orientation. Assumes both soles are on the ground (double support).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

CONTROLLER_URDF = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../../assets/robonionv2_controller.urdf")
)

# Sole centre in *_foot_roll_link, from the foot mesh *_foot_visual.stl
SOLE_CENTRE = np.array([-0.0195, 0.0, -0.056])

# 4 bar linkage
PASSIVE_COUPLING = {  # passive joint suffix: (actuated joint suffix, gain)
    "knee_pitch_joint": ("front_thigh_pitch_joint", -1.0),
    "back_thigh_pitch_joint": ("front_thigh_pitch_joint", 1.0),
    "front_shin_pitch_joint": ("ankle_pitch_joint", -1.0),
    "back_shin_pitch_joint": ("ankle_pitch_joint", -1.0),
}

GRAVITY = 9.81


@dataclass
class EstimatorState:
    c: np.ndarray
    # CoM (x, y) in the support frame S [m].
    cdot: np.ndarray
    # CoM velocity (x, y) in S [m/s].
    xi: np.ndarray
    # DCM (x, y) in S [m].
    com_height: float
    # CoM height above the sole plane [m].
    fallen: bool
    # CoM height below min_com_height: the robot is no longer standing on both soles.


class StabilizerEstimator:
    """Estimates CoM and DCM relative to the support frame from encoders and the AHRS.

    The support frame S has its origin at the midpoint of the two sole centres, z
    opposite to gravity and x along the base heading.
    """

    def __init__(
        self,
        actuated_joint_names: list[str],
        dt: float,
        omega: float | None = None,
        vel_cutoff: float = 50.0,
        min_com_height: float = 0.25,
        urdf_path: str = CONTROLLER_URDF,
    ):
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.dt = dt
        self.omega = omega
        self.min_com_height = min_com_height
        self.alpha = dt / (dt + 1.0 / (2.0 * math.pi * vel_cutoff))

        act = [self.model.joints[self.model.getJointId(name)] for name in actuated_joint_names]
        self._act_idx_q = np.array([j.idx_q for j in act])
        self._act_idx_v = np.array([j.idx_v for j in act])
        self._passive = []  # (idx_q, idx_v, index into the actuated vector, gain)
        for side in ("left", "right"):
            for passive, (active, gain) in PASSIVE_COUPLING.items():
                j = self.model.joints[self.model.getJointId(f"{side}_{passive}")]
                self._passive.append((j.idx_q, j.idx_v, actuated_joint_names.index(f"{side}_{active}"), gain))

        self._imu = self.model.getFrameId("imu_link")
        self._feet = [self.model.getFrameId(f"{side}_foot_roll_link") for side in ("left", "right")]
        self._cdot = np.zeros(2)

    def full_configuration(self, q_act: np.ndarray, base_rotation: np.ndarray = np.eye(3)) -> np.ndarray:
        """
        Pinocchio configuration with passive joints filled in by the parallelogram coupling.
        """
        q = pin.neutral(self.model)
        q[3:7] = pin.Quaternion(base_rotation).coeffs()
        q[self._act_idx_q] = q_act
        for idx_q, _, src, gain in self._passive:
            q[idx_q] = gain * q_act[src]
        return q

    def base_rotation(self, q_act: np.ndarray, imu_quat_xyzw: np.ndarray) -> np.ndarray:
        """Gravity-aligned base rotation (roll and pitch only) from the AHRS orientation."""
        q = self.full_configuration(q_act)
        pin.framesForwardKinematics(self.model, self.data, q)
        R_base_imu = self.data.oMf[self._imu].rotation
        x, y, z, w = (float(v) for v in imu_quat_xyzw / np.linalg.norm(imu_quat_xyzw))
        R_world_base = pin.Quaternion(w, x, y, z).matrix() @ R_base_imu.T
        yaw = math.atan2(R_world_base[1, 0], R_world_base[0, 0])
        return pin.utils.rotate("z", -yaw) @ R_world_base

    def full_velocity(self, qd_act: np.ndarray) -> np.ndarray:
        """
        Pinocchio velocity with zero base twist and passive joint rates from the coupling.
        """
        v = np.zeros(self.model.nv)
        v[self._act_idx_v] = qd_act
        for _, idx_v, src, gain in self._passive:
            v[idx_v] = gain * qd_act[src]
        return v

    def update(
        self, q_act: np.ndarray, qd_act: np.ndarray, imu_quat_xyzw: np.ndarray, gyro: np.ndarray
    ) -> EstimatorState:
        q = self.full_configuration(q_act, self.base_rotation(q_act, imu_quat_xyzw))
        v = self.full_velocity(qd_act)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        com = pin.centerOfMass(self.model, self.data, q)
        J_com = pin.jacobianCenterOfMass(self.model, self.data, q)

        # The gyro sits on the upper body, behind torso_pitch: solve its reading for the pelvis rate.
        J_imu = pin.getFrameJacobian(self.model, self.data, self._imu, pin.LOCAL)[3:]
        v[3:6] = np.linalg.solve(J_imu[:, 3:6], gyro - J_imu[:, 6:] @ v[6:])

        soles, J_soles = [], []
        for f in self._feet:
            M = self.data.oMf[f]
            r = M.rotation @ SOLE_CENTRE
            J = pin.getFrameJacobian(self.model, self.data, f, pin.LOCAL_WORLD_ALIGNED)
            soles.append(M.translation + r)
            J_soles.append(J[:3] - pin.skew(r) @ J[3:])
        origin = 0.5 * (soles[0] + soles[1])

        # Base linear velocity moves CoM and soles alike, so it cancels and stays zero in v.
        c = (com - origin)[:2]
        cdot_raw = ((J_com - 0.5 * (J_soles[0] + J_soles[1])) @ v)[:2]
        self._cdot += self.alpha * (cdot_raw - self._cdot)

        # When the robot falls the feet-flat model puts the CoM near or below the soles.
        com_height = float(com[2] - origin[2])
        fallen = com_height < self.min_com_height
        omega = self.omega if self.omega is not None else math.sqrt(GRAVITY / max(com_height, self.min_com_height))
        return EstimatorState(
            c=c, cdot=self._cdot.copy(), xi=c + self._cdot / omega, com_height=com_height, fallen=fallen
        )
