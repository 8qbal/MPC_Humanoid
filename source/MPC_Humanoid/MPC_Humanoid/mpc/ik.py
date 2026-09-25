# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Velocity IK for the stabilizer. Runs on an internal copy of the controller model
integrated from its own joint targets (not from the encoders): both soles fixed,
CoM to the LIPM reference, pelvis upright, torso at its default angle. Arms and
head are held at their default angles.
"""

from __future__ import annotations

import re

import numpy as np
import pinocchio as pin
from scipy.optimize import lsq_linear

from .estimator import CONTROLLER_URDF, PASSIVE_COUPLING, SOLE_CENTRE

IK_JOINTS = (
    ".*_hip_yaw_joint",
    ".*_hip_roll_joint",
    ".*_front_thigh_pitch_joint",
    ".*_ankle_pitch_joint",
    ".*_ankle_roll_joint",
    "torso_pitch_joint",
)
# World-frame rows of a sole twist that are tracked. Sole pitch always equals pelvis
# pitch through the parallelograms, so it is left to the pelvis task.
SOLE_ROWS = [0, 1, 2, 3, 5]


class StabilizerIK:
    def __init__(
        self,
        actuated_joint_names: list[str],
        q_default_act: np.ndarray,
        dt: float,
        w_sole: float = 100.0,
        w_com: float = 10.0,
        w_height: float = 0.1,
        w_pelvis: float = 1.0,
        w_torso: float = 0.1,
        k_sole: float = 50.0,
        k_com: float = 20.0,
        k_pelvis: float = 20.0,
        k_torso: float = 10.0,
        damping: float = 1e-3,
        soft_limit_factor: float = 0.9,
        urdf_path: str = CONTROLLER_URDF,
    ):
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.dt = dt
        self.q_default_act = np.asarray(q_default_act, dtype=float)
        self.w = dict(sole=w_sole, com=w_com, height=w_height, pelvis=w_pelvis, torso=w_torso, damping=damping)
        self.k = dict(sole=k_sole, com=k_com, pelvis=k_pelvis, torso=k_torso)

        joints = [self.model.joints[self.model.getJointId(n)] for n in actuated_joint_names]
        self._act_idx_q = np.array([j.idx_q for j in joints])
        self._ik_act = [i for i, n in enumerate(actuated_joint_names) if any(re.fullmatch(p, n) for p in IK_JOINTS)]
        self._torso = self._ik_act.index(actuated_joint_names.index("torso_pitch_joint"))

        nx = 6 + len(self._ik_act)
        self._G = np.zeros((self.model.nv, nx))
        self._G[:6, :6] = np.eye(6)
        for col, a in enumerate(self._ik_act):
            self._G[joints[a].idx_v, 6 + col] = 1.0
        self._passive = []  # (idx_q, index into the actuated vector, gain)
        for side in ("left", "right"):
            for passive, (active, gain) in PASSIVE_COUPLING.items():
                j = self.model.joints[self.model.getJointId(f"{side}_{passive}")]
                a = actuated_joint_names.index(f"{side}_{active}")
                self._G[j.idx_v, 6 + self._ik_act.index(a)] = gain
                self._passive.append((j.idx_q, a, gain))

        ik_q = np.array([joints[a].idx_q for a in self._ik_act])
        ik_v = np.array([joints[a].idx_v for a in self._ik_act])
        lo, hi = self.model.lowerPositionLimit[ik_q], self.model.upperPositionLimit[ik_q]
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo) * soft_limit_factor
        self._ik_idx_q = ik_q
        self._q_min, self._q_max = mid - half, mid + half
        self._qd_max = self.model.velocityLimit[ik_v]

        self._feet = [self.model.getFrameId(f"{side}_foot_roll_link") for side in ("left", "right")]
        self.reset(self.q_default_act)

    def reset(self, q_act: np.ndarray):
        """
        Restart the internal model at q_act with the base upright and the origin of the
        support frame S (midpoint of the two sole centres) at the world origin.
        """
        q = pin.neutral(self.model)
        q[self._act_idx_q] = q_act
        for idx_q, src, gain in self._passive:
            q[idx_q] = gain * q_act[src]
        pin.framesForwardKinematics(self.model, self.data, q)
        soles = [self.data.oMf[f].act(SOLE_CENTRE) for f in self._feet]
        q[:3] -= 0.5 * (soles[0] + soles[1])
        pin.framesForwardKinematics(self.model, self.data, q)
        self.q = q
        self._sole_targets = [self.data.oMf[f].copy() for f in self._feet]
        self.com = pin.centerOfMass(self.model, self.data, q).copy()
        self.com_height = float(self.com[2])

    @property
    def q_des(self) -> np.ndarray:
        """Joint targets for the actuated joints; arms and head stay at their defaults."""
        q_des = self.q_default_act.copy()
        q_des[self._ik_act] = self.q[self._act_idx_q[self._ik_act]]
        return q_des

    def step(self, c_ref: np.ndarray, cdot_ref: np.ndarray) -> np.ndarray:
        """Advance the internal model one tick towards the CoM reference (x, y) in S; returns q_des."""
        model, data, q, G = self.model, self.data, self.q, self._G
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        self.com = pin.centerOfMass(model, data, q).copy()
        J_com = pin.jacobianCenterOfMass(model, data, q)

        rows, targets, weights = [], [], []

        def task(J, target, w):
            rows.append(J @ G)
            targets.append(target)
            weights.append(np.full(len(target), w))

        for f, T in zip(self._feet, self._sole_targets):
            M = data.oMf[f]
            J = pin.getFrameJacobian(model, data, f, pin.LOCAL_WORLD_ALIGNED)
            e = np.concatenate([T.translation - M.translation, pin.log3(T.rotation @ M.rotation.T)])
            task(J[SOLE_ROWS], self.k["sole"] * e[SOLE_ROWS], self.w["sole"])
        task(J_com[:2], cdot_ref + self.k["com"] * (c_ref - self.com[:2]), self.w["com"])
        task(J_com[2:], self.k["com"] * np.array([self.com_height - self.com[2]]), self.w["height"])
        J_base = pin.getJointJacobian(model, data, 1, pin.LOCAL_WORLD_ALIGNED)
        task(J_base[3:], self.k["pelvis"] * pin.log3(data.oMi[1].rotation.T), self.w["pelvis"])
        J_torso = np.zeros((1, model.nv))
        J_torso[0, np.flatnonzero(G[:, 6 + self._torso])] = 1.0
        torso_err = self.q_default_act[self._ik_act[self._torso]] - q[self._ik_idx_q[self._torso]]
        task(J_torso, self.k["torso"] * np.array([torso_err]), self.w["torso"])

        sw = np.sqrt(np.concatenate(weights))
        nx = G.shape[1]
        A = np.vstack([sw[:, None] * np.vstack(rows), np.sqrt(self.w["damping"]) * np.eye(nx)])
        b = np.concatenate([sw * np.concatenate(targets), np.zeros(nx)])

        q_ik = q[self._ik_idx_q]
        lo = np.concatenate([np.full(6, -np.inf), np.maximum(-self._qd_max, (self._q_min - q_ik) / self.dt)])
        hi = np.concatenate([np.full(6, np.inf), np.minimum(self._qd_max, (self._q_max - q_ik) / self.dt)])
        x = lsq_linear(A, b, bounds=(lo, hi), method="bvls").x
        self.q = pin.integrate(model, q, G @ x * self.dt)
        return self.q_des
