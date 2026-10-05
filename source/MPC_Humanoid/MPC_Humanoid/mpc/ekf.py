# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Base position estimation by fusing leg kinematics and the IMU (stage 3).

Extended Kalman filter of Bloesch et al., "State Estimation for Legged Robots - Consistent Fusion of Leg
Kinematics and IMU", RSS 2012 (references/papers/bloesch2012_rss.pdf):

x = [r, v, R, p_left, p_right, b_f, b_w]     IMU position, velocity, orientation (world); foot contact points
                                             (world); accelerometer and gyro bias (IMU frame)

The filter body is the IMU frame (imu_link on upper_body_link), so the accelerometer and gyro enter the
prediction directly; the foot positions relative to it, s_i = R^T (p_i - r), come from the encoders through
torso pitch and the legs. The base pose follows from the same kinematics. A foot in contact may slip by the
process noise on p_i; a foot in the air is dropped and its contact point re-initialised on touchdown.

Leg kinematics and the IMU leave absolute position and yaw unobservable (paper section IV). The yaw of the AHRS
orientation (gyro + magnetometer fused inside the MTi-630) is an extra measurement, so the yaw stays bounded and
the x, y axes of the filter follow the AHRS heading.

Differences from the paper: the orientation error is a world-frame rotation vector (R = exp(dphi^) R_hat), which
makes the orientation block of F the identity; the contact point of a foot follows its contact mode (sole centre
when flat, the edge it rolls on otherwise, CONTACT_MODES) and moves with the sole kinematics when the mode changes;
the discrete process noise is first order in dt.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from .model import CONTACT_MODES, CONTROLLER_URDF, GRAVITY, PASSIVE_COUPLING

_G = np.array([0.0, 0.0, -GRAVITY])
_R, _V, _PHI, _P, _BF, _BW = 0, 3, 6, 9, 15, 18
_N = 21


def _yaw(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def _wrap(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)


def _skew(a: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


class LegKinematicsEkf:
    """Base position and orientation from encoders, accelerometer and gyro, without contact sensing.

    Noise parameters are continuous-time densities (std^2 per Hz) for the IMU and the foot slip, and std for the
    kinematics. The gyro default is the Xsens MTi-630 leaflet value (0.007 deg/s/sqrt(Hz), also the env noise). The
    accelerometer density is far above the leaflet's (60 ug/sqrt(Hz), 3.5e-7): one 200 Hz sample held for a tick
    misrepresents the touchdown impacts (up to 31 m/s^2, world z averaging 0.16 m/s^2 over a stepping run in Isaac);
    with the leaflet value z drifted 325 mm in 5 s with a 3 mm std, with 0.1 the error stayed within 2 mm. The
    encoder std is about one Dynamixel tick (4096 per turn).

    yaw_std: of the AHRS yaw, the MTi-630 heading accuracy (about 1 deg in a clean magnetic field; near motors and
    steel it can be worse).

    estimate_bias: off by default; the paper switches it at runtime because typical gaits lie close to the cases
    where the biases are unobservable, and the Isaac IMU has no bias.
    """

    def __init__(
        self,
        act_names: list[str],
        accel_density: float = 0.1,
        gyro_density: float = np.radians(0.007) ** 2,
        accel_bias_density: float = 1e-8,
        gyro_bias_density: float = 1e-10,
        slip_density: float = 1e-6,
        kinematics_std: float = 0.002,
        encoder_std: float = 2 * np.pi / 4096,
        yaw_std: float = np.radians(1.0),
        estimate_bias: bool = False,
    ):
        self.act_names = list(act_names)
        self.model = pin.buildModelFromUrdf(CONTROLLER_URDF, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        self._idx_q = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in self.act_names])
        self._passive = []  # (idx_q, index into the actuated vector, gain)
        for side in ("left", "right"):
            for passive, (active, gain) in PASSIVE_COUPLING.items():
                j = self.model.joints[self.model.getJointId(f"{side}_{passive}")]
                self._passive.append((j.idx_q, self.act_names.index(f"{side}_{active}"), gain))
        self._imu = self.model.getFrameId("imu_link")
        self._feet = [self.model.getFrameId(f"{side}_foot_roll_link") for side in ("left", "right")]

        self.Qf, self.Qw = accel_density, gyro_density
        self.Qbf, self.Qbw = accel_bias_density, gyro_bias_density
        self.Qp = slip_density
        self.Rs, self.Ra = kinematics_std**2, encoder_std**2
        self.Ryaw = yaw_std**2
        self.estimate_bias = estimate_bias
        self.reset()

    def reset(self):
        self.r = self.v = self.R = None
        self.p = np.zeros((2, 3))
        self.bf, self.bw = np.zeros(3), np.zeros(3)
        self.P = np.zeros((_N, _N))
        self.modes = ["air", "air"]
        self._imu_last: tuple[np.ndarray, np.ndarray] | None = None
        self._r_lin = self._v_lin = self._R_lin = None  # first-estimate linearization point (a priori state)
        self._p_lin = np.zeros((2, 3))

    def _kinematics(self, q_act: np.ndarray, points: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Contact points in the IMU frame (2 x 3) and the base pose in the IMU frame (R_ib, p_ib)."""
        q = pin.neutral(self.model)
        q[self._idx_q] = q_act
        for idx_q, src, gain in self._passive:
            q[idx_q] = gain * q_act[src]
        pin.framesForwardKinematics(self.model, self.data, q)
        imu = self.data.oMf[self._imu]
        s = np.array([imu.actInv(self.data.oMf[f].act(c)) for f, c in zip(self._feet, points)])
        return s, imu.rotation.T, -imu.rotation.T @ imu.translation

    def _kinematics_cov(self, q_act: np.ndarray, points: list[np.ndarray], s: np.ndarray) -> np.ndarray:
        """Covariance of each contact point in the IMU frame (2 x 3 x 3): model error plus encoder noise
        through the leg Jacobian (paper eq. 25), the Jacobian by forward differences."""
        eps = 1e-6
        J = np.zeros((2, 3, len(q_act)))
        for k in range(len(q_act)):
            dq = q_act.copy()
            dq[k] += eps
            J[:, :, k] = (self._kinematics(dq, points)[0] - s) / eps
        return self.Rs * np.eye(3) + self.Ra * J @ J.transpose(0, 2, 1)

    def _init(self, s: np.ndarray, R_ahrs: np.ndarray, contact: list[bool]):
        """Start with the AHRS orientation and the lowest contact point at height 0, x = y = 0 below the IMU."""
        self.R = R_ahrs.copy()
        feet = self.R @ s.T
        self.r = np.r_[0.0, 0.0, -feet[2, contact].min()]
        self.v = np.zeros(3)
        self.p = self.r + feet.T
        self.P = np.zeros((_N, _N))
        self.P[_PHI : _PHI + 3, _PHI : _PHI + 3] = np.diag([np.radians(0.5) ** 2, np.radians(0.5) ** 2, 1e-4])
        self.P[_V : _V + 3, _V : _V + 3] = 1e-4 * np.eye(3)
        if self.estimate_bias:
            self.P[_BF : _BF + 3, _BF : _BF + 3] = 0.05**2 * np.eye(3)
            self.P[_BW : _BW + 3, _BW : _BW + 3] = np.radians(0.5) ** 2 * np.eye(3)
        for i in (0, 1):
            self._reset_foot(i)
        self._r_lin, self._v_lin, self._R_lin = self.r.copy(), self.v.copy(), self.R.copy()
        self._p_lin = self.p.copy()

    def _reset_foot(self, i: int):
        """Drop the foot from the estimate: no correlation, uncertainty far beyond any step (the paper's 'infinite')."""
        sl = slice(_P + 3 * i, _P + 3 * i + 3)
        self.P[sl, :] = 0.0
        self.P[:, sl] = 0.0
        self.P[sl, sl] = 1.0 * np.eye(3)

    def _predict(self, f: np.ndarray, w: np.ndarray, dt: float):
        """IMU propagation (paper eqs. 30-35) and the covariance with first-estimate Jacobians (eqs. 71-79)."""
        f, w = f - self.bf, w - self.bw
        a = self.R @ f + _G
        r_prior, v_prior, R_prior = self._r_lin, self._v_lin, self._R_lin
        self.r = self.r + dt * self.v + 0.5 * dt * dt * a
        self.v = self.v + dt * a
        self.R = self.R @ pin.exp3(w * dt)

        # world-frame accelerations that carry the a priori states from k to k+1 exactly, so the linearized
        # system keeps position and yaw unobservable
        a1 = (self.r - r_prior - dt * v_prior) / (0.5 * dt * dt) - _G
        a2 = (self.v - v_prior) / dt - _G
        F = np.eye(_N)
        F[_R : _R + 3, _V : _V + 3] = dt * np.eye(3)
        F[_R : _R + 3, _PHI : _PHI + 3] = -0.5 * dt * dt * _skew(a1)
        F[_V : _V + 3, _PHI : _PHI + 3] = -dt * _skew(a2)
        if self.estimate_bias:
            F[_R : _R + 3, _BF : _BF + 3] = -0.5 * dt * dt * R_prior
            F[_V : _V + 3, _BF : _BF + 3] = -dt * R_prior
            F[_PHI : _PHI + 3, _BW : _BW + 3] = -dt * R_prior

        Q = np.zeros((_N, _N))
        Q[_R : _R + 3, _R : _R + 3] = dt**3 / 3 * self.Qf * np.eye(3)
        Q[_R : _R + 3, _V : _V + 3] = Q[_V : _V + 3, _R : _R + 3] = dt**2 / 2 * self.Qf * np.eye(3)
        Q[_V : _V + 3, _V : _V + 3] = dt * self.Qf * np.eye(3)
        Q[_PHI : _PHI + 3, _PHI : _PHI + 3] = dt * self.Qw * np.eye(3)
        for i in (0, 1):
            if self.modes[i] != "air":
                sl = slice(_P + 3 * i, _P + 3 * i + 3)
                Q[sl, sl] = dt * self.Qp * np.eye(3)  # isotropic, so R Qp R^T = Qp
        if self.estimate_bias:
            Q[_BF : _BF + 3, _BF : _BF + 3] = dt * self.Qbf * np.eye(3)
            Q[_BW : _BW + 3, _BW : _BW + 3] = dt * self.Qbw * np.eye(3)
        self.P = F @ self.P @ F.T + Q

    def _correct(self, s: np.ndarray, S: np.ndarray, feet: list[int], R_ahrs: np.ndarray):
        """Leg-kinematics update (paper eqs. 46-53) plus the AHRS yaw, Jacobians at the first estimates."""
        m = 3 * len(feet) + 1
        y, H, Rk = np.zeros(m), np.zeros((m, _N)), np.zeros((m, m))
        for k, i in enumerate(feet):
            rows = slice(3 * k, 3 * k + 3)
            y[rows] = s[i] - self.R.T @ (self.p[i] - self.r)
            H[rows, _R : _R + 3] = -self._R_lin.T
            H[rows, _PHI : _PHI + 3] = self._R_lin.T @ _skew(self._p_lin[i] - self._r_lin)
            H[rows, _P + 3 * i : _P + 3 * i + 3] = self._R_lin.T
            Rk[rows, rows] = S[i]
        y[-1] = _wrap(_yaw(R_ahrs) - _yaw(self.R))
        eps = 1e-6
        H[-1, _PHI : _PHI + 3] = [
            _wrap(_yaw(pin.exp3(eps * e) @ self._R_lin) - _yaw(self._R_lin)) / eps for e in np.eye(3)
        ]
        Rk[-1, -1] = self.Ryaw

        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + Rk)
        dx = K @ y
        IKH = np.eye(_N) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rk @ K.T
        self.r = self.r + dx[_R : _R + 3]
        self.v = self.v + dx[_V : _V + 3]
        self.R = pin.exp3(dx[_PHI : _PHI + 3]) @ self.R
        self.p = self.p + dx[_P : _P + 6].reshape(2, 3)
        if self.estimate_bias:
            self.bf = self.bf + dx[_BF : _BF + 3]
            self.bw = self.bw + dx[_BW : _BW + 3]

    def update(
        self,
        q_act: np.ndarray,
        qd_act: np.ndarray,
        accel: np.ndarray,
        gyro: np.ndarray,
        imu_quat_xyzw: np.ndarray,
        modes: list[str],
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """One tick. q_act, qd_act in act_names order; accel (with gravity, as an accelerometer reads it) and gyro
        in the IMU frame; of the AHRS quaternion, the initial orientation and then the yaw are used. modes: contact
        mode per foot (left, right) from ContactModeMonitor. Returns the base position, velocity and rotation in the
        world."""
        q_act = np.asarray(q_act, dtype=float)
        points = [CONTACT_MODES[m][0] for m in modes]
        s, R_ib, p_ib = self._kinematics(q_act, points)
        contact = [m != "air" for m in modes]

        x, y, z, w = (float(c) for c in imu_quat_xyzw / np.linalg.norm(imu_quat_xyzw))
        R_ahrs = pin.Quaternion(w, x, y, z).matrix()
        if self.r is None:
            self.modes = list(modes)
            self._init(s, R_ahrs, contact)
        else:
            self._predict(*self._imu_last, dt)
            if any(contact):
                prev = self._kinematics(q_act, [CONTACT_MODES[m][0] for m in self.modes])[0]
            for i in (0, 1):
                if contact[i] and self.modes[i] == "air":
                    self._reset_foot(i)
                    self.p[i] = self.r + self.R @ s[i]
                    self._p_lin[i] = self.p[i]
                elif contact[i] and modes[i] != self.modes[i]:
                    shift = self.R @ (s[i] - prev[i])  # same sole, other contact point
                    self.p[i] += shift
                    self._p_lin[i] += shift
            self.modes = list(modes)
            self._r_lin, self._v_lin, self._R_lin = self.r.copy(), self.v.copy(), self.R.copy()
            self._correct(s, self._kinematics_cov(q_act, points, s), [i for i in (0, 1) if contact[i]], R_ahrs)

        self._imu_last = (np.asarray(accel, dtype=float), np.asarray(gyro, dtype=float))
        eps = 1e-6
        _, _, p_ib_eps = self._kinematics(q_act + eps * np.asarray(qd_act, dtype=float), points)
        p_ib_d = (p_ib_eps - p_ib) / eps  # the base moves relative to the IMU through torso pitch
        v_base = self.v + self.R @ (np.cross(self._imu_last[1] - self.bw, p_ib) + p_ib_d)
        return self.r + self.R @ p_ib, v_base, self.R @ R_ib
