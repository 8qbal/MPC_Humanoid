# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Gait plan for stepping in place: contact schedule, DCM / CoM reference and swing-foot
height.

Timeline: stand -> DS (ZMP from the middle of the soles to the first stance sole) -> SS -> DS -> ... -> SS ->
DS (ZMP back to the middle) -> stand. The ZMP is constant in single support and moves linearly in double
support; the DCM follows from integrating xi_dot = omega (xi - p) backwards from rest at the end, the CoM from
c_dot = omega (xi - c) forwards from rest at the start (both in closed form per phase). The reference is only a
feedforward for the NMPC cost; feedback comes from the NMPC.

Positions are in the yaw-free world frame of RigidContactModel (xy of the sole centres, z up).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .model import GRAVITY


@dataclass
class GaitParams:
    t_ss: float = 0.30
    t_ds: float = 0.10
    clearance: float = 0.02  # m, peak sole lift
    n_steps: int = 10
    t_stand: float = 1.5  # s standing before the first weight shift and after the last step
    first_swing: int = 1  # 0 left, 1 right
    v_descent: float = 0.1  # m/s, sole-centre descent once single support lasts longer than planned


@dataclass
class Phase:
    t0: float
    t1: float
    zmp0: np.ndarray
    zmp1: np.ndarray
    swing: int | None  # foot in the air (0 left, 1 right), None in double support
    t_nom: float  # planned duration; stretch() changes t1 but not t_nom


class GaitPlan:
    def __init__(self, params: GaitParams, feet_xy: np.ndarray, com_height: float):
        """feet_xy: (2, 2) sole-centre xy of the left and right foot; com_height above the soles."""
        self.p = params
        self.feet_xy = np.asarray(feet_xy, dtype=float)
        self.omega = math.sqrt(GRAVITY / com_height)
        mid = self.feet_xy.mean(0)

        phases, t = [], 0.0

        def add(dt, z0, z1, swing=None):
            nonlocal t
            phases.append(Phase(t, t + dt, np.array(z0), np.array(z1), swing, dt))
            t += dt

        add(params.t_stand, mid, mid)
        swing, prev = params.first_swing, mid
        for _ in range(params.n_steps):
            stance = self.feet_xy[1 - swing]
            add(params.t_ds, prev, stance)
            add(params.t_ss, stance, stance, swing)
            prev, swing = stance, 1 - swing
        add(params.t_ds, prev, mid)
        add(params.t_stand, mid, mid)
        self.phases = phases
        self._integrate()

    @property
    def duration(self) -> float:
        return self.phases[-1].t1

    def stretch(self, i: int, dt: float):
        """Lengthen phase i by dt (negative: shorten, to 1 ms at least) and shift every later phase."""
        dt = max(dt, 1e-3 - (self.phases[i].t1 - self.phases[i].t0))
        self.phases[i].t1 += dt
        for ph in self.phases[i + 1 :]:
            ph.t0 += dt
            ph.t1 += dt
        self._integrate()

    def _integrate(self):
        w, n = self.omega, len(self.phases)
        self._B = [None] * n  # xi = p + p_dot / w + B exp(w (t - t1))
        xi = self.phases[-1].zmp1.copy()
        for i in reversed(range(n)):
            ph = self.phases[i]
            T = ph.t1 - ph.t0
            pd = (ph.zmp1 - ph.zmp0) / T
            self._B[i] = xi - ph.zmp1 - pd / w
            xi = ph.zmp0 + pd / w + self._B[i] * math.exp(-w * T)
        # c = p + B / 2 exp(w (t - t1)) + D exp(-w (t - t0)), D from continuity of c
        self._D, c = [None] * n, self.phases[0].zmp0.copy()
        for i, ph in enumerate(self.phases):
            T = ph.t1 - ph.t0
            self._D[i] = c - ph.zmp0 - 0.5 * self._B[i] * math.exp(-w * T)
            c = ph.zmp1 + 0.5 * self._B[i] + self._D[i] * math.exp(-w * T)

    def index(self, t: float) -> int:
        for i, ph in enumerate(self.phases):
            if t < ph.t1:
                return i
        return len(self.phases) - 1

    def modes(self, t: float) -> tuple[str, str]:
        swing = self.phases[self.index(t)].swing
        return tuple("air" if swing == i else "flat" for i in (0, 1))

    def reference(self, t: float) -> dict[str, np.ndarray]:
        """ZMP, DCM, CoM position / velocity / acceleration (xy) at time t."""
        i = self.index(t)
        ph, w = self.phases[i], self.omega
        tau = min(max(t, ph.t0), ph.t1) - ph.t0
        T = ph.t1 - ph.t0
        pd = (ph.zmp1 - ph.zmp0) / T
        p = ph.zmp0 + pd * tau
        eB = self._B[i] * math.exp(w * (tau - T))
        eD = self._D[i] * math.exp(-w * tau)
        c = p + 0.5 * eB + eD
        return {
            "zmp": p,
            "dcm": p + pd / w + eB,
            "com": c,
            "com_vel": pd + 0.5 * w * eB - w * eD,
            "com_acc": w * w * (c - p),
        }

    def swing(self, t: float) -> tuple[int | None, float, float]:
        """(swing foot, sole-centre height, swing progress s); height 0 outside single support.

        z = h 64 s^3 (1 - s)^3 with s = (t - t0) / t_nom: peak h at mid swing, zero velocity and acceleration at
        lift-off and planned touchdown. A phase stretched beyond t_nom (late touchdown) keeps descending at
        v_descent."""
        ph = self.phases[self.index(t)]
        if ph.swing is None:
            return None, 0.0, 0.0
        s = max((t - ph.t0) / ph.t_nom, 0.0)
        if s > 1.0:
            return ph.swing, -self.p.v_descent * (t - ph.t0 - ph.t_nom), s
        return ph.swing, self.p.clearance * 64 * s**3 * (1 - s) ** 3, s
