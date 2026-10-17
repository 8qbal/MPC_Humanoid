# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""

Gait plan: contact schedule, footholds, DCM / CoM reference and swing-foot height.

Each single support k swings one foot to footholds[k]. Nominally a step lands step_length(k) ahead of the stance
foot at the initial foot spacing: zero for stepping in place (vx = 0); walking ramps the step length up over the
first ramp_steps steps and down to zero (feet side by side) over the last ones. replan() moves a foothold (ALIP, or
the measured landing) and every later one with it.

"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .model import GRAVITY


@dataclass
class GaitParams:
    t_ss: float = 0.30
    t_ds: float = 0.20  # 0.10 slid the soles apart at weight transfer with the 0.038 armature (2026-10-15)
    clearance: float = 0.02  # m, peak sole lift
    n_steps: int = 10
    t_stand: float = 1.5  # s standing before the first weight shift and after the last step
    first_swing: int = 1  # 0 left, 1 right
    v_descent: float = 0.1  # m/s, sole-centre descent once single support lasts longer than planned
    vx: float = 0.0  # m/s, commanded forward speed
    ramp_steps: int = 2  # steps to reach vx at the start and to stop at the end
    alip: bool = False  # footholds from the ALIP planner (alip.py) instead of the nominal ones


@dataclass
class Phase:
    t0: float
    t1: float
    zmp0: np.ndarray
    zmp1: np.ndarray
    swing: int | None  # foot in the air (0 left, 1 right), None in double support
    t_nom: float  # planned duration; stretch() changes t1 but not t_nom
    step: int | None = None  # index into GaitPlan.footholds of the single support


class GaitPlan:
    def __init__(self, params: GaitParams, feet_xy: np.ndarray, com_height: float, spacing: np.ndarray | None = None):
        """feet_xy: (2, 2) sole-centre xy of the left and right foot; com_height above the soles; spacing: left minus
        right sole centre of the nominal footholds (default: of feet_xy)."""
        self.p = params
        self.feet_xy = np.asarray(feet_xy, dtype=float)
        self.omega = math.sqrt(GRAVITY / com_height)
        self.spacing = self.feet_xy[0] - self.feet_xy[1] if spacing is None else np.asarray(spacing, dtype=float)

        n = params.n_steps
        self.swings = [(params.first_swing + k) % 2 for k in range(n)]
        self.footholds = [np.zeros(2) for _ in range(n)]

        phases, t = [], 0.0

        def add(dt, swing=None, step=None):
            nonlocal t
            phases.append(Phase(t, t + dt, np.zeros(2), np.zeros(2), swing, dt, step))
            t += dt

        add(params.t_stand)
        for k in range(n):
            add(params.t_ds)
            add(params.t_ss, self.swings[k], k)
        add(params.t_ds)
        add(params.t_stand)

        self.phases = phases
        self._nominal(0)

    def step_length(self, k: int) -> float:
        """Nominal step length of step k: the swing foot lands this far ahead of the stance foot."""
        r = self.p.ramp_steps + 1
        f = min((k + 1) / r, (self.p.n_steps - 1 - k) / r, 1.0)
        return self.p.vx * (self.p.t_ss + self.p.t_ds) * f

    def stance_xy(self, k: int) -> np.ndarray:
        """Position of the foot that stands during step k (k = n_steps: after the last step, the last stance)."""
        foot = 1 - self.swings[k] if k < len(self.swings) else self.swings[-1]
        for j in reversed(range(k)):
            if self.swings[j] == foot:
                return self.footholds[j]
        return self.feet_xy[foot]

    def feet_after(self, k: int) -> np.ndarray:
        """(2, 2) foot positions after the first k steps."""
        feet = self.feet_xy.copy()
        for j in range(k):
            feet[self.swings[j]] = self.footholds[j]
        return feet

    def replan(self, k: int, xy: np.ndarray):
        """Foothold of step k to xy; the later ones follow at their nominal offsets."""
        self.footholds[k] = np.asarray(xy, dtype=float).copy()
        self._nominal(k + 1)

    def _nominal(self, k0: int):
        for k in range(k0, self.p.n_steps):
            side = 1.0 if self.swings[k] == 0 else -1.0
            self.footholds[k] = self.stance_xy(k) + side * self.spacing + np.array([self.step_length(k), 0.0])
        self._set_zmps()

    def _set_zmps(self):
        """ZMP of every phase from the footholds: at the stance foot in single support, moving from one stance foot
        to the next in double support, at the middle of the soles while standing."""
        n = self.p.n_steps
        stand0, stand1 = self.feet_xy.mean(0), self.feet_after(n).mean(0)
        prev = stand0
        for i, ph in enumerate(self.phases):
            if ph.step is not None:
                ph.zmp0 = ph.zmp1 = self.stance_xy(ph.step)
            elif i == 0:
                ph.zmp0 = ph.zmp1 = stand0
            elif i == len(self.phases) - 1:
                ph.zmp0 = ph.zmp1 = stand1
            else:
                nxt = self.phases[i + 1]
                ph.zmp0 = prev
                ph.zmp1 = self.stance_xy(nxt.step) if nxt.step is not None else stand1
            prev = ph.zmp1
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

    def shift(self, dt: float):
        """Move the whole plan dt later."""
        for ph in self.phases:
            ph.t0 += dt
            ph.t1 += dt

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

    def phase(self, t: float) -> Phase:
        return self.phases[self.index(t)]

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


def swing_xy(lift: np.ndarray, target: np.ndarray, s: float) -> np.ndarray:
    """Sole-centre xy of a swing from lift to target at progress s: minimum jerk, there by s = XY_DONE so the sole
    comes down vertically."""
    r = min(max(s / XY_DONE, 0.0), 1.0)
    return lift + (target - lift) * r**3 * (10 - 15 * r + 6 * r * r)


XY_DONE = 0.85
