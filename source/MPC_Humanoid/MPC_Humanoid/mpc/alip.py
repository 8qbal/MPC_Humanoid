# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
ALIP foot placement (Gong & Grizzle 2022), per horizontal axis, as step-to-step feedback about the walking orbit.

State about the stance contact point p: x = c - p and the angular momentum L about p, divided by m H so it has
velocity units, l = L / (m H) = cd + h_G / (m H), with h_G the centroidal angular momentum (arms, swing leg, torso;
model.Ag). With a constant CoM height H, xd = l and ld = w^2 x, w = sqrt(g / H).

The contact switches once per step period T = t_ss + t_ds: the double support is taken as an instant switch at its
middle (the ZMP crosses from foot to foot during it). L about the new contact equals L about the old one when the
CoM moves horizontally, so the step u = p_new - p_old only shifts x. The step for which l at the end of the next step
reaches l_des is

    u = x- - (l_des - cosh(wT) l-) / (w sinh(wT)),   (x-, l-) predicted at the switch.

About a reference with the same l_des the step changes by du = dx- + coth(wT) / w dl-. The flat-footed robot does
not move like the point-foot LIP of the plan within a step: its CoP moves along the sole and the NMPC tracks the DCM
reference only on average, so du about the plan carries a large offset per foot (up to 0.25 m sagittal at 0.1 m/s in
Isaac, docs/stage3.md). The controller removes the offset it learns over the steps and uses du only as feedback on
what changes from step to step.
"""

from __future__ import annotations

import copy
import math

import numpy as np

from .gait import GaitPlan, swing_xy
from .model import GRAVITY, RigidContactModel

# Step relative to the stance foot [m]: sagittal reach of the crouched leg within U_RANGE_DEG is ~85 mm per foot at
# constant height (nmpc.py), lateral keeps the feet apart and within the hip roll range.
MAX_STEP_X = 0.12
MIN_WIDTH, MAX_WIDTH = 0.08, 0.18
# Walking: the ALIP foothold of a step is set once, at swing progress ALIP_AT. The offset of du is learned at
# ALIP_RATE per step, ALIP_GAIN and ALIP_MAX [m] apply to what is left. At 0.1 m/s in Isaac du at progress 0.5 stayed
# within +-5 mm of its offset from step to step: lateral +-31-37 mm per swing foot; sagittal ALIP_V_GAIN x the plan's
# CoM velocity, 1.5-1.6 s at 0.05 and 0.1 m/s through the speed ramps (a constant offset lagged over the ramp, and a
# 33 mm longer step put the stance sole on its toe). Learned only while the plan moves faster than ALIP_V_MIN [m/s].
ALIP_AT, ALIP_RATE, ALIP_GAIN, ALIP_MAX, ALIP_V_MIN = 0.5, 0.7, 0.5, 0.05, 0.01
# The first two steps start from standing (lateral du -21 mm against -33 mm later): not learned
ALIP_LEARN_FROM = 2
# du offsets per swing progress, for a du away from ALIP_AT: the sagittal ratio fell from 1.2 s at 0.5 to 0.5 s at 0.7
# and the lateral offset from +-30 to +-25 mm; step-to-step spread <= 5 mm (steady) and 13 mm (ramps) from 0.4 to 0.9,
# 30-50 mm before 0.3 (0.05 and 0.1 m/s, docs/stage3.md). A du further than ALIP_OUTLIER [m] from its offset (a push)
# is not learned.
ALIP_BINS, ALIP_OUTLIER = (0.4, 0.5, 0.6, 0.7, 0.8, 0.9), 0.03


def _blend(old: float | None, new: float) -> float:
    return new if old is None else old + ALIP_RATE * (new - old)


def predict(x: np.ndarray, lm: np.ndarray, w: float, t: float) -> tuple[np.ndarray, np.ndarray]:
    ch, sh = math.cosh(w * t), math.sinh(w * t)
    return ch * x + sh / w * lm, w * sh * x + ch * lm


def step_change(dx: np.ndarray, dl: np.ndarray, t_switch: float, period: float, w: float) -> np.ndarray:
    """du for a deviation (dx, dl) of the state now from the reference; t_switch: time to the contact switch (rest
    of the single support plus half the double support)."""
    dx_m, dl_m = predict(dx, dl, w, max(t_switch, 0.0))
    return dx_m + dl_m / (w * math.tanh(w * period))


def clamp_step(u: np.ndarray, left_lands: bool) -> np.ndarray:
    side = 1.0 if left_lands else -1.0
    return np.array([np.clip(u[0], -MAX_STEP_X, MAX_STEP_X), side * np.clip(side * u[1], MIN_WIDTH, MAX_WIDTH)])


class FootholdAdapter:
    def __init__(self, model: RigidContactModel):
        self.model = model
        self.reset()

    def reset(self) -> None:
        self._ref = None  # (step, plan before its ALIP foothold)
        # learned du per progress bin: lateral per swing foot, sagittal per plan velocity
        self._table = {s: [None, None, None] for s in ALIP_BINS}
        self._offset = self._table[ALIP_AT]
        self._learned = (None, set())  # (step, bins learned in it)
        self.held = False  # held or recovering since the last touchdown

    def restart(self) -> None:
        self._ref = None

    def reference_plan(self, plan: GaitPlan, step: int | None) -> GaitPlan:
        return self._ref[1] if self._ref is not None and self._ref[0] == step else plan

    def update(self, t: float, q, qd, plan: GaitPlan, soles: np.ndarray, lift_xy: np.ndarray) -> None:
        self._alip(t, q, qd, plan, soles, lift_xy)
        self._learn_offsets(t, q, qd, plan, soles)

    def _alip(self, t, q, qd, plan, soles, lift_xy):
        """Foothold of the swinging foot: nominal plus the ALIP feedback (alip.py) about the learned orbit. The last
        step too: without it the CoM ran 30% faster than the plan sideways into the stop and fell (0.1 m/s)."""
        ph = plan.phase(t)
        k = ph.step
        if k is None or (t - ph.t0) / ph.t_nom < ALIP_AT:
            return
        if self._ref is not None and self._ref[0] == k:
            return
        self._ref = (k, copy.deepcopy(plan))
        if self.held:
            # the plan was paused: its reference CoM velocity is not the robot's, du read -0.1 m for a step that
            # then landed behind the stance foot (0.1 m/s, 2026-10-17)
            return
        du, v = self._alip_du(t, q, qd, plan, ph, soles)
        i = ph.swing
        fb = np.zeros(2)
        if k >= ALIP_LEARN_FROM:
            off = self._offset
            offset = du[1] if off[i] is None else off[i]
            fb[1] = np.clip(ALIP_GAIN * (du[1] - offset), -ALIP_MAX, ALIP_MAX)
            if off[2] is not None:
                fb[0] = np.clip(ALIP_GAIN * (du[0] - off[2] * v), -ALIP_MAX, ALIP_MAX)
            off[i] = _blend(offset, du[1])
            if abs(v) > ALIP_V_MIN:
                off[2] = _blend(off[2], du[0] / v)
        self._move_foothold(t, ph, plan, fb, plan, soles, lift_xy)

    def _learn_offsets(self, t, q, qd, plan, soles):
        """du offsets of the bins other than ALIP_AT (learned in _alip), once per step as the swing passes them."""
        ph = plan.phase(t)
        k = ph.step
        if k is None or k < ALIP_LEARN_FROM or self.held:
            return
        if self._learned[0] != k:
            self._learned = (k, set())
        prog = (t - ph.t0) / ph.t_nom
        due = [b for b in ALIP_BINS if b != ALIP_AT and b <= prog < b + 0.1 and b not in self._learned[1]]
        if not due:
            return
        self._learned[1].add(due[0])
        ref = self.reference_plan(plan, k)
        du, v = self._alip_du(t, q, qd, ref, ph, soles)
        off, i = self._table[due[0]], ph.swing
        if off[i] is None or abs(du[1] - off[i]) < ALIP_OUTLIER:
            off[i] = _blend(off[i], du[1])
        if abs(v) > ALIP_V_MIN and (off[2] is None or abs(du[0] - off[2] * v) < ALIP_OUTLIER):
            off[2] = _blend(off[2], du[0] / v)

    def _alip_du(self, t, q, qd, plan, ph, soles):
        """ALIP step change du about the reference of plan, and the plan's sagittal CoM velocity."""
        r = plan.reference(t)
        stance = soles[:2, 1 - ph.swing]
        com, comd = np.array(self.model.f_com(q)).ravel(), np.array(self.model.f_comd(q, qd)).ravel()
        h = self.model.Ag @ qd
        lm = comd[:2] + np.array([h[1], -h[0]]) * plan.omega**2 / (GRAVITY * self.model.mass)
        du = step_change(
            com[:2] - stance - (r["com"] - plan.stance_xy(ph.step)),
            lm - r["com_vel"],
            ph.t1 - t + 0.5 * plan.p.t_ds,
            plan.p.t_ss + plan.p.t_ds,
            plan.omega,
        )
        return du, r["com_vel"][0]

    def _move_foothold(self, t, ph, ref, fb, plan, soles, lift_xy):
        """Foothold of the current step to the stance sole + (nominal step of plan ref) + fb, within the step limits."""
        k, i = ph.step, ph.swing
        stance = soles[:2, 1 - i]
        old = plan.footholds[k].copy()
        plan.replan(k, stance + clamp_step(ref.footholds[k] - ref.stance_xy(k) + fb, i == 0))
        # the swing reference stays continuous: swing_xy is linear in its lift and target points
        f = swing_xy(np.zeros(2), np.ones(2), (t - ph.t0) / ph.t_nom)[0]
        if f < 1.0:
            lift_xy[i] -= (plan.footholds[k] - old) * f / (1.0 - f)
