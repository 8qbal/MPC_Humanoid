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

import math

import numpy as np

# Step relative to the stance foot [m]: sagittal reach of the crouched leg within U_RANGE_DEG is ~85 mm per foot at
# constant height (nmpc.py), lateral keeps the feet apart and within the hip roll range.
MAX_STEP_X = 0.12
MIN_WIDTH, MAX_WIDTH = 0.08, 0.18


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
