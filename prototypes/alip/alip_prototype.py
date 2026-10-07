"""Offline ALIP footstep planner for Robonion.

Reduced-order study of the Angular-Momentum-based Linear Inverted Pendulum (ALIP) of Gong and Grizzle [1]
used as a footstep planner. The script contains no simulator dependency (numpy and matplotlib only).

Model. For each horizontal axis the state is the CoM position relative to the stance foot and the angular
momentum about the stance foot, at constant CoM height H and total mass m:

    sagittal  (x, L_y):   x_dot = L_y / (m H),    L_y_dot = m g x                     [1, Eq. 7]
    lateral   (y, Lp):    y_dot = Lp / (m H),     Lp_dot  = m g y,    Lp := -L_x

Both axes share the linear system d/dt [q, L] = [[0, 1/(mH)], [mg, 0]] [q, L], whose closed-form solution over a
time t is the matrix returned by `transition` [1, Eq. 8]. The angular momentum about the contact point is
continuous across a stance-foot change, so at touchdown only the position is re-referenced:

    q_new = q_end - u,    L_new = L_end,

with u the landing point of the swing foot relative to the previous stance foot.

Foot placement. With the predicted end-of-step momentum L_hat [1, Eq. 14] and the desired momentum at the end of
the next step L_des, the CoM position relative to the new stance foot is [1, Eq. 15]

    q_new = (L_des - cosh(l T) L_hat) / (m H l sinh(l T)),    l = sqrt(g / H).

The desired momenta follow from the periodic orbit of the gait (derived in this work, verified by `self_check`):

    sagittal, step length d = v T:           L_y,des = m H l coth(l T / 2) d / 2
    lateral, step width w:                   Lp_des  = s m H l tanh(l T / 2) w / 2

with s = +1 (-1) for a left (right) stance foot in the current step; L_des refers to the end of the next step.

Model parameters and their provenance are listed in README.md.

[1] Y. Gong and J. Grizzle, "Angular Momentum about the Contact Point for Control of Bipedal Locomotion:
    Validation in a LIP-based Controller", arXiv:2008.10763.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

GRAVITY = 9.81
SERVO_LIMIT = 4.08  # rad/s, hip servos (references/docs/joint_info.md)
DIVERGENCE_LIMIT = 0.6  # m, CoM distance from the stance foot beyond which a walk counts as failed


@dataclass
class Params:
    """Model, gait and kinematic-limit parameters (SI units)."""

    m: float = 7.933  # total mass
    H: float = 0.48  # CoM height
    T: float = 0.40  # time between touchdowns
    T_swing: float = 0.30  # swing-foot flight time
    t_replan: float = 0.20  # time after touchdown at which the landing point is fixed
    w: float = 0.11  # nominal lateral distance between the feet
    u_max_x: float = 0.20  # maximum sagittal landing distance from the stance foot
    u_max_y_out: float = 0.25  # maximum lateral landing distance from the stance foot
    w_min: float = 0.07  # minimum lateral foot-to-foot distance
    l_leg: float = 0.38  # effective hip-to-sole length for the joint-speed estimate
    adapt: bool = True  # False: land on the nominal footprint without feedback
    limits: bool = True  # enforce reach and servo limits
    ell: float = field(init=False)

    def __post_init__(self) -> None:
        self.ell = math.sqrt(GRAVITY / self.H)

    @property
    def gain(self) -> float:
        return self.m * self.H * self.ell


def transition(p: Params, t: float) -> np.ndarray:
    """State transition of [position, angular momentum] over time t [1, Eq. 8]."""
    c, s = math.cosh(p.ell * t), math.sinh(p.ell * t)
    return np.array([[c, s / p.gain], [p.gain * s, c]])


def desired_momentum_sagittal(p: Params, v: float) -> float:
    """Angular momentum L_y at the end of a step for a periodic gait at forward speed v."""
    return p.gain / math.tanh(p.ell * p.T / 2) * v * p.T / 2


def desired_momentum_lateral(p: Params, side: int) -> float:
    """Angular momentum L' at the end of a step with stance foot `side` (+1 left, -1 right)."""
    return -side * p.gain * math.tanh(p.ell * p.T / 2) * p.w / 2


def foot_placement(p: Params, state: np.ndarray, t: float, l_des: float) -> tuple[float, float]:
    """Evaluate [1, Eq. 14, 15] for one axis.

    Args:
        state: (position, momentum) relative to the stance foot, t seconds after touchdown.
        l_des: desired momentum at the end of the next step.

    Returns:
        CoM position relative to the new stance foot, and the predicted CoM position at the end of the step.
    """
    q_end, l_end = transition(p, p.T - t) @ state
    q_new = (l_des - math.cosh(p.ell * p.T) * l_end) / (p.gain * math.sinh(p.ell * p.T))
    return q_new, q_end


def landing_point(
    p: Params, sag: np.ndarray, lat: np.ndarray, t: float, side: int, v_des: float
) -> tuple[float, float]:
    """Landing point of the swing foot relative to the stance foot.

    Args:
        sag: (x, L_y) relative to the stance foot, t seconds after touchdown (x forward).
        lat: (y, L') relative to the stance foot, with y to the left and L' = -L_x.
        side: stance foot, +1 left and -1 right.
        v_des: commanded forward speed.

    Returns:
        (u_x, u_y), clipped to the reach limits when `p.limits` is set.
    """
    x_new, x_end = foot_placement(p, np.asarray(sag, float), t, desired_momentum_sagittal(p, v_des))
    y_new, y_end = foot_placement(p, np.asarray(lat, float), t, desired_momentum_lateral(p, -side))
    ux, uy = x_end - x_new, y_end - y_new
    if not p.adapt:
        ux, uy = v_des * p.T, -side * p.w
    if p.limits:
        ux = float(np.clip(ux, -p.u_max_x, p.u_max_x))
        uy = float(np.clip(-side * uy, p.w_min, p.u_max_y_out)) * (-side)
    return float(ux), float(uy)


def _apply_impulse(p: Params, sag: np.ndarray, lat: np.ndarray, push: dict) -> None:
    sag[1] += p.H * push["Jx"]
    lat[1] += p.H * push["Jy"]  # L' = -L_x, so a lateral impulse J_y adds H J_y


def _peak_joint_speed(p: Params, travel: np.ndarray, v_com: np.ndarray) -> float:
    """Peak hip joint speed of a swing foot on a minimum-jerk path, relative to the moving CoM [rad/s]."""
    s = np.linspace(0.0, 1.0, 61)
    profile = 30 * s**2 * (1 - s) ** 2
    relative = travel[None, :] / p.T_swing * profile[:, None] - v_com[None, :]
    return float(np.abs(relative).max() / p.l_leg)


def simulate(p: Params, v_des: float = 0.2, n_steps: int = 14, push: dict | None = None, ramp_steps: int = 3) -> dict:
    """Simulate `n_steps` steps with the ALIP footstep rule.

    The walk starts with the left foot as stance foot. The lateral axis starts on its periodic orbit; the
    sagittal axis starts at rest and the commanded speed ramps up over `ramp_steps` steps.

    Args:
        push: optional impulse {"step": k, "t": time after touchdown [s], "Jx": N s, "Jy": N s} in the stance frame.

    Returns:
        Dictionary with the CoM and stance-foot positions at each touchdown, a per-step log, and the failure
        status (CoM divergence or servo limit exceeded).
    """
    side = 1
    sag = np.array([0.0, 0.0])
    lat = np.array([-p.w / 2, p.gain * math.tanh(p.ell * p.T / 2) * p.w / 2])
    foot = np.array([0.0, p.w / 2])
    prev_u = np.array([0.0, side * p.w])
    times, com_xy, feet_xy, log = [], [], [], []
    t_abs = 0.0
    failed, reason = False, ""
    for k in range(n_steps):
        v = v_des * min(1.0, (k + 1) / ramp_steps)
        t_push = push["t"] if push and push["step"] == k else None

        segments = [(0.0, p.t_replan)]
        if t_push is not None and t_push < p.t_replan:
            segments = [(0.0, t_push), (t_push, p.t_replan)]
        t_now = 0.0
        for _, end in segments:
            sag, lat = transition(p, end - t_now) @ sag, transition(p, end - t_now) @ lat
            t_now = end
            if t_push is not None and len(segments) == 2 and end == t_push:
                _apply_impulse(p, sag, lat, push)

        v_com = np.array([sag[1], lat[1]]) / (p.m * p.H)
        ux, uy = landing_point(p, sag, lat, p.t_replan, side, v)

        if t_push is not None and t_push >= p.t_replan:
            sag, lat = transition(p, t_push - p.t_replan) @ sag, transition(p, t_push - p.t_replan) @ lat
            _apply_impulse(p, sag, lat, push)
            sag, lat = transition(p, p.T - t_push) @ sag, transition(p, p.T - t_push) @ lat
        else:
            sag, lat = transition(p, p.T - p.t_replan) @ sag, transition(p, p.T - p.t_replan) @ lat

        peak_joint = _peak_joint_speed(p, np.array([ux, uy]) + prev_u, v_com)
        times.append(t_abs + p.T)
        com_xy.append(foot + np.array([sag[0], lat[0]]))
        feet_xy.append(foot.copy())
        log.append(
            {
                "step": k,
                "side": side,
                "ux": ux,
                "uy": uy,
                "x_end": float(sag[0]),
                "y_end": float(lat[0]),
                "L_end": float(sag[1]),
                "Lp_end": float(lat[1]),
                "peak_joint": peak_joint,
            }
        )
        if peak_joint > SERVO_LIMIT and p.limits:
            failed, reason = True, f"servo limit {peak_joint:.2f} rad/s at step {k}"

        foot = foot + np.array([ux, uy])
        sag = np.array([sag[0] - ux, sag[1]])
        lat = np.array([lat[0] - uy, lat[1]])
        prev_u = np.array([ux, uy])
        side = -side
        t_abs += p.T
        if abs(sag[0]) > DIVERGENCE_LIMIT or abs(lat[0]) > DIVERGENCE_LIMIT:
            failed, reason = True, f"CoM {sag[0]:+.2f},{lat[0]:+.2f} m from the stance foot at step {k}"
            break
    return {
        "t": np.array(times),
        "com": np.array(com_xy),
        "feet": np.array(feet_xy),
        "log": log,
        "failed": failed,
        "reason": reason,
    }


def self_check(p: Params) -> dict:
    """Compare a long simulated walk with the closed-form periodic orbit and the matrix exponential."""
    v = 0.3
    result = simulate(p, v_des=v, n_steps=40, ramp_steps=1)
    steady = result["log"][-10:]
    system = np.array([[0, 1 / (p.m * p.H)], [p.m * GRAVITY, 0]])
    eigenvalues, eigenvectors = np.linalg.eig(system)
    exponential = (eigenvectors @ np.diag(np.exp(eigenvalues * 0.17)) @ np.linalg.inv(eigenvectors)).real
    return {
        "step_length": float(np.mean([s["ux"] for s in steady])),
        "expected_step_length": v * p.T,
        "step_width": float(np.mean([abs(s["uy"]) for s in steady])),
        "expected_step_width": p.w,
        "mean_speed": float((result["com"][-1, 0] - result["com"][-11, 0]) / (10 * p.T)),
        "expected_speed": v,
        "matrix_exp_error": float(np.abs(exponential - transition(p, 0.17)).max()),
    }


def max_recoverable_impulse(
    p: Params, direction: str, push_step: int = 6, t_push: float = 0.15, v_des: float = 0.2
) -> float:
    """Largest impulse [N s] in `direction` ("x+", "x-", "y+", "y-") after which the walk does not fail.

    The impulse is applied t_push seconds after touchdown of step `push_step` (left stance for even steps)
    and the limit is found by bisection.
    """
    key = "Jx" if direction[0] == "x" else "Jy"
    sign = 1.0 if direction[1] == "+" else -1.0

    def survives(impulse: float) -> bool:
        push = {"step": push_step, "t": t_push, "Jx": 0.0, "Jy": 0.0, key: sign * impulse}
        return not simulate(p, v_des=v_des, n_steps=push_step + 12, push=push)["failed"]

    low, high = 0.0, 20.0
    if not survives(0.0):
        return 0.0
    if survives(high):
        return high
    for _ in range(40):
        mid = (low + high) / 2
        low, high = (mid, high) if survives(mid) else (low, mid)
    return low


def _diverged(result: dict) -> bool:
    return result["failed"] and "CoM" in result["reason"]


def plot_nominal_walk(p: Params, path: Path) -> dict:
    result = simulate(p, v_des=0.2, n_steps=14)
    fig, (top, bottom) = plt.subplots(1, 2, figsize=(11, 4))
    top.plot(result["com"][:, 0], result["com"][:, 1], "-o", ms=3, label="CoM at touchdown")
    for i, foot in enumerate(result["feet"]):
        top.plot(foot[0], foot[1], "s", color="tab:red" if i % 2 == 0 else "tab:green", ms=7)
    top.plot([], [], "s", color="tab:red", label="left stance foot")
    top.plot([], [], "s", color="tab:green", label="right stance foot")
    top.set(xlabel="x forward [m]", ylabel="y left [m]", ylim=(-0.12, 0.12), title="Footsteps, v = 0.2 m/s (top view)")
    top.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3, fontsize=8)
    top.grid(alpha=0.3)
    bottom.plot([s["ux"] for s in result["log"]], "-o", label="step length $u_x$ [m]")
    bottom.axhline(0.2 * p.T, ls="--", color="gray", label="$v_{des}\\,T$")
    bottom.plot([abs(s["uy"]) for s in result["log"]], "-o", label="step width $|u_y|$ [m]")
    bottom.axhline(p.w, ls="--", color="gray")
    bottom.set(xlabel="step", title="Planned landing points")
    bottom.legend()
    bottom.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return {
        "final_speed": float((result["com"][-1, 0] - result["com"][-5, 0]) / (4 * p.T)),
        "max_lateral_com": float(np.abs(result["com"][:, 1]).max()),
    }


def plot_lateral_push(p: Params, path: Path) -> None:
    fig, (left, right) = plt.subplots(1, 2, figsize=(11, 4))
    for impulse, color in ((0.0, "k"), (0.5, "tab:blue"), (1.0, "tab:orange")):
        push = {"step": 6, "t": 0.15, "Jx": 0.0, "Jy": impulse}
        result = simulate(p, v_des=0.2, n_steps=18, push=push)
        left.plot(result["com"][:, 0], result["com"][:, 1], "-o", ms=3, color=color, label=f"$J_y$ = {impulse} N s")
        right.plot([s["uy"] for s in result["log"]], "-o", ms=3, color=color)
    left.set(xlabel="x [m]", ylabel="y [m]", ylim=(-0.05, 0.3), title="CoM path, lateral push at step 6")
    left.text(0.05, 0.22, "$J_y$ = 1.0 N s: CoM diverges\n(outside the plotted range)", color="tab:orange")
    left.legend()
    left.grid(alpha=0.3)
    right.set(xlabel="step", ylabel="lateral landing point $u_y$ [m]", title="Lateral landing points")
    right.xaxis.set_major_locator(MaxNLocator(integer=True))
    right.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_servo_feasibility(rows: list[dict], path: Path) -> None:
    valid = [r for r in rows if r["peak_joint_speed"] is not None]
    fig, ax = plt.subplots(figsize=(5.5, 4))
    ax.plot([r["v"] for r in valid], [r["peak_joint_speed"] for r in valid], "-o")
    ax.axhline(SERVO_LIMIT, color="r", ls="--", label=f"servo limit {SERVO_LIMIT} rad/s")
    ax.set(xlabel="forward speed [m/s]", ylabel="estimated peak hip joint speed [rad/s]", title="Servo feasibility")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def run(out: Path) -> dict:
    """Run all experiments, write figures and results.json to `out`, and return the results."""
    out.mkdir(parents=True, exist_ok=True)
    p = Params()
    names = ("m", "H", "T", "T_swing", "t_replan", "w", "u_max_x", "u_max_y_out", "w_min", "l_leg")
    results: dict = {"assumptions": {name: getattr(p, name) for name in names}}
    results["selfcheck"] = self_check(p)
    results["nominal_0.2"] = plot_nominal_walk(p, out / "fig1_nominal_walk.png")
    plot_lateral_push(p, out / "fig2_lateral_push.png")

    table = {}
    for name, reach in (("narrow reach 0.15 m", 0.15), ("medium reach 0.25 m", 0.25), ("wide reach 0.35 m", 0.35)):
        q = Params(u_max_y_out=reach, u_max_x=min(reach, 0.30))
        table[name] = {d: round(max_recoverable_impulse(q, d), 2) for d in ("x+", "x-", "y+", "y-")}
    results["max_push_Ns_by_reach_assumption"] = table
    open_loop = simulate(Params(adapt=False), v_des=0.2, n_steps=14)
    results["open_loop_footsteps_no_push"] = {"failed": open_loop["failed"], "reason": open_loop["reason"]}

    rows = []
    for v in (0.1, 0.2, 0.3, 0.4, 0.5):
        r = simulate(p, v_des=v, n_steps=14)
        rows.append(
            {
                "v": v,
                "step_length": round(float(np.mean([s["ux"] for s in r["log"][-6:]])), 3),
                "peak_joint_speed": None if _diverged(r) else round(max(s["peak_joint"] for s in r["log"][-6:]), 2),
                "failed": r["failed"],
                "reason": r["reason"],
            }
        )
    results["speed_sweep"] = rows
    plot_servo_feasibility(rows, out / "fig3_servo_feasibility.png")

    timing = []
    for period in (0.30, 0.35, 0.40, 0.45, 0.50, 0.60):
        row = {"T": period, "T_swing": round(period - 0.1, 2)}
        for v in (0.3, 0.4):
            r = simulate(Params(T=period, T_swing=period - 0.1, t_replan=period / 2), v_des=v, n_steps=16)
            peak = max(s["peak_joint"] for s in r["log"][-6:])
            row[f"peak_joint_rad_s_at_{v}"] = None if _diverged(r) else round(peak, 2)
            row[f"note_at_{v}"] = "needs steps beyond the reach limit" if _diverged(r) else ""
        timing.append(row)
    results["timing_sweep"] = timing

    results["heading_bias_lateral_error_cm_per_5m"] = {
        f"{b} deg": round(100 * 5 * math.tan(math.radians(b)), 1) for b in (0.5, 1, 2, 3, 5)
    }
    (out / "results.json").write_text(json.dumps(results, indent=2))
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=Path("."), help="output directory for figures and results.json")
    args = parser.parse_args()
    print(json.dumps(run(args.out), indent=2))


if __name__ == "__main__":
    main()
