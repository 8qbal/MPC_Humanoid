"""
Offline ALIP footstep prototype for Robonion (no Isaac, numpy + matplotlib only).

Model: Gong & Grizzle, "Angular Momentum about the Contact Point for Control of Bipedal Locomotion:
Validation in a LIP-based Controller", arXiv:2008.10763, Eq. (7), (8), (14), (15).

Per axis the state is (position of the CoM relative to the stance foot, angular momentum about the stance foot):
  sagittal  (x, Ly):   x_dot =  Ly/(mH),  Ly_dot =  m g x        (paper Eq. 7)
  lateral   (y, L'):   y_dot =  L'/(mH),  L'_dot  =  m g y        with L' = -Lx  (derived, see README)
Both axes therefore share one linear system  d/dt [pos, L] = [[0, 1/(mH)], [m g, 0]] [pos, L].

Angular momentum about the contact point does not jump when the stance foot changes (CoM at constant height),
so at touchdown only the position is re-referenced:  pos_new = pos_end - u,  L_new = L_end,
u = landing position of the swing foot relative to the old stance foot.

Foot placement (paper Eq. 15) with the predicted end-of-step momentum L_hat and the desired momentum at the end of
the NEXT step L_des:
  pos_new = (L_des - cosh(lT) L_hat) / (m H l sinh(lT)),   l = sqrt(g/H)

Steady-state L_des for a periodic gait (derived here, checked numerically in selfcheck()):
  sagittal, step length d = v T:       Ly_des = m H l coth(lT/2) d / 2
  lateral, step width w, stance side s (+1 left, -1 right):  L'_des = s * m H l tanh(lT/2) w / 2

ASSUMPTIONS (not from the paper or the robot; replace with measured values):
  - one inverted-pendulum period T = single support + double support = 0.4 s (Robonion gait: 0.3 + 0.1), the
    double support is folded into the stance period (instant foot switch)
  - CoM height H = 0.48 m (the value hard-coded in Iqbal's nmpc.py), mass 7.933 kg (sum of the link masses in
    assets/robonionv2_controller.urdf)
  - reach limits U_MAX_X, U_MAX_Y_OUT, W_MIN, leg length L_LEG: rough numbers from the 0.2 m + 0.2 m parallelogram
    legs and the 0.11 m hip spacing in references/docs/joint_info.md
  - swing foot on a minimum-jerk path in T_swing; joint speed = (foot speed relative to the moving CoM) / leg length,
    which ignores knee flexion, foot lift and the passive parallelogram coupling: an order-of-magnitude check
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

G = 9.81
SERVO_LIMIT = 4.08  # rad/s, front-thigh / hip-roll servos (references/docs/joint_info.md)


@dataclass
class P:
    m: float = 7.933
    H: float = 0.48
    T: float = 0.40  # period between touchdowns [s]
    T_swing: float = 0.30  # time the swing foot is in the air [s]
    t_replan: float = 0.20  # re-plan the landing point this long after touchdown [s]
    w: float = 0.11  # nominal lateral distance between the feet [m]
    u_max_x: float = 0.20  # max sagittal landing distance from the stance foot [m]
    u_max_y_out: float = 0.25  # max lateral landing distance [m]
    w_min: float = 0.07  # min lateral foot-to-foot distance (sole width 0.067 m) [m]
    l_leg: float = 0.38  # effective hip-to-sole length used to turn foot speed into joint speed [m]
    adapt: bool = True  # False: always land on the nominal footprint (no ALIP feedback)
    limits: bool = True
    ell: float = field(init=False)

    def __post_init__(self):
        self.ell = math.sqrt(G / self.H)


def A(p: P, t: float) -> np.ndarray:
    """State transition of [pos, L] over time t (paper Eq. 8)."""
    c, s = math.cosh(p.ell * t), math.sinh(p.ell * t)
    k = p.m * p.H * p.ell
    return np.array([[c, s / k], [k * s, c]])


def L_des_sagittal(p: P, v: float) -> float:
    return p.m * p.H * p.ell / math.tanh(p.ell * p.T / 2) * v * p.T / 2


def L_des_lateral(p: P, side: int) -> float:
    """Desired L' at the end of a step whose stance foot is `side` (+1 left, -1 right)."""
    return -side * p.m * p.H * p.ell * math.tanh(p.ell * p.T / 2) * p.w / 2


def place(p: P, state_t: np.ndarray, t: float, L_des: float) -> float:
    """CoM position relative to the NEW stance foot that gives L_des at the end of the next step (Eq. 14 + 15)."""
    rest = p.T - t
    pos_end, L_end = A(p, rest) @ state_t
    k = p.m * p.H * p.ell
    return (L_des - math.cosh(p.ell * p.T) * L_end) / (k * math.sinh(p.ell * p.T)), pos_end


def landing_point(p: P, sag: np.ndarray, lat: np.ndarray, t: float, side: int, v_des: float) -> tuple[float, float]:
    """THE function to reuse in the real controller.

    sag = (x, Ly), lat = (y, L') measured t seconds after touchdown, expressed relative to the stance foot (x forward,
    y left, L' = -Lx); side = +1 left / -1 right stance foot; v_des = commanded forward speed.
    Returns the landing point (ux, uy) of the swing foot relative to the stance foot, clipped to the reach limits."""
    x_new, x_end = place(p, np.asarray(sag, float), t, L_des_sagittal(p, v_des))
    y_new, y_end = place(p, np.asarray(lat, float), t, L_des_lateral(p, -side))
    ux, uy = x_end - x_new, y_end - y_new
    if not p.adapt:
        ux, uy = v_des * p.T, -side * p.w
    if p.limits:
        ux = float(np.clip(ux, -p.u_max_x, p.u_max_x))
        uy = float(np.clip(-side * uy, p.w_min, p.u_max_y_out)) * (-side)
    return float(ux), float(uy)


def simulate(
    p: P,
    v_des: float = 0.2,
    n_steps: int = 14,
    push: dict | None = None,
    ramp_steps: int = 3,
    heading_bias_deg: float = 0.0,
):
    """Walk n_steps. push = {step, t, Jx, Jy} impulse [N s] at time t of that step (stance foot frame).

    Returns a dict with the CoM / foot paths in a world frame (x forward, y left, left stance first) and the
    per-step diagnostics. `failed` is set when the CoM leaves +-0.6 m of the stance foot or a servo limit is hit."""
    side = +1  # stance foot: +1 left, -1 right
    sag = np.array([0.0, 0.0])
    # lateral sway already running: start on the periodic orbit (a standstill needs a weight-shift phase that
    # this model does not have); sagittal axis starts at rest and ramps up to v_des
    lat = np.array([-p.w / 2, p.m * p.H * p.ell * math.tanh(p.ell * p.T / 2) * p.w / 2])
    foot = np.array([0.0, p.w / 2])  # world position of the stance foot, left foot first
    prev_u = np.array([0.0, side * p.w])  # where the previous foot landed relative to the previous stance foot
    com_t, com_xy, feet_xy, log = [], [], [], []
    t_abs = 0.0
    failed, reason = False, ""
    for k in range(n_steps):
        v = v_des * min(1.0, (k + 1) / ramp_steps)
        t_push = push["t"] if push and push["step"] == k else None
        # integrate until the replanning time (with the push in between, if any)
        segs = [(0.0, p.t_replan)]
        if t_push is not None and t_push < p.t_replan:
            segs = [(0.0, t_push), (t_push, p.t_replan)]
        t_now = 0.0
        for a, b in segs:
            sag, lat = A(p, b - t_now) @ sag, A(p, b - t_now) @ lat
            t_now = b
            if t_push is not None and abs(b - t_push) < 1e-12 and a == 0.0 and len(segs) == 2:
                sag[1] += p.H * push["Jx"]
                lat[1] += p.H * push["Jy"]  # L' = -Lx, Delta Lx = -H Jy
        # re-plan with the state at t_replan
        v_com = np.array([sag[1], lat[1]]) / (p.m * p.H)
        ux, uy = landing_point(p, sag, lat, p.t_replan, side, v)
        # finish the step (a push after the replanning time is felt, but the target is already fixed)
        if t_push is not None and t_push >= p.t_replan:
            sag, lat = A(p, t_push - p.t_replan) @ sag, A(p, t_push - p.t_replan) @ lat
            sag[1] += p.H * push["Jx"]
            lat[1] += p.H * push["Jy"]
            sag, lat = A(p, p.T - t_push) @ sag, A(p, p.T - t_push) @ lat
        else:
            sag, lat = A(p, p.T - p.t_replan) @ sag, A(p, p.T - p.t_replan) @ lat
        # swing foot travel and the servo check: the foot moves from where the previous stance foot is (-prev_u)
        # to the landing point u, on a minimum-jerk path in T_swing; the hip moves with the CoM, so what the leg
        # joints see is the foot speed relative to the CoM (hip pitch for x, hip roll for y)
        travel = np.array([ux, uy]) + prev_u
        sgrid = np.linspace(0.0, 1.0, 61)
        prof = 30 * sgrid**2 * (1 - sgrid) ** 2
        rel = travel[None, :] / p.T_swing * prof[:, None] - v_com[None, :]
        peak_joint = float(np.abs(rel).max() / p.l_leg)
        # world-frame bookkeeping (optional heading bias rotates the commanded forward direction)
        com_world = foot + np.array([sag[0], lat[0]])
        com_t.append(t_abs + p.T)
        com_xy.append(com_world.copy())
        feet_xy.append(foot.copy())
        log.append(
            dict(step=k, side=side, ux=ux, uy=uy, x_end=float(sag[0]), y_end=float(lat[0]), L_end=float(sag[1]),
                 Lp_end=float(lat[1]), peak_joint=float(peak_joint))
        )
        if peak_joint > SERVO_LIMIT and p.limits:
            failed, reason = True, f"servo limit {peak_joint:.2f} rad/s at step {k}"
        # touchdown: switch stance foot
        foot = foot + np.array([ux, uy])
        sag = np.array([sag[0] - ux, sag[1]])
        lat = np.array([lat[0] - uy, lat[1]])
        prev_u = np.array([ux, uy])
        side = -side
        t_abs += p.T
        if abs(sag[0]) > 0.6 or abs(lat[0]) > 0.6:
            failed, reason = True, f"CoM {sag[0]:+.2f},{lat[0]:+.2f} m from the stance foot at step {k}"
            break
    return dict(t=np.array(com_t), com=np.array(com_xy), feet=np.array(feet_xy), log=log, failed=failed, reason=reason)


def selfcheck(p: P) -> dict:
    """Closed-form periodic orbit vs. simulation: steady-state x0 = -d/2, y0 = -w/2 and the formulas above."""
    v = 0.3
    r = simulate(p, v_des=v, n_steps=40, ramp_steps=1)
    d_steady = np.mean([s["ux"] for s in r["log"][-10:]])
    w_steady = np.mean([abs(s["uy"]) for s in r["log"][-10:]])
    speed = (r["com"][-1, 0] - r["com"][-11, 0]) / (10 * p.T)
    # linearization check of the ODE against the paper's matrix exponential
    M = np.array([[0, 1 / (p.m * p.H)], [p.m * G, 0]])
    w_, V = np.linalg.eig(M)
    Aexp = (V @ np.diag(np.exp(w_ * 0.17)) @ np.linalg.inv(V)).real
    err = float(np.abs(Aexp - A(p, 0.17)).max())
    return dict(step_length=float(d_steady), expected_step_length=v * p.T, step_width=float(w_steady),
                expected_step_width=p.w, mean_speed=float(speed), expected_speed=v, matrix_exp_error=err)


def max_push(p: P, direction: str, side_step: int = 6, t_push: float = 0.15, v_des: float = 0.2) -> float:
    """Largest impulse [N s] (bisection) after which the 14-step walk still succeeds. Push at time t_push of
    step `side_step` (even steps: left stance)."""
    def ok(J):
        push = dict(step=side_step, t=t_push, Jx=0.0, Jy=0.0)
        push[{"x+": "Jx", "x-": "Jx", "y+": "Jy", "y-": "Jy"}[direction]] = J if direction[1] == "+" else -J
        return not simulate(p, v_des=v_des, n_steps=side_step + 12, push=push)["failed"]

    lo, hi = 0.0, 20.0
    if not ok(0.0):
        return 0.0
    if ok(hi):
        return hi
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def main(out: Path):
    out.mkdir(parents=True, exist_ok=True)
    p = P()
    res: dict = {"assumptions": {k: getattr(p, k) for k in ("m", "H", "T", "T_swing", "t_replan", "w", "u_max_x",
                                                            "u_max_y_out", "w_min", "l_leg")}}
    res["selfcheck"] = selfcheck(p)

    # ---- Figure 1: nominal walk, 0.2 m/s
    r = simulate(p, v_des=0.2, n_steps=14)
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(r["com"][:, 0], r["com"][:, 1], "-o", ms=3, label="CoM at touchdown")
    for i, f in enumerate(r["feet"]):
        ax[0].plot(f[0], f[1], "s", color="tab:red" if i % 2 == 0 else "tab:green", ms=7)
    ax[0].plot([], [], "s", color="tab:red", label="left stance foot")
    ax[0].plot([], [], "s", color="tab:green", label="right stance foot")
    ax[0].set_xlabel("x forward [m]"); ax[0].set_ylabel("y left [m]"); ax[0].set_ylim(-0.12, 0.12)
    ax[0].legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3, fontsize=8)
    ax[0].set_title("ALIP footsteps, v_des = 0.2 m/s (top view)"); ax[0].grid(alpha=0.3)
    ax[1].plot([s["ux"] for s in r["log"]], "-o", label="step length u_x [m]")
    ax[1].axhline(0.2 * p.T, ls="--", color="gray", label="v_des * T")
    ax[1].plot([abs(s["uy"]) for s in r["log"]], "-o", label="|step width| [m]")
    ax[1].axhline(p.w, ls="--", color="gray")
    ax[1].set_xlabel("step"); ax[1].legend(); ax[1].grid(alpha=0.3); ax[1].set_title("Planned landing points")
    fig.tight_layout(); fig.savefig(out / "fig1_nominal_walk.png", dpi=140); plt.close(fig)
    res["nominal_0.2"] = dict(final_speed=float((r["com"][-1, 0] - r["com"][-5, 0]) / (4 * p.T)),
                              max_lateral_com=float(np.abs(r["com"][:, 1]).max()))

    # ---- Figure 2: lateral push recovery
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for J, c in ((0.0, "k"), (0.5, "tab:blue"), (1.0, "tab:orange")):
        rr = simulate(p, v_des=0.2, n_steps=18, push=dict(step=6, t=0.15, Jx=0.0, Jy=J))
        ax[0].plot(rr["com"][:, 0], rr["com"][:, 1], "-o", ms=3, color=c, label=f"lateral push {J} N s (+y)")
        ax[1].plot([s["uy"] for s in rr["log"]], "-o", ms=3, color=c)
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].legend(); ax[0].grid(alpha=0.3)
    ax[0].set_ylim(-0.05, 0.3); ax[0].text(0.05, 0.22, "1.0 N s: leaves the plot\n(the model falls)", color="tab:orange")
    ax[0].set_title("CoM path, push at step 6 (left stance)")
    ax[1].set_xlabel("step"); ax[1].set_ylabel("lateral landing u_y [m]"); ax[1].grid(alpha=0.3)
    ax[1].set_title("The planner moves the foot to catch the push")
    fig.tight_layout(); fig.savefig(out / "fig2_lateral_push.png", dpi=140); plt.close(fig)

    # ---- Max recoverable impulse, three reach assumptions (N s)
    table = {}
    for name, umax in (("narrow reach 0.15 m", 0.15), ("medium reach 0.25 m", 0.25), ("wide reach 0.35 m", 0.35)):
        q = P(u_max_y_out=umax, u_max_x=min(umax, 0.30))
        table[name] = {d: round(max_push(q, d), 2) for d in ("x+", "x-", "y+", "y-")}
    res["max_push_Ns_by_reach_assumption"] = table
    ol = simulate(P(adapt=False), v_des=0.2, n_steps=14)
    res["open_loop_footsteps_no_push"] = dict(failed=ol["failed"], reason=ol["reason"])

    # ---- Speed sweep and servo feasibility
    rows = []
    for v in (0.1, 0.2, 0.3, 0.4, 0.5):
        rr = simulate(p, v_des=v, n_steps=14)
        div = rr["failed"] and "CoM" in rr["reason"]
        rows.append(dict(v=v, step_length=round(float(np.mean([s["ux"] for s in rr["log"][-6:]])), 3),
                         peak_joint_speed=None if div else round(max(s["peak_joint"] for s in rr["log"][-6:]), 2),
                         failed=rr["failed"], reason=rr["reason"]))
    res["speed_sweep"] = rows
    fig, ax = plt.subplots(figsize=(5.5, 4))
    ok_rows = [r_ for r_ in rows if r_["peak_joint_speed"] is not None]
    ax.plot([r_["v"] for r_ in ok_rows], [r_["peak_joint_speed"] for r_ in ok_rows], "-o")
    ax.axhline(SERVO_LIMIT, color="r", ls="--", label=f"servo limit {SERVO_LIMIT} rad/s")
    ax.set_xlabel("forward speed [m/s]"); ax.set_ylabel("est. peak swing-leg joint speed [rad/s]")
    ax.legend(); ax.grid(alpha=0.3); ax.set_title("Servo feasibility (rough estimate)")
    fig.tight_layout(); fig.savefig(out / "fig3_servo_feasibility.png", dpi=140); plt.close(fig)

    # ---- Which gait timing reaches 0.3-0.4 m/s inside the servo limit? (double support fixed at 0.1 s)
    timing = []
    for T in (0.30, 0.35, 0.40, 0.45, 0.50, 0.60):
        row = dict(T=T, T_swing=round(T - 0.1, 2))
        for v in (0.3, 0.4):
            rr = simulate(P(T=T, T_swing=T - 0.1, t_replan=T / 2), v_des=v, n_steps=16, ramp_steps=3)
            diverged = rr["failed"] and "CoM" in rr["reason"]
            row[f"peak_joint_rad_s_at_{v}"] = None if diverged else round(max(x["peak_joint"] for x in rr["log"][-6:]), 2)
            row[f"note_at_{v}"] = "needs steps beyond the reach limit" if diverged else ""
        timing.append(row)
    res["timing_sweep"] = timing

    # ---- Heading bias -> lateral error (geometry, 5 m walk)
    res["heading_bias_lateral_error_cm_per_5m"] = {f"{b} deg": round(100 * 5 * math.tan(math.radians(b)), 1)
                                                   for b in (0.5, 1, 2, 3, 5)}
    (out / "results.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else Path("out"))
