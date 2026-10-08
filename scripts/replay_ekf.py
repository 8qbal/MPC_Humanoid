"""Replay the base-position estimator on runs recorded by scripts/record_truth.py and score its drift.

The leg-kinematics + IMU EKF (mpc/ekf.py, Bloesch et al., RSS 2012) only observes: its output does not change the
recorded encoder, AHRS, gyro and accelerometer inputs, so replaying them with other noise parameters is valid open
loop and needs no Isaac. The error is the estimated base displacement minus the simulator displacement since the
anchor time, so it contains the heading error of the AHRS but not the (unobservable) start position. The anchor
(default 0.5 s, before the crouch) skips the first ticks, where the robot settles on the ground in the simulator
while the estimator starts from the contact height.

Needs numpy, pinocchio, casadi and matplotlib, no Isaac Sim / Isaac Lab.

Usage:
    uv run python scripts/replay_ekf.py runs/seed0.npz runs/seed1.npz runs/seed2.npz --plot drift.png
    uv run python scripts/replay_ekf.py runs/*.npz --set accel_density=0.01 --set slip_density=1e-5
    uv run python scripts/replay_ekf.py runs/*.npz --sweep accel_density 0.001 0.01 0.1 1
"""

import argparse
import inspect
from pathlib import Path

import numpy as np

from MPC_Humanoid.mpc.ekf import LegKinematicsEkf


def parse_value(text: str) -> float | bool:
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    return float(text)


def replay(rec: dict[str, np.ndarray], **params: float | bool) -> np.ndarray:
    ekf = LegKinematicsEkf(list(rec["act_names"]), **params)
    dt = float(rec["step_dt"])
    inputs = zip(rec["q_act"], rec["qd_act"], rec["accel"], rec["gyro"], rec["quat"], rec["modes"])
    return np.array([ekf.update(q, qd, a, w, quat, list(m), dt)[0] for q, qd, a, w, quat, m in inputs])


def drift(estimate: np.ndarray, truth: np.ndarray, start: int) -> np.ndarray:
    """Error of the estimated displacement against the true displacement since tick `start`, per tick [m]; NaN
    before `start`."""
    err = (estimate - estimate[start]) - (truth - truth[start])
    err[:start] = np.nan
    return err


def summarize(rec: dict[str, np.ndarray], err: np.ndarray, start: int) -> dict[str, float]:
    dt = float(rec["step_dt"])
    horizontal = np.linalg.norm(err[:, :2], axis=1)
    gait = int(np.argmax(~np.isnan(rec["plan_time"])))
    steps = slice(gait, None)
    slope = np.polyfit(np.arange(len(horizontal))[steps] * dt, horizontal[steps], 1)[0]
    return {
        "duration_s": len(err) * dt,
        "gait_start_s": gait * dt,
        "err_x_mm": 1e3 * err[-1, 0],
        "err_y_mm": 1e3 * err[-1, 1],
        "err_z_mm": 1e3 * err[-1, 2],
        "err_xy_start_gait_mm": 1e3 * horizontal[gait],
        "err_xy_end_mm": 1e3 * horizontal[-1],
        "err_xy_max_mm": 1e3 * np.nanmax(horizontal),
        "growth_mm_per_s": 1e3 * slope,
        "true_travel_mm": 1e3 * np.linalg.norm(rec["truth_pos"][-1, :2] - rec["truth_pos"][start, :2]),
    }


def plot(path: str, names: list[str], recs: list[dict[str, np.ndarray]], errs: list[np.ndarray]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    for name, rec, err in zip(names, recs, errs):
        t = np.arange(len(err)) * float(rec["step_dt"])
        axes[0].plot(t, 1e3 * np.linalg.norm(err[:, :2], axis=1), label=name)
        axes[1].plot(t, 1e3 * err[:, 2], label=name)
        axes[2].plot(1e3 * err[:, 0], 1e3 * err[:, 1], label=name)
        axes[2].plot(1e3 * err[-1, 0], 1e3 * err[-1, 1], "k.")
    gait = int(np.argmax(~np.isnan(recs[0]["plan_time"]))) * float(recs[0]["step_dt"])
    for ax in axes[:2]:
        ax.axvline(gait, color="0.5", ls="--", lw=0.8)
        ax.set_xlabel("time [s]")
    axes[0].set_ylabel("horizontal error [mm]")
    axes[0].set_title("Horizontal drift (dashed: gait starts)")
    axes[1].set_ylabel("z error [mm]")
    axes[1].set_title("Vertical error")
    axes[2].set_xlabel("x error [mm]")
    axes[2].set_ylabel("y error [mm]")
    axes[2].set_title("Direction of the drift (dot: end of run)")
    axes[2].axis("equal")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def report(names: list[str], stats: list[dict[str, float]]) -> None:
    header = f"{'run':<24}{'x':>8}{'y':>8}{'z':>8}{'xy@gait':>9}{'xy end':>8}{'xy max':>8}{'mm/s':>7}{'travel':>8}"
    print(
        "error in mm at the end of the run: displacement of the estimate minus the true displacement since the anchor"
    )
    print(header)
    for name, s in zip(names, stats):
        print(
            f"{name:<24}{s['err_x_mm']:>8.1f}{s['err_y_mm']:>8.1f}{s['err_z_mm']:>8.1f}"
            f"{s['err_xy_start_gait_mm']:>9.1f}{s['err_xy_end_mm']:>8.1f}{s['err_xy_max_mm']:>8.1f}"
            f"{s['growth_mm_per_s']:>7.2f}{s['true_travel_mm']:>8.1f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("records", nargs="+", help=".npz files written by scripts/record_truth.py")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="LegKinematicsEkf argument")
    parser.add_argument("--sweep", nargs="+", metavar=("NAME", "VALUE"), help="one argument over several values")
    parser.add_argument("--anchor", type=float, default=0.5, help="[s] time at which the error is set to zero")
    parser.add_argument("--plot", default=None, help="save the drift figure to this .png")
    args = parser.parse_args()

    valid = set(inspect.signature(LegKinematicsEkf.__init__).parameters) - {"self", "act_names"}
    params = dict(item.split("=", 1) for item in args.set)
    unknown = (set(params) | set(args.sweep[:1] if args.sweep else [])) - valid
    if unknown:
        parser.error(f"unknown parameter {sorted(unknown)}; valid: {sorted(valid)}")
    params = {k: parse_value(v) for k, v in params.items()}

    recs = [dict(np.load(p)) for p in args.records]
    start = round(args.anchor / float(recs[0]["step_dt"]))
    names = [Path(p).stem for p in args.records]
    for name, rec in zip(names, recs):
        if bool(rec["fell"]):
            print(f"warning: {name} ends in a fall")

    if args.sweep:
        key, values = args.sweep[0], [parse_value(v) for v in args.sweep[1:]]
        print(f"{key:>14}{'xy end [mm]':>14}{'xy max [mm]':>14}   (mean over {len(recs)} run(s))")
        for value in values:
            stats = [
                summarize(r, drift(replay(r, **{**params, key: value}), r["truth_pos"], start), start) for r in recs
            ]
            end = np.mean([s["err_xy_end_mm"] for s in stats])
            peak = np.mean([s["err_xy_max_mm"] for s in stats])
            print(f"{value:>14g}{end:>14.1f}{peak:>14.1f}")
        return

    estimates = [replay(r, **params) for r in recs]
    errs = [drift(e, r["truth_pos"], start) for e, r in zip(estimates, recs)]
    report(names, [summarize(r, e, start) for r, e in zip(recs, errs)])

    online = [np.abs(e - r["ekf_pos"]).max() for e, r in zip(estimates, recs)]
    if not params:
        print(f"replay vs online estimate of the recorded run: max difference {1e3 * max(online):.3f} mm")
    if args.plot:
        plot(args.plot, names, recs, errs)
        print(f"figure -> {args.plot}")


if __name__ == "__main__":
    main()
