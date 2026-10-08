"""Animate a run recorded by scripts/record_truth.py: simulator base path against the estimated base path.

Left: top view of the base displacement since the anchor (mm), simulator in black, estimate in red. Centre: horizontal
estimation error. Right: contact of the left and right foot (the modes the estimator used; filled while not in the
air). The estimate is the online estimate of the recorded run, so the script needs only numpy and matplotlib.

Usage:
    uv run python scripts/animate_run.py runs/seed0.npz --out seed0.gif
    uv run python scripts/animate_run.py runs/seed0.npz --out seed0.mp4 --speed 1
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("record", help=".npz written by scripts/record_truth.py")
    parser.add_argument("--out", required=True, help="output .gif or .mp4")
    parser.add_argument("--anchor", type=float, default=0.5, help="[s] time at which the displacements are zero")
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed relative to simulated time")
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()

    d = np.load(args.record)
    dt = float(d["step_dt"])
    a = round(args.anchor / dt)
    truth = 1e3 * (d["truth_pos"][a:] - d["truth_pos"][a])
    est = 1e3 * (d["ekf_pos"][a:] - d["ekf_pos"][a])
    t = np.arange(len(truth)) * dt + args.anchor
    err = np.linalg.norm((est - truth)[:, :2], axis=1)
    contact = d["modes"][a:] != "air"
    gait = float(np.argmax(~np.isnan(d["plan_time"]))) * dt
    frames = np.arange(0, len(t), max(1, round(args.speed / (args.fps * dt))))

    fig, (ax_p, ax_e, ax_c) = plt.subplots(1, 3, figsize=(14, 4.2), gridspec_kw={"width_ratios": [1.1, 1, 1]})
    pad = 2.0
    xs, ys = np.r_[truth[:, 0], est[:, 0]], np.r_[truth[:, 1], est[:, 1]]
    span = max(np.ptp(xs), np.ptp(ys)) / 2 + pad
    ax_p.set_xlim(xs.mean() - span, xs.mean() + span)
    ax_p.set_ylim(ys.mean() - span, ys.mean() + span)
    ax_p.set_aspect("equal")
    ax_p.set_xlabel("x [mm]")
    ax_p.set_ylabel("y [mm]")
    ax_p.set_title("Base displacement, top view")
    (path_t,) = ax_p.plot([], [], "k-", lw=1.2, label="simulator")
    (path_e,) = ax_p.plot([], [], "r-", lw=1.2, label="estimate")
    (dot_t,) = ax_p.plot([], [], "ko")
    (dot_e,) = ax_p.plot([], [], "ro")
    ax_p.legend(loc="upper left")

    ax_e.plot(t, err, color="0.8")
    ax_e.set_xlim(t[0], t[-1])
    ax_e.set_ylim(0, max(1.0, 1.1 * err.max()))
    ax_e.axvline(gait, color="0.5", ls="--", lw=0.8)
    ax_e.set_xlabel("time [s]")
    ax_e.set_ylabel("horizontal error [mm]")
    ax_e.set_title("Estimation error (dashed: gait starts)")
    (line_e,) = ax_e.plot([], [], "r-")

    for row in (0, 1):
        ax_c.fill_between(t, row, row + 0.8, where=contact[:, row], color="0.85", step="post")
    ax_c.set_xlim(t[0], t[-1])
    ax_c.set_yticks([0.4, 1.4], ["left foot", "right foot"])
    ax_c.set_xlabel("time [s]")
    ax_c.set_title("Contact (filled: on the ground)")
    ax_c.axvline(gait, color="0.5", ls="--", lw=0.8)
    cursor_c = ax_c.axvline(t[0], color="r")
    cursor_e = ax_e.axvline(t[0], color="r")
    fig.tight_layout()

    def draw(k: int):
        path_t.set_data(truth[: k + 1, 0], truth[: k + 1, 1])
        path_e.set_data(est[: k + 1, 0], est[: k + 1, 1])
        dot_t.set_data([truth[k, 0]], [truth[k, 1]])
        dot_e.set_data([est[k, 0]], [est[k, 1]])
        line_e.set_data(t[: k + 1], err[: k + 1])
        cursor_c.set_xdata([t[k], t[k]])
        cursor_e.set_xdata([t[k], t[k]])
        return path_t, path_e, dot_t, dot_e, line_e, cursor_c, cursor_e

    anim = FuncAnimation(fig, draw, frames=frames, blit=False)
    writer = FFMpegWriter(fps=args.fps) if args.out.endswith(".mp4") else PillowWriter(fps=args.fps)
    anim.save(args.out, writer=writer, dpi=100)
    print(f"{len(frames)} frames, {len(frames) / args.fps:.1f} s -> {args.out}")


if __name__ == "__main__":
    main()
