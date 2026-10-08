"""Replay a run recorded by scripts/record_truth.py as a stick figure of the robot (side and front view).

Each frame draws the robot from the recorded Isaac Lab simulation: the measured joint angles of the actuated joints
(passive parallelogram joints from PASSIVE_COUPLING), the simulator base position and the AHRS base orientation,
through the controller URDF. Soles are green while the estimator treats the foot as in contact, grey in the air.
The dashed line marks the start position of the base, the trail its path. It is a kinematic replay, not a render of
the simulator, and needs no Isaac Sim / Isaac Lab, only numpy, pinocchio and matplotlib.

Usage:
    uv run python scripts/animate_robot.py runs/seed0.npz --out robot_seed0.gif
    uv run python scripts/animate_robot.py runs/stand_seed0.npz --out robot_stand.mp4 --speed 2
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

from MPC_Humanoid.mpc.model import CONTROLLER_URDF, PASSIVE_COUPLING, SOLE_CORNERS

VIEWS = ((0, "side view (x forward)", "x [m]"), (1, "front view (y left)", "y [m]"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("record", help=".npz written by scripts/record_truth.py")
    parser.add_argument("--out", required=True, help="output .gif or .mp4")
    parser.add_argument("--anchor", type=float, default=0.5, help="[s] time at which the base displacement is zero")
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed relative to simulated time")
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()

    d = np.load(args.record)
    act_names = list(d["act_names"])
    dt = float(d["step_dt"])
    anchor = round(args.anchor / dt)
    model = pin.buildModelFromUrdf(CONTROLLER_URDF, pin.JointModelFreeFlyer())
    data = model.createData()
    idx_q = np.array([model.joints[model.getJointId(n)].idx_q for n in act_names])
    passive = [
        (model.joints[model.getJointId(f"{side}_{name}")].idx_q, act_names.index(f"{side}_{active}"), gain)
        for side in ("left", "right")
        for name, (active, gain) in PASSIVE_COUPLING.items()
    ]
    imu = model.getFrameId("imu_link")
    feet = [model.getFrameId(f"{side}_foot_roll_link") for side in ("left", "right")]
    order = (0, 1, 3, 2)

    def pose(k: int) -> tuple[np.ndarray, list[np.ndarray]]:
        q = pin.neutral(model)
        q[idx_q] = d["q_act"][k]
        for i, src, gain in passive:
            q[i] = gain * d["q_act"][k][src]
        x, y, z, w = d["quat"][k] / np.linalg.norm(d["quat"][k])
        pin.framesForwardKinematics(model, data, q)
        R_base = pin.Quaternion(w, x, y, z).matrix() @ data.oMf[imu].rotation.T
        q[:3], q[3:7] = d["truth_pos"][k], pin.Quaternion(R_base).coeffs()
        pin.framesForwardKinematics(model, data, q)
        joints = np.array([data.oMi[j].translation for j in range(model.njoints)])
        soles = [(data.oMf[f].rotation @ SOLE_CORNERS.T).T + data.oMf[f].translation for f in feet]
        return joints, soles

    frames = np.arange(0, len(d["q_act"]), max(1, round(args.speed / (args.fps * dt))))
    start = d["truth_pos"][anchor]
    centre = d["truth_pos"][:, :2].mean(0)
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    parents = [model.parents[j] for j in range(model.njoints)]
    artists = []
    for ax, (axis, title, label) in zip(axes, VIEWS):
        ax.set_xlim(centre[axis] - 0.45, centre[axis] + 0.45)
        ax.set_ylim(-0.05, 0.75)
        ax.set_aspect("equal")
        ax.axhline(0, color="k", lw=1)
        ax.axvline(start[axis], color="0.5", ls="--", lw=0.8)
        ax.set_xlabel(label)
        ax.set_ylabel("z [m]")
        ax.set_title(title)
        ax.grid(alpha=0.3)
        bones = [ax.plot([], [], "-", color="C0", lw=2)[0] for _ in range(model.njoints)]
        soles = [ax.plot([], [], "-", lw=3)[0] for _ in range(2)]
        (trail,) = ax.plot([], [], "r-", lw=1)
        artists.append((axis, bones, soles, trail))
    label_text = fig.suptitle("")
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    def draw(k: int):
        joints, sole_points = pose(k)
        contact = d["modes"][k] != "air"
        for axis, bones, soles, trail in artists:
            for j in range(2, model.njoints):
                a, b = joints[parents[j]], joints[j]
                bones[j].set_data([a[axis], b[axis]], [a[2], b[2]])
            for i in (0, 1):
                pts = sole_points[i][list(order)]
                pts = np.vstack([pts, pts[:1]])
                soles[i].set_data(pts[:, axis], pts[:, 2])
                soles[i].set_color("green" if contact[i] else "0.6")
            trail.set_data(d["truth_pos"][: k + 1, axis], d["truth_pos"][: k + 1, 2])
        shift = 1e3 * (d["truth_pos"][k] - start)
        label_text.set_text(
            f"t = {k * dt:5.2f} s   base since {args.anchor:g} s:  x {shift[0]:6.1f} mm   y {shift[1]:6.1f} mm"
        )
        return []

    anim = FuncAnimation(fig, draw, frames=frames, blit=False)
    writer = FFMpegWriter(fps=args.fps) if args.out.endswith(".mp4") else PillowWriter(fps=args.fps)
    anim.save(args.out, writer=writer, dpi=90)
    print(f"{len(frames)} frames, {len(frames) / args.fps:.1f} s -> {args.out}")


if __name__ == "__main__":
    main()
