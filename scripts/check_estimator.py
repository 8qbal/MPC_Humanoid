"""Validate the stabilizer estimator against simulator ground truth.

The robot stands (servos hold the default pose, no stabilizer yet) and is pushed
on the upper body every --push_interval seconds, alternating along x and y. Each
push has a known impulse: a constant horizontal force impulse / duration applied
at the upper_body_link CoM for --push_duration. The estimator runs on what the real robot
measures: actuated joint encoders and velocities, and the noisy AHRS orientation
and gyro from the env observations. Ground truth (body CoMs and CoM velocities,
foot poses, passive joint angles) is read from the simulator only for this
comparison. Reports:

  - CoM c and DCM xi in the support frame S: estimate vs. ground truth, every 0.1 s,
  - per push: peak DCM excursion (estimate and truth),
  - parallelogram coupling: passive joint angles predicted from the actuated
    ones vs. the angles PhysX produces through the loop closures.

Usage:
    uv run python scripts/check_estimator.py                  # 1 N*s pushes, x/y alternating every 1 s
    uv run python scripts/check_estimator.py --push 1.5 --viz kit
"""

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--seconds", type=float, default=7.0)
parser.add_argument("--push", type=float, default=1.0, help="push impulse [N*s], negative pushes backwards/right")
parser.add_argument("--push_time", type=float, default=1.0, help="time of the first push [s]")
parser.add_argument("--push_interval", type=float, default=1.0, help="[s]")
parser.add_argument("--push_duration", type=float, default=0.05, help="[s]")
parser.add_argument("--first_axis", choices=("x", "y"), default="x", help="axis of the first push")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import numpy as np
import torch
from isaaclab.envs import ManagerBasedEnv

from MPC_Humanoid.env import MpcHumanoidRobonionEnvCfg
from MPC_Humanoid.mpc.estimator import GRAVITY, PASSIVE_COUPLING, SOLE_CENTRE, StabilizerEstimator


def quat_to_R(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = q_xyzw / np.linalg.norm(q_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main() -> None:
    cfg = MpcHumanoidRobonionEnvCfg()
    cfg.scene.num_envs = 1
    env = ManagerBasedEnv(cfg)
    robot = env.scene["robot"]
    joint_names = list(robot.joint_names)
    body_names = list(robot.body_names)

    act_names = list(env.action_manager.get_term("joint_pos")._joint_names)
    act_ids = [joint_names.index(n) for n in act_names]

    terms = env.observation_manager.active_terms["state"]
    dims = [math.prod(d) for d in env.observation_manager.group_obs_term_dim["state"]]
    starts = np.cumsum([0] + dims)
    obs_slice = {t: slice(starts[i], starts[i + 1]) for i, t in enumerate(terms)}

    estimator = StabilizerEstimator(act_names, dt=env.step_dt)

    passive_pairs = [
        (joint_names.index(f"{side}_{p}"), joint_names.index(f"{side}_{a}"), gain)
        for side in ("left", "right")
        for p, (a, gain) in PASSIVE_COUPLING.items()
    ]
    feet = [body_names.index(f"{side}_foot_roll_link") for side in ("left", "right")]
    root = body_names.index("lower_body_link")
    torso = body_names.index("upper_body_link")
    default_q = robot.data.default_joint_pos.torch[0].cpu().numpy()
    mass = robot.data.body_mass.torch[0].cpu().numpy()

    obs, _ = env.reset()
    action = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
    steps = int(round(args.seconds / env.step_dt))
    push_len = max(1, int(round(args.push_duration / env.step_dt)))
    interval = int(round(args.push_interval / env.step_dt))
    first = int(round(args.push_time / env.step_dt))
    axes = "xy" if args.first_axis == "x" else "yx"
    pushes = {k: axes[i % 2] for i, k in enumerate(range(first, steps - push_len, interval))}
    force_mag = args.push / (push_len * env.step_dt)
    print(f"{len(pushes)} pushes of {args.push:+.2f} N*s every {args.push_interval:.2f} s, x/y alternating: "
          f"{force_mag:.1f} N on upper_body_link for {push_len * env.step_dt:.3f} s")
    rows, passive_err = [], []

    for step in range(steps):
        if step in pushes:
            force = torch.zeros(1, 1, 3, device=env.device)
            force[0, 0, "xy".index(pushes[step])] = force_mag
            robot.permanent_wrench_composer.set_forces_and_torques_index(forces=force, body_ids=[torso], is_global=True)
        elif step - push_len in pushes:
            robot.permanent_wrench_composer.reset()
        obs, _ = env.step(action)
        t = (step + 1) * env.step_dt

        state = obs["state"][0].cpu().numpy()
        q_all = state[obs_slice["joint_pos_rel"]] + default_q
        qd_all = state[obs_slice["joint_vel_rel"]]
        est = estimator.update(
            q_all[act_ids], qd_all[act_ids], state[obs_slice["imu_orientation"]], state[obs_slice["imu_ang_vel"]]
        )
        if est.fallen:
            print(f"t={t:5.2f}s | estimator reports a fall (CoM height {est.com_height:.3f} m), stopping")
            break

        q_true = robot.data.joint_pos.torch[0].cpu().numpy()
        passive_err.append(max(abs(q_true[p] - g * q_true[s]) for p, s, g in passive_pairs))

        com_w = (mass[:, None] * robot.data.body_com_pos_w.torch[0].cpu().numpy()).sum(0) / mass.sum()
        comd_w = (mass[:, None] * robot.data.body_com_lin_vel_w.torch[0].cpu().numpy()).sum(0) / mass.sum()
        pose = robot.data.body_link_pose_w.torch[0].cpu().numpy()
        soles = [pose[f, :3] + quat_to_R(pose[f, 3:]) @ SOLE_CENTRE for f in feet]
        R_root = quat_to_R(pose[root, 3:])
        yaw = math.atan2(R_root[1, 0], R_root[0, 0])
        R_yaw = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        c_true = R_yaw @ (com_w - 0.5 * (soles[0] + soles[1]))[:2]
        xi_true = c_true + R_yaw @ comd_w[:2] / math.sqrt(GRAVITY / est.com_height)
        rows.append((t, est.c, c_true, est.xi, xi_true))

        if step % int(0.1 / env.step_dt) == 0:
            mark = f"  <-- push {pushes[step]}" if step in pushes else ""
            print(
                f"t={t:5.2f}s | c est={est.c.round(4)} true={c_true.round(4)}"
                f" | xi est={est.xi.round(4)} true={xi_true.round(4)}"
                f" err={np.abs(est.xi - xi_true).max() * 1000:5.1f} mm{mark}"
            )

    c_err = np.array([np.abs(r[1] - r[2]).max() for r in rows])
    xi_err = np.array([np.abs(r[3] - r[4]).max() for r in rows])
    print("\nPER PUSH (peak xi change from the value just before the push, within the following interval)")
    for k, axis in pushes.items():
        if k >= len(rows):
            break
        a = "xy".index(axis)
        window = rows[k : k + interval]
        est_before, true_before = rows[k - 1][3][a], rows[k - 1][4][a]
        peak = max(window, key=lambda r: abs(r[4][a] - true_before))
        print(f"t={rows[k][0]:5.2f}s along {axis}: truth {(peak[4][a] - true_before) * 1000:+6.1f} mm, "
              f"estimate {(peak[3][a] - est_before) * 1000:+6.1f} mm (at t={peak[0]:.2f}s)")
    print("\nSUMMARY")
    print(f"CoM c in S, estimate vs truth:  max {c_err.max() * 1000:.2f} mm, rms {np.sqrt((c_err**2).mean()) * 1000:.2f} mm")
    print(f"DCM xi in S, estimate vs truth: max {xi_err.max() * 1000:.2f} mm, rms {np.sqrt((xi_err**2).mean()) * 1000:.2f} mm")
    print(f"parallelogram coupling, passive angle error: max {math.degrees(max(passive_err)):.3f} deg")
    print(f"root height at end: {robot.data.root_pos_w.torch[0, 2].item():.3f} m")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
