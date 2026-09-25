"""Run the stage-1 stabilizer in the Robinion env and push the robot.

The controller only sees what the real robot measures (joint encoders, AHRS
orientation, gyro). The robot is pushed on the upper body every --push_interval
seconds, alternating along x and y. Foot slip is read from the simulator for
evaluation only. Reports every 0.1 s and, per push, the peak DCM excursion and
whether the robot kept its feet.

Usage:
    uv run python scripts/run_stabilizer.py                    # 1 N*s pushes, x/y alternating every 1.5 s
    uv run python scripts/run_stabilizer.py --push 2.0 --viz kit
    uv run python scripts/run_stabilizer.py --push 0           # stand only
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--seconds", type=float, default=8.0)
parser.add_argument("--push", type=float, default=1.0, help="push impulse [N*s], negative pushes backwards/right")
parser.add_argument("--push_time", type=float, default=2.0, help="time of the first push [s]")
parser.add_argument("--push_interval", type=float, default=1.5, help="[s]")
parser.add_argument("--push_duration", type=float, default=0.05, help="[s]")
parser.add_argument("--first_axis", choices=("x", "y"), default="x", help="axis of the first push")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import math

import numpy as np
import torch
from isaaclab.envs import ManagerBasedEnv

from MPC_Humanoid.env import MpcHumanoidRobonionEnvCfg
from MPC_Humanoid.mpc.controller import StabilizerController


def main() -> None:
    cfg = MpcHumanoidRobonionEnvCfg()
    cfg.scene.num_envs = 1
    cfg.sim.device = args.device
    env = ManagerBasedEnv(cfg)
    robot = env.scene["robot"]
    joint_names = list(robot.joint_names)
    body_names = list(robot.body_names)

    action_term = env.action_manager.get_term("joint_pos")
    act_names = list(action_term._joint_names)
    act_ids = [joint_names.index(n) for n in act_names]

    terms = env.observation_manager.active_terms["state"]
    dims = [math.prod(d) for d in env.observation_manager.group_obs_term_dim["state"]]
    starts = np.cumsum([0] + dims)
    obs_slice = {t: slice(starts[i], starts[i + 1]) for i, t in enumerate(terms)}

    default_q = robot.data.default_joint_pos.torch[0].cpu().numpy()
    q_default_act = default_q[act_ids]
    controller = StabilizerController(act_names, q_default_act, env.step_dt)
    print(f"CoM height {controller.ik.com_height:.3f} m, omega {controller.omega:.2f} rad/s")

    feet = [body_names.index(f"{side}_foot_roll_link") for side in ("left", "right")]
    torso = body_names.index("upper_body_link")

    obs, _ = env.reset()
    steps = int(round(args.seconds / env.step_dt))
    push_len = max(1, int(round(args.push_duration / env.step_dt)))
    interval = int(round(args.push_interval / env.step_dt))
    first = int(round(args.push_time / env.step_dt))
    axes = "xy" if args.first_axis == "x" else "yx"
    pushes = {} if args.push == 0 else {k: axes[i % 2] for i, k in enumerate(range(first, steps - push_len, interval))}
    force_mag = args.push / (push_len * env.step_dt)
    print(f"{len(pushes)} pushes of {args.push:+.2f} N*s every {args.push_interval:.2f} s, x/y alternating: "
          f"{force_mag:.1f} N on upper_body_link for {push_len * env.step_dt:.3f} s")

    state = obs["state"][0].cpu().numpy()
    controller.reset(state[obs_slice["joint_pos_rel"]][act_ids] + q_default_act)
    feet_start = robot.data.body_link_pose_w.torch[0, feet, :2].cpu().numpy()
    rows = []

    for step in range(steps):
        if step in pushes:
            force = torch.zeros(1, 1, 3, device=env.device)
            force[0, 0, "xy".index(pushes[step])] = force_mag
            robot.permanent_wrench_composer.set_forces_and_torques_index(forces=force, body_ids=[torso], is_global=True)
        elif step - push_len in pushes:
            robot.permanent_wrench_composer.reset()

        state = obs["state"][0].cpu().numpy()
        out = controller.step(
            state[obs_slice["joint_pos_rel"]][act_ids] + q_default_act,
            state[obs_slice["joint_vel_rel"]][act_ids],
            state[obs_slice["imu_orientation"]],
            state[obs_slice["imu_ang_vel"]],
        )
        action = torch.tensor(out.q_des - q_default_act, dtype=torch.float32, device=env.device).unsqueeze(0)
        obs, _ = env.step(action)
        t = (step + 1) * env.step_dt

        slip = np.linalg.norm(robot.data.body_link_pose_w.torch[0, feet, :2].cpu().numpy() - feet_start, axis=1).max()
        p0 = out.mpc.p0 if out.mpc is not None else np.full(2, np.nan)
        rows.append((t, out.est.xi, p0, slip, out.mode))

        if step % int(0.1 / env.step_dt) == 0 or step in pushes:
            mark = f"  <-- push {pushes[step]}" if step in pushes else ""
            print(
                f"t={t:5.2f}s | xi={out.est.xi.round(4)} p0={p0.round(4)} c_ref={out.c_ref.round(4)}"
                f" | foot slip {slip * 1000:5.1f} mm | {out.mode}{'' if out.recoverable else ' (xi outside soles)'}{mark}"
            )
        if out.mode == "safe_stop":
            print(f"t={t:5.2f}s | safe_stop (CoM height {out.est.com_height:.3f} m), stopping")
            break

    print("\nPER PUSH (peak xi change from the value just before the push, within the following interval)")
    for k, axis in pushes.items():
        if k >= len(rows):
            break
        a = "xy".index(axis)
        window = rows[k : k + interval]
        before = rows[k - 1][1][a]
        peak = max(window, key=lambda r: abs(r[1][a] - before))
        print(f"t={rows[k][0]:5.2f}s along {axis}: xi {(peak[1][a] - before) * 1000:+6.1f} mm, "
              f"peak |p0| {max(abs(r[2][a]) for r in window) * 1000:5.1f} mm, "
              f"foot slip {window[-1][3] * 1000:5.1f} mm, {'ok' if window[-1][4] == 'stand' else 'safe_stop'}")
    print(f"\nfinal mode: {rows[-1][4]}, root height {robot.data.root_pos_w.torch[0, 2].item():.3f} m, "
          f"max foot slip {max(r[3] for r in rows) * 1000:.1f} mm")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
