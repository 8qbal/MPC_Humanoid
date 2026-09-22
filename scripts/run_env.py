"""Load MpcHumanoidRobinionEnvCfg and play the simulation with zero commanded effort.

The actuator PD holds the crouch pose, so the robot just stands there. Close the window
(or Ctrl-C) to stop.

Usage:
    uv run python scripts/run_env.py --viz kit       # Isaac Sim window
    uv run python scripts/run_env.py                 # headless, prints state every second
    uv run python scripts/run_env.py --viz kit --num_envs 4
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument(
    "--seconds", type=float, default=None, help="stop after this many sim seconds (default: run until closed)"
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402

from isaaclab.envs import ManagerBasedEnv  # noqa: E402

from MPC_Humanoid.env import MpcHumanoidRobinionEnvCfg  # noqa: E402


def main() -> None:
    cfg = MpcHumanoidRobinionEnvCfg()
    cfg.scene.num_envs = args.num_envs
    env = ManagerBasedEnv(cfg)
    robot = env.scene["robot"]
    print(f"joints ({len(robot.joint_names)}): {robot.joint_names}")
    print(f"bodies ({len(robot.body_names)}): {robot.body_names}")

    obs, _ = env.reset()
    zero_action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    steps_per_second = int(round(1.0 / env.step_dt))
    step = 0
    while simulation_app.is_running():
        obs, _ = env.step(zero_action)
        step += 1
        if step % steps_per_second == 0:
            t = step * env.step_dt
            z = robot.data.root_pos_w.torch[0, 2].item()
            print(f"t={t:6.1f}s  base z={z:.3f} m  obs finite={bool(torch.isfinite(obs['state']).all())}")
        if args.seconds is not None and step * env.step_dt >= args.seconds:
            break
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
