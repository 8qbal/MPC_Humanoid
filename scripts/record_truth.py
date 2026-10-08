"""Record a stepping run together with the simulator ground truth, for scripts/replay_ekf.py.

The run is the one of scripts/run_stepping.py (crouch, calibrate, step in place) without the push and the scoring. The
controller sees only what the real robot measures. Per tick the file holds the controller inputs and targets (the
format of scripts/benchmark_stepping.py), the contact modes the base estimator used, the online base position of the
estimator, and the simulator base position, which is never given to the controller.

Use one env per process, one run per seed (the seed sets the sensor-noise realization):

    uv run python scripts/record_truth.py --seed 0 --record runs/seed0.npz
    uv run python scripts/record_truth.py --seed 1 --record runs/seed1.npz
    uv run python scripts/record_truth.py --seed 2 --record runs/seed2.npz

Standing control (no steps, about 10 s at the crouched pose; a longer run needs a larger --timeout):

    uv run python scripts/record_truth.py --steps 0 --t_stand 5 --record runs/stand_seed0.npz
    uv run python scripts/record_truth.py --steps 0 --t_stand 30 --timeout 60 --record runs/stand60_seed0.npz

The controller must be built first (scripts/build_controller.py). acados needs ACADOS_SOURCE_DIR=$HOME/acados and
LD_LIBRARY_PATH=$HOME/acados/lib. Runs headless unless --viz kit is given.
"""

import argparse
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--steps", type=int, default=10)
parser.add_argument("--clearance", type=float, default=0.02, help="[m] peak sole lift")
parser.add_argument("--t_ss", type=float, default=0.30, help="[s] single support")
parser.add_argument("--t_ds", type=float, default=0.10, help="[s] double support")
parser.add_argument("--t_stand", type=float, default=1.5, help="[s] standing before the first and after the last step")
parser.add_argument("--timeout", type=float, default=8.0, help="[s] run past the nominal gait duration at most")
parser.add_argument("--record", required=True, help=".npz file to write")
parser.add_argument("--seed", type=int, default=0, help="env seed: the sensor-noise realization")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import numpy as np
import torch

from isaaclab.envs import ManagerBasedEnv

from MPC_Humanoid.env import MpcHumanoidRobonionEnvCfg
from MPC_Humanoid.mpc.controller import RobonionController
from MPC_Humanoid.mpc.gait import GaitParams

FALL_HEIGHT = 0.45  # m, root height below which the robot has fallen
END_MARGIN = 0.5  # s run past the end of the plan


def main() -> None:
    cfg = MpcHumanoidRobonionEnvCfg()
    cfg.scene.num_envs = 1
    cfg.seed = args.seed
    cfg.sim.device = args.device
    env = ManagerBasedEnv(cfg)

    robot = env.scene["robot"]
    joint_names = list(robot.joint_names)
    act_names = list(env.action_manager.get_term("joint_pos")._joint_names)
    act_ids = [joint_names.index(n) for n in act_names]
    terms = env.observation_manager.active_terms["state"]
    dims = [math.prod(d) for d in env.observation_manager.group_obs_term_dim["state"]]
    starts = np.cumsum([0] + dims)
    obs_slice = {t: slice(starts[i], starts[i + 1]) for i, t in enumerate(terms)}
    q_default_act = robot.data.default_joint_pos.torch[0].cpu().numpy()[act_ids]

    params = GaitParams(
        t_ss=args.t_ss, t_ds=args.t_ds, clearance=args.clearance, n_steps=args.steps, t_stand=args.t_stand
    )
    controller = RobonionController(act_names, q_default_act, env.step_dt, params)

    obs, _ = env.reset()
    record = {k: [] for k in ("q_act", "qd_act", "quat", "gyro", "accel", "q_des", "modes", "plan_time")}
    record |= {"truth_pos": [], "ekf_pos": []}
    fell = False

    while True:
        tp = controller.status.plan_time
        if controller.plan is not None and (
            tp > controller.plan.duration + END_MARGIN or tp > params.n_steps * (args.t_ss + args.t_ds) + args.timeout
        ):
            break

        truth = robot.data.root_pos_w.torch[0].cpu().numpy()
        state = obs["state"][0].cpu().numpy()
        inputs = (
            state[obs_slice["joint_pos_rel"]][act_ids] + q_default_act,
            state[obs_slice["joint_vel_rel"]][act_ids],
            state[obs_slice["imu_orientation"]],
            state[obs_slice["imu_ang_vel"]],
            state[obs_slice["imu_lin_acc"]],
        )
        q_des = controller.step(*inputs)

        values = (*inputs, q_des, controller.ekf.modes, np.nan if tp is None else tp, truth, controller._ekf_base[0])
        for key, value in zip(record, values):
            record[key].append(np.array(value, dtype=float if key != "modes" else str))

        action = torch.tensor(q_des - q_default_act, dtype=torch.float32, device=env.device).unsqueeze(0)
        obs, _ = env.step(action)

        if robot.data.root_pos_w.torch[0, 2].item() < FALL_HEIGHT:
            fell = True
            break

    n = len(record["q_act"])
    Path(args.record).parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.record,
        act_names=np.array(act_names),
        q_default_act=q_default_act,
        step_dt=env.step_dt,
        params=np.array([params.t_ss, params.t_ds, params.clearance, params.n_steps]),
        seed=args.seed,
        fell=fell,
        **{k: np.array(v) for k, v in record.items()},
    )
    outcome = "FELL" if fell else "no fall"
    print(f"recorded {n} ticks ({n * env.step_dt:.2f} s), seed {args.seed}, {outcome} -> {args.record}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
