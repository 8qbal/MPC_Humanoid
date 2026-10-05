"""Run the stage-2 stepping-in-place controller in the Robonion env, optionally with one push while stepping.

The controller only sees what the real robot measures (joint encoders, AHRS orientation, gyro, accelerometer): it
crouches, calibrates the AHRS bias, then steps in place (docs/stage2.md). The push (constant force on upper_body_link)
comes in the middle of the 5th single support (--push_at mid_ss) or of the double support after it (ds). Simulator
ground truth (sole heights, foot displacement, root height) is read for scoring only. Use one env per process:
multi-env runs of this asset differ from single-env runs (see docs/stage1.md).

The controller must be built first (scripts/build_controller.py, no Isaac); this script only loads it. acados needs
ACADOS_SOURCE_DIR=$HOME/acados and LD_LIBRARY_PATH=$HOME/acados/lib.

Usage:
    uv run python scripts/run_stepping.py                            # 10 steps, 2 cm clearance
    uv run python scripts/run_stepping.py --steps 20 --clearance 0.03
    uv run python scripts/run_stepping.py --push x+1.75              # push +x with 1.75 N*s mid single support
    uv run python scripts/run_stepping.py --viz kit                  # with the Isaac Sim window
    uv run python scripts/run_stepping.py --record run.npz           # inputs for scripts/benchmark_stepping.py
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--steps", type=int, default=10)
parser.add_argument("--clearance", type=float, default=0.02, help="[m] peak sole lift")
parser.add_argument("--t_ss", type=float, default=0.30, help="[s] single support")
parser.add_argument("--t_ds", type=float, default=0.10, help="[s] double support")
parser.add_argument("--push", default=None, help="axis and impulse [N*s], e.g. x+1.75 or y-1.0")
parser.add_argument("--push_at", default="mid_ss", choices=("mid_ss", "ds"))
parser.add_argument("--push_duration", type=float, default=0.05, help="[s]")
parser.add_argument("--record", default=None, help="save the controller inputs and outputs per tick to this .npz")
parser.add_argument("--seed", type=int, default=0, help="env seed: the sensor-noise realization (AHRS bias, gyro)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import math
import time

import numpy as np
import pinocchio as pin
import torch

from isaaclab.envs import ManagerBasedEnv

from MPC_Humanoid.env import MpcHumanoidRobonionEnvCfg
from MPC_Humanoid.mpc.controller import RobonionController
from MPC_Humanoid.mpc.gait import GaitParams
from MPC_Humanoid.mpc.model import SOLE_CORNERS

H_AIR = 0.001  # m, lowest sole corner above its standing height: the foot is in the air (scoring only)
PUSH_SS = 4  # index of the single support the push is timed on (the 5th)
END_MARGIN = 0.5  # s run past the end of the plan
TIMEOUT = 8.0  # s past the nominal gait duration, in case retiming and pauses keep extending the plan


def sole_lows(pose: np.ndarray, feet: list[int]) -> np.ndarray:
    lows = []
    for b in feet:
        x, y, z, w = (float(v) for v in pose[b, 3:])
        R = pin.Quaternion(w, x, y, z).matrix()
        lows.append((pose[b, :3][:, None] + R @ SOLE_CORNERS.T)[2].min())
    return np.array(lows)


def main() -> None:
    cfg = MpcHumanoidRobonionEnvCfg()
    cfg.scene.num_envs = 1
    cfg.seed = args.seed
    cfg.sim.device = args.device
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
    default_q = robot.data.default_joint_pos.torch[0].cpu().numpy()
    q_default_act = default_q[act_ids]
    feet = [body_names.index(f"{side}_foot_roll_link") for side in ("left", "right")]
    torso = body_names.index("upper_body_link")

    params = GaitParams(t_ss=args.t_ss, t_ds=args.t_ds, clearance=args.clearance, n_steps=args.steps)
    controller = RobonionController(act_names, q_default_act, env.step_dt, params)

    obs, _ = env.reset()
    push_t = push_end = None
    push_modes = "-"
    ground = feet0 = None
    in_air = [False, False]
    lifts = landings = 0
    displacement = np.zeros(2)
    fell = False
    ticks, solve_times = [], []
    qp_failures = held = 0
    record = {k: [] for k in ("q_act", "qd_act", "quat", "gyro", "accel", "q_des")}
    step = 0

    while True:
        tp = controller.status.plan_time
        if controller.plan is not None:
            if push_t is None:
                ss = [ph for ph in controller.plan.phases if ph.swing is not None][PUSH_SS]
                push_t = 0.5 * (ss.t0 + ss.t1) if args.push_at == "mid_ss" else ss.t1 + 0.5 * args.t_ds
            if tp > controller.plan.duration + END_MARGIN or tp > params.n_steps * (args.t_ss + args.t_ds) + TIMEOUT:
                break

        if args.push and tp is not None and push_end is None and tp >= push_t:
            force = torch.zeros(1, 1, 3, device=env.device)
            force[0, 0, "xy".index(args.push[0])] = float(args.push[1:]) / args.push_duration
            robot.permanent_wrench_composer.set_forces_and_torques_index(forces=force, body_ids=[torso], is_global=True)
            push_end = tp + args.push_duration
            push_modes = "/".join(controller.status.modes)
        elif push_end is not None and tp is not None and tp >= push_end:
            robot.permanent_wrench_composer.reset()
            push_end = math.inf

        state = obs["state"][0].cpu().numpy()
        inputs = (
            state[obs_slice["joint_pos_rel"]][act_ids] + q_default_act,
            state[obs_slice["joint_vel_rel"]][act_ids],
            state[obs_slice["imu_orientation"]],
            state[obs_slice["imu_ang_vel"]],
            state[obs_slice["imu_lin_acc"]],
        )

        t0 = time.perf_counter()
        q_des = controller.step(*inputs)
        if args.record:
            for k, v in zip(record, (*inputs, q_des)):
                record[k].append(np.array(v, dtype=float))

        st = controller.status
        if st.plan_time is not None:
            ticks.append(time.perf_counter() - t0)
            held += st.holding
            if st.solve_time is not None:
                solve_times.append(st.solve_time)
                qp_failures += st.qp_status not in (0, 2)

        action = torch.tensor(q_des - q_default_act, dtype=torch.float32, device=env.device).unsqueeze(0)
        obs, _ = env.step(action)
        step += 1

        pose = robot.data.body_link_pose_w.torch[0].cpu().numpy()
        if st.plan_time is not None:
            lows = sole_lows(pose, feet)
            if ground is None:
                ground, feet0 = lows.min(), pose[feet, :2].copy()
            for i in (0, 1):
                up = lows[i] - ground > H_AIR
                lifts += up and not in_air[i]
                landings += in_air[i] and not up
                in_air[i] = up
            displacement = np.maximum(displacement, np.linalg.norm(pose[feet, :2] - feet0, axis=1))

        if robot.data.root_pos_w.torch[0, 2].item() < 0.45:
            fell = True
            break

    result = "FALL" if fell else "ok"
    push = f" push {args.push} at plan t={push_t:.2f} ({args.push_at}, modes {push_modes})" if args.push else ""
    print(
        f"\nRESULT stepping {params.n_steps} steps, clearance {1000 * args.clearance:.0f} mm{push}: {result} at "
        f"t={step * env.step_dt:.2f} s, steps completed {landings} (lift-offs {lifts}), max foot displacement L/R "
        f"{(displacement * 1000).round(1)} mm, targets held {held * env.step_dt:.2f} s"
    )

    if ticks:
        tk = np.array(ticks) * 1000
        print(
            f"controller tick (estimation + NMPC) mean/p95/max {tk.mean():.2f}/{np.percentile(tk, 95):.2f}/"
            f"{tk.max():.2f} ms; NMPC solve mean/max {np.mean(solve_times) * 1000:.2f}/"
            f"{np.max(solve_times) * 1000:.2f} ms, QP failures {qp_failures}"
        )

    if args.record:
        np.savez(
            args.record,
            act_names=np.array(act_names),
            q_default_act=q_default_act,
            step_dt=env.step_dt,
            params=np.array([params.t_ss, params.t_ds, params.clearance, params.n_steps]),
            **{k: np.array(v) for k, v in record.items()},
        )

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
