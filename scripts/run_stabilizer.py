"""Run the stage-1 standing controller in the Robinion env and push the robot once.

The controller only sees what the real robot measures (joint encoders, AHRS orientation, gyro). It
calibrates the AHRS bias during the first 0.5 s, moves the CoM to the sole centre over the next 1 s,
and the push (constant force on upper_body_link) comes at --push_time. Simulator ground truth (foot
displacement, sole tilt, root height) is read for scoring only. Use one env per process: multi-env
runs of this asset differ from single-env runs (see docs/stage1.md).

The first run builds the acados solver (~1.5 min, into outputs/acados/); acados needs
ACADOS_SOURCE_DIR and LD_LIBRARY_PATH (see README).

Usage:
    uv run python scripts/run_stabilizer.py --push x+2.5               # push +x with 2.5 N*s
    uv run python scripts/run_stabilizer.py --push y-2.25 --passive    # same push, servos hold the default pose
    uv run python scripts/run_stabilizer.py --viz kit                  # stand only, with the Isaac Sim window
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--seconds", type=float, default=4.0)
parser.add_argument("--push", default="x+0", help="axis and impulse [N*s], e.g. x+2.5 or y-2.25")
parser.add_argument("--push_time", type=float, default=2.0, help="[s]")
parser.add_argument("--push_duration", type=float, default=0.05, help="[s]")
parser.add_argument("--passive", action="store_true", help="no controller: servos hold the default pose")
parser.add_argument("--seed", type=int, default=0, help="env seed: the sensor-noise realization (AHRS bias, gyro)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import math

import numpy as np
import pinocchio as pin
import torch

from isaaclab.envs import ManagerBasedEnv

from MPC_Humanoid.env import MpcHumanoidRobonionEnvCfg
from MPC_Humanoid.mpc.controller import StandingController


def sole_tilt(quat_xyzw: np.ndarray) -> np.ndarray:
    """(roll, pitch) of a body with yaw removed [deg]."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    R = pin.Quaternion(w, x, y, z).matrix()
    R = pin.utils.rotate("z", -math.atan2(R[1, 0], R[0, 0])) @ R
    return np.degrees([math.atan2(R[2, 1], R[2, 2]), -math.asin(float(np.clip(R[2, 0], -1.0, 1.0)))])


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

    controller = None if args.passive else StandingController(act_names, q_default_act, env.step_dt)
    axis, impulse = args.push[0], float(args.push[1:])
    push_start = int(round(args.push_time / env.step_dt))
    push_len = max(1, int(round(args.push_duration / env.step_dt)))

    obs, _ = env.reset()
    feet_start = robot.data.body_link_pose_w.torch[0, feet, :2].cpu().numpy()
    tilt_start = None
    q_des = q_default_act.copy()
    displacement, tilt_peak, fell = np.zeros(2), 0.0, False
    solve_times, qp_failures, held, mode_changes, last_modes = [], 0, 0, [], ["flat", "flat"]
    for step in range(int(round(args.seconds / env.step_dt))):
        if step == push_start and impulse != 0:
            force = torch.zeros(1, 1, 3, device=env.device)
            force[0, 0, "xy".index(axis)] = impulse / (push_len * env.step_dt)
            robot.permanent_wrench_composer.set_forces_and_torques_index(forces=force, body_ids=[torso], is_global=True)
        elif step == push_start + push_len:
            robot.permanent_wrench_composer.reset()

        if controller is not None:
            state = obs["state"][0].cpu().numpy()
            q_des = controller.step(
                state[obs_slice["joint_pos_rel"]][act_ids] + q_default_act,
                state[obs_slice["joint_vel_rel"]][act_ids],
                state[obs_slice["imu_orientation"]],
                state[obs_slice["imu_ang_vel"]],
            )
            st = controller.status
            if st.solve_time is not None:
                solve_times.append(st.solve_time)
                qp_failures += st.qp_status != 0
            held += st.holding
            if st.modes != last_modes:
                mode_changes.append(f"{(step + 1) * env.step_dt:.3f} {st.modes[0]}/{st.modes[1]}")
                last_modes = st.modes
        action = torch.tensor(q_des - q_default_act, dtype=torch.float32, device=env.device).unsqueeze(0)
        obs, _ = env.step(action)

        pose = robot.data.body_link_pose_w.torch[0].cpu().numpy()
        tilt = np.array([sole_tilt(pose[b, 3:]) for b in feet])
        if tilt_start is None:
            tilt_start = tilt
        tilt_peak = max(tilt_peak, np.abs(tilt - tilt_start).max())
        displacement = np.maximum(displacement, np.linalg.norm(pose[feet, :2] - feet_start, axis=1))
        if robot.data.root_pos_w.torch[0, 2].item() < 0.45:
            fell = True
            break

    result = "FALL" if fell else "ok"
    print(
        f"\nRESULT {'passive' if args.passive else 'controller'} push {args.push}: {result}, "
        f"peak sole tilt {tilt_peak:.1f} deg, max foot displacement L/R {(displacement * 1000).round(0)} mm"
    )
    if solve_times:
        print(
            f"NMPC solve mean/max {np.mean(solve_times) * 1000:.2f}/{np.max(solve_times) * 1000:.2f} ms, "
            f"QP failures {qp_failures}, servo targets held {held * env.step_dt:.2f} s"
        )
        print(f"contact modes: {'; '.join(mode_changes[:12]) or 'flat/flat throughout'}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
