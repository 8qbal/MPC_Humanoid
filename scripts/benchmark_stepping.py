"""Time the stage-2 stepping controller without Isaac, by replaying a recorded run (for the Jetson AGX Orin).

Record the inputs once with Isaac (`scripts/run_stepping.py --record run.npz`), copy the file to the target machine
and replay it there: every tick gets the recorded encoder, AHRS and gyro values (as float32, as Isaac and the
drivers deliver them), so the controller takes the same path as in the recorded run. The replay is open loop; on
another CPU the targets can differ by rounding, which is reported (the recorded targets are the reference).

Needs no Isaac Sim / Isaac Lab: numpy, casadi, pinocchio, acados_template and the acados libraries
(ACADOS_SOURCE_DIR, LD_LIBRARY_PATH). The first run generates and compiles the solver and the compiled functions
into outputs/acados/ (a few minutes on the Jetson).

Usage:
    uv run python scripts/benchmark_stepping.py run.npz
    uv run python scripts/benchmark_stepping.py run.npz --repeat 3
"""

import argparse
import time

import numpy as np

from MPC_Humanoid.mpc.gait import GaitParams
from MPC_Humanoid.mpc.stepping import SteppingController


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("record", help=".npz written by scripts/run_stepping.py --record")
    parser.add_argument("--repeat", type=int, default=1, help="replays of the whole run, all timed")
    args = parser.parse_args()

    d = np.load(args.record)
    t_ss, t_ds, clearance, n_steps = d["params"]
    params = GaitParams(t_ss=float(t_ss), t_ds=float(t_ds), clearance=float(clearance), n_steps=int(n_steps))
    inputs = [d[k].astype(np.float32) for k in ("q_act", "qd_act", "quat", "gyro")]
    controller = SteppingController(
        list(d["act_names"]), d["q_default_act"].astype(np.float32), float(d["step_dt"]), params
    )

    ticks, solves, diff = [], [], 0.0
    for rep in range(args.repeat):
        controller.reset()
        for k in range(len(inputs[0])):
            t0 = time.perf_counter()
            q_des = controller.step(*(x[k] for x in inputs))
            dt = time.perf_counter() - t0
            st = controller.status
            if st.plan_time is not None:
                ticks.append(dt)
                if st.solve_time is not None:
                    solves.append(st.solve_time)
            if rep == 0:  # later replays start from the solver's warm-start memory of the previous one
                diff = max(diff, float(np.abs(q_des - d["q_des"][k]).max()))

    tk, sv = 1e3 * np.array(ticks), 1e3 * np.array(solves)
    print(f"{args.record}: {len(tk)} controller ticks ({args.repeat} replay(s)), budget {1e3 * d['step_dt']:.1f} ms")
    print(
        f"  tick (estimation + NMPC) mean {tk.mean():.2f}, p50 {np.percentile(tk, 50):.2f}, p95 "
        f"{np.percentile(tk, 95):.2f}, p99 {np.percentile(tk, 99):.2f}, max {tk.max():.2f} ms; over budget "
        f"{100 * np.mean(tk > 1e3 * d['step_dt']):.1f} % of ticks"
    )
    print(f"  acados solve time per tick mean {sv.mean():.2f}, p95 {np.percentile(sv, 95):.2f} ms")
    print(f"  max |servo target - recorded| {diff:.2e} rad")


if __name__ == "__main__":
    main()
