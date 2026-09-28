# MPC_Humanoid

MPC controller for **Robinion v2**, a Dynamixel-actuated humanoid with parallelogram-linkage legs, simulated in Isaac
Lab. The project is an external Isaac Lab project: an installable Python package (`source/MPC_Humanoid`), the robot
asset, and reference material.

The controller is an NMPC on a rigid-contact floating-base model (acados), re-solved every control tick (200 Hz), that
outputs joint position targets for the servos (Dynamixel position mode). It only uses what the real robot measures:
joint encoders and an Xsens MTi-630 AHRS (orientation + gyro). The robot has no foot force sensors; the contact state of
each foot is estimated.

## Status

| Stage | State |
|---|---|
| Robot model (URDF → USD, actuators, env) | done |
| 1. Stabilizer: standing push recovery without stepping | done (design D): stands, never worse than the passive robot under pushes, better in 2–3 of 10 cases |
| 2. Stepping in place | done: 20 steps at 2–3 cm clearance with the estimator only, ≈ 4.3 ms per tick on the desktop (Jetson not measured); resumes stepping after pushes of +1.75 / −1.0 N·s sagittal (−1.75 in 2 of 3 noise seeds) and ±0.5 / +1.0 N·s lateral |
| 3. Fast walking (0.3–0.4 m/s) without arm swing | not started |
| 4. Fast walking with arm swing | not started |

Everything tried for stage 1, with results, is in [`docs/stage1.md`](docs/stage1.md); the stage-2 design, gait rules
and results are in [`docs/stage2.md`](docs/stage2.md).

## Installation

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then create the project environment:

```bash
uv sync
```

Commit both `pyproject.toml` files and `uv.lock` so collaborators use the same environment. Always run project code
through `uv run`.

### acados

The NMPC uses [acados](https://github.com/acados/acados). `pyproject.toml` takes its Python interface
(`acados_template`) from a local build at `~/acados/interfaces/acados_template`, so acados must first be built there
following its installation guide. Code generation also needs the `t_renderer` binary (0.2.1) in `~/acados/bin`; on the
Jetson AGX Orin take its `linux-arm64` build.

Every run that uses the controller needs:

```bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$HOME/acados/lib:$LD_LIBRARY_PATH
```

The first run generates and compiles the solver into `outputs/acados/` (≈ 1.5 min for standing, ≈ 3 min for
stepping); later runs reuse it. Delete that
folder after changing the NMPC formulation.

## Run

```bash
uv run python scripts/run_env.py --viz kit          # load the Robinion env; servos hold the default pose
uv run python scripts/run_stabilizer.py                          # stage-1 standing controller, stand only
uv run python scripts/run_stabilizer.py --push x+2.5             # one push: +x, 2.5 N*s at t = 2 s
uv run python scripts/run_stabilizer.py --push y-2.25 --passive  # same push, servos only hold the default pose
uv run python scripts/run_stabilizer.py --push x-1.75 --viz kit
uv run python scripts/run_stepping.py --steps 20                 # stage-2 stepping in place, 2 cm clearance
uv run python scripts/run_stepping.py --push x+1.75              # one push in the middle of a single support
```

Without `--viz kit` the scripts run headless.

## Layout

```text
source/MPC_Humanoid/MPC_Humanoid/
  robots/robonionv2.py    ArticulationCfg: USD asset, initial pose, actuator models (the one robot config)
  env/                    plain ManagerBasedEnv the controller drives (joint position actions, IMU/AHRS observations)
  mpc/
    model.py              rigid-contact floating-base model (contact mode per foot: flat, edge, air)
    estimator.py          AHRS/gyro reading, contact-mode monitor, floating-base state estimator
    nmpc.py               acados NMPC on the model, outputs servo targets
    controller.py         StandingController: calibration, estimation, NMPC while both soles are flat
    gait.py               gait plan for stepping in place: contact schedule, DCM/CoM reference, swing spline
    gait_nmpc.py          the NMPC with per-node gait references, compiled contact algebra, split RTI
    stepping.py           SteppingController: crouch, calibration, estimation, gait rules, NMPC
  tasks/mpc_humanoid/     gym-registered RL task tree (currently only the generated cart-pole placeholder)
assets/                   robonionv2.usd (simulation) and robonionv2_controller.urdf (controller model)
references/               URDF, joint/link reference doc, upstream submodules
scripts/                  runnable scripts (see Run)
outputs/                  scratch (git-ignored): acados build, stage-1/2 prototypes, validation scripts and data
tools/asset/              URDF -> USD pipeline and controller URDF export
docs/
  PLAN.md                 roadmap and stage status
  stage1.md               stage-1 designs tried, equations and results
  stage2.md               stage-2 gait design, gait rules and results
```

## Robot asset

`assets/robonionv2.usd` is generated from `references/robinion_description/robinion2.urdf` and then hand-corrected
(parallelogram loop closures, floating base, inertia fixes). The controller's Pinocchio model,
`assets/robonionv2_controller.urdf`, is exported from that corrected USD. Read
[`references/docs/joint_info.md`](references/docs/joint_info.md) before regenerating or editing either file; it
documents every link/joint value, the leg mechanism and the exact pipeline.

## Development

Run formatting and lint checks through the project environment:

```bash
uv run pre-commit run --all-files
```

To configure VS Code, run the `setup_python_env` task or invoke its command directly:

```bash
uv run python .vscode/tools/setup_vscode.py
```

## Troubleshooting

If Pylance cannot resolve simulator modules, run the VS Code setup command above and reload the window. If indexing uses
too much memory, remove unused simulator extension paths from `.vscode/settings.json`.
