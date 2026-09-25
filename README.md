# MPC_Humanoid

MPC controller for **Robinion v2**, a Dynamixel-actuated humanoid with parallelogram-linkage legs, simulated in Isaac
Lab. The project is an external Isaac Lab project: an installable Python package (`source/MPC_Humanoid`), the robot
asset, and reference material.

The controller is a DCM MPC re-solved every control tick (200 Hz) that outputs joint position targets for the servos
(Dynamixel position mode). It only uses what the real robot measures: joint encoders and an Xsens MTi-630 AHRS. The
robot has no foot force sensors.

## Status

| Stage | State |
|---|---|
| Robot model (URDF → USD, actuators, env) | done |
| 1. Stabilizer: standing push recovery without stepping | implemented, **not yet stable in simulation** |
| 2. Stepping in place | not started |
| 3. Fast walking (0.3–0.4 m/s) without arm swing | not started |
| 4. Fast walking with arm swing | not started |

Details, test results and next steps for stage 1 are in [`docs/PLAN.md`](docs/PLAN.md) (*Stage 1 status*).

## Installation

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then create the project environment:

```bash
uv sync
```

Commit both `pyproject.toml` files and `uv.lock` so collaborators use the same environment. Always run project code
through `uv run`.

## Run

```bash
uv run python scripts/run_env.py --viz kit          # load the Robinion env; servos hold the default pose
uv run python scripts/check_estimator.py            # validate the CoM/DCM estimator against simulator truth under pushes
uv run python scripts/run_stabilizer.py             # run the stage-1 stabilizer with x/y pushes (1 N*s every 1.5 s)
uv run python scripts/run_stabilizer.py --push 0    # stand only
uv run python scripts/run_stabilizer.py --push 2.0 --viz kit
```

Without `--viz kit` the scripts run headless.

## Layout

```text
source/MPC_Humanoid/MPC_Humanoid/
  robots/robonionv2.py    ArticulationCfg: USD asset, initial pose, actuator models (the one robot config)
  env/                    plain ManagerBasedEnv the controller drives (joint position actions, IMU/AHRS observations)
  mpc/
    estimator.py          [A] CoM / DCM estimator from encoders + AHRS
    dcm_mpc.py            [B] DCM MPC: ZMP plan over the horizon (OSQP)
    ik.py                 [D] velocity IK on the Pinocchio model with the parallelogram coupling
    controller.py         [C] LIPM step, [E] safety, and the StabilizerController that chains all blocks
  tasks/mpc_humanoid/     gym-registered RL task tree (currently only the generated cart-pole placeholder)
assets/                   robonionv2.usd (simulation) and robonionv2_controller.urdf (controller model)
references/               URDF, joint/link reference doc, upstream submodules
scripts/                  runnable scripts (see Run)
tools/asset/              URDF -> USD pipeline and controller URDF export
docs/
  PLAN.md                 roadmap and stage status
  stabilizer.md           stage-1 variables and equations
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
