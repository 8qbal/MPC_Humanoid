# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

An Isaac Lab external project: an installable Python package (`source/MPC_Humanoid`)
plus assets and reference material for **Robinion**, a Dynamixel-actuated humanoid
with parallelogram-linkage legs. The goal is an MPC-controlled robot. The real-to-sim
model (`ArticulationCfg` in `source/MPC_Humanoid/MPC_Humanoid/robots/robonionv2.py`
backed by `assets/robonionv2.usd`) is done; work has moved on to the controller.

## Working with the user

Do not create files or write code on your own initiative. Discuss the approach with
the user first and wait for their go-ahead before creating or editing any file. Then
implement only what was agreed. Propose anything extra (helper modules, scripts, doc
updates) in one sentence and let the user decide.

`docs/PLAN.md` is a roadmap for humans to read, not a task list for you to follow. Its
phases and checklists do not tell you what to do next or what the current state is
(Phase 0, the model, is already finished). Take the task from the user, and use the
plan only as background on decisions already made.

## Environment

- Package manager: **uv**. Always run project code as `uv run <cmd>`, never call
  `python`/pip tools directly — the interpreter and all Isaac Sim/Isaac Lab packages
  live in the project's `.venv`, resolved by `uv`.
- Python 3.12, workspace member `source/MPC_Humanoid` (see `pyproject.toml`).
- `uv sync` installs/updates the environment. **Do not run installs, builds, or
  downloads on the user's behalf unless explicitly asked** — the user runs these
  themselves; describe the command instead of executing it when the task is
  install/build/download in nature.
- Isaac Sim ships in two independent places on this machine — do not assume they
  are interchangeable:
  - This project's `.venv` (`isaacsim` pip package, currently 6.1.0.0): headless
    Python only (`uv run isaaclab ...`, `uv run python ...`). No GUI.
  - `/home/tkuai/isaacsim` (standalone Isaac Sim Full app, currently 6.0.1-rc.7):
    the interactive GUI. Its bundled `urdf_usd_converter` may be a different,
    possibly older version than the one in `.venv` — check both before assuming a
    GUI re-import behaves like the CLI converter (see `references/docs/joint_info.md`
    for a concrete version-skew bug this caused).

## Commands

```bash
uv run isaaclab zero_agent --task <TASK_NAME> --num_envs 16   # sanity-check an env, no policy
uv run isaaclab random_agent --task <TASK_NAME> --num_envs 16
uv run isaaclab train --rl_library <RL_LIBRARY> --task <TASK_NAME>
uv run isaaclab play --rl_library <RL_LIBRARY> --task <TASK_NAME> --checkpoint latest
uv run isaaclab train_multigpu --rl_library <RL_LIBRARY> --task <TASK_NAME> --num_gpus 2
uv run isaaclab benchmark runtime --task <TASK_NAME> --num_envs 16 --num_steps 1000
uv run isaaclab benchmark training --rl_library <RL_LIBRARY> --task <TASK_NAME> --max_iterations 10
uv run pre-commit run --all-files                              # lint/format (ruff, codespell, etc.)
uv run python scripts/run_env.py --viz kit                    # load the Robinion MPC env and play it (headless without --viz)
```

`physics=<PRESET>` selects one of the physics presets defined per task (e.g. the
`MpcHumanoidPhysicsCfg` presets such as `newton_mjwarp` / `newton_kamino` / `physx`).

Linting: `ruff` (line length 120, `E,F,I,UP,W`) + `ruff-format` + `codespell`, run
via pre-commit. Match existing code style (docstrings, type hints, `from __future__
import annotations`) rather than introducing a new one. There is no test suite yet.

## Comments

Do not over-comment. Default to no comments — well-named code should speak for
itself. Only add a comment when the *why* is non-obvious: a hidden constraint, a
physical/mechanical fact (e.g. why a joint is passive), a workaround for a specific
upstream bug, or a numeric value whose source (URDF field, datasheet, mesh
computation) isn't otherwise documented nearby. Never comment on *what* the code
does, restate the function/variable name, or leave narration of the current task.

To configure VS Code (Pylance resolving simulator modules), run:
```bash
uv run python .vscode/tools/setup_vscode.py
```
If Pylance still can't resolve simulator modules after this, reload the window; if
indexing uses too much memory, remove unused simulator extension paths from
`.vscode/settings.json`.

## Repository layout

```
source/MPC_Humanoid/MPC_Humanoid/   installable package: robot config, env, mpc, RL tasks
assets/                             USD assets actually loaded by ArticulationCfg
references/                         URDF, CAD-derived docs, git submodules (IK, RL refs)
scripts/                            run_env.py: load + play the env (not part of the package)
tools/asset/                        URDF -> USD pipeline scripts (fix, flatten, check)
outputs/                            scratch / generated output, not source of truth
docs/PLAN.md                        human-facing roadmap for the MPC controller (not a task list)
```

`references/robinion_description` and the other `references/*` folders are **git
submodules** (see `.gitmodules`) — treat their contents as upstream/generated, not
project source to hand-edit, except where noted below.

### Package architecture (`source/MPC_Humanoid/MPC_Humanoid/`)

- `robots/robonionv2.py` — the single `ROBINION_CFG` `ArticulationCfg`: USD asset
  path, initial pose, and per-actuator-group `DCMotorCfg`/`ImplicitActuatorCfg`
  (legs, torso/arms, head, and the passive parallelogram joints). This is the one
  source of truth every env/task imports the robot from.
- `env/` — the plain (non-RL) `ManagerBasedEnvCfg` that the MPC controller will
  drive directly (`MpcHumanoidRobinionEnvCfg` in `robinion_env_cfg.py`): scene
  (ground + `ROBINION_CFG` + light), an `ActionsCfg` (joint-effort passthrough over
  the actuated joints only), and an `ObservationsCfg` with a single `state` group
  (base lin/ang vel, projected gravity, relative joint pos/vel). Not gym-registered
  — there is no training loop here, so it stays a `ManagerBasedEnvCfg`, not
  `ManagerBasedRLEnvCfg`. `observations` and `actions` are required (`MISSING`) on
  the base class, so any env cfg here must define both.
- `mpc/` — the controller stack (currently only a placeholder `controller.py`).
  Kept independent of `tasks/` — it's a plain module, not an RL task.
- `tasks/mpc_humanoid/` — Isaac Lab's generated RL-task tree (gym-registered,
  manager-based, `config/<variant>/` + shared `mdp/`). Currently only holds the
  generated cart-pole placeholder task (`config/cartpole/`); keep it isolated until a real Robinion RL/benchmark task is registered here, then
  retire it so it can't be run by accident as the humanoid task. **Don't put the MPC
  controller or its non-RL env here** — this tree is specifically for gym-registered
  RL tasks with reward/termination managers.

Never conflate `robots/` (asset + actuator config) with the USD asset's raw
inertial/collision data — `robonionv2.py` deliberately keeps them separate.

## The URDF → USD pipeline (assets/robonionv2.usd)

This is the asset `ROBINION_CFG` in `robots/robonionv2.py` loads. It is derived
from `references/robinion_description/robinion2.urdf` by a converter that only
understands trees, so a fixed set of physics facts (parallelogram loop joints,
floating base, foot inertia, etc.) must be re-authored by hand after every
conversion. **Read `references/docs/joint_info.md` in full before touching this
asset** — it documents every link/joint value, the parallelogram leg mechanism,
known defects in the URDF, and the exact pipeline below. Do not regenerate or
hand-edit `assets/robonionv2.usd` without reading it first.

```bash
uv run urdf_usd_converter references/robinion_description/robinion2.urdf assets/robonionv2 \
    -p robinion_description=$PWD/references/robinion_description   # 1. URDF -> raw USD
uv run python tools/asset/fix_robinion_usd.py assets/robonionv2/robinion.usda  # 2. hand-made physics fixes
uv run python tools/asset/flatten_usd.py assets/robonionv2/robinion.usda assets/robonionv2.usd  # 3. standalone asset
uv run python tools/asset/check_urdf_vs_usd.py assets/robonionv2.usd            # 4. verify against URDF+STL
```

Rules when working on this pipeline:
- `assets/robonionv2/Payload/*.usda` is raw converter output — never hand-edit it.
  All authored fixes go in `assets/robonionv2/robinion.usda` (the root layer), so a
  diff against a fresh conversion shows exactly what was added by hand.
- `assets/robonionv2.usd` is a generated, flattened artifact — regenerate it with
  `flatten_usd.py`, don't edit it directly.
- After *any* change to the URDF, the converter fixes, or the asset, run
  `tools/asset/check_urdf_vs_usd.py` and compare the SUMMARY against the expected one
  documented in `joint_info.md`. New/changed ERR or WARN lines mean something
  regressed — do not silently accept them.
- `tools/asset/fix_robinion_usd.py` is idempotent (safe to re-run on a file already
  partly edited by hand in the Isaac Sim GUI).

## When editing robot/task config

- `robots/robonionv2.py` deliberately keeps the actuator model (stiffness/damping/
  armature, estimated from the Dynamixel XH540-W270/XH430-W350 datasheets; effort/velocity limits
  taken verbatim from the URDF, which the user confirmed correct) separate from the
  USD asset's raw inertial/collision data — don't fold one into the other.
- The 4 parallelogram-passive joints (`*_knee_pitch_joint`, `*_back_thigh_pitch_joint`,
  `*_front_shin_pitch_joint`, `*_back_shin_pitch_joint`) are unmotored by design
  (`ImplicitActuatorCfg` with zero stiffness/damping/effort) — do not add a drive to
  them without confirming with the user first; see *Leg mechanism* in
  `joint_info.md` for why they are mechanically constrained, not free.

## Documentation conventions

- `references/docs/joint_info.md` is the living reference for link/joint physical
  data and the USD build pipeline; update it in the same change whenever you touch
  URDF numbers, the conversion scripts, or the fix list. Keep it in English.
- Numbers in that doc are sourced from the URDF/STL/CAD, not guessed — when adding
  a new fact, say where it came from (URDF field, mesh computation, datasheet).
