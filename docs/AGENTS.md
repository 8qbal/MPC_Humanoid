# AGENTS.md

Instructions for AI coding agents (Codex, etc.) working in this repository.

## What this project is

An Isaac Lab external project: an installable Python package (`source/MPC_Humanoid`)
plus assets and reference material for **Robinion**, a Dynamixel-actuated humanoid
with parallelogram-linkage legs. The long-term goal is an MPC-controlled robot. The
current phase is to build a correct, high-fidelity real-to-sim model first: an
MPC/RL-ready `ArticulationCfg` (`source/MPC_Humanoid/MPC_Humanoid/robots/robonionv2.py`)
backed by a USD asset that matches the robot's URDF/CAD exactly.

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
uv run python scripts/list_envs.py --show_presets        # list envs/presets
uv run isaaclab zero_agent --task <TASK_NAME>              # sanity-check an env
uv run isaaclab train --rl_library <LIB> --task <TASK_NAME>
uv run pre-commit run --all-files                           # lint/format (ruff, codespell, etc.)
```

Linting: `ruff` (line length 120, `E,F,I,UP,W`) + `ruff-format` + `codespell`, run
via pre-commit. Match existing code style (docstrings, type hints, `from __future__
import annotations`) rather than introducing a new one.

## Repository layout

```
source/MPC_Humanoid/MPC_Humanoid/   installable package: tasks, robot configs
assets/                             USD assets actually loaded by ArticulationCfg
references/                         URDF, CAD-derived docs, git submodules (IK, RL refs)
scripts/                            standalone utilities (not part of the package)
outputs/                            scratch / generated output, not source of truth
```

`references/robinion_description` and the other `references/*` folders are **git
submodules** (see `.gitmodules`) — treat their contents as upstream/generated, not
project source to hand-edit, except where noted below.

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
uv run python scripts/fix_robinion_usd.py assets/robonionv2/robinion.usda  # 2. hand-made physics fixes
uv run python scripts/flatten_usd.py assets/robonionv2/robinion.usda assets/robonionv2.usd  # 3. standalone asset
uv run python scripts/check_urdf_vs_usd.py assets/robonionv2.usd            # 4. verify against URDF+STL
```

Rules when working on this pipeline:
- `assets/robonionv2/Payload/*.usda` is raw converter output — never hand-edit it.
  All authored fixes go in `assets/robonionv2/robinion.usda` (the root layer), so a
  diff against a fresh conversion shows exactly what was added by hand.
- `assets/robonionv2.usd` is a generated, flattened artifact — regenerate it with
  `flatten_usd.py`, don't edit it directly.
- After *any* change to the URDF, the converter fixes, or the asset, run
  `scripts/check_urdf_vs_usd.py` and compare the SUMMARY against the expected one
  documented in `joint_info.md`. New/changed ERR or WARN lines mean something
  regressed — do not silently accept them.
- `scripts/fix_robinion_usd.py` is idempotent (safe to re-run on a file already
  partly edited by hand in the Isaac Sim GUI).

## When editing robot/task config

- `robots/robonionv2.py` deliberately keeps the actuator model (stiffness/damping/
  armature, derived from the Dynamixel XH540-W270/AX-12A datasheets) separate from the
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
