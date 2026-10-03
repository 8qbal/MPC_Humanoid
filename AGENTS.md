# AGENTS.md

Instructions for AI coding agents (Codex, etc.) working in this repository.

## What this project is

An Isaac Lab external project: an installable Python package (`source/MPC_Humanoid`)
plus assets and reference material for **Robonion**, a Dynamixel-actuated humanoid
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
uv run isaaclab zero_agent --task <TASK_NAME>              # sanity-check an env
uv run isaaclab train --rl_library <LIB> --task <TASK_NAME>
uv run pre-commit run --all-files                           # lint/format (ruff, codespell, etc.)
uv run python scripts/run_env.py --viz kit                  # load the Robonion env (headless without --viz)
uv run python scripts/build_controller.py                   # build the controller (C functions + acados solver), no Isaac
uv run python scripts/run_stepping.py --steps 20             # stage-2 stepping in place (crouch, calibrate, step)
uv run python scripts/run_stepping.py --push x+1.75          # one push mid single support while stepping
uv run python scripts/run_stepping_nocalib.py --viz kit      # same, spawned crouched, no AHRS calibration
```

Linting: `ruff` (line length 120, `E,F,I,UP,W`) + `ruff-format` + `codespell`, run
via pre-commit. Match existing code style (docstrings, type hints, `from __future__
import annotations`) rather than introducing a new one.

## Comments

Do not over-comment. Default to no comments — well-named code should speak for
itself. Only add a comment when the *why* is non-obvious: a hidden constraint, a
physical/mechanical fact (e.g. why a joint is passive), a workaround for a specific
upstream bug, or a numeric value whose source (URDF field, datasheet, mesh
computation) isn't otherwise documented nearby. Never comment on *what* the code
does, restate the function/variable name, or leave narration of the current task.

## Repository layout

```
source/MPC_Humanoid/MPC_Humanoid/   installable package: robots/ (robot config), env/ (plain env), mpc/ (controller), tasks/ (RL)
assets/                             USD assets (robonionv2.usd) + controller URDF (robonionv2_controller.urdf)
references/                         URDF, CAD-derived docs, git submodules (IK, RL refs)
scripts/                            run_env.py (load + play the env), build_controller.py (build the controller, no Isaac),
                                    run_stepping.py (stepping in place + push), run_stepping_nocalib.py (same, spawned
                                    crouched, no AHRS calibration), benchmark_stepping.py (replay a recorded run, no Isaac)
tools/asset/                        URDF -> USD pipeline scripts (fix, flatten, check), controller URDF export
outputs/                            scratch (git-ignored): acados build, stage-1/2 prototypes, validation scripts/data
docs/PLAN.md                        human-facing roadmap (not a task list); docs/stage1.md, stage2.md: stage records
                                    (stage records local only, git-ignored; last committed at git tag stage2)
```

`references/robinion_description` and the other `references/*` folders are **git
submodules** (see `.gitmodules`) — treat their contents as upstream/generated, not
project source to hand-edit, except where noted below.

## Controller (`mpc/`)

Stage 1 (standing) is done with the rigid-contact NMPC; `docs/stage1.md` records every
design tried (LIPM/DCM MPC + IK, flat-foot NMPC, soft-contact NMPC, rigid-contact NMPC) with
numbers. Stage 2 (stepping in place) is done on the same NMPC; `docs/stage2.md`
records the gait design, the gait rules and why, and every result. Read both before changing
the controller. The stage-1 standing controller (its 25-node NMPC and push script) is
retired: straight-leg standing only needs the servos to hold the pose,
and every active mode (balancing, stepping) runs at the crouched pose. Its code is at git tag
`stage2`; its results stay in `docs/stage1.md` as a record.

**Project state (stage 2 closed by the user on 2026-09-26).** Next, in this order: (1) the
Jetson AGX Orin tick benchmark, run by the user (`docs/stage2.md`, *Benchmark on the Jetson
AGX Orin*); its result decides whether the tick architecture must change (node count, horizon,
NMPC in its own thread); (2) a hardware check of the contact-mode monitor and the estimator
(standing, slow stepping), whose thresholds (0.2 deg, 15 ms lead, 2 ticks) were tuned in
Isaac only; (3) stage 3, footstep adaptation. Known issues, all listed under *Open* in
`docs/stage2.md`:
- Tick 4.2-4.3 ms mean / 4.6-4.8 ms p95 on the desktop, 2 % of ticks over 5 ms; Jetson unknown.
- Backward pushes while stepping: −x 1.75 N·s survived in 2 of 3 seeds, −x 2.0 falls. Both
  passed with 4 PhysX velocity iterations; the drop comes from the simulator fix (0
  iterations), not the controller, and was accepted by the user.
- After a push the gait holds the targets and waits (up to 3.6 s), no active recovery;
  lateral pushes beyond ±0.5 / +1.0 N·s need step placement (stage 3).
- Landing drift ~0.3 mm per step (needs pelvis yaw in the model); ground-height error
  beyond ±5 mm falls.
- All results come from Isaac only; limits within one push step are not precise.

- `model.py`: floating base (x, y, z, roll, pitch) + 9 independent joints
  (`MODEL_JOINTS`: hip roll, front thigh, ankle pitch, ankle roll per leg, torso pitch);
  passive joints from `PASSIVE_COUPLING`, yaw/arms/head fixed, mass matrix frozen at the
  standing pose. Contacts are rigid rows with a contact mode per foot (`flat`, `toe`,
  `heel`, `left_edge`, `right_edge`, `air`); the contact wrench is eliminated in closed
  form (position-mode servos leave it no freedom). In double support the two sole-pitch
  rows are identical (parallelograms), so the right one is switched off. Servo K/D/armature
  and effort limits come from `robots/robonion_params.py` (also used by `robonionv2.py`; no Isaac
  imports, so `mpc/` runs on the Jetson) — never duplicate them.
- `estimator.py`: `AhrsGyro`, `ContactModeMonitor` (contact mode per foot from sole tilt,
  tilt rate, sole height and the model-predicted normal force; there is no contact
  sensing), `ContactProjectionEstimator` (state on the contact manifold). `compiled=True`
  (used by `controller.py`) runs `RigidContactModel`, `AhrsGyro` and the estimator as generated C.
- `nmpc.py`: `RobonionNMPC`, acados SQP-RTI NMPC, outputs servo targets. The caller passes the
  plan per node (contact modes, DCM / CoM-velocity and swing-sole references); edge modes are
  never planned. Per-node references and gates are parameters, compiled contact algebra, RTI
  split (`prepare` / `feedback`), 16 nodes over 0.5 s. Cost weights are hardcoded constants
  by user decision (no config); `W_U = 0.1` was chosen over 10; swing weight 1e6 (1e4 barely
  lifted the foot); non-uniform node grids broke the tuning.
- `gait.py`: `GaitPlan` — contact schedule (T_ss 0.3 s, T_ds 0.1 s, clearance 2 cm; user
  decision, chosen against the 1.1 and 1.5-1.7 Hz modes and the servo rate limit), closed-form
  DCM/CoM reference, swing-sole spline, `stretch()` for retiming.
- `controller.py`: `RobonionController` — crouch ramp (0.2 rad; straight legs are singular and
  too slow for the swing), AHRS calibration, then the gait. `nominal_pose()` gives the crouched
  pose rounded to 6 decimals, so a float32 default pose builds the same model. Gait rules
  (peel-off, retiming on early/late touchdown, "catch" when the stance sole rolls in single
  support, hold + plan pause + posture reset + standing recovery in double support) are each
  justified by an Isaac failure in `docs/stage2.md`, and run as a state machine (`State`: STEP,
  HOLD, RESET, CATCH, RECOVER). Edge detection while stepping uses the predicted tilt
  (`MONITOR_LEAD` 15 ms: tilt + lead · rate beyond 0.2°) on 2 consecutive ticks
  (`MONITOR_TICKS`; a touchdown impact reads 12-19 deg/s for one tick). The asset runs with 0
  PhysX TGS velocity iterations (`robonionv2.py`): with 4, a flat sole read up to 3.6 deg/s of
  false tilt rate.
- Build: `scripts/build_controller.py` (no Isaac) generates and compiles the C functions, the
  contact algebra (`alg.so`) and the acados solver into `outputs/acados/`; runtime
  (`run_stepping.py`, `benchmark_stepping.py`) only loads them and raises `FileNotFoundError`
  when a build is missing. Every build is named by a hash of what it computes
  (`robonion_model_<hash>.so`, `robonion_ahrs_*`, `robonion_projection_*`, folder
  `robonion_nmpc_<hash>/` from the OCP expressions and numbers), so rerun the build script
  after changing the model, the NMPC or the default pose; a stale build is never loaded.
  acados needs `ACADOS_SOURCE_DIR=$HOME/acados` and `LD_LIBRARY_PATH=$HOME/acados/lib`.
- Test in Isaac with **one env per process, runs in sequence**: multi-env runs of this
  asset give different outcomes, and parallel Isaac processes can crash on `sim.reset()`.
  Compare every push against the push tables for seeds 0, 1, 2 in `docs/stage2.md`, one push
  per run, and run each case with seeds 0, 1, 2 (`--seed`, the sensor-noise realization):
  results near a limit flip between seeds.
- Validation tools and data stay in `outputs/stage1_prototypes/` (git-ignored, user
  decision): `validate3.py` (model + estimator vs `ident.npz` / `truth_*.npz`),
  `nmpc3_offline.py` (closed loop on the hybrid rigid-contact plant), `isaac_nmpc3.py
  --truth_state` (NMPC fed with simulator truth). They run from that folder with
  `uv run --project ../.. python <script>`. `model3.py` there is the validated prototype of
  `mpc/model.py`; re-check the package model against it after model changes. Stage-2
  prototypes and test inputs (ground-height error, edge policies, `--truth_state`, traces) are
  in `outputs/stage2_prototypes/` (`gait_offline.py`, `isaac_gait.py`), run the same way.
  The prototypes in both folders import the package modules as they were before the rename to
  `nmpc.py` / `controller.py` / `robonion_params.py`, so they only run with the package at git
  tag `stage2` (check that tag out first).
- Open: a stepping tick costs 4.2-4.3 ms mean / 4.6-4.8 ms p95 in Isaac on the desktop (compiled
  estimation + NMPC), near the 5 ms budget; the real robot runs on a Jetson AGX Orin (not
  measured: `scripts/run_stepping.py --record run.npz` on the desktop, then
  `scripts/build_controller.py` and `scripts/benchmark_stepping.py run.npz` on the Jetson, no
  Isaac needed). Lateral push
  recovery while stepping needs footstep adaptation (stage 3). Landing drift ~0.3 mm per step
  needs pelvis yaw in the model (walking / turning stage).

## The URDF → USD pipeline (assets/robonionv2.usd)

This is the asset `Robonion_CFG` in `robots/robonionv2.py` loads. It is derived
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
