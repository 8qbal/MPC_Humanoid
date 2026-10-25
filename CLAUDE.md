# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
uv run isaaclab zero_agent --task <TASK_NAME> --num_envs 16   # sanity-check an env, no policy
uv run isaaclab random_agent --task <TASK_NAME> --num_envs 16
uv run isaaclab train --rl_library <RL_LIBRARY> --task <TASK_NAME>
uv run isaaclab play --rl_library <RL_LIBRARY> --task <TASK_NAME> --checkpoint latest
uv run isaaclab train_multigpu --rl_library <RL_LIBRARY> --task <TASK_NAME> --num_gpus 2
uv run isaaclab benchmark runtime --task <TASK_NAME> --num_envs 16 --num_steps 1000
uv run isaaclab benchmark training --rl_library <RL_LIBRARY> --task <TASK_NAME> --max_iterations 10
uv run pre-commit run --all-files                              # lint/format (ruff, codespell, etc.)
uv run python scripts/run_env.py --viz kit                    # load the Robonion MPC env and play it (headless without --viz)
uv run python scripts/build_controller.py                     # build the controller (C functions + acados solver), no Isaac
uv run python scripts/run_stepping.py --steps 20                # stage-2 stepping in place (crouch, calibrate, step)
uv run python scripts/run_stepping.py --push x+1.75            # one push mid single support while stepping
uv run python scripts/run_stepping_nocalib.py --viz kit        # spawned crouched, no AHRS calibration (holds: bias > 0.2 deg)
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
assets/                             USD assets (robonionv2.usd) + controller URDF (robonionv2_controller.urdf)
references/                         URDF, CAD-derived docs, git submodules (IK, RL refs)
scripts/                            run_env.py (load + play the env), build_controller.py (build the controller, no Isaac),
                                    run_stepping.py (stepping in place, walking, pushes), run_stepping_nocalib.py (spawned
                                    crouched, no AHRS calibration), benchmark_stepping.py (replay a recorded run, no Isaac)
tools/asset/                        URDF -> USD pipeline scripts (fix, flatten, check), controller URDF export
outputs/                            scratch (git-ignored): acados build, stage-1/2 prototypes, validation scripts/data
docs/PLAN.md                        human-facing roadmap for the MPC controller (not a task list)
docs/stage1.md                      stage-1 record: the four designs tried, equations, validation, push tables
docs/stage2.md                      stage-2 record: gait design, gait rules, tick time, push tables
                                    (both stage records are local only, git-ignored; last committed at git tag stage2)
```

`references/robinion_description` and the other `references/*` folders are **git
submodules** (see `.gitmodules`) — treat their contents as upstream/generated, not
project source to hand-edit, except where noted below.

### Package architecture (`source/MPC_Humanoid/MPC_Humanoid/`)

- `robots/robonionv2.py` — the single `Robonion_CFG` `ArticulationCfg`: USD asset
  path, initial pose (`DEFAULT_JOINT_POS` from `robots/robonion_params.py`), and
  per-actuator-group `DCMotorCfg`/`ImplicitActuatorCfg` (one group per servo type and servo count, and
  the unpowered parallelogram back bars). This is the one
  source of truth every env/task imports the robot from.
- `env/` — the plain (non-RL) `ManagerBasedEnvCfg` that the MPC controller drives
  directly (`MpcHumanoidRobonionEnvCfg` in `robonion_env_cfg.py`, sim dt 1 ms,
  decimation 5 = 200 Hz): scene (ground + robot + IMU/AHRS sensors + light), an
  `ActionsCfg` with `joint_pos` (`JointPositionActionCfg` over the actuated joints, targets
  relative to the default pose), and an `ObservationsCfg` with a single `state` group:
  gyro, accelerometer and AHRS orientation (with the MTi-630 noise model), relative joint
  pos/vel, and simulator-only base lin/ang vel and projected gravity, which the controller
  must not use (the real robot has no such measurement). Not gym-registered
  — there is no training loop here, so it stays a `ManagerBasedEnvCfg`, not
  `ManagerBasedRLEnvCfg`. `observations` and `actions` are required (`MISSING`) on
  the base class, so any env cfg here must define both.
- `mpc/` — the controller stack (model, estimator, NMPC, gait plan, controller; see
  *Controller* below). Kept independent of `tasks/` — it's a plain module, not an RL task.
- `tasks/` — entry point for gym-registered Isaac Lab RL tasks (`import_packages` in `tasks/__init__.py`,
  `isaaclab.tasks` entry point in `pyproject.toml`). Empty: the generated cart-pole placeholder was removed on
  2026-10-22. **Don't put the MPC controller or its non-RL env here** — this tree is specifically for
  gym-registered RL tasks with reward/termination managers.

Never conflate `robots/` (asset + actuator config) with the USD asset's raw
inertial/collision data — `robonionv2.py` deliberately keeps them separate.

## Controller (`mpc/`)

Stage 1 (standing) is done with the rigid-contact NMPC; `docs/stage1.md` records every
design tried (LIPM/DCM MPC + IK, flat-foot NMPC, soft-contact NMPC, rigid-contact NMPC) with
numbers. Stage 2 (stepping in place) is done on the same NMPC; `docs/stage2.md`
records the gait design, the gait rules and why, and every result. Read both before changing
the controller. The stage-1 standing controller (its 25-node NMPC and push script) is
retired: straight-leg standing only needs the servos to hold the pose,
and every active mode (balancing, stepping) runs at the crouched pose. Its code is at git tag
`stage2`; its results stay in `docs/stage1.md` as a record.

**Project state (stage 2 closed by the user on 2026-09-26; stage 3 set on 2026-10-04, widened
2026-10-07 and 2026-10-09, closed by the user on 2026-10-22; stage 4 not decided yet).** Stage 3 is base
position estimation, the whole body in the NMPC, and walking. Closed with: 40 steps in place clean, straight
walking clean up to 0.15 m/s, ±1.0 N·s pushes while walking fall in 1 of 24 runs at 0.05 and at 0.1 m/s; after a
push the gait holds and restarts (no recovery step). Next proposed: servo identification before fast walking. Working notes and every result: `docs/stage3.md` (local, git-ignored); the summary is in
`docs/PLAN.md`, *Stage 3*. Branch `feat/movement-gait`.
- Estimation:
  - The supervisor rejected the stage-2 base x, y, z: `ContactProjectionEstimator` pins the contact
    points (`cref`) in the world, so any foot slip becomes a permanent position error.
  - The controller now takes base x, y, z and velocity from the leg-kinematics + IMU EKF of
    Bloesch et al. 2012 (`ekf.py`).
- Tick time: the NMPC runs every second tick (`NMPC_EVERY = 2`, 100 Hz).
  - The tick stays 5 ms. Estimation, monitor and gait rules stay at 200 Hz: the user kept all the
    5 ms tuning.
  - A plain 10 ms tick is the fallback only if the whole-body NMPC does not fit.
  - 50 Hz was tried and rejected (falls, slip).
- Whole body: pelvis yaw, hip yaw and the 8 arm joints in the NMPC, for balance and yaw damping. Built by
  session a3 on `feat/arms-yaw`, merged 2026-10-10 (51adb38).
  - Arm targets as NMPC inputs with a servo-error cost, a ground yaw-moment cost (`W_MZ`), Radau with 2 stages
    on the first 8 intervals and 1 on the rest (1 stage everywhere rolled the soles onto edges in Isaac), HPIPM
    SPEED_ABS with tolerances 1e-5.
  - Base yaw comes from the EKF; `--no_arms` / `--no_hip_yaw` hold those joints at the default pose on the same
    build.
  - Isaac push table at 7485037: full 15/15, yaw + arms 14/15, yaw only 15/15. Tick 4.0-4.1 ms mean / 4.9-5.1 ms
    p95.
- Servos per servo type (2026-10-13, 09807f2; see *When editing robot/task config*): Isaac full 13/15, +x 1.75
  passes 1/3. The bisect found no single cause (new limits, new K/D or the 3.14 rad/s torso rate alone each drop
  one seed); +x 1.75 sits at the limit. Re-check once the measured servo parameters are in.
- EKF drift against the Isaac truth (2026-10-12): x, y ≤ 1-3 mm over 11-24 s with up to 74 mm foot slip, z
  -0.07 mm/s. A 40-step run without push falls after ~29 clean steps (stance sole rolls in single support): a
  gait issue, open.
- Upstream `robinion_description` 55ac2c2 adopted (2026-10-15, 6e2e9c5..2f225f4): the URDF is copied to
  `assets/robonionv2.urdf` and the asset is built from it; servo parameters and map follow its
  `config/actuators.yaml` (14.8 V, armature 0.038 as a BAM prior, servos on knee and front shin). The 0.038
  armature needed T_ds 0.2 s and `MONITOR_TICKS` 3; Isaac push table 14/15 (+x 1.75 falls on seed 1, the other
  two hold 5-8 s), slip without push 3-4 mm per 10 steps (was ≤ 1.5 mm). Bisect and diagnosis in
  `docs/stage3.md`.
- Walking: straight walking at a commanded speed with an ALIP footstep planner (user decision
  2026-10-09), built on `feat/movement-gait` (9a50d8f, 2cfe9f8, ad56ba0; `--vx`, `--ramp_steps` in
  `run_stepping.py`). `mpc/alip.py` gives the step change du about the plan; the controller removes a learned
  offset per swing foot (`ALIP_BINS` per swing progress) and uses du as step-to-step feedback. After a hold it
  restarts with a new plan from the current soles.
  - `W_DCM = 1000` (was 100, ad56ba0): at 100 the servo-rate cost kept the planned DCM 19 mm RMS behind the walking
    reference. Battery of 51 runs: falls 1/51 (was 2/51), no holds walking without push, holds after walking pushes
    halved; in-place x-1.0 now holds up to 7.3 s (no fall).
  - Speed limit 0.15 m/s (user, 2026-10-18): clean on 6/6 runs. At 0.2 m/s 3/6 stop and restart, at 0.25 m/s
    holds and one fall: the soles roll to an edge at weight transfer and in single support. Tracking harder
    (runtime weights) made it worse.
  - Capture steps (a step adapted to a push within the current swing) were tried twice and reverted.
- The user dropped the Jetson AGX Orin tick benchmark (the Jetson is assumed to keep up).
- Later:
  - a hardware check of the contact-mode monitor and the estimator, whose thresholds (0.2 deg,
    15 ms lead, 3 ticks) were tuned in Isaac only;
  - stage 4: fast walking and turning.

Known issues from stage 2, listed under *Open* in `docs/stage2.md`:
- Stage-2 tick: 4.2-4.3 ms mean / 4.6-4.8 ms p95 on the desktop. With the 100 Hz NMPC and the
  compiled EKF: 2.34 / 2.96 ms (offline replay).
- Backward pushes while stepping: −x 1.75 N·s survived in 2 of 3 seeds, −x 2.0 falls. Both
  passed with 4 PhysX velocity iterations; the drop comes from the simulator fix (0
  iterations), not the controller, and was accepted by the user.
- After a push the gait holds the targets and waits (up to 3.6 s), no active recovery;
  lateral pushes beyond ±0.5 / +1.0 N·s need step placement (ALIP, stage 3).
- Landing drift ~0.3 mm per step (needs pelvis yaw in the model); ground-height error
  beyond ±5 mm falls.
- All results come from Isaac only; limits within one push step are not precise.

- `model.py`: floating base (x, y, z, roll, pitch, yaw) + 9 independent joints
  (`MODEL_JOINTS`: hip roll, front thigh, ankle pitch, ankle roll per leg, torso pitch), hip yaw
  (`HIP_YAW_JOINTS`) and the 8 arm joints (`ARM_JOINTS`); yaw, hip yaw and arms can be built off (the
  stage-2 model). Parallelogram joints follow `PASSIVE_COUPLING` (`robots/robonion_params.py`); the
  servos on knee and front shin add to the coordinate they follow (`drive_of`). Head fixed, mass matrix frozen at the
  standing pose. Contacts are rigid rows (6 per sole with yaw) with a contact mode per foot (`flat`, `toe`,
  `heel`, `left_edge`, `right_edge`, `air`); the contact wrench is eliminated in closed
  form (position-mode servos leave it no freedom). In double support the two sole-pitch
  rows are identical (parallelograms), so the right one is switched off. Servo K/D/armature
  and effort limits come from `robots/robonion_params.py` (also used by `robonionv2.py`; no Isaac
  imports, so `mpc/` runs on the Jetson) — never duplicate them.
- `estimator.py`: `AhrsGyro`, `ContactModeMonitor` (contact mode per foot from sole tilt,
  tilt rate, sole height and the model-predicted normal force; there is no contact
  sensing), `ContactProjectionEstimator` (state on the contact manifold). `compiled=True`
  (used by `controller.py`) runs `RigidContactModel`, `AhrsGyro` and the estimator as generated C.
- `ekf.py`: `LegKinematicsEkf` (Bloesch et al. 2012): base position, velocity and orientation, and one
  contact point per sole. Its inputs are the IMU, the leg kinematics, the AHRS tilt and the monitor's
  contact modes. `compiled=True` (used by `controller.py`) generates the leg kinematics and their
  Jacobian as C (`robonion_ekf_<hash>.so`).
- `nmpc.py`: `RobonionNMPC`, acados SQP-RTI NMPC, outputs servo targets. The caller passes the
  plan per node (contact modes, DCM / CoM-velocity and swing-sole references); edge modes are
  never planned. Per-node references and gates are parameters, compiled contact algebra, RTI
  split (`prepare` / `feedback`), 16 nodes over 0.5 s. Cost weights are hardcoded constants
  by user decision (no config); `W_U = 0.1` was chosen over 10; swing weight 1e6 (1e4 barely
  lifted the foot); non-uniform node grids broke the tuning.
- `gait.py`: `GaitPlan` — contact schedule (T_ss 0.3 s, T_ds 0.2 s, clearance 2 cm; user
  decisions: 0.3/0.1 s chosen against the 1.1 and 1.5-1.7 Hz modes and the servo rate limit, T_ds 0.2 s since
  2026-10-15 because 0.1 s slid the soles apart at weight transfer with the 0.038 armature), closed-form
  DCM/CoM reference, swing-sole spline, `stretch()` for retiming.
- `controller.py`: `RobonionController` — crouch ramp (0.2 rad; straight legs are singular and
  too slow for the swing), AHRS calibration, then the gait. `nominal_pose()` gives the crouched
  pose rounded to 6 decimals, so a float32 default pose builds the same model. Gait rules
  (peel-off, retiming on early/late touchdown, "catch" when the stance sole rolls in single
  support, hold + plan pause + posture reset + standing recovery in double support) are each
  justified by an Isaac failure in `docs/stage2.md`, and run as a state machine (`State`: STEP,
  HOLD, RESET, CATCH, RECOVER). Edge detection while stepping uses the predicted tilt
  (`MONITOR_LEAD` 15 ms: tilt + lead · rate beyond 0.2°) on 3 consecutive ticks
  (`MONITOR_TICKS`; a touchdown impact reads 12-19 deg/s for one tick, and with the 0.038 armature the
  0.1-0.15° sole wobble after touchdown lasts 2 ticks). The asset runs with 0
  PhysX TGS velocity iterations (`robonionv2.py`): with 4, a flat sole read up to 3.6 deg/s of
  false tilt rate. `NMPC_EVERY = 2`: feedback on every second tick, RTI preparation on the tick
  between. Targets between the two follow u + v·dt. When node 0's contact modes change, the plan is
  retimed or the NMPC restarts after a hold, that tick only prepares and the feedback follows on the
  next one (at most one deferral in a row; a full solve does not fit the tick). No peel-off while the
  targets are held (HOLD, RESET, CATCH).
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
- Open:
  - Tick with the whole-body NMPC: 4.0-4.1 ms mean / 4.9-5.1 ms p95 in Isaac on the desktop.
    - After +x 1.75 N·s the p95 reaches 7.4-7.8 ms in some runs; not investigated.
    - yaw + arms (`--no_hip_yaw`) fell once at +x 1.75 (seed 1); after +x 1.75 the gait holds for 3-7 s.
    - The real robot runs on a Jetson AGX Orin (not measured, benchmark dropped by the user). If
      needed: `scripts/run_stepping.py --record run.npz` on the desktop, then
      `scripts/build_controller.py` and `scripts/benchmark_stepping.py run.npz` on the Jetson, no
      Isaac needed.
  - Lateral push recovery while stepping needs footstep placement (ALIP, stage 3).
  - Landing drift (~0.3 mm per step in stage 2): with the whole body the foot moves ≤ 1.2 mm over 10 steps
    without a push; not measured per step.
- The laptop is shared with other projects: run Isaac and other heavy jobs under
  `flock /tmp/laptop_heavy.lock <command>` (agreed with `slam_project`, 15 GB RAM).

## The URDF → USD pipeline (assets/robonionv2.usd)

This is the asset `Robonion_CFG` in `robots/robonionv2.py` loads. It is derived
from `references/robinion_description/urdf/robinion2.urdf` by a converter that only
understands trees, so a fixed set of physics facts (parallelogram loop joints,
floating base, foot inertia, etc.) must be re-authored by hand after every
conversion. **Read `references/docs/joint_info.md` in full before touching this
asset** — it documents every link/joint value, the parallelogram leg mechanism,
known defects in the URDF, and the exact pipeline below. Do not regenerate or
hand-edit `assets/robonionv2.usd` without reading it first.

```bash
uv run urdf_usd_converter references/robinion_description/urdf/robinion2.urdf assets/robonionv2 \
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

- `robots/robonionv2.py` deliberately keeps the actuator model separate from the USD asset's raw
  inertial/collision data — don't fold one into the other. The actuator model is one `DCMotorCfg`
  per servo type and servo count, from `SERVOS` / `SERVO_JOINTS` in `robots/robonion_params.py`, which follow
  the upstream URDF and `config/actuators.yaml` (user, 2026-10-15): XH540-W270 on hip roll, front thigh (2
  servos), knee (2), front shin, ankle pitch and ankle roll; XM540-W270 on hip yaw, torso pitch, shoulders and
  elbow pitch; XM430-W350 on elbow yaw and the head. 14.8 V figures, K = stall / 0.217 rad, armature and
  friction BAM priors until the servos are identified. `mpc/model.py` and `mpc/nmpc.py` take the same values.
- Knee and front shin carry servos (user, 2026-10-15, superseding "5 motors per leg"); their targets follow
  front thigh and ankle pitch through the parallelogram gain (`env/actions.py`, `CoupledJointPositionAction`),
  so each leg keeps 2 pitch DOF. The back bars (`*_back_thigh_pitch_joint`, `*_back_shin_pitch_joint`) are
  unpowered (`ImplicitActuatorCfg` with zero stiffness/damping/effort); see *Leg mechanism* in
  `joint_info.md`.

## Documentation conventions

- `references/docs/joint_info.md` is the living reference for link/joint physical
  data and the USD build pipeline; update it in the same change whenever you touch
  URDF numbers, the conversion scripts, or the fix list. Keep it in English.
- Numbers in that doc are sourced from the URDF/STL/CAD, not guessed — when adding
  a new fact, say where it came from (URDF field, mesh computation, datasheet).
