# Stage 3 — base position estimation: working notes

Discussion document for branch `iqbal/cleanup-verify`.  The proposal is in [`PLAN.md`](PLAN.md), *Stage 3*; this file records what the code does now and the questions still open.

## Status

| | State |
|---|---|
| EKF | `mpc/ekf.py`, `LegKinematicsEkf` (Bloesch et al. 2012), commits `8bc030e`, `e292001` |
| Controller | already uses the EKF base x, y, z and velocity (`controller.py`, `_estimate`); PLAN.md and CLAUDE.md still say the stage-2 estimator stays until the offline results are in |
| Validation | standing only: `outputs/stage3_prototypes/ekf_standing.py`, squat 0.08 rad + sway 0.05 rad, 10 s, seed 0, both soles flat, no slip, no stepping |
| Not done | PLAN.md validation steps 1–3: recorded stepping run, offline replay, seeds 0, 1, 2, one push |

Standing result (seed 0): EKF error x ≈ −0.8 mm, y ≈ −2.5 mm, z ≈ 0 after 10 s, inside ±3σ (≈ ±8 mm at the end).  Stage-2 estimator peaks: 1.7 mm in x, 9.5 mm in y.

## Estimation per tick

Four classes, created in `controller.py:123-127`:

| Class | File | Gives | Needed for |
|---|---|---|---|
| `AhrsGyro` | `estimator.py:36` | pelvis roll, pitch and rates | NMPC state, monitor, calibration |
| `ContactModeMonitor` | `estimator.py:145` | contact mode per foot (no contact sensing) | EKF, gait logic, NMPC node 0 |
| `ContactProjectionEstimator` | `estimator.py:272` | `q`, `qd` that satisfy the active contact rows `c(q) = cref` | NMPC (rigid-contact model) |
| `LegKinematicsEkf` | `ekf.py:43` | base position, velocity, orientation; foot contact points | base x, y, z of the NMPC state |

```text
AhrsGyro -> roll, pitch, rates (minus calibration bias)
EKF      -> base x, y, z, velocity, foot points p_i          (contact modes of the previous tick)
Monitor  -> contact modes
cref[feet in contact] <- p_i  ->  Projection -> q, qd ;  q[:3], qd[:3] <- EKF
Monitor.release -> feet with too little predicted normal force -> air
gait logic -> NMPC
```

Projection weights (`estimator.py:293`): base x, y, z 1e-8, roll/pitch 0.01, joints 1.  The contact mismatch is therefore absorbed mostly by base x, y, z, which the EKF then overwrites; what remains of the projection is the roll/pitch correction and `qd`.  Whether the state handed to the NMPC still satisfies `c(q) = cref` has not been measured.

### NMPC state (37) and its sources

`x = [q (14), qd (14), u (9)]` (`nmpc.py:126`), set every tick from `x0 = [q, qd, u_prev]` (`nmpc.py:342`).

| Index | State | Source |
|---|---|---|
| 0–2 | base x, y, z | EKF |
| 3–4 | roll, pitch | `AhrsGyro` − bias, then projection |
| 5–13 | 9 joints | encoders, through projection |
| 14–16 | base velocity | EKF |
| 17–18 | roll, pitch rate | gyro through `AhrsGyro`, then projection |
| 19–27 | joint velocities | servo velocities, through projection |
| 28–36 | servo targets `u` | the controller's own output of the previous tick (exact) |

Contact modes are not state: they enter the NMPC as per-node parameters (`flags`, `pc`, `contact`, `isflat`).

## EKF against PLAN.md and the paper

Same as the paper: state, IMU prediction, first-estimate (observability-constrained) Jacobians, Joseph-form update, foot covariance reset on touchdown, bias estimation off.

Differences:
1. AHRS yaw is an extra measurement (`_correct`, last row).  The paper leaves yaw unobservable; neither PLAN.md nor the module docstring lists it.
2. The contact point follows the contact mode (sole centre, toe, heel, edge), not one sole-centre point as in PLAN.md.  Listed in the module docstring.
3. The EKF gets the contact modes of the previous tick (`controller.py:171`, `self.status.modes`): touchdown and lift-off arrive 5 ms late.
4. Leg kinematics are computed twice: CasADi in `model.py` (9 model joints, others frozen at default, compiled) and Pinocchio in `ekf.py` (all actuated joints, Python, Jacobian by finite differences, ~1 + n_act forward-kinematics calls per tick).

## Observability

From the paper (`references/papers/bloesch2012_rss.pdf`):
- §IV-A: "The four dimensional manifold composed of robot position and yaw angle … is always unobservable."
- Hardware, Fig. 3: "All three positions are affected by some drift, amounting up to 10 % of the traveled distance", from "inaccurate leg kinematics and the fault-prone contact detection".

Encoders measure `p_i − r`, the IMU measures accelerations and rates: shifting the body and every foot together, or rotating them about gravity, leaves every reading unchanged.  The target is a small drift with an honest covariance, not zero drift.

For control, the NMPC model is invariant to x, y translation and only uses relative quantities (base against `cref`, DCM against the support).  As long as the base and `cref` come from the same estimate they drift together.  Absolute drift matters for placing feet at world targets, and z matters through the flat-ground assumption (ground-height error beyond ±5 mm falls, `stage2.md`).

## Foot slip

Slip is only process noise on a foot in contact: `slip_density = 1e-6` m²/s, ≈ 1 mm σ after 1 s.

| Noise | EKF | Env (`robonion_env_cfg.py`) as a density |
|---|---|---|
| Accelerometer | `accel_density = 0.1` (m/s²)²/Hz | std 0.008 at 200 Hz → 3.2e-7 |
| Gyro | `gyro_density` = 1.5e-8 rad²/s | std 0.002 at 200 Hz → 2e-8 |

The accelerometer is trusted ~3·10⁵ times less than its simulated noise, the foot positions much more than the base.  A real slip then goes almost entirely into the base position, as with the stage-2 pinning.  Only the IMU, over short windows, can separate slip from base motion.  Not tested: the standing run has no slip.

## Measurements on the real robot

The simulation is cleaner than the hardware:

| Signal | Isaac now | Real robot |
|---|---|---|
| Joint position | exact, no noise | Dynamixel 12-bit encoder (0.088°), horn mounting offsets |
| Joint velocity | exact | `Present Velocity`, 0.229 rpm ≈ 0.024 rad/s units, noisy; filtering adds lag |
| AHRS roll/pitch | truth + white noise 0.02° + constant bias (calibrated out) | MTi-630 error grows with accelerations and touchdown impacts, correlated with the motion |
| AHRS yaw | truth + noise | depends on the magnetic field near motors and steel |
| Gyro | truth + white noise | close; plus servo vibration and impacts |
| Accelerometer | instantaneous PhysX sample (0 velocity iterations, spikes at impacts) | vibration, impacts, mounting |
| Timing | all signals simultaneous, no delay | servos over RS-485, IMU over USB, separate clocks, bus read/write time per tick |

The contact-mode monitor flags an edge at 0.2° of predicted sole tilt (`MONITOR_LEAD` 15 ms, `MONITOR_TICKS` 2).  The sole tilt comes from the AHRS roll/pitch plus the leg joints, so a dynamic AHRS error of that size, encoder quantisation and joint-velocity noise act directly on that threshold.

## Open questions

1. Keep the EKF wired into the controller, or go back to the stage-2 estimator until validation (and update PLAN.md / CLAUDE.md accordingly)?
2. Projection: lock base x, y, z to the EKF and project only roll/pitch and `qd` (option 1), or drop the projection (option 2, untested with the NMPC)?
3. One source for the leg kinematics: a CasADi function in `model.py` (contact points in the IMU frame + Jacobian), compiled, used by the EKF?
4. Noise tuning: `accel_density` near the real accelerometer noise (with a margin for impacts) and a larger `slip_density`; sweep both on a recorded run.
5. AHRS yaw as a measurement: keep, and write it down as a difference from the paper, or remove?
6. Contact modes of the current tick for the EKF instead of the previous one?
7. Make the simulation less clean before the hardware check: encoder quantisation, joint-velocity noise, latency, correlated AHRS error.
8. Validation as in PLAN.md: record stepping runs (seeds 0, 1, 2, one push, one with slip), replay offline, drift of x, y, z against the truth and the stage-2 estimator, error inside 3σ.

## Project audit (2026-10-06)

A review of the whole project, from the first commit (`a5dcf09`) to `c2e3dd9` plus the uncommitted working tree. **Nothing has been changed.** Every fix below is only a proposal. Each one is applied only after the user approves it, and each one comes with the paper(s) it rests on (*References* at the end). Process and repository items rest on a project rule instead of a paper.

Method:
- Read: every module in `mpc/`, `robots/`, `env/`, `scripts/`, the package `__init__`, the git history, the working-tree diff (including the `references/robinion_description` submodule), and the stage docs.
- Run (read-only):
  - `tools/asset/check_urdf_vs_usd.py`: the SUMMARY matches the one expected in `joint_info.md` (7 INFO, 2 WARN).
  - Loaded the current controller build with `build=False`: it loads, so it is up to date.
  - Timed `LegKinematicsEkf.update` alone.
- Not run: Isaac (no stepping run with the current code), and `ruff` (`uv run ruff` fails because ruff is not in `.venv`; pre-commit installs its own copy).

Status words: **verified** = checked by running or by an exact reading of the code; **suspected** = follows from the code but was not measured.

### Count

| Severity | Count | IDs |
|---|---|---|
| High: invalidates results, or a hardware risk | 4 | A1, B1, B3, F1 |
| Medium: wrong or unmeasured behaviour, or a validation gap | 12 | A2, A3, B2, C1, C2, E1, F2, F3, F4, F5, G1, H1 |
| Low: cosmetic, or not observed so far | 10 | A4, D1, E2, F6, F7, G2, G3, H2, H3, I1 |
| **Total** | **26** | |

C1, F4 and F7 are open questions 7, 4 and 5 above, with more detail. Only one of the 26 is a verified wrong code path (H2). F1 is a suspected one. The rest are model gaps, untested assumptions, stale documents and missing reports. `gait.py` (`GaitPlan`) had no finding.

### A. Repository and process

**A1 (High, verified): the asset pipeline reads a URDF that git does not track, and the tracked copy was edited by hand.**
- `CLAUDE.md`, `joint_info.md`, `robonion_params.py:3` and `check_urdf_vs_usd.py:33` (default `--urdf`) all use `references/robinion_description/robinion2.urdf`. That file is *untracked* in the submodule (`git status`: `?? robinion2.urdf`), so a fresh clone does not have it.
- The submodule tracks `urdf/robinion2.urdf`. It carries an uncommitted local edit: `left_foot_roll_link` `izz −0.000023 → 0.0`.
- That edit breaks two project rules:
  - corrections go into the USD overrides, never into `references/`;
  - a negative moment is replaced with a physical positive value, not 0. PhysX rejects 0, and the USD uses 5.0e-4 (fix 4).
- Options:
  1. revert the submodule edit and point the pipeline at the tracked `urdf/robinion2.urdf` (the two files differ only by that edit);
  2. keep the root copy, but say where it comes from (generated from `urdf/robinion2.xacro`?).
- Rule: `CLAUDE.md` / memory.

**A2 (Medium, verified): the code as committed has never stepped in Isaac.**
- `8bc030e` put the EKF in the loop.
- `c2e3dd9` raised the left front-thigh effort from 9.9 to 19.8 N·m, in both the actuator config and the NMPC effort bound (so the build hash changed too).
- The only stage-3 run is the standing prototype. The stage-2 push tables and tick times therefore describe a different controller and a different simulated robot.
- Proposal: before any further change, rerun the stage-2 baseline with the current code (no push; the push table; seeds 0, 1, 2; one env per process). That gives every later fix a reference to compare against.
- Rule: `CLAUDE.md` (*Test in Isaac*).

**A3 (Medium, verified): the documents contradict the code.**
- `CLAUDE.md` (*Project state*) and `PLAN.md` (*Stage 3*) say the EKF is "not started" and that the controller keeps the stage-2 estimator, but `controller.py` uses the EKF.
- `CLAUDE.md` (*Controller*) describes the estimation chain without `ekf.py`.
- `stage2.md:213` and `stage2.md:235` still say lateral push recovery is "stage 3"; it is now stage 4.
- This waits for open question 1: the text depends on whether the EKF stays in the loop.

**A4 (Low, verified by reading): the uncommitted `env/robonion_env_cfg.py` edit breaks the lint rules and the docstring.**
- Trailing whitespace on lines 52 and 88 (ruff W291; ruff-format would rewrite them).
- The blank lines between classes were removed.
- The module docstring now reads "…directly / plain ``ManagerBasedEnvCfg``", a broken sentence.
- The TODO on line 126 has an unclosed parenthesis.
- Proposal: run pre-commit before committing (the user runs it).

Info, not counted: the working-tree change to `assets/robonionv2_controller.urdf` (header, left thigh 19.8) is a correct regeneration by `export_controller_urdf.py`. The committed copy was the stale one. The controller does not read effort from the URDF.

### B. `robots/` (`robonion_params.py`, `robonionv2.py`)

**B1 (High, hardware question): 19.8 N·m on the front thigh is twice what one XH540-W270 can deliver.**
- XH540-W270 stall torque [R1]: 9.9 N·m at 12.0 V, 9.2 N·m at 11.1 V. No-load speed: 39 rpm (4.08 rad/s) at 12 V, 36 rpm (3.77 rad/s) at 11.1 V. So 19.8 = 2 × 9.9.
- Either the front thigh has two servos or a 2:1 transmission, or the URDF value 19.8 (right) is the typo and the left 9.9 is correct.
- Two servos: K, D and armature of that joint should double. A 2:1 gear: the torque doubles, the speed halves and the armature goes up 4×. The config uses single-servo values (K 42, D 2.4, armature 0.003, 4.08 rad/s) either way.
- With one servo, the NMPC allows 0.8 × 19.8 = 15.8 N·m and Isaac saturates at 19.8. Both are optimistic, and in single support the thigh is the most loaded joint.
- Proposal: check on the robot how the front thigh is driven, then set effort, K, D, armature and speed together. Basis: actuator identification for sim-to-real [P9, P8].

**B2 (Medium, verified against the datasheets): the URDF limits do not match one servo model or one voltage.**
- `legs` (9.9 N·m, 4.08 rad/s) are the 12 V figures. `XH540_DAMPING = 9.2 / 3.77` (`robonion_params.py:15`) uses the 11.1 V figures, and the comment says "use 11.1 V".
- `yaw` (hip yaw, elbow yaw: 4.1 N·m, 4.82 rad/s = 46 rpm) matches the XM430-W350 at 12 V [R2]. It matches neither the XH540 (memory: body = XH540) nor the XH430-W350 (3.4 N·m, 30 rpm [R2]). The group still uses XH540 stiffness, damping and armature.
- `torso_arms` 3.14 rad/s = 30 rpm, which is the XH430-W350 speed, not an XH540 speed.
- Hip-yaw deflection is one of the three causes of the 0.3 mm landing drift (`stage2.md`, *Foot drift*). If the hip yaw is really an XM/XH430, its stiffness is about 2.5× lower and the drift on hardware would be larger.
- The user confirmed the URDF limits, so this is a question, not a fix: which servo sits on each joint, and at which supply voltage? Basis [P8, P9].

**B3 (High, sim-to-real): the servo model has never been identified on the hardware.**
- The NMPC dynamics rest on τ = K(u − q) − D·q̇ (`model.py:224`, `nmpc.py:120`), with K, D and armature estimated from the datasheet.
- The Dynamixel position loop (P/I/D gains in its control table) is not modelled, and neither are friction (Stribeck, load-dependent), backlash or the torque–speed curve under load.
- The plan, the effort constraints and the contact-force elimination all inherit this error.
- Proposal: identify each servo type on a pendulum bench, with the same position gains as on the robot, then check K, D and armature against the measurements. Basis:
  - [P8] pendulum-bench identification and extended friction models for Dynamixel-class servos (library BAM);
  - [P9] actuator identification and latency as the two largest sim-to-real gaps;
  - [P10] learned actuator model, an alternative only if the analytic model fails.

### C. `env/robonion_env_cfg.py`

**C1 (Medium) = open question 7: the sensors are idealised.**
- Encoders: exact, no quantisation.
- Joint velocities: exact.
- Timing: no latency between sensors, and all signals simultaneous.
- AHRS error: white noise plus a constant bias.
- Accelerometer: no bias.
- Basis: [P9] (simulated latency and actuator model).

**C2 (Medium): the env cannot produce slip, so the main reason for stage 3 cannot be tested.**
- Ground friction is 1.0 and the feet use the PhysX default 0.5 (≈ 0.75 combined, `nmpc.py:62`). Nothing lowers it.
- `PLAN.md` asks for "one run with slip".
- Proposal: a scene variant with a low-friction patch, or lower friction on the feet, for the validation runs only. Basis: [P2] and [P7] test their estimators on slippery ground.

### D. `mpc/model.py` (`RigidContactModel`)

The mass matrix is frozen at the crouched standing pose. That is an accepted design, recorded in `stage1.md`, and not counted.

**D1 (Low): `-march=native` builds (`model.py:145`, `nmpc.py:271`) are not portable, and the build hash does not cover the compiler flags.** A build copied from the desktop to the Jetson fails to load instead of being rebuilt. This is already handled by "build on the target" (`build_controller.py`), so it is only a documentation point.

### E. `mpc/estimator.py`

`AhrsGyro`: no finding. The compiled and Pinocchio paths are equivalent. Yaw rate leaks into the roll rate as sin(pitch)·ψ̇, which is negligible.

**E1 (Medium, hardware): touchdown detection works below the sensor resolution.**
- `ContactModeMonitor` lands a foot when its lowest corner is within `h_land = 0.5 mm` of the other foot (`estimator.py:171`).
- The encoder resolution is 0.088° ≈ 0.7 mm at the sole (`stage2.md`, *Foot drift*).
- A 0.26° AHRS roll error over the 0.11 m stance width is also 0.5 mm. The dynamic AHRS error at touchdown is not modelled (C1).
- `tilt_on = 0.2°` faces the same limit (already in `stage2.md`, *Open*).
- On hardware, touchdown and edge detection would mostly react to noise.
- Proposal: a probabilistic contact estimate that fuses the gait-phase prior, kinematic height and velocity, and the model-predicted force, instead of hard thresholds. Basis:
  - [P5] MIT Cheetah 3: fuses a gait-phase prior, foot height and estimated force; 99.3 % accuracy, 4–5 ms delay, no contact sensors;
  - [P6] HyQ: contact from joint torques, no foot sensors.

**E2 (Low, not observed): the projection weights span 1e-8 to 1 (`estimator.py:293`), an ill-conditioned KKT matrix.** No failure has been seen. With the EKF supplying base x, y, z, the 1e-8 base weight is only there so that the base absorbs the contact mismatch, and that becomes moot if open question 2 locks the base to the EKF.

### F. `mpc/ekf.py` (`LegKinematicsEkf`) and how `controller.py` uses it

**F1 (High, suspected): for one tick after a contact-mode change, the contact reference sits on the old contact point.**
- How it happens:
  - The EKF gets the *previous* tick's modes (`controller.py:171`).
  - `_estimate` then writes the EKF foot point into `cref` for every foot the EKF thinks is in contact (`controller.py:214-217`).
  - When the monitor switches a foot that stays in contact (flat ↔ toe / heel / edge), `update_refs` has just moved `cref` to the new contact point. `_estimate` overwrites it with the EKF point of the old mode, while `pc` already names the new point.
- Size of the error: 8.75 cm along x for toe/heel (sole centre x −0.0195 m against +0.068 / −0.107 m), 3.35 cm along y for an edge.
- Effect on the NMPC:
  - The model turns a contact error into acceleration through Baumgarte stabilisation [P13]: α²(c − cref) with α = 20, so 400 × 0.0875 ≈ 35 m/s².
  - Edge → flat during HOLD: `_transition` moves to RECOVER and runs the NMPC in that same tick with this `cref` on every node (`controller.py:300-313`, `_nodes`).
- Effect on the wrench and `release()`:
  - The base q[:3] is overwritten by the EKF after the projection (`controller.py:221`), so c(q) ≠ cref by that distance.
  - `f_wrench`, and through it `release()`, sees that error and can release a loaded foot (HOLD → CATCH).
- Verification first: record ‖crow(q) − cref‖ and fz per tick in a stepping run with pushes (edges happen there).
- Options:
  1. run the EKF after `monitor.update` with the current modes (open question 6);
  2. in `_estimate`, overwrite `cref` only for feet whose EKF mode equals the current mode.
- Basis: [P1] §III, where the measurement model at time k uses the contact set of time k; [P5] shows that contact-timing errors of a few ms matter.

**F2 (Medium, suspected): the stack has two roll/pitch estimates, and they are never compared.**
- The base position comes from the EKF orientation R: gyro-integrated, initialised from the raw AHRS (bias not removed), and corrected through the leg kinematics.
- The projection, the monitor and the NMPC use `AhrsGyro` roll/pitch minus the bias calibrated at the start.
- An angle difference δ moves the EKF base relative to the projected joints by δ·ℓ (ℓ ≈ 0.5 m from the IMU to the soles): 1 mm per 0.11°. That adds to the c(q) ≠ cref of open question 2.
- Proposal: log both, then feed one orientation (the EKF's) to every consumer. Basis: [P1], and [P4], a single filter for the full base state of a humanoid with flat feet.

**F3 (Medium, measured standalone): the EKF adds ≈ 0.4 ms to a tick that was already close to its 5 ms budget.**
- `update` takes 0.38 ms mean, 0.50 ms p95 on the desktop (Python, 300 calls).
- Each tick calls Pinocchio forward kinematics about 24 times:
  - the finite-difference Jacobian runs over all 21 actuated joints, including 10 arm/head columns that are always zero (`ekf.py:116-120`);
  - plus `prev` on every tick, even without a mode change (`ekf.py:239`);
  - plus the velocity step (`ekf.py:255`).
- Added to the stage-2 4.2–4.3 ms mean and 4.6–4.8 ms p95, this predicts ≈ 4.6–4.7 ms mean and ≈ 5.1–5.3 ms p95: more ticks over 5 ms. Not yet measured in Isaac.
- Proposal: open question 3, contact points and their Jacobian from one compiled CasADi function over the leg and torso joints only. Basis: [P15] code generation, [P16] analytic kinematic derivatives.

**F4 (Medium) = open question 4: the noise densities are far from the simulated sensors, and slip goes into the base.**
- Options beyond retuning:
  - a Mahalanobis gate on each leg's kinematic residual, so that a slipping foot is rejected rather than absorbed [P2];
  - an HMM slip estimator [P7].
- Basis for consistency: the invariant EKF [P3], whose linearised error dynamics do not depend on the estimate.

**F5 (Medium): EKF z drift becomes a ground-height error for the NMPC.**
- z is unobservable [P1, §IV], but the NMPC references assume the ground at z = 0 in the model frame: `contact_refs` sets z = 0 (`model.py:295`), and the swing target is an absolute height (`gait.py:145`).
- An EKF z error of e therefore looks to the NMPC like a ground-height error of e. Stage 2 falls beyond ±5 mm.
- Proposal: express the swing and contact z references relative to the estimated stance-foot height, rather than to an absolute 0, so that only relative (observable) quantities enter the NMPC. Basis: [P1] §IV, [P3]; no paper found yet for this exact choice on a position-controlled humanoid.

**F6 (Low, verified): the `ekf.py` docstring says the discrete process noise is "first order in dt", but `_predict` uses the dt³/3 and dt²/2 terms (`ekf.py:172-174`).** Fix the text, or the code, once a choice is made.

**F7 (Low) = open question 5: the yaw alignment is frozen at `_start`.**
- `_align_ekf` (`controller.py:224-230`) takes the yaw once, and the model has no yaw.
- AHRS yaw is an EKF measurement with σ = 1°. Near motors and steel, a magnetic disturbance would rotate the estimated x, y relative to the model frame.
- Basis: [P1] (yaw unobservable from kinematics + IMU), [P3].

### G. `mpc/nmpc.py` (`RobonionNMPC`)

**G1 (Medium, verified): solver problems do not show up in the results.**
- `nan_resets` is counted (`nmpc.py:353`) but no script prints it.
- `run_stepping.py:146` counts QP status 2 (iteration limit) as a success.
- A run can print "QP failures 0" while the solver restarted from NaN or stopped at `qp_solver_iter_max = 100`.
- Proposal: report NaN resets and status 2 as separate numbers. Basis: [P14], whose RTI guarantees assume each QP is solved.

**G2 (Low, verified): the NMPC computes the DCM with `OMEGA = √(g/0.48)` (`nmpc.py:60`), while `GaitPlan` and RECOVER use ω from the measured crouched CoM height, 0.475 m (`gait.py:49`).**
- The difference is ≈ 0.5 %, at most ~0.1 mm of DCM at stepping speeds.
- Proposal: one ω, taken from the plan. Basis: [P12] DCM definition.

**G3 (Low, not observed): the bounds on u (`idxbx`) are hard constraints.** After HOLD / RESET / CATCH have moved u without the NMPC, a u more than `U_RATE_MAX × 31.25 ms` outside `U_RANGE_DEG` makes the QP infeasible. This has not been seen. It would show up as a G1 status once G1 is reported.

### H. `scripts/`

**H1 (Medium, verified): `run_stepping.py --record` cannot feed the stage-3 validation.**
- It saves the controller inputs and outputs (`run_stepping.py:103`), but not the simulator base pose (truth), the contact modes, or the EKF state and covariance.
- `PLAN.md` step 1 needs all of these.
- Proposal: a recording option for them (truth used for scoring only), or a stage-3 recording prototype in `outputs/stage3_prototypes/`. Rule: `PLAN.md`, *Stage 3* validation.

**H2 (Low, verified by reading): `run_stepping.py:110` picks the 5th single support for the push even without `--push`.** `--steps 4` or fewer raises `IndexError`.

**H3 (Low): `run_stepping_nocalib.py` patches module globals (`controller.CROUCH`, `GAIT_START`, …).** If one of those names is renamed, the assignment silently creates a new attribute and the patch has no effect.

### I. Package and `tasks/`

**I1 (Low): the package `__init__.py` catches `ImportError` and passes.** That also hides real import errors inside `tasks/`, not only a missing Isaac Lab. Narrowing the check to the Isaac Lab import would keep the Jetson case. The cart-pole placeholder and `ui_extension_example.py` are template leftovers, already known in `CLAUDE.md`.

### Decisions for the user

Nothing is applied until approved. Suggested order (each item depends on the ones before it):
1. A1: the URDF source of the pipeline.
2. B1 / B2: which servo, and how many, drive each joint, and the supply voltage.
3. A2: rerun the baseline with the current code. Then G1 and H1, so that the rerun and every later run report solver failures and record the truth.
4. F1: verify first, then option 1 or 2 (together with open question 6).
5. F2, F3, F4, F5: the estimator questions, together with open questions 2–5.
6. B3, C1, C2, E1: sim-to-real work (servo identification, sensor realism, slip test, contact estimation), before the hardware check.
7. A3, A4, F6, G2, H2, H3, I1, D1: documentation and small fixes, once the decisions above are made.

### Decisions and follow-up (2026-10-06)

**A1: left as is (user decision).**
- Nothing at runtime reads the `references/` URDF: Isaac loads `assets/robonionv2.usd`, and the controller loads `assets/robonionv2_controller.urdf`.
- Only the asset pipeline reads it: converter step 1 and the `check_urdf_vs_usd.py` default. So it matters only when the USD is regenerated.
- The submodule edit stays local.

**B1: applied (user decision): one XH540-W270 on each front thigh, 9.9 N·m on both sides.**
- `URDF_LIMITS["thigh"]` was removed. The front thighs are back in `legs` (`robonion_params.py`), in the `legs` actuator group (`robonionv2.py`), and in `_effort` (`model.py`). The NMPC thigh bound is now 0.8 × 9.9 = 7.9 N·m.
- `joint_info.md` is updated.
- Fix 7 changed too (user approved): it now sets 9.9 on both front thighs in the USD metadata.
  - Regenerated: fix → flatten → `check_urdf_vs_usd.py` → `export_controller_urdf.py`.
  - The SUMMARY is again 7 INFO / 2 WARN, but the effort WARN is now `right_front_thigh_pitch_joint: 9.9 != urdf 19.8`; `joint_info.md` is updated to match.
  - The controller URDF diff is only that effort, and its CoM matches Pinocchio.
  - Copies of the previous `robinion.usda` and `robonionv2.usd` are in the session scratchpad, not in the repo.
- **The NMPC hash changed: rerun `scripts/build_controller.py` before any run.**
- All results above (F1, F3) were measured *before* this change, with 19.8.

**B3: deferred (user decision).** The servo identification is not done yet on purpose.

**F1: confirmed in Isaac.**
- Run: `run_stepping.py --push x+1.75 --seed 0` (EKF in the loop, thigh 19.8), through a scratch tracing subclass that logs |crow(q) − cref| on the active rows after `_estimate`. The tracer is not in the repo.
- Result: ok, 10 steps, targets held 1.47 s, largest foot displacement 19.7 mm.
- Contact mismatch: on 17 estimates the EKF mode differed from the monitor mode for a foot in contact:

| t [s] | monitor | EKF | contact error | state, NMPC |
|---|---|---|---|---|
| 5.415 | toe/toe | flat/flat | 83 mm | HOLD |
| 5.750 | flat/flat | toe/toe | **92 mm** | **RECOVER, NMPC ran** (the case predicted above) |
| 6.300, 6.530, 8.070, 10.490 | flat ↔ edge on one foot | | 16–34 mm | **STEP, NMPC ran** |
| 6.420, 7.905, 10.225 | | | 33–34 mm | CATCH |

- Without a mismatch the error is not zero either: median 3.1 mm, p95 5.2 mm, max 38 mm. This is the c(q) ≠ cref of open question 2 and F2, measured: with α² = 400, 3 mm acts as ≈ 1.2 m/s² of constraint acceleration in the NMPC model.
- Whether a mismatch tick caused a wrong `release()` cannot be read from this run. Releases also mark every normal lift-off: 174 release ticks, several during mismatch ticks (6.300, 6.420, 7.905, 10.225).
- Proposal unchanged (option 1 or 2). It touches the code that runs in the Isaac loop, so it would be checked with the full A2 rerun (seeds 0, 1, 2, push table) before and after. Waiting for approval.

**F3: measured in Isaac, worse than predicted: now High.**
- Same run without the tracer: tick mean 4.97 ms, p95 7.60 ms, max 29.5 ms; NMPC solve 1.28 ms mean.
- Stage 2 was 4.2–4.3 ms mean and 4.6–4.8 ms p95.
- The mean is now at the 5 ms budget. Most of the tick is outside the solver, so the Python EKF is the first suspect (open question 3).
- Severity count after this: High 5 (A1, B1, B3, F1, F3), Medium 11; B1 is now applied.

**Question (2026-10-07): does the contact monitor need to be this precise?**

The monitor gives six modes per foot (flat, toe, heel, left_edge, right_edge, air), with an edge flagged at 0.2° of predicted tilt. Who uses what:

| User | What it needs from the mode |
|---|---|
| Gait logic, state machine (`_transition`) | only *air / flat / some edge*. Any edge means HOLD, RESET or CATCH; which edge does not matter. The peel-off rule turns an edge into `air` |
| NMPC | only flat and air. Edges are never planned, and the NMPC does not run while a sole is on an edge (stage-1 rule) |
| Projection (`ContactProjectionEstimator`), `f_wrench` / `release()` | *which* edge: it sets the contact point and the active rows |
| EKF | which edge: today the contact point follows the mode. In the paper and in `PLAN.md` it is one point per foot, contact / no contact |

So:
- Control needs three states: flat, tilted, air.
- The edge identity is used only by estimation.
- F1 comes exactly from that part: the EKF contact point jumps between the sole centre and the edge on every mode change.

*Why the 0.2° threshold is so tight* (`stage1.md`, *From "acting in every mode" to "only while flat"*): an NMPC that keeps acting while a sole rolls fell in all 10 pushes. The flat-contact model is wrong as soon as a sole rolls, so the roll has to be caught early. The 15 ms lead caught a heel 20 ms earlier than the angle alone.

*On hardware this precision is probably not reachable* (E1):
- 0.2° is about the size of the AHRS error, and the encoders resolve 0.7 mm at the sole.
- The threshold itself was tuned on simulator artefacts (`stage2.md`, *Open*).

**Possible simplification (not decided):**
1. The EKF goes back to the paper: contact / no contact only, one point per foot at the sole centre [P1]. The EKF no longer depends on the edge identity, and F1's jump is gone by construction.
   - While a sole rolls, its sole centre moves; the EKF reads that as slip and absorbs it through the slip noise.
   - Alternatively, the foot orientation can be a state [P4] (flat-foot EKF).
   - The projection then gets its edge contact point from the EKF sole-centre point plus the measured sole orientation, not from the EKF directly.
2. Control keeps the three states (flat, tilted, air).
3. On hardware, "tilted" and "air" become probabilities that fuse the gait phase, kinematics and the model force, instead of hard 0.2° / 0.5 mm thresholds [P5]. For the edge identity, `landing_mode`-style geometry (which side is lower) is enough.

This would replace F1 options 1 and 2. It still needs a run against the A2 baseline: the edge identity also drives the projection, and through it `release()`.

**Question: does walking need its own state machine?** Not a new one.
- `controller.py` already has one (`State`: STEP, HOLD, RESET, CATCH, RECOVER), and every state comes from an Isaac failure (`stage2.md`). The DS/SS phases themselves are a time schedule in `GaitPlan`, retimed by events (early or late touchdown, pause).
- Walking (after stage 4) changes the plan rather than the states:
  - footholds that are not the lift-off point (step length, and stage-4 footstep adaptation);
  - replanning per step;
  - start and stop of a walk.
- The existing STEP state and `GaitPlan` would carry these. A separate machine would duplicate the edge and recovery rules.
- Basis: [P5], an event-based contact state machine on top of a time-based gait schedule (contact events retime the plan, as `_retime` does here).
- Not for now: stage 3 comes first.

### Scope question: whole body (arms) in the NMPC while walking (raised 2026-10-06, not decided)

The user: in stage 3 the whole body should move while walking, so the arms go into the NMPC.
- This is not in stage 3 as `PLAN.md` defines it (base position estimation).
- Open: does it add to stage 3, replace it, or come after it? Is it the supervisor's requirement?

**Today**
- The model has 9 joints (`MODEL_JOINTS`). Arms, head and yaw are frozen at the default pose, and the mass matrix is frozen at the standing pose.
- The 8 arm joints are already actuated in the env (shoulder pitch and roll, elbow yaw and pitch, × 2). They just hold their default.

**What arms in the NMPC would cost.** Estimates, not measured:

| | Model joints | x = q, q̇, u | Controls |
|---|---|---|---|
| Now | 9 | 37 | 9 |
| + all 8 arm joints | 17 | 61 | 17 |
| + shoulder pitch and roll only | 13 | 49 | 13 |

- The Riccati recursion in HPIPM scales roughly with (nx + nu)³ per stage: (78/46)³ ≈ 4.9× for all arm joints, (62/46)³ ≈ 2.4× for the shoulders only.
- The integrator sensitivities grow with nx².
- The tick is already at the 5 ms budget (F3). Without first moving the EKF to compiled code and freeing time elsewhere, arms do not fit.
- The arms also move the CoM and the inertia that the frozen mass matrix assumes. That needs checking against `outputs/stage1_prototypes/model3.py`, as for every model change.

**What the arms are for decides the design**
1. Cancelling the vertical (yaw) angular momentum of the legs: the main effect in human walking. Without arm swing the vertical ground-reaction moment rises 63 % [P17]. The model has no yaw and no yaw contact row, so the NMPC cannot use the arms for this until pelvis yaw is added. Pelvis yaw is already needed for the 0.3 mm landing drift, and is planned for walking.
2. Balance and push recovery through angular momentum in the sagittal and lateral planes [P18, P19, P20]. The model can represent this, but torso pitch already does part of it sagittally.
3. Natural-looking motion only: no need for the NMPC.

**Options** (all need user approval; none started)
- (a) All 8 arm joints in the NMPC. Most capable, and the most expensive (≈ 5× QP); worth it only together with pelvis yaw (purpose 1).
- (b) Shoulder pitch and roll only (4 joints). About half the cost of (a); covers purposes 1 (with yaw) and 2.
- (c) Arms outside the NMPC: a counter-phase swing tied to the gait phase, sent as fixed targets. The NMPC model receives the arm angles per node as known parameters instead of states, so their effect on CoM and gravity is still modelled. Almost no cost; covers purpose 3, and partly 1, without optimising it.

**Suggested order**
- First finish stage 3: the EKF validation, F1, and F3, which frees tick time.
- Then (c) as the first step, and (b) only if the push or walking results show the arms are needed.

**Questions for the user**
1. Is this part of stage 3 (a change to `PLAN.md`), or a stage after it?
2. What should the arms do: purpose 1, 2 or 3?
3. Which joints: all 8, or the shoulders only?

**User decision (2026-10-07):** the arms are part of **stage 3**, for **balance (purpose 2) and yaw damping (purpose 1)**, with **all 8 arm joints**. This is option (a). Stage 3 is therefore base estimation *and* arms in the NMPC. `PLAN.md` and `CLAUDE.md` still say otherwise and need updating (waiting for approval).

#### What option (a) with yaw damping requires

Yaw damping only works if the model can feel yaw. Every point below is a change to `model.py`, `nmpc.py` and the estimation chain. None is started.

1. **Pelvis yaw as a coordinate.**
   - q = [x, y, z, roll, pitch, **yaw**, joints].
   - Today the base rotation is Ry(pitch) Rx(roll) (`model.py:190`). It becomes Rz(yaw) Ry(pitch) Rx(roll).
   - The NMPC frame then no longer needs `_align_ekf` to freeze the yaw (F7): the EKF yaw goes straight into the state.
2. **A yaw contact row per sole:** the z component of the sole x axis, against a reference, for a flat sole. Each foot then has 6 rows instead of 5.
   - The yaw moment needs a friction limit. For a rectangular sole, the closed-form contact wrench cone bounds M_z by linear inequalities in f_x, f_y, f_z, M_x and M_y [P22]. These replace the 4 friction rows per sole used now.
   - Without this row, the NMPC can produce a yaw moment the ground cannot carry; that slip is the landing drift of `stage2.md`.
3. **Hip yaw joints (2) in the model.** Not yet decided, but recommended:
   - The arm reaction moment reaches the soles through the hip yaw servos.
   - Hip-yaw servo deflection is one of the three causes of the 0.3 mm landing drift (`stage2.md`).
   - With the hip yaw frozen, the model takes the leg as rigid in yaw.
4. **All 8 arm joints in the model:** shoulder pitch, shoulder roll, elbow yaw, elbow pitch × 2.
   - Servo torque τ = K(u − q) − D·q̇ as for the legs.
   - Limits from `URDF_LIMITS` (`torso_arms`, `yaw` for the elbow yaw).
   - `U_RANGE_DEG` per arm joint: still to be chosen, and depends on the collision with the torso.
5. **Mass matrix.**
   - Moving arms change the inertia much more than the legs do today.
   - Before deciding between keeping M frozen at the standing pose and recomputing it per node as a parameter (like the contact algebra), check the error against a full M(q) along a recorded arm swing.
   - The same check must be run against `outputs/stage1_prototypes/model3.py`.
6. **Cost.** New terms:
   - centroidal angular momentum about z (yaw damping) [P18, P19];
   - angular momentum about x and y (balance) [P18, P20];
   - arms back to the default pose when there is nothing to do.
   - The weights are to be tuned in the prototype, as in stage 2. The weights stay hardcoded constants, by an earlier user decision.
7. **Estimation.**
   - `AhrsGyro` removes the yaw today (`estimator.py:117`). It would hand on the yaw and its rate, or the EKF yaw would go into the state.
   - The monitor and the projection carry 6 base coordinates and 12 contact rows.

**Problem size** (estimates, not measured):

| | q | x = q, q̇, u | u | contact rows | QP per stage (nx + nu)³ |
|---|---|---|---|---|---|
| Now | 14 | 37 | 9 | 10 | 1× |
| (a) + yaw, without hip yaw | 23 | 63 | 17 | 12 | ≈ 5.3× |
| (a) + yaw + hip yaw | 25 | 69 | 19 | 12 | ≈ 7× |

**Blocker: tick time.**
- The tick is already at the budget (F3: 4.97 ms mean, 7.60 ms p95), and the HPIPM solve alone is 1.28 ms. At ≈ 5–7× that is 6–9 ms of solve alone.
- So time has to be found before the arms or together with them. Candidates, each to be measured in the prototype:
  - the compiled EKF (F3, open question 3);
  - more partial condensing or fewer nodes (non-uniform grids already broke the tuning once, `nmpc.py:56`);
  - NMPC at 100 Hz with interpolated targets in between [P14];
  - the arms as a centroidal (momentum) term with full kinematics instead of full dynamics [P21].

**Question (2026-10-07): why not a 10 ms tick instead of 5 ms?**

It is possible: `decimation = 10` in `robonion_env_cfg.py`, with `dt` passed through to the controller. What it does to the time budget and to stage 2:

*Time budget.*
- Of today's 4.97 ms tick, ≈ 1.3 ms is the solve and ≈ 3.7 ms is everything else (estimation, the EKF in Python, parameter preparation).
- With the arms the solve alone is ≈ 6–9 ms (estimate), so the tick becomes ≈ 10–13 ms: still over even 10 ms.
- So a 10 ms tick does not replace making the rest of the tick faster; it only adds room.
- "Compiled EKF" means the same as for `AhrsGyro`, the projection and the model today: the computation is generated as C by CasADi (`compile_functions`) instead of running Pinocchio in Python. The kinematics then take ~µs instead of the ≈ 25 Pinocchio calls per tick (F3).

*What was tuned at 5 ms and would need retuning:*
- the edge trigger `MONITOR_TICKS = 2` (10 ms → 20 ms, later than the 15 ms lead it is meant to catch);
- `MONITOR_LEAD`;
- the `rate_ticks` filter against touchdown impacts;
- the servo rate limit per tick (`U_RATE_MAX·dt`, double the step);
- the swing (60 → 30 ticks per single support);
- the reaction to a push and to touchdown (+5 ms latency).
- All stage-2 results (push tables, ground-height error) were measured at 5 ms and would have to be run again.

*Hardware.*
- The Dynamixel bus (sync read of 21 servos, sync write of the targets, RS-485), the IMU over USB and the computation all have to fit into one tick on the Jetson.
- The bus time has not been measured. If it alone takes a few ms, 10 ms may be needed on the robot anyway.

*Middle option: multi-rate.*
- Estimation, contact monitor and gait logic stay at 200 Hz.
- The NMPC runs every second tick (100 Hz), and the targets between are held or interpolated from the NMPC solution.
- Edge and touchdown detection keep their tuning, and the NMPC gets 10 ms. RTI already splits preparation and feedback [P14].
- Running the MPC slower than the low-level loop is standard: the MIT Cheetah 3 solves its MPC at 20–30 Hz under a much faster leg controller [P23]. Here the servos' internal position loop is the fast loop.
- Not tested.

**User decision (2026-10-07): option 3, multi-rate plus a compiled EKF.** Recorded here only; nothing is implemented yet.
- Tick stays 5 ms (env `decimation = 5`). AHRS, EKF, contact monitor, projection and gait logic run every tick (200 Hz), so the stage-2 edge and touchdown tuning stays valid.
- The NMPC runs every second tick (100 Hz). In the tick between, the servo targets come from the last NMPC solution: held, or interpolated along its first interval; which one is to be chosen in the prototype.
- The EKF kinematics (contact points in the IMU frame, their Jacobian, the base-velocity term) become one CasADi function, compiled like the others, over the leg and torso joints only (F3, open question 3).
- Open, for the prototype:
  - which tick the NMPC runs on relative to a contact-mode change: run it at once on an event, or wait for its slot;
  - how the RTI preparation is spread over the two ticks [P14];
  - the tick time with and without the arms.
- Order unchanged:
  1. F1;
  2. compiled EKF and multi-rate, measured against the A2 baseline;
  3. then yaw and the arms in the prototype.

**Validation: what to measure.**
- Stepping in place hardly creates any yaw moment: the swing foot rises almost vertically.
- So the yaw-damping effect will only show in walking (stage 4) or under lateral pushes. Even there, stage 2 saw 0.3 mm landing drift per step and a stance foot that did not move.
- Proposed metrics (Isaac truth, scoring only):
  - pelvis yaw;
  - the yaw rotation of each stance sole;
  - the push table of `stage2.md` with and without arms (seeds 0, 1, 2);
  - the tick time.

**Proposed order** (needs approval):
1. Estimation fixes first: F1, then F3 (compiled EKF). This frees tick time, and the arms build on the estimated yaw.
2. Prototype in `outputs/stage3_prototypes/`, in steps:
   - pelvis yaw and the yaw contact row only (no arms; check that it still steps the same);
   - then the 8 arm joints;
   - then the momentum costs.
3. Measure the tick time after each step, and move to the package only when it fits.

References for this section:
- [P17] S. H. Collins, P. G. Adamczyk, A. D. Kuo, *Dynamic Arm Swinging in Human Walking*, Proc. R. Soc. B 276, 2009, https://pmc.ncbi.nlm.nih.gov/articles/PMC2817299.
- [P18] D. E. Orin, A. Goswami, S.-H. Lee, *Centroidal Dynamics of a Humanoid Robot*, Autonomous Robots 35(2–3), 2013, pp. 161–176.
- [P19] S. Kajita, F. Kanehiro, K. Kaneko, K. Fujiwara, K. Harada, K. Yokoi, H. Hirukawa, *Resolved Momentum Control: Humanoid Motion Planning Based on the Linear and Angular Momentum*, IROS 2003, pp. 1644–1650.
- [P20] A. Hofmann, M. Popovic, H. Herr, *Exploiting Angular Momentum to Enhance Bipedal Center-of-Mass Control*, ICRA 2009.
- [P21] H. Dai, A. Valenzuela, R. Tedrake, *Whole-Body Motion Planning with Centroidal Dynamics and Full Kinematics*, IEEE-RAS Humanoids 2014.
- [P23] J. Di Carlo, P. M. Wensing, B. Katz, G. Bledt, S. Kim, *Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control*, IROS 2018.
- [P22] S. Caron, Q.-C. Pham, Y. Nakamura, *Stability of Surface Contacts for Humanoid Robots: Closed-Form Formulae of the Contact Wrench Cone for Rectangular Support Areas*, ICRA 2015.

### References

Datasheets:
- [R1] ROBOTIS e-Manual, XH540-W270: https://emanual.robotis.com/docs/en/dxl/x/xh540-w270
- [R2] ROBOTIS e-Manual, XM430-W350: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350; XH430-W350 figures as listed by servodatabase.com: https://servodatabase.com/servo/dynamixel/xh430-w350-r (check against the ROBOTIS XH430-W350 page before using them).

Papers:
- [P1] M. Bloesch, M. Hutter, M. Hoepflinger, S. Leutenegger, C. Gehring, C. D. Remy, R. Siegwart, *State Estimation for Legged Robots – Consistent Fusion of Leg Kinematics and IMU*, RSS 2012, DOI 10.15607/RSS.2012.VIII.003 (`references/papers/bloesch2012_rss.pdf`).
- [P2] M. Bloesch, C. Gehring, P. Fankhauser, M. Hutter, M. Hoepflinger, R. Siegwart, *State Estimation for Legged Robots on Unstable and Slippery Terrain*, IROS 2013, pp. 6058–6064: UKF with outlier rejection.
- [P3] R. Hartley, M. Ghaffari, R. M. Eustice, J. W. Grizzle, *Contact-Aided Invariant Extended Kalman Filtering for Robot State Estimation*, IJRR 2020, arXiv:1904.09251 (RSS 2018 version: arXiv:1805.10410).
- [P4] N. Rotella, M. Bloesch, L. Righetti, S. Schaal, *State Estimation for a Humanoid Robot*, IROS 2014, pp. 952–958, arXiv:1402.5450.
- [P5] G. Bledt, P. M. Wensing, S. Ingersoll, S. Kim, *Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains*, ICRA 2018, http://hdl.handle.net/1721.1/120350.
- [P6] M. Camurri, M. Fallon, S. Bazeille, A. Radulescu, V. Barasuol, D. G. Caldwell, C. Semini, *Probabilistic Contact Estimation and Impact Detection for State Estimation of Quadruped Robots*, IEEE RA-L 2(2), 2017, pp. 1023–1030.
- [P7] F. Jenelten, J. Hwangbo, F. Tresoldi, C. D. Bellicoso, M. Hutter, *Dynamic Locomotion on Slippery Ground*, IEEE RA-L 4(4), 2019, pp. 4170–4176.
- [P8] M. Duclusaud, G. Passault, V. Padois, O. Ly, *Extended Friction Models for the Physics Simulation of Servo Actuators*, arXiv:2410.08650 (library: https://bam.readthedocs.io).
- [P9] J. Tan, T. Zhang, E. Coumans, A. Iscen, Y. Bai, D. Hafner, S. Bohez, V. Vanhoucke, *Sim-to-Real: Learning Agile Locomotion For Quadruped Robots*, RSS 2018, DOI 10.15607/RSS.2018.XIV.010.
- [P10] J. Hwangbo, J. Lee, A. Dosovitskiy, D. Bellicoso, V. Tsounis, V. Koltun, M. Hutter, *Learning Agile and Dynamic Motor Skills for Legged Robots*, Science Robotics 2019, arXiv:1901.08652.
- [P11] G. P. Huang, A. I. Mourikis, S. I. Roumeliotis, *Observability-based Rules for Designing Consistent EKF SLAM Estimators*, IJRR 29(5), 2010, pp. 502–528: the first-estimate Jacobians the EKF uses.
- [P12] J. Englsberger, C. Ott, A. Albu-Schäffer, *Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion*, IEEE T-RO 31(2), 2015.
- [P13] J. Baumgarte, *Stabilization of Constraints and Integrals of Motion in Dynamical Systems*, Comput. Methods Appl. Mech. Eng. 1, 1972, pp. 1–16.
- [P14] M. Diehl, H. G. Bock, J. P. Schlöder, *A Real-Time Iteration Scheme for Nonlinear Optimization in Optimal Feedback Control*, SIAM J. Control Optim. 43(5), 2005, pp. 1714–1736.
- [P15] J. A. E. Andersson, J. Gillis, G. Horn, J. B. Rawlings, M. Diehl, *CasADi: A Software Framework for Nonlinear Optimization and Optimal Control*, Math. Prog. Comp. 11, 2019.
- [P16] J. Carpentier, G. Saurel, G. Buondonno, J. Mirabel, F. Lamiraux, O. Stasse, N. Mansard, *The Pinocchio C++ Library*, IEEE/SICE SII 2019.
