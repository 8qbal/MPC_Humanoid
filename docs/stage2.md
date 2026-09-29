# Stage 2 — stepping in place: design and results

Goal: step in place with the design-D NMPC of stage 1 (see [`stage1.md`](stage1.md)), using only what the real robot measures (joint encoders, Xsens MTi-630 AHRS orientation and gyro; no foot force sensors), every control tick (200 Hz), outputting Dynamixel position-mode servo targets.

## Status

| | Result |
|---|---|
| Stepping in place, Isaac, estimator only | 10 and 20 steps at 2 cm, 20 steps at 3 cm clearance, no fall, no held targets, no QP failure, seeds 0, 1, 2 (`scripts/run_stepping.py`) |
| Tick time (desktop) | estimation + NMPC 4.2–4.3 ms mean, 4.6–4.8 ms p95 in Isaac (3.8 / 4.3 ms replayed without Isaac); measurement-to-target latency ≈ 1.3 ms; Jetson not measured |
| Early / late touchdown | handled up to ±5 mm of ground-height error in Isaac (3 seeds); −10 mm falls |
| Pushes while stepping (mid single support) | resumes stepping after +1.75 / −1.0 N·s sagittal in all 3 seeds (−1.75 in 2 of 3); lateral ±0.5 and +0.75 / +1.0 N·s in all 3 seeds, −1.0 in 1 of 3; see *Pushes* |
| Closed | 2026-09-26 by user decision, with the limits under *Open* accepted |
| Package | `mpc/gait.py`, `mpc/nmpc.py` (`RobonionNMPC`), `mpc/controller.py` (`RobonionController`), `robots/robonion_params.py`, `scripts/build_controller.py`, `scripts/run_stepping.py`, `scripts/benchmark_stepping.py` |

All Isaac numbers are GPU PhysX with 0 TGS velocity iterations (see *Pitfalls*), **one environment per process**, runs in sequence (as in stage 1); package results are given for sensor-noise seeds 0, 1, 2.

## Architecture

One layer: the design-D NMPC is the whole-body tracker; a gait scheduler supplies the contact plan per node (double support → single support with the swing foot in `air`), a DCM/ZMP reference and a swing-foot spline, both only as cost references.  A two-layer design (DCM planner + tracker) was rejected: design A in stage 1 showed that a DCM feedback loop through the compliant servos is unstable; the NMPC already models the servos.

```text
encoders + AHRS + gyro -> AhrsGyro -> ContactModeMonitor -> ContactProjectionEstimator -> (q, qd, modes)
gait plan (gait.py) -> per-node modes, DCM / CoM velocity, swing-sole target -> gait logic (controller.py)
  -> RobonionNMPC (acados SQP-RTI, split preparation / feedback) -> servo targets of the 9 model joints
```

Node 0 of the contact plan is always the measured mode; later nodes follow the plan.  Edge modes are never planned.

## Gait parameters and why

Numbers from the package `RigidContactModel` (linearised, before any Isaac run):

| Question | Result |
|---|---|
| Swing clearance with straight legs (`LEG_CROUCH = 0`) | 3 cm needs 0.39 rad on front thigh and ankle pitch together; with a 0.15 s lift phase ≈ 4.9 rad/s, above the 4.08 rad/s servo limit.  Straight legs are also singular for leg length (stage 1) |
| Same with a 0.2 rad crouch (both joints, negative sign as `LEG_CROUCH`) | 2 cm: 0.18 rad, 3 cm: 0.24 rad; pelvis 8 mm lower, CoM height 0.475 m (ω = 4.55 s⁻¹) |
| Modes, double support, crouch 0.2 | 1.07 Hz, 1.73 Hz (ζ 0.3–0.4) |
| Modes, single support, CoM over the stance sole | 0.22 Hz (ζ 0.3, nearly neutral), 1.1 Hz, 1.5 Hz, 2.3–2.8 Hz (ζ 0.7–0.9); no unstable eigenvalue |
| Stance hip roll in single support | 5 N·m static (63 % of 0.8 × 9.9 N·m), 6.8° sag; uncompensated targets would drop the swing hip ≈ 13 mm |
| Lateral CoM sway (periodic LIPM, ZMP at the sole centres) | T_ss/T_ds 0.3/0.1 s: ±16 mm; 0.35/0.1: ±19 mm; 0.4/0.1: ±22 mm; 0.6/0.2: ±36 mm.  The ZMP moves ±55 mm, the CoM does not have to |

Chosen (user decision): crouch 0.2 rad ramped by the controller after start (`LEG_CROUCH` in `robots/robonion_params.py` stays 0, so the stage-1 tables still hold), **T_ss 0.30 s, T_ds 0.10 s** (sway 1.25 Hz and steps 2.5 Hz avoid the 1.1 Hz and 1.5–1.7 Hz modes; 0.45–0.5 s steps would sway at 1.0–1.1 Hz), clearance 2 cm (3 cm also works), step width 0.110 m (the standing width).  0.6/0.2 s is the tested fallback.

## Gait plan (`mpc/gait.py`)

Stand (1.5 s) → DS (ZMP from the middle of the soles to the first stance sole) → SS → DS → … → DS (ZMP back to the middle) → stand.  ZMP constant in single support, linear in double support; the DCM is integrated backwards in closed form from rest at the end (ξ̇ = ω(ξ − p)), the CoM forwards from rest at the start (ċ = ω(ξ − c)).  Checked numerically: ODE residual 3e-6 m/s (CoM), CoM at rest at both ends within 0.4 mm.  Swing sole height `z = h · 64 s³(1 − s)³` (zero velocity and acceleration at both ends); with the crouch, 2 cm needs 1.99 rad/s peak joint rate (49 % of the limit).  `stretch()` lengthens or shortens one phase and moves the later ones (retiming, pauses); a phase stretched beyond its planned duration keeps its sole reference descending at 0.1 m/s.

## NMPC (`mpc/nmpc.py`, `RobonionNMPC`)

Same model, dynamics, constraints and weights as the stage-1 NMPC (25 nodes, retired; code at git tag `stage2`), plus:

- per-node references as parameters: DCM, CoM velocity, swing-sole position (all nodes written with one `set_flat`);
- per-node gates: leg-length cost only for feet in contact, swing-position cost only for feet in the air, the 5 N minimum normal force only for feet in contact — all constraint bounds are then constant;
- swing-position weight **1e6**: at 1e4 the foot reached 3 mm of a 10 mm target (the servo-rate cost dominated), at 1e5 7.6 mm, at 1e6 9.4 mm;
- `U_RANGE_DEG` is centred on the crouched pose (the model is built at the crouch), so the lift fits.

### Tick time

Offline (HybridSim, 2 cm, per controller tick on the desktop), then Isaac with the estimator:

| Change | Tick mean / p95 | Latency (measurement → target) |
|---|---|---|
| Start (Python parameter loop, CasADi VM for the contact algebra) | 8.3 / 10.7 ms | 8.3 ms |
| Contact algebra compiled to C, batched inverse, one `set_flat`, one `get_flat` | 6.7 / 9.1 ms | same |
| HPIPM partial condensing to 5 stages, `SPEED` mode, warm start; IRK Jacobian reuse | 5.2 / 5.7 ms | same |
| RTI split: preparation for the next tick after the targets are out | 5.3 / 5.9 ms | 2.0 ms |
| 16 nodes × 31.25 ms instead of 25 × 20 ms (same 0.5 s horizon) | 3.4 / 3.8 ms | 1.2 ms |
| Isaac, estimator + monitor (1.1 ms, Python) + NMPC, package | 4.7 / 5.2 ms | ≈ 1.3 ms |
| **Estimation path compiled to C (`compiled=True`: contact model, `AhrsGyro`, estimator), Isaac** | **4.2–4.3 / 4.6–4.8 ms** | ≈ 1.3 ms |
| Same, replayed without Isaac (`scripts/benchmark_stepping.py`, 20 steps at 3 cm) | 3.8 / 4.3 ms (p99 6.2, 2 % of ticks over 5 ms) | |

The 16-node grid behaves like 25 nodes (CoM sway, peak torque 58 %, landing timing).  Non-uniform grids (8 × 20 ms + 8 × 42.5 ms, 12 × 20 + 6 × 43.3, 10 × 25 + 6 × 41.7) are faster but raised the torso torque to 140–174 % of the servo limit and one fell: the weights are tuned for uniform intervals (acados scales the stage cost by the interval).  Full condensing was slower (QP 8 ms).

`scripts/build_controller.py` (no Isaac) generates and compiles the compiled functions (one shared library each), the contact algebra (`alg.so`) and the acados solver into `outputs/acados/`; `run_stepping.py` and `benchmark_stepping.py` only load them and stop with `FileNotFoundError` when a build is missing.  Every build is named by a hash of what it computes (the functions; for the solver folder `robonion_nmpc_<hash>` the OCP expressions and numbers), so a changed model never loads a stale build.  The controller rounds its crouched pose to 6 decimals (`nominal_pose`), so the float32 default pose from Isaac builds the same model as the exact one of the build script.  Servo constants and the default pose live in `robots/robonion_params.py` (no Isaac imports), so `mpc/` and the benchmark run on a machine without Isaac Sim.  Not yet measured on the Jetson AGX Orin (see the next section).

### Benchmark on the Jetson AGX Orin

`scripts/benchmark_stepping.py` times the controller without Isaac by replaying a recorded run: every tick gets the recorded encoder, AHRS and gyro values (float32, as delivered), so the controller takes the same path as in Isaac (open loop).  The controller stack (`mpc/`, `robots/robonion_params.py`) imports without Isaac Lab, Isaac Sim and torch (checked with those imports blocked).

**1. Record on the desktop (Isaac):**

```bash
uv run python scripts/run_stepping.py --steps 20 --clearance 0.03 --record run_20x30.npz
uv run python scripts/run_stepping.py --push x+1.75 --record run_push.npz   # also covers the hold / recover path
```

The regression recordings can be used as they are (`outputs/final2/*.npz`, e.g. `s0_steps20_h30.npz`).

**2. Copy to the Jetson:** the repository (at least `source/MPC_Humanoid/`, `scripts/benchmark_stepping.py`, `assets/robonionv2_controller.urdf`) and the `.npz` files.  `outputs/` is not needed; the solver and the compiled functions are built there on the Jetson (step 4).

**3. Python environment on the Jetson.** The project's `uv` environment pulls in `isaaclab[isaacsim]`, which does not exist for the Jetson, so use a separate one with only what the controller needs (not tried on the Jetson yet):

```bash
uv venv ~/robonion-venv --python 3.12
uv pip install --python ~/robonion-venv numpy scipy casadi pin
uv pip install --python ~/robonion-venv -e ~/acados/interfaces/acados_template
```

acados itself has to be built from source on the Jetson (`~/acados`, as on the desktop).  `acados_template` renders the solver code with the `t_renderer` binary, which it downloads for x86 only; on the Jetson build it from source (tera_renderer, Rust) and put it into `~/acados/bin`.

**4. Run:**

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks        # maximum clocks; note the power mode with the result
export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$HOME/acados/lib
PYTHONPATH=source/MPC_Humanoid ~/robonion-venv/bin/python scripts/build_controller.py   # once, a few minutes
PYTHONPATH=source/MPC_Humanoid ~/robonion-venv/bin/python scripts/benchmark_stepping.py run_20x30.npz --repeat 3
```

The benchmark does not build anything; without the build step it stops with `FileNotFoundError`.

**5. Reading the output:**

| Line | Meaning | Desktop reference (`s0_steps20_h30.npz`, `--repeat 2`) |
|---|---|---|
| `tick (estimation + NMPC) mean, p50, p95, p99, max; over budget` | wall time of `RobonionController.step()` per tick, and the share of ticks over the 5 ms tick | 3.78, 3.73, 4.29, 6.18, 8.51 ms; 2.0 % |
| `acados solve time per tick` | the NMPC feedback solve alone | 1.10 mean, 1.39 p95 |
| `max \|servo target - recorded\|` | difference to the targets of the recorded run; rounding only (≈ 1e-6 rad or less).  A large value means the replay took another path and the timing is not comparable | 0 |

The budget is met when p95 stays below 5 ms and the ticks over budget stay rare (a late tick holds the previous targets for one more 5 ms).

## Controller logic (`mpc/controller.py`, `RobonionController`)

Timeline: 0–0.5 s standing at the default pose, 0.5–1.3 s crouch ramp, 1.5–2.0 s AHRS bias calibration (both soles flat), gait from 2.0 s.

| Situation | Rule | Why |
|---|---|---|
| Foot scheduled to swing tips onto an edge while the other sole is flat | planned `air`, NMPC keeps running | An unloaded foot peels off over its outer edge; the stage-1 hold rule froze the robot for 4.4 s at the first step (offline) |
| Swing foot touches down before the planned end | single support ends there (`stretch`), DS starts | Offline, 10 mm lower ground (75 ms early) fell without it and passed with it |
| Swing foot still in the air at the planned end | phase extended tick by tick, up to 0.15 s, reference descending | |
| Stance sole on an edge while the other foot is in the air | **catch**: stance-leg targets held, swing leg back to its targets at lift-off at the servo rate limit, step ended | Isaac, y ±1.0 N·s mid single support: catch passed y−1.0; holding everything (stage-1 rule) and landing the swing foot with the NMPC (stance sole planned flat) both fell |
| Sole on an edge, both feet down | targets held (stage-1 rule), plan paused (0.6 s of DS kept ahead) once the hold lasts > 0.1 s | Without the pause the plan ran on during the hold and every scheduled step was consumed within ~0.5 s |
| Sole resting still on an edge (tilt span < 0.3° over 0.2 s) | targets move to the standing posture at 0.5 rad/s | After pushes the soles can rest on their outer edges indefinitely; the monitor only returns to `flat` when the tilt crosses zero.  Resuming the NMPC with the sole planned flat rolled it again within ~50 ms |
| After any hold with both feet down | NMPC stands as in stage 1 (every node flat, DCM to the middle, CoM at rest), plan paused, until DCM < 10 mm from the middle and CoM speed < 0.03 m/s for 0.2 s | Resuming the plan right away (its DCM reference is a moving ZMP) re-tipped the soles |

The rules are a state machine (`State` in `controller.py`: STEP, HOLD, RESET, CATCH, RECOVER; the contact modes of each tick select the next state).  It replaced the equivalent chain of flags and gave the same servo targets (0 rad difference) on 14 recorded runs that pass through all five states.

### Edge detection while stepping

`ContactModeMonitor` flags a flat sole as on an edge when its tilt angle is beyond 0.2° (immediately) or when the **predicted tilt** `tilt + 0.015 s · rate` is beyond 0.2° on **2 consecutive ticks** (`MONITOR_LEAD`, `MONITOR_TICKS`).  Stage 1 keeps its 3°/s rate trigger (`lead=None`, one tick).  How it got there:

| Trigger | Problem in Isaac |
|---|---|
| rate > 3°/s (stage 1) | flat stance sole read −4°/s while weight moved (Isaac joint velocities, see *Pitfalls*); the 1 cm run fell at 4.3 s |
| rate > 10°/s, one tick | the setting with 4 TGS velocity iterations; replaced together with them (a margin against a simulator error, not a detection rule) |
| rate trigger that must persist over ticks, 0 velocity iterations | steps got stuck: without a push the plan kept pausing, targets held 1.0–1.3 s and the 10 steps not finished within 12 s (3 seeds) |
| predicted tilt, one tick | touchdown impact: 12–14°/s on a single tick at ≈ 0° tilt (next tick ≈ 0°/s) held the targets 0.36–0.83 s over 20 steps at 3 cm, in all three seeds |
| **predicted tilt, 2 ticks** | none found; a real roll is flagged 5 ms later than with one tick |

The predicted tilt does not flag a sole whose rate brings it back to level; the second tick rejects the one-tick impact spike, which a minimum rate (e.g. 3°/s) would not (the spike is 12–19°/s) while it would miss real slow rolls (2.2–2.9°/s, −x 1.0 and 20-step runs).  Checked offline first: every edge flag in the 42 recorded runs of the one-tick version, replayed through the controller (`monitor_trace.py`, `replay_rec.py --trace`).

## Results

### Offline (HybridSim) vs Isaac, simulator state and contact modes

Prototype, 25 nodes, T_ss 0.3, T_ds 0.1:

| | Offline | Isaac, simulator state |
|---|---|---|
| Weight shift only (no lift) | CoM y −19.4 / +11.2 mm, rms error 5.1 mm | −19.6 / +11.3 mm, rms 5.0 mm |
| 1 cm, 6 steps | peak sole 9.4 mm; lift-off / touchdown +10 / −15 ms | 9.4 mm; +35 / −35 ms (air = > 1 mm above the ground) |
| 2 cm, 20 steps | 18.9 mm; CoM y −16.9 / +13.4 mm; peak servo torque 58 % | 19.1 mm; −16.9 / +13.1 mm; 56 % |

With the estimator instead of the simulator state (Isaac, 2 cm, 20 steps): CoM error rms 3.1 mm (x) / 0.1 mm (y), contact modes equal to the simulator's in 91 % of the ticks (the rest is the 1 mm air threshold), no held targets.

### Ground-height error (Isaac, estimator)

The controller is told the ground under a landing foot is higher or lower than it is (test input of the prototype); rerun with the final monitor (`isaac_gait.py --lead 0.015 --rate_ticks 2`) and seeds 0, 1, 2: ±5 mm ok in all six runs.  First results:

| Error | Touchdown vs plan | With retiming | Without |
|---|---|---|---|
| −5 mm (lower) | −60 ms | ok | ok |
| +5 mm (higher) | +15 ms | ok | ok |
| +10 mm | +40 ms | ok | – |
| −10 mm | −80 ms | falls (catch: 4.3 s; hold: 3.5 s) | falls (3.3 s) |

### Clearance and timing (Isaac, estimator)

| Run | Result |
|---|---|
| 2 cm, 20 steps | ok, peak sole 18.9 mm, servo torque 58 %, joint rate 1.1 rad/s |
| 3 cm, 20 steps | ok, peak sole 29.3 mm, servo torque 66 %, joint rate 1.7 rad/s |
| T_ss 0.6, T_ds 0.2, 10 steps | ok, CoM y ±35 mm, servo torque 54 % |

### Pushes while stepping

One push on `upper_body_link` for 0.05 s in the middle of the 5th single support (stance: left foot), 10 steps planned; "steps" = steps completed after the push (5 planned).

Package (`scripts/run_stepping.py --push ... --seed N`, 16 nodes); per cell: result, steps completed of the 10 planned, time with held servo targets, largest foot displacement:

| Push (N·s) | seed 0 | seed 1 | seed 2 | before: 4 velocity iterations, 10°/s one-tick trigger (seeds 0 / 1 / 2) |
|---|---|---|---|---|
| +x 1.0 | ok 10, 0.00 s, 3 mm | ok 10, 0.00 s, 3 mm | ok 10, 0.00 s, 3 mm | ok/ok/ok |
| +x 1.75 | ok 10, 1.72 s, 6 mm | ok 10, 1.71 s, 6 mm | ok 10, 2.40 s, 8 mm | ok/ok/ok |
| +x 2.5 | **fall** 5, 1.22 s, 84 mm | **fall** 5, 1.23 s, 85 mm | **fall** 5, 1.23 s, 84 mm | fall/fall/fall |
| −x 1.0 | ok 10, 0.53 s, 2 mm | ok 10, 0.49 s, 2 mm | ok 10, 0.48 s, 2 mm | ok/ok/ok |
| −x 1.75 | ok 10, 3.60 s, 12 mm | ok 10, 3.51 s, 11 mm | **fall** 6, 4.61 s, 109 mm | ok/ok/ok |
| −x 2.0 | **fall** 5, 1.56 s, 92 mm | **fall** 5, 1.60 s, 90 mm | **fall** 5, 1.56 s, 91 mm | ok/ok/ok |
| +y 0.5 | ok 10, 0.00 s, 3 mm | ok 10, 0.00 s, 3 mm | ok 10, 0.00 s, 3 mm | ok/ok/ok |
| +y 0.75 | ok 10, 1.41 s, 11 mm | ok 10, 0.02 s, 3 mm | ok 10, 3.89 s, 12 mm | fall/ok/fall |
| +y 1.0 | ok 10, 0.10 s, 3 mm | ok 10, 0.10 s, 3 mm | ok 10, 0.10 s, 3 mm | ok/fall/fall |
| −y 0.5 | ok 10, 0.00 s, 2 mm | ok 10, 0.00 s, 2 mm | ok 10, 0.00 s, 2 mm | ok/ok/ok |
| −y 1.0 | **fall** 5, 1.75 s, 87 mm | ok 10, 1.01 s, 15 mm | **fall** 5, 1.22 s, 86 mm | fall/fall/fall |
| −y 1.5 | **fall** 5, 0.84 s, 84 mm | **fall** 5, 0.84 s, 84 mm | **fall** 5, 0.84 s, 84 mm | fall/fall/fall |

The last column is the package before the stage-2 closing fixes, on the same noise (`run_stepping_cfg.py --vel_iters 4 --old_monitor --uncompiled`).  Mixed runs separate the two changes: −x 2.0 with 0 velocity iterations and the old 10°/s trigger falls in all 3 seeds with the same trace as now, and −x 1.75 with 0 velocity iterations falls in 2 of 3 with the old trigger, 1 of 3 now; with 4 velocity iterations the predicted-tilt trigger survives −x 1.75 in all 3.  So the backward limit dropped with the simulator change, not with the monitor (mechanism not investigated).  The lateral pushes gained (+y 0.75 and +y 1.0 in all 3 seeds).

Every push that the robot survives ends with the full 10 steps: after a hold it stands (stage-1 behaviour) until calm and then steps again.  The same pushes with the 25-node prototype: +x 2.5 and −x 2.0 fell, +y 0.75 passed, +y 1.0 fell.

Development history of the gait rules on the same pushes (prototype, 25 nodes): with only the stage-1 hold rule, every sagittal push ≥ 1.4 N·s was survived but the robot never stepped again (the plan ran on during the hold and every step was consumed); plan pause, posture reset and standing recovery made it resume.

For comparison, standing (stage 1, design D, 0 velocity iterations) survives +x 2.5, −x 2.0 and +y 2.0.  While stepping the sagittal limit is somewhat lower (+x 1.75, −x 1.75 in 2 of 3 seeds), the lateral one much lower: a push outward over the stance sole (+y here) moves the DCM past the stance sole with the only foot that could catch it on the wrong side; capturing it needs step placement (stage 3).  Results near the limit flip with the sensor-noise seed (−y 1.0 passes in 1 of 3, −x 1.75 in 2 of 3) and with small changes of the gait logic, so each case is run with 3 seeds.

### Foot drift

Each swing foot lands ≈ 0.31 mm forward of where it lifted off (≈ 3 mm per foot over 20 steps); the stance foot does not move (0.00 mm per single support), so it is not slip.  Sending the swing foot to its initial place instead of its lift-off point did not change it (+0.45 mm per step), the simulator-state run shows the same, and with 0.6/0.2 s timing it is +0.12 mm per step: it happens at touchdown.

Cause: kinematics under load that the model does not see, in three parts of ≈ 0.15 mm each: hip-yaw servo deflection, the soft parallelogram loop closure in Isaac, and the double-support projection plus tracking error.  A hip-yaw encoder alone does not help, because the model has neither pelvis yaw nor a yaw contact row.  The drift is also below the encoder resolution (0.088° ≈ 0.7 mm at the sole).  Left as is (user decision); the real fix (pelvis yaw as a coordinate, a yaw contact row, hip yaw driven by the NMPC) belongs to the walking / turning stage.

## Pitfalls found

- A solver found in its build folder is loaded without rebuilding: a stale prototype build of the same name once crashed the compiled contact algebra (wrong dimension), and a solver folder shared by name was overwritten when the default pose arrived as float32 (−0.20000000298 instead of −0.2 changed the acados hash), so the next run looked for a `.so` that did not exist.  Hence the separate build: the solver folder is named `robonion_nmpc_<hash>` from the OCP expressions and numbers, and the crouched pose is rounded to 6 decimals.
- **PhysX TGS velocity iterations change the joint velocities after the positions are integrated**: with 4 (the default before), a sole lying flat read up to 3.6°/s of tilt rate from encoders + gyro while weight moved (the simulator state gives the same wrong rate, so it is the simulator, not the estimator); with 0 it is 0.55°/s.  `robots/robonionv2.py` now sets `solver_velocity_iteration_count=0`.  This changes the dynamics slightly: in stage 1, 19 of 20 push outcomes stayed the same (−y 2.25 now falls, see `stage1.md`); while stepping, −x 1.75 and −x 2.0 became marginal (next section).
- Results near a limit depend on the sensor-noise realization (AHRS bias, gyro noise): one run per push is not enough.  Every table here uses seeds 0, 1, 2 (`--seed` in `run_stepping.py`, `isaac_gait.py`).
- A touchdown reads as a 12–19°/s sole tilt rate on a single tick at ≈ 0° tilt (Isaac impact, gyro + encoders); a one-tick rate or predicted-tilt trigger takes it for an edge.
- The monitor returns an edge to `flat` only when the tilt crosses zero; a sole resting tilted keeps its mode forever.
- Editing prototype files while a batch of Isaac runs imports them breaks the later runs of the batch (a broken docstring once killed six runs silently).
- Offline success is necessary, not sufficient: HybridSim has no slip and is accurate laterally only for ≈ 0.1 s (stage 1); the tilt-rate problems above exist only in Isaac.

## Open

- Contact-mode monitor thresholds (0.2°, 15 ms lead, 2 ticks) and the estimator were tuned in Isaac, including simulator artefacts (touchdown spike, false tilt rate): check them on the robot (standing, slow stepping) before relying on them.
- Jetson AGX Orin timing: record a run on the desktop (`scripts/run_stepping.py --record run.npz`), build and replay it on the Jetson with `scripts/build_controller.py` and `scripts/benchmark_stepping.py run.npz` (no Isaac needed).
- Lateral push recovery while stepping needs footstep adaptation (stage 3).
- Landing drift ≈ 0.31 mm per step: needs pelvis yaw in the model (walking / turning stage).
- Ground-height errors beyond ±5 mm (−10 mm falls).
- −x 1.75 / −x 2.0 N·s while stepping were survived with 4 TGS velocity iterations and are marginal / fall with 0 (see *Pushes*).

## Prototype code

In `outputs/stage2_prototypes/` (git-ignored scratch), run from that folder with `uv run --project ../.. python <script>`.  They import the package modules as they were before the rename to `nmpc.py` / `controller.py` / `robonion_params.py`, so they only run with the package at git tag `stage2` (check that tag out first):

| File | Content |
|---|---|
| `check_gait.py` | gait reference check (DCM / CoM consistency, sway, swing joint rates) |
| `gait.py`, `gait_nmpc.py`, `gait_loop.py` | prototype plan, NMPC (grid, solver options, swing weight as options) and gait logic with test inputs (`ground_error`, `edge_policy`, `swing_target`, `no_events`) |
| `gait_offline.py` | closed loop on the stage-1 `HybridSim` (`--split`, `--grid`, `--ground_error`, `--save`) |
| `isaac_gait.py` | Isaac run: estimator or `--truth_state`, pushes, ground-height error, estimation error, foot placement per step, `--trace` of contact modes and plan phases, `--lead` / `--rate_ticks` / `--rate_on` of the monitor, `--seed` |
| `run_stepping_cfg.py` | copy of `scripts/run_stepping.py` with switches for paired runs on the same noise: `--vel_iters`, `--old_monitor` (10°/s, one tick), `--uncompiled`, `--pred_ticks` |
| `vel_check.py`, `vel_flat.py`, `vel_episodes.py`, `vel_analyze.py` | Isaac joint velocity vs joint-angle derivative at every 1 ms substep, solver variants; false tilt rate on flat soles |
| `monitor2.py`, `monitor_trace.py` | monitor variants (persistence, predicted tilt); tilt / rate / predicted tilt around given times of a `--record` file |
| `replay_rec.py` | replay a `--record` file through the package (or `stepping_sm.py`, `compiled.py`) controller: target difference, state / hold timeline |
| `stepping_sm.py`, `compiled.py`, `pkg_next/` | prototypes of the state machine, of the compiled estimation path and of the package version that took both (all moved into the package) |
| `drift_analyze.py`, `drift_est.py` | landing drift per step, from simulator truth and from the estimator |
