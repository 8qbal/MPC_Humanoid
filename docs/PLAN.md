# Isaac Lab MPC Plan for Robinion Humanoid Movement

## Goal and scope

Build a simulation-first, receding-horizon controller for the Robinion v2 humanoid in Isaac Lab.  The first usable milestone is stable standing and commanded planar walking on a flat floor; rough terrain, vision, and hardware deployment are later milestones.

The controller must respect the robot's floating base, contact forces, actuator torque/speed limits, joint limits, and the two parallelogram leg mechanisms.  It must control only independent actuated coordinates: hip yaw, hip roll, front-thigh pitch (hip pitch motor), ankle pitch (ankle pitch motor), ankle roll, torso, and optionally arms.  The passive four-bar joints (`*_back_thigh_pitch`, `*_knee_pitch`, `*_front_shin_pitch`, `*_back_shin_pitch`) remain uncommanded and are constrained by the USD loop closures -- note `front_shin_pitch`, not `ankle_pitch`, is the passive one; the ankle-pitch motor drives the shin parallelogram from its distal end (user-confirmed, see `references/docs/joint_info.md`).

## Current roadmap

Target: fast (0.3--0.4 m/s), reasonably natural walking that stays stable under pushes, with MPC as the only control law (no RL).  Servos run in Dynamixel position mode (mode 3), so the controller outputs joint position targets.  The real robot has no foot force/contact sensors; the `ContactSensorCfg` in simulation is ground truth for validation only.

The core controller is a DCM MPC re-solved every control tick (200 Hz) from the measured state, so it acts as both planner and stabilizer:

```text
estimator -> c, c_dot, xi = c + c_dot / omega
  -> DCM MPC (ZMP, later footstep position and timing)
  -> CoM reference -> IK (Pinocchio + parallelogram coupling) -> q_des -> JointPositionAction
```

Stages, each built on the previous one:

1. **Stabilizer** -- double-support standing that recovers from pushes without stepping (ZMP moved inside the hull of both soles).
2. **Stepping in place** -- gait scheduler, single support, alternating support polygon, swing-foot spline.
3. **Fast walking without arm swing** -- footstep position and step timing as MPC decision variables, 0.3--0.4 m/s.
4. **Fast walking** -- add counter-phase arm swing and other naturalness tuning.
5. *Maybe:* **stochastic MPC** -- chance-constrained or tube MPC on top of the same QP for noise and disturbance robustness.

Success metrics: maximum recoverable push impulse (N·s) per direction, velocity RMSE, falls per 100 pushes, no servo hitting its velocity limit.

## Architecture

```text
velocity command / mode (stand, walk, stop)
          |
          v
gait scheduler ------------------> contact sequence, nominal step timing
          |                                  |
          v                                  v
DCM MPC (200 Hz) <----------------- state estimator
          |       (CoM, DCM, feet, contacts estimated from gait schedule + IMU + kinematics)
          v
ZMP plan, CoM reference, footstep position and timing
          |
          v
swing-foot spline + IK (Pinocchio, parallelogram coupling)
          |
          v
joint position targets --> Dynamixel position mode (sim: JointPositionAction) --> contacts
```

1. The DCM MPC is a small QP re-solved every control tick from the measured state, so it is both the gait planner and the balance stabilizer.  The ZMP is its input; the servos never receive it directly -- the realised ZMP follows from the commanded CoM motion.
2. The IK maps the CoM reference, stance/swing foot poses and torso orientation to independent joint position targets.  The foot task is 5-D: foot pitch is locked to pelvis pitch by the parallelograms.
3. Centroidal or whole-body MPC (e.g. whole-body MPC with a learned residual model) may later replace the DCM MPC + IK behind the same controller interface; the DCM controller is the baseline they are compared against.  Neither is on the critical path.

Use OSQP (or `qpsolvers`) for the MPC and IK QPs and Pinocchio for kinematics; both are already in `.venv`.  `acados_template` stays available for later nonlinear MPC variants.

## Phase 0 — Freeze interfaces and validate the simulation model

1. Read and preserve the asset workflow in `AGENTS.md` and `references/docs/joint_info.md`; do not regenerate `assets/robonionv2.usd` unless the asset pipeline is deliberately being changed.
2. Replace the cart-pole placeholder task with a Robinion-specific direct or manager-based environment.  The scene must load `ROBINION_CFG`, a flat ground plane, contact sensors, and deterministic physics settings.
3. Create one authoritative `RobinionModelSpec` containing the ordered independent actuated joints, passive joints, foot body names, pelvis body name, nominal standing pose, limits, torque/speed limits, foot sole dimensions, mass, and control periods.  Reuse it in task configuration, observation extraction, MPC, WBC, and tests to prevent ordering errors.
4. Add a headless smoke script that spawns one robot, resets it to the crouch pose, steps for 10 seconds under gravity, and reports NaNs, joint-limit violations, base height, and contact forces.
5. Verify physically before controller work: correct left/right foot contacts, no self-collision explosions, stable passive-link loop closure, correct base frame convention (X forward/Y left/Z up), and expected total mass.  Record baseline traces and solver/physics version.

**Exit criteria:** the unactuated robot settles without numerical instability; foot contact forces and body/joint names are confirmed programmatically.

## Phase 1 — State, kinematics, and reference generation

1. Implement a batched `HumanoidState` adapter from Isaac Lab articulation and contact-sensor buffers.  At minimum expose base pose/twist, independent joint positions/velocities, all body poses/velocities, CoM, centroidal momentum, left/right foot poses, and contact state/normal force.
2. Implement robust contact classification with hysteresis and a minimum dwell time, estimated from gait schedule, leg kinematics and IMU (the real robot has no foot force sensors).  The sim `ContactSensorCfg` feeds the same interface only as ground truth for comparison and logging; never infer contact only from foot height.
3. Implement the independent-coordinate mapping for each parallelogram leg and tests that compare reconstructed passive angles with measured USD articulation angles.
4. Add kinematics helpers for the sole frame, CoM, support polygon, stance/swing selection, and terrain height query.  Keep all tensor shapes explicit as `(num_envs, ...)` and define one frame convention for every vector.
5. Add command and reference modules: constant velocity command, stop command, bounded yaw-rate command, phase-based alternating contact schedule, and swing-foot trajectories (quintic horizontal interpolation plus configurable clearance).
6. Start with a fixed cadence, double-support transition, and fixed nominal step length/width.  Add adaptive footsteps only after nominal gait succeeds.

**Exit criteria:** visualized/logged reference feet alternate correctly; state and reference unit tests pass for single- and multi-environment batches.

## Phase 2 — DCM MPC

1. Model: LIPM with constant CoM height, `omega = sqrt(g / h)`, DCM `xi = c + c_dot / omega`, exact discretisation `xi[k+1] = e^(omega*dt) xi[k] + (1 - e^(omega*dt)) p[k]`.  Use the robot mass and CoM height from the verified model.
2. Decision variables: ZMP `p[k]` over the horizon (start: 0.02 s nodes, 0.5 s horizon for standing; about 1 s / two steps for walking).  From stage 3 add the next footstep positions and the current step duration (via `sigma = e^(-omega*T)` to stay a QP).
3. Cost: DCM/velocity tracking, ZMP near the sole centre, ZMP rate, footstep deviation from nominal, step duration deviation from nominal.  Expose weights through a versioned configuration file rather than hard-coding them.
4. Hard constraints: ZMP inside the support polygon (hull of both soles in double support, stance sole in single support), footstep reachability and minimum lateral spacing, step-duration bounds derived from servo speed limits, and a terminal DCM (capturability) constraint.
5. Re-solve every control tick from the measured DCM; apply only the first ZMP and integrate it to the next CoM reference.  On solver failure, reuse the shifted previous solution for one tick and enter `safe_stop` after a bounded number of failures.
6. Write an offline deterministic test harness using recorded Isaac Lab states.  Test feasibility for standing, pushes of known impulse, stepping in place and slow walking before connecting to live simulation.

**Exit criteria:** solves meet the 5 ms control deadline, the ZMP stays inside the support polygon, and the offline harness recovers pushes whose DCM stays inside the support polygon.

## Phase 3 — IK and Isaac Lab position actuation

1. Implement a kinematic IK QP (Pinocchio model in independent coordinates via the linear parallelogram coupling) over joint velocities, integrated to joint position targets.
2. Track, in priority/weighted form: CoM reference, stance-foot stationarity, swing-foot trajectory (5-D, no foot pitch), torso orientation via `torso_pitch_joint`, nominal posture, and low arm/head motion.  Hold arms and head at safe poses until the arm-swing stage.
3. Send the IK output through `JointPositionActionCfg`; the `DCMotorCfg` stiffness/damping model the Dynamixel position loop and should be identified from the register gains used on the real robot.  Never command the four passive linkage joints.
4. Because mode 3 has no current limit, add software guards: joint-position clamps, joint-velocity limits, per-tick target-jump limits, base-tilt, base-height and contact-loss checks.  Clamp commands before writing them and report which guard triggered.
5. Implement controller modes: `reset`, `stand`, `walk`, `recover`, and `safe_stop`.  Require a verified stable stand before allowing walk; a fall, persistent solver failure, or invalid state must enter `safe_stop` and reset the environment.

**Exit criteria:** the stabilizer holds the robot under small pushes; IK tracks a swing foot without sliding the stance foot; no commands are sent to passive joints.

## Phase 4 — Closed-loop gait progression

1. Integrate scheduler, DCM MPC, IK, and environment in a single `MpcHumanoidController` called at the configured decimated control rate.  Keep a pure-Python offline mode and a live Isaac Lab mode behind the same API.
2. Follow the stages in *Current roadmap*: stabilizer, stepping in place, fast walking without arm swing, fast walking with arm swing.  Within walking, tune slow forward walking, then lateral velocity, yaw turns, stop/start transitions and recovery pushes.
3. Use a curriculum in command magnitude and perturbation strength.  Do not increase speed until the previous stage meets its quantitative criteria over multiple randomized resets.
4. Record episodes in structured logs (configuration hash, seed, physics dt, state, contact state, measured vs planned ZMP, MPC status/solve time/cost, planned footsteps, IK residuals, actions, and termination reason).  Add an optional visualization of planned CoM, ZMP and foot placements.

**Exit criteria:** repeatable 0.3--0.4 m/s straight walking for a predefined duration and reset set without falls, deadline misses, or safety-guard violations, and push recovery by stepping.

## Phase 5 — Validation, robustness, and sim-to-real preparation

1. Create automated regression scenarios: stand, zero-velocity command, forward/lateral/yaw commands, abrupt stop, low-friction ground, small height variations, impulse pushes, delayed/noisy state estimates, and contact-sensor noise.
2. Define success metrics: maximum recoverable push impulse per direction, fall rate, distance/time before failure, velocity RMSE, stance-foot slip, peak joint torque/velocity, MPC/IK feasible-solve rate, p50/p95 solve latency, and safety-stop count.
3. Add randomized but bounded mass, CoM, joint damping/friction, actuator strength/latency, contact friction, and sensor noise.  Keep nominal performance and robust performance as separate reports.
4. Compare the DCM MPC against a standing-PD baseline and the existing RL reference only as benchmarks; do not mix their controllers in the first MPC evaluation.
5. Before hardware, replace simulator-only state inputs with an estimator interface, calibrate encoder zero offsets and joint/foot frames, identify the Dynamixel position-loop response (gains, latency) and limits, add an independent emergency-stop path, and start with a tethered standing test.  Hardware actuation is out of scope until these checks are approved.

## Proposed repository changes

```text
source/MPC_Humanoid/MPC_Humanoid/
  robots/robonionv2.py                 # retain asset + actuator configuration
  tasks/mpc_humanoid/
    config/robinion/                   # real Isaac Lab task and physics presets
    mdp/                                # observations, actions, events, terminations
  mpc/
    model_spec.py                       # one joint/body/contact source of truth
    state.py                            # Isaac Lab state adapter
    contacts.py                         # contact filtering and schedules
    references.py                       # command, footsteps, swing trajectories
    dcm_mpc.py                          # DCM MPC QP (ZMP, footsteps, step timing)
    ik.py                               # kinematic IK QP -> joint position targets
    controller.py                       # modes, rate scheduling, fallbacks
    logging.py                           # episode diagnostics
scripts/
  run_mpc.py                            # live headless/interactive runner
  validate_mpc.py                       # deterministic scenario suite
tests/
  test_model_spec.py
  test_parallelogram_mapping.py
  test_contacts.py
  test_dcm_mpc.py
  test_ik.py
  test_mpc_integration.py
```

Keep the generated cart-pole configuration isolated until the Robinion task is registered and its smoke tests pass; then retire or rename the placeholder registration so users cannot accidentally run it as the humanoid task.

## Decisions required before implementation

1. ~~Confirm the physical motor assignment for the thigh stage~~ -- resolved: 5 motors/leg (hip yaw, hip roll, hip pitch = `front_thigh_pitch`, ankle pitch, ankle roll), `front_shin_pitch` is passive not `ankle_pitch`; the left/right front-thigh effort asymmetry (9.9 vs 19.8 N·m) in the URDF is confirmed real, not a typo -- both per user confirmation, see `references/docs/joint_info.md`.
2. ~~Measure and confirm the real effective parallelogram link length~~ -- resolved: use the URDF value of 0.20 m (user decision); the 0.18 m in the IK reference script is not used.
3. ~~Confirm the intended low-level hardware command mode~~ -- resolved: Dynamixel position control (mode 3, no current limit); the controller emits joint position targets (user decision).
4. ~~Select the initial walking target~~ -- resolved: flat ground, staged per *Current roadmap* up to 0.3--0.4 m/s with push recovery (user decision).  Still open: target compute hardware and deadline.
5. ~~Foot sole dimensions~~ -- resolved: take them from the CAD (foot collision geometry) for the ZMP constraints (user decision).
6. ~~Foot sensors~~ -- resolved: the real robot has no foot force/contact sensors; contact is estimated and there is no measured CoP/ZMP (user decision).

Until these are resolved, use conservative simulator-only limits, document them as assumptions, and keep all values configurable.
