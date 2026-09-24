# Isaac Lab MPC Plan for Robinion Humanoid Movement

## Goal and scope

Build a simulation-first, receding-horizon controller for the Robinion v2 humanoid in Isaac Lab.  The first usable milestone is stable standing and commanded planar walking on a flat floor; rough terrain, vision, and hardware deployment are later milestones.

The controller must respect the robot's floating base, contact forces, actuator torque/speed limits, joint limits, and the two parallelogram leg mechanisms.  It must control only independent actuated coordinates: hip yaw, hip roll, front-thigh pitch (hip pitch motor), ankle pitch (ankle pitch motor), ankle roll, torso, and optionally arms.  The passive four-bar joints (`*_back_thigh_pitch`, `*_knee_pitch`, `*_front_shin_pitch`, `*_back_shin_pitch`) remain uncommanded and are constrained by the USD loop closures -- note `front_shin_pitch`, not `ankle_pitch`, is the passive one; the ankle-pitch motor drives the shin parallelogram from its distal end (user-confirmed, see `references/docs/joint_info.md`).

## Architecture

```text
velocity / waypoint command
          |
          v
contact & footstep scheduler ----> desired contact sequence
          |                                  |
          v                                  v
centroidal MPC <------------------- robot state estimator
          |       (CoM, momentum, contacts, feet)
          v
desired CoM / wrench / swing-foot trajectories
          |
          v
whole-body inverse dynamics QP
          |
          v
joint torque or impedance targets --> Isaac Lab articulation --> contacts
```

Use a two-level controller rather than a single full-body nonlinear MPC initially:

1. A centroidal MPC optimizes center-of-mass motion and contact wrenches over a fixed contact schedule.  This is small enough to solve consistently at 50--100 Hz.
2. A whole-body controller (WBC) converts the MPC outputs into torque commands while enforcing floating-base dynamics, stance-foot constraints, friction cones, joint bounds, and swing-foot tracking.  Run it at the physics control rate (target 200--500 Hz).
3. Keep a full-body nonlinear MPC prototype behind the same controller interface for later comparison.  Do not make it the critical path to first walking.

`acados_template` is already declared as a local project dependency; use it for the centroidal optimal-control problem.  Use a sparse QP backend (for example OSQP or the solver available through Isaac Lab) for the WBC after checking dependency compatibility with the existing environment.

## Phase 0 — Freeze interfaces and validate the simulation model

1. Read and preserve the asset workflow in `AGENTS.md` and `references/docs/joint_info.md`; do not regenerate `assets/robonionv2.usd` unless the asset pipeline is deliberately being changed.
2. Replace the cart-pole placeholder task with a Robinion-specific direct or manager-based environment.  The scene must load `ROBINION_CFG`, a flat ground plane, contact sensors, and deterministic physics settings.
3. Create one authoritative `RobinionModelSpec` containing the ordered independent actuated joints, passive joints, foot body names, pelvis body name, nominal standing pose, limits, torque/speed limits, foot sole dimensions, mass, and control periods.  Reuse it in task configuration, observation extraction, MPC, WBC, and tests to prevent ordering errors.
4. Add a headless smoke script that spawns one robot, resets it to the crouch pose, steps for 10 seconds under gravity, and reports NaNs, joint-limit violations, base height, and contact forces.
5. Verify physically before controller work: correct left/right foot contacts, no self-collision explosions, stable passive-link loop closure, correct base frame convention (X forward/Y left/Z up), and expected total mass.  Record baseline traces and solver/physics version.

**Exit criteria:** the unactuated robot settles without numerical instability; foot contact forces and body/joint names are confirmed programmatically.

## Phase 1 — State, kinematics, and reference generation

1. Implement a batched `HumanoidState` adapter from Isaac Lab articulation and contact-sensor buffers.  At minimum expose base pose/twist, independent joint positions/velocities, all body poses/velocities, CoM, centroidal momentum, left/right foot poses, and contact state/normal force.
2. Implement robust contact classification with hysteresis and a minimum dwell time.  Log both raw normal force and the filtered contact state; never infer contact only from foot height.
3. Implement the independent-coordinate mapping for each parallelogram leg and tests that compare reconstructed passive angles with measured USD articulation angles.
4. Add kinematics helpers for the sole frame, CoM, support polygon, stance/swing selection, and terrain height query.  Keep all tensor shapes explicit as `(num_envs, ...)` and define one frame convention for every vector.
5. Add command and reference modules: constant velocity command, stop command, bounded yaw-rate command, phase-based alternating contact schedule, and swing-foot trajectories (quintic horizontal interpolation plus configurable clearance).
6. Start with a fixed cadence, double-support transition, and fixed nominal step length/width.  Add adaptive footsteps only after nominal gait succeeds.

**Exit criteria:** visualized/logged reference feet alternate correctly; state and reference unit tests pass for single- and multi-environment batches.

## Phase 2 — Centroidal MPC

1. Define the reduced state as CoM position/velocity and angular momentum (or centroidal momentum), with each contacting foot wrench as the control.  Use the robot mass and gravity from the verified model.
2. Formulate a finite-horizon, multiple-shooting OCP with a 0.01--0.02 s node interval and an initial 0.5--1.0 s horizon.  Warm-start each solve by shifting the prior solution.
3. Penalize velocity/yaw tracking, CoM reference error, angular momentum, foot-wrench magnitude/rate, and terminal state error.  Expose weights through a versioned configuration file rather than hard-coding them.
4. Add hard constraints: normal force non-negativity and maximum, linearized friction cone, center-of-pressure bounds within the foot sole, zero wrench for swing feet, conservative CoM/base-height bounds, and optional wrench-rate limits.
5. Generate a contact schedule parameter per horizon node; use double support around contact transitions.  On solver failure or deadline miss, hold the last feasible wrench for one tick and transition to a safe stand controller after a bounded number of failures.
6. Write an offline deterministic test harness using recorded Isaac Lab states.  Test solver feasibility for standing, left/right single support, and a low-speed walk before connecting the optimizer to live simulation.

**Exit criteria:** solves meet the control deadline on the target machine, respect all force/friction constraints, and track a standing and low-speed walking reference in the offline harness.

## Phase 3 — Whole-body controller and Isaac Lab actuation

1. Implement a constrained inverse-dynamics QP over generalized accelerations, actuated torques, and contact forces.  Include floating-base rigid-body dynamics, stance-foot acceleration constraints, joint acceleration/torque bounds, friction constraints, and passive-joint handling consistent with the four-bar mechanism.
2. Track, in priority/weighted form: MPC net wrench and CoM acceleration, stance-foot stationarity, swing-foot trajectory, torso orientation, nominal posture, and low arm/head motion.  During initial walking, hold arms and head at safe poses.
3. Map WBC output to Isaac Lab.  Prefer `JointEffortActionCfg` for independently actuated joints when simulator dynamics and actuator models support it; otherwise use explicitly documented impedance targets plus feed-forward torque.  Never command the four passive linkage joints.
4. Add torque, torque-rate, velocity, joint-position, base-tilt, base-height, and contact-loss safety guards.  Clamp commands before writing them to the simulator and report which guard triggered.
5. Implement controller modes: `reset`, `stand`, `walk`, `recover`, and `safe_stop`.  Require a verified stable stand before allowing walk; a fall, persistent solver failure, or invalid state must enter `safe_stop` and reset the environment.

**Exit criteria:** stand controller holds the robot under small pushes; WBC tracks a swing foot without sliding the stance foot; no commands are sent to passive joints.

## Phase 4 — Closed-loop gait progression

1. Integrate scheduler, centroidal MPC, WBC, and environment in a single `MpcHumanoidController` called at the configured decimated control rate.  Keep a pure-Python offline mode and a live Isaac Lab mode behind the same API.
2. Tune in this order: static double support, weight shifts, single-leg swing in place, slow stepping, low-speed forward walking, lateral velocity, yaw turns, stop/start transitions, then recovery pushes.
3. Use a curriculum in command magnitude and perturbation strength.  Do not increase speed until the previous stage meets its quantitative criteria over multiple randomized resets.
4. Record episodes in structured logs (configuration hash, seed, physics dt, state, contact state, MPC status/solve time/cost, planned wrenches, WBC residuals, actions, and termination reason).  Add an optional visualization of planned CoM and foot placements.

**Exit criteria:** repeatable 0.1--0.2 m/s straight walking for a predefined duration and reset set without falls, deadline misses, or safety-guard violations.

## Phase 5 — Validation, robustness, and sim-to-real preparation

1. Create automated regression scenarios: stand, zero-velocity command, forward/lateral/yaw commands, abrupt stop, low-friction ground, small height variations, impulse pushes, delayed/noisy state estimates, and contact-sensor noise.
2. Define success metrics: fall rate, distance/time before failure, velocity RMSE, stance-foot slip, peak torque/velocity, friction violations, MPC/WBC feasible-solve rate, p50/p95 solve latency, and safety-stop count.
3. Add randomized but bounded mass, CoM, joint damping/friction, actuator strength/latency, contact friction, and sensor noise.  Keep nominal performance and robust performance as separate reports.
4. Compare centroidal MPC/WBC against a standing-PD baseline and the existing RL reference only as benchmarks; do not mix their controllers in the first MPC evaluation.
5. Before hardware, replace simulator-only state inputs with an estimator interface, calibrate encoder zero offsets and joint/foot frames, validate torque/current conversions and Dynamixel limits, add an independent emergency-stop path, and start with a tethered standing test.  Hardware actuation is out of scope until these checks are approved.

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
    centroidal.py                       # acados OCP and warm-start policy
    whole_body_qp.py                    # inverse-dynamics QP
    controller.py                       # modes, rate scheduling, fallbacks
    logging.py                           # episode diagnostics
scripts/
  run_mpc.py                            # live headless/interactive runner
  validate_mpc.py                       # deterministic scenario suite
tests/
  test_model_spec.py
  test_parallelogram_mapping.py
  test_contacts.py
  test_centroidal_mpc.py
  test_whole_body_qp.py
  test_mpc_integration.py
```

Keep the generated cart-pole configuration isolated until the Robinion task is registered and its smoke tests pass; then retire or rename the placeholder registration so users cannot accidentally run it as the humanoid task.

## Decisions required before implementation

1. ~~Confirm the physical motor assignment for the thigh stage~~ -- resolved: 5 motors/leg (hip yaw, hip roll, hip pitch = `front_thigh_pitch`, ankle pitch, ankle roll), `front_shin_pitch` is passive not `ankle_pitch`; the left/right front-thigh effort asymmetry (9.9 vs 19.8 N·m) in the URDF is confirmed real, not a typo -- both per user confirmation, see `references/docs/joint_info.md`.
2. ~~Measure and confirm the real effective parallelogram link length~~ -- resolved: use the URDF value of 0.20 m (user decision); the 0.18 m in the IK reference script is not used.
3. Confirm the intended low-level hardware command mode: current/torque, position with current limit, or position-only.  This determines whether the hardware-facing WBC emits torque or impedance targets.
4. Select the initial walking target (recommended: flat ground, 0.1 m/s forward, no arm swing) and the target compute hardware/deadline.

Until these are resolved, use conservative simulator-only limits, document them as assumptions, and keep all values configurable.
