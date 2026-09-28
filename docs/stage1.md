# Stage 1 — standing stabilizer: designs tried and results

Goal: double-support standing that recovers from pushes without stepping, running every control tick (200 Hz) and outputting joint position targets for the Dynamixel servos (position mode 3).  Only what the real robot measures is used: joint encoders and the Xsens MTi-630 AHRS (orientation + gyro).  There are no foot force/contact sensors.

## Status

| Design | Where | Standing | Pushes |
|---|---|---|---|
| A. DCM MPC + LIPM + IK (below) | removed from the package (git history) | **unstable**: oscillates, `safe_stop` within 0.8 s | – |
| A + Smith predictor | prototype, reverted | stable | worse than the passive baseline |
| B. NMPC, flat-foot reduced model | prototype (acados, ≈ 1 ms/solve) | stable, CoM to the sole centre, servo sag compensated automatically | ≈ passive baseline; depends mostly on where the CoM is held |
| C. NMPC, soft-contact floating-base model | prototype (acados IRK, 5–55 ms/solve) | stable | promising once (+x 2.5 N·s: 2.5° sole tilt vs 15.9° passive, offline) but the QP fails after larger pushes |
| D. NMPC, rigid contacts, contact mode per foot, acting in every mode | prototype | stable | worse than passive: fights the sole rolling on an edge |
| **D. same, NMPC only while both soles are flat** | **package: `mpc/model.py`, `estimator.py`, `nmpc.py`, `controller.py`; `scripts/run_stabilizer.py`** | stable | **never worse than passive, better in 2–3 of 10 pushes (2 with the stage-2 PhysX settings)** |

Stage 1 is closed with design D in the package; A–C remain only as records here and as prototypes (see *Prototype code*).  Push recovery beyond the passive limit of the soles has to come from stepping (stage 2 on), not from a standing controller.

All results below are Isaac Lab, GPU PhysX, **one environment per process**, one push per run (constant force on `upper_body_link` for 0.05 s at t = 2 s), unless stated otherwise.  Multi-environment runs gave different outcomes than single-environment runs, even for env 0 (see *Pitfalls*), so they are not used for any number here.

## Passive baseline (servos hold the default pose)

Largest push on `upper_body_link` (constant force for 0.05 s) that the robot survives with the servos only holding the default pose, from `scripts/check_estimator.py` (removed with design A; fall = root height well below the standing 0.554 m, or the estimator reporting `fallen`).  The theoretical limit is `J_max = m ω · margin`, with the ZMP jumping straight to the sole edge and the standing CoM 2.6 cm behind the sole centre.

| Direction | Survives | Falls | Theoretical limit |
|---|---|---|---|
| +x (forward) | 2.5 N·s | 3.0 N·s | 4.0 N·s |
| −x (backward) | 1.75 N·s | 2.0 N·s | 2.1 N·s |
| +y (left) | 2.0 N·s | 2.5 N·s | 3.1 N·s |
| −y (right) | 2.25 N·s | 2.5 N·s | 3.1 N·s |

The stabilizer has to beat these numbers; backward is the weakest direction because the standing CoM is close to the heel.

Single-push rerun on GPU PhysX (one env, deterministic: repeated runs identical).  "slip" = survived but a foot slid more than 5 mm; numbers are the peak foot slip:

| Push (N·s) | +x 2.5 | +x 3.0 | −x 1.75 | −x 2.0 | −x 2.5 | +y 2.0 | +y 2.5 | −y 2.25 | −y 2.5 |
|---|---|---|---|---|---|---|---|---|---|
| passive | ok (6 mm) | fall | fall | fall | fall | slip 59 mm | fall | fall | fall |

The −x 1.75 N·s result differs from the table above (CPU PhysX, alternating pushes from `scripts/check_estimator.py`).

## Design A — DCM MPC + LIPM + IK (removed from the package)

Double-support standing controller that recovers from pushes without stepping.  It runs every control tick (200 Hz) and outputs joint position targets for the Dynamixel servos (position mode 3).  No foot force sensors and no simulator ground truth (`base_lin_vel`, `base_ang_vel`, `projected_gravity`) are used.  Blocks: [A] estimator, [B] DCM MPC, [C] LIPM step, [D] IK, [E] safety.

### Frames

- **S (support frame):** origin at the midpoint of the two sole centres on the ground, z opposite to gravity, x forward, y left.  All horizontal quantities (`c`, `xi`, `p`) are expressed in S.
- **B (base frame):** `base_link` of the controller model.

### Variables

#### Inputs (from the env / hardware)

| Symbol | Code name | Dim | Unit | Source |
|---|---|---|---|---|
| `q_a` | `q_act` | 21 | rad | encoders; env obs `joint_pos_rel` + `q_default` |
| `q̇_a` | `qd_act` | 21 | rad/s | encoders; env obs `joint_vel_rel` |
| `quat` | `imu_quat_xyzw` | 4 (x, y, z, w) | — | AHRS orientation of `imu_link` (MTi-630); env obs `imu_orientation`, normalised before use |
| `ω_I` | `gyro` | 3 | rad/s | gyro in the `imu_link` frame (MTi-630); env obs `imu_ang_vel` |

#### Internal signals

| Symbol | Code name | Dim | Unit | Produced by | Meaning |
|---|---|---|---|---|---|
| `q` | `q_full` | 29 | rad | [A] | all revolute joints incl. passive ones, `q = G q_a` |
| `c` | `c` | 2 | m | [A] | measured CoM (x, y) in S |
| `ω_B` | `v[3:6]` | 3 | rad/s | [A] | pelvis angular velocity, from the gyro minus the torso-pitch rate |
| `ċ` | `cdot` | 2 | m/s | [A] | measured CoM velocity from gyro + joint velocities (Jacobians), low-pass filtered |
| `ξ` | `xi` | 2 | m | [A] | measured DCM, `xi = c + ċ / ω` |
| `h_c` | `com_height` | 1 | m | [A] | estimated CoM height above the sole plane |
| `fallen` | `fallen` | bool | — | [A] | `h_c < h_min`: the feet-flat model no longer holds (robot falling or down) |
| `p` | `p` | 2 × N | m | [B] | planned ZMP over the horizon (per axis) |
| `p₀` | `p0` | 2 | m | [B] | first planned ZMP, the only one applied |
| `p_prev` | `p_prev` | 2 | m | [B] | `p0` of the previous tick (for the ZMP-rate cost) |
| `c_ref` | `c_ref` | 2 | m | [C] | CoM reference for the next tick |
| `ċ_ref` | `cdot_ref` | 2 | m/s | [C] | CoM velocity reference for the next tick |
| `v` | `v` | 6 + 11 | m/s, rad/s | [D] | IK velocity: base twist + leg (10) and torso (1) joint velocities |
| `q_des` | `q_des` | 21 | rad | [D], [E] | joint position targets |

#### Output

| Symbol | Code name | Dim | Unit | Destination |
|---|---|---|---|---|
| `q_des − q_default` | `action` | 21 | rad | env action term `joint_pos` (`JointPositionActionCfg`, `use_default_offset=True`) |

#### Parameters

| Symbol | Code name | Initial value | Unit | Source / note |
|---|---|---|---|---|
| `g` | `g` | 9.81 | m/s² | |
| `m` | `mass` | 7.9325 | kg | total mass of `assets/robonionv2_controller.urdf` (= USD, after the fixes) |
| `h` | `h` | 0.480 | m | CoM height above the sole at the standing pose (straight legs), from FK of the controller model |
| `ω` | `omega` | `sqrt(g / h)` ≈ 4.52 | rad/s | |
| `dt` | `dt` | 0.005 | s | sim dt 1 ms × decimation 5 |
| `Δ` | `dt_mpc` | 0.02 | s | MPC node interval |
| `N` | `horizon` | 25 | — | 0.5 s horizon |
| `a` | `a` | `e^(ω Δ)` ≈ 1.09 | — | |
| `Q`, `R`, `S` | `w_xi`, `w_zmp`, `w_dzmp` | 1, 0.1, 1 | — | DCM tracking, ZMP centring, ZMP rate |
| `m_zmp` | `zmp_margin` | 0.01 | m | safety margin inside the sole edges |
| `ξ_ref`, `p_ref` | `xi_ref`, `p_ref` | (0, 0) | m | origin of S = sole centre (user decision): equal 8.75 cm margin to heel and toe edges; the current standing CoM sits at x ≈ −0.028 in S |
| `p_min`, `p_max` | `zmp_bounds` | x: ±0.0875, y: ±0.0885 | m | hull of both soles in S at the standing pose; from the foot mesh `*_foot_visual.stl` (CAD) and FK |
| sole size | `sole_*` | length 0.175 (+0.068 / −0.107 from the ankle-roll axis), width 0.067, depth 0.056 below the ankle-roll axis | m | foot mesh `*_foot_visual.stl` (CAD) |
| `f_c` | `vel_cutoff` | 50 | Hz | low-pass cutoff for `ċ`; 20 Hz lagged the push peak by ≈ 14 %, 50 Hz by ≈ 6 % (`scripts/check_estimator.py`) |
| `h_min` | `min_com_height` | 0.25 | m | below this the estimator reports `fallen` |
| `sole_centre` | `SOLE_CENTRE` | (−0.0195, 0, −0.056) | m | sole centre in `*_foot_roll_link`, from the foot mesh (CAD) |
| `q̇_max` | `qd_max` | 4.08 (legs) | rad/s | URDF velocity limit |
| `θ_max` | `tilt_limit` | 20 | deg | base roll/pitch that triggers `safe_stop` |

### Equations

#### [A] Estimator

Parallelogram coupling (`q = G q_a`):

```text
knee = −front_thigh        back_thigh = front_thigh
front_shin = −ankle_pitch  back_shin  = front_shin
```

Model: `assets/robonionv2_controller.urdf` (Pinocchio, free-flyer root `lower_body_link`).  Forward kinematics with the base orientation taken from the AHRS (roll/pitch only, yaw removed, base position 0) gives the CoM and both sole centres `o_L`, `o_R`; `o = (o_L + o_R) / 2` is the origin of S.

```text
R_WB = R_WI(quat) · R_BI(q)ᵀ, then yaw removed          (IMU sits on upper_body_link)
c    = (CoM − o)_xy
h_c  = (CoM − o)_z
```

The CoM velocity comes from the gyro and the joint velocities through Jacobians, not from differentiating `c` (differentiating the orientation-noisy `c` gave ±8 cm of noise on `ξ`):

```text
v    = [ 0 ; ω_B ; G q̇_a ]                               base linear velocity left 0: it moves CoM and soles alike and cancels
ω_B  : solved from ω_I = J_I,ang v                       (J_I = LOCAL Jacobian of imu_link; removes the torso-pitch rate)
ċ    = lowpass( [ J_com − (J_oL + J_oR) / 2 ]_xy v , f_c )
J_o  = J_lin − [R s]× J_ang                              (sole-centre point Jacobian, LOCAL_WORLD_ALIGNED, s = sole_centre)
ξ    = c + ċ / ω                                         ω = sqrt(g / h), the same constant as [B] and [C]
fallen = h_c < h_min
```

Validation (`scripts/check_estimator.py`, standing robot, 1 N·s pushes on `upper_body_link`, x/y alternating): parallelogram coupling error ≤ 0.014°; without sensor noise `c` within 0.07 mm and the push peak of `ξ` within 1.4 mm; with the env's AHRS noise model (bias per reset ≈ 0.2°, small white jitter) `c` carries a constant offset of ≈ 4.4 mm and `ξ` changes follow the truth within ≈ 2 mm.

#### [B] DCM MPC (x and y solved separately)

```text
ξ[k+1] = a ξ[k] + (1 − a) p[k]
ξ      = Φ ξ₀ + Γ p,   Φ[k] = a^k,   Γ[k, j] = a^(k−1−j) (1 − a) for j < k, else 0

min_p   Q Σ_{k=1..N} (ξ[k] − ξ_ref)² + R Σ_{k=0..N−1} (p[k] − p_ref)² + S Σ_k (p[k] − p[k−1])²
s.t.    p_min + m_zmp ≤ p[k] ≤ p_max − m_zmp
        p_min ≤ ξ[N] ≤ p_max
```

Only `p₀ = p[0]` is applied.  `H` is constant; each tick only the linear term and the terminal bounds change.

#### [C] LIPM step

```text
c_ref⁺ = p₀ + (c_ref − p₀) cosh(ω dt) + (ċ_ref / ω) sinh(ω dt)
ċ_ref⁺ = (c_ref − p₀) ω sinh(ω dt) + ċ_ref cosh(ω dt)
```

`c_ref` is integrated from its previous value (not from the measured `c`) so the servo targets stay smooth; feedback enters through the measured `ξ` in [B].

#### [D] IK

```text
min_v   Σ_i w_i ‖J_i G v − (K_i e_i + ẋ_i_ref)‖² + λ ‖v‖²
tasks:  left/right sole fixed (5-D each, no pitch), CoM xy → c_ref / ċ_ref,
        CoM height → h, pelvis upright (roll, pitch, yaw → 0), torso_pitch → q_default;
        arms and head held at q_default
s.t.    q_min ≤ q + q̇ dt ≤ q_max,   |q̇| ≤ q̇_max
q_des⁺ = q_des + q̇ dt
```

The 21 actuated joints are legs 10, torso 1, arms 8, head 2; the IK moves only the legs and the torso.

- **Pelvis upright.** Sole pitch always equals pelvis pitch through the two parallelograms, so the 5-D sole tasks leave pelvis pitch free; without this task the IK could tilt the pelvis and both soles together (toes or heels lifting on the real robot).  Roll and yaw keep the upper body level and facing forward.
- **CoM height, low weight.** `h` is a CoM height, so the height task holds the CoM (not the pelvis) at `h`, matching the constant-height LIPM.  At the straight-leg default pose (`_LEG_CROUCH = 0` in `robots/robonionv2.py`) the height is singular: bending the legs changes it only to second order.  Its weight is kept low (0.1 against 10 for CoM xy) so the IK does not chase an unreachable height with large joint velocities; the CoM drops ≈ 1.3 mm when it shifts ≈ 3 cm.  A crouched default pose would make the height controllable.

#### [E] Safety

```text
q_des ← clamp(q_des, soft joint limits)
|q_des − q_des_prev| ≤ q̇_max dt
|roll| or |pitch| > θ_max  → safe_stop
ξ outside [p_min, p_max]   → flag "not recoverable without stepping"
```

### Why design A fails

Measured response of the real CoM to the IK CoM command (MPC off; step of 10 mm and a 5 mm chirp 0.2–4 Hz, servo stiffness 42 N·m/rad):

| | x | y |
|---|---|---|
| step: static gain / overshoot / 50 % time | 1.16 / 22 % / 145 ms | 1.18 / 29 % / 155 ms |
| gain (phase) at 1.0 Hz | 1.36 (−26°) | 1.44 (−28°) |
| gain (phase) at 1.7 Hz (resonance) | 1.61 (−64°) | 1.92 (−110°) |
| gain (phase) at 3.0 Hz | 0.59 (−139°) | 0.45 (−150°) |

A second-order fit `K ωn² / (s² + 2ζωn s + ωn²)` needs no delay: x `K = 1.136, fn = 1.87 Hz, ζ = 0.362`, y `K = 1.135, fn = 1.54 Hz, ζ = 0.294` (error 0.45 / 0.64 mm on a 5 mm signal).  The static gain > 1 is gravity bending the compliant servos towards the lean.  An offline loop of the design-A controller against this fit reproduces the Isaac behaviour (K = 1 unstable, blend K = 0.6 oscillates, K = 0.3 stable, as found earlier in Isaac), so the failure is the DCM feedback loop through the servo lag and resonance, not a bug in one block.

### Attempts on design A (all reverted)

- **Feedback blend** `ξ_mpc = ξ_model + K (ξ_meas − ξ_model)` (earlier session): K ≥ 0.6 falls, K ≤ 0.3 stands but resists pushes no better than the baseline.
- **Velocity damping** `c_cmd −= kd (ċ_meas − ċ_ref) / ω`: stable for kd ≥ 0.5 on the nominal fit, but oscillates or falls with 20–40 ms extra delay.
- **Smith predictor** (MPC fed with `ξ_ref-model + (ξ_meas − ξ_predicted)`, the second-order fit as predictor) + static-gain correction `c_cmd = c_stand + (c_ref − c_stand) / K` + `ξ_ref` ramped from the standing CoM to the sole centre over 1 s.  Offline it is stable for plant gain ×0.8–1.4, +40 ms delay, fn ±30 %, ζ ×0.5–2, and swapped x/y plants.  In Isaac it **stands** (6 s, no oscillation, 0.0 mm foot slip, `ξ` settles ≈ 2 mm from the target).
- **Push response of the Smith version** with a gain on the unpredicted part `ξ_meas − ξ_predicted` (`disturbance_gain`):

| Push (N·s) | +x 2.5 | +x 3.0 | −x 1.75 | −x 2.0 | −x 2.5 | +y 2.0 | +y 2.5 | −y 2.25 | −y 2.5 |
|---|---|---|---|---|---|---|---|---|---|
| gain 0.25 | fall | fall | ok | fall | fall | slip 59 mm | fall | fall | fall |
| gain 0 | fall | fall | ok | ok | fall | slip 44 mm | fall | slip 60 mm | fall |

  With gain 1.0 (`scripts/run_stabilizer.py`, alternating pushes) 1 N·s along y already ends in `safe_stop`, 2 N·s along x falls, and a +x 1 N·s push swings `ξ` −44 mm backwards: the LIPM model treats the robot as a free pendulum and pulls it back at full strength while the servos already pull it back passively, so it rebounds.  The more the controller reacts, the worse; the "gain 0" gains only come from holding the CoM at the sole centre instead of 27 mm behind it (better backwards, worse forwards).

## Design B — NMPC on a flat-foot reduced model (prototype)

**Model.**  With both feet flat and both legs at equal joint angles the soles stay flat and 0.110 m apart for any hip roll / front thigh / ankle pitch (checked with FK), with `ankle_roll = −hip_roll`.  So double-support standing reduces to 4 coordinates `s = [hip_roll, front_thigh, ankle_pitch, torso_pitch]` (8 states), with the passive parallelogram joints from the linear coupling, hip yaw 0 and arms/head fixed at their defaults.  Rigid-body dynamics (Lagrange) from the masses/inertias of `assets/robonionv2_controller.urdf` (no fitting), servos `τ = K (q_des − q) − D q̇` on every actuated joint (`K = 42 N·m/rad`, `D = 2.4 N·m·s/rad`, armature 0.003 from `robots/robonionv2.py`), gravity, and the exact ZMP from CoM acceleration and angular momentum.  The mass matrix and centroidal angular-momentum map are frozen at the standing pose ("lean" model: 368 instead of 63 000 CasADi instructions per evaluation; CoM within 0.003 mm and ZMP within 0.04 mm of the full model on the chirp data).

Validation against the Isaac chirp/step data (model driven by the same servo targets):

| | Isaac | model |
|---|---|---|
| x: static gain / overshoot / 50 % time | 1.160 / 22 % / 145 ms | 1.163 / 20 % / 145 ms |
| x: CoM rms error (5 mm signal) | | 0.63 mm |
| y: static gain / overshoot | 1.183 / 29 % | 1.184 / 24 % |
| y: CoM rms error | | 1.35 mm (resonance peak too flat) |

Static joint sag matches within 0.01°.  Linearised modes: 1.78 Hz and 1.96 Hz (lightly damped), 2.6 Hz, 5.1 Hz and one fast mode at −201 s⁻¹ (needs ≈ 5 ms integration sub-steps).

**NMPC.**  State `[s, ṡ, u]`, control `u̇` (u = servo targets in reduced coordinates), horizon 0.5 s with 25 nodes, cost on DCM `c + ċ/ω` to the target, CoM velocity, torso pitch, `u − u_default` and `u̇`, soft ZMP bounds (sole hull − 1 cm), hard servo torque bounds `|K (u − q)| ≤ 0.8 effort`.  The state is taken from the encoders (joint angles projected onto `s`).  The controller outputs servo targets directly (no IK).

- ipopt (CasADi Opti) first: 0.2–0.5 s per solve, usable only because the simulator waits for the controller.
- **acados SQP-RTI + HPIPM, ERK 4 × 4: 0.96 ms mean / 1.8 ms max per solve on the desktop**, run every tick (200 Hz) in Isaac, no failed solve.  Needs `t_renderer` 0.2.1 in `~/acados/bin` (`linux-arm64` build for the Jetson AGX Orin).
- **Standing:** stable, the CoM moves smoothly from −27 mm to the sole centre (−0.3 mm), no foot slip; the NMPC commands e.g. the torso 1.14° beyond its measured angle to cancel the gravity sag without any hand-tuned compensation.

Pushes, acados at 200 Hz (ipopt at 50 Hz gave the same picture; the borderline cases flip between the two):

| CoM target | +x 2.5 | +x 3.0 | +x 3.5 | −x 1.75 | −x 2.0 | −x 2.5 | +y 2.0 | +y 2.5 | −y 2.25 | −y 2.5 |
|---|---|---|---|---|---|---|---|---|---|---|
| sole centre | fall | fall | fall | ok | ok | fall | slip 37 mm | fall | slip 60 mm | fall |
| standing CoM (−27 mm, same as passive) | ok (4 mm) | fall (ipopt: ok) | fall | fall | fall | fall | slip 49 mm | fall | fall | fall |

Lowering the torso weight (to use the upper body, "hip strategy") changed nothing.  Conclusion: stable and fast, but the reactive benefit while standing is small; where the CoM is held matters more.

## Sole tilt during pushes

Traces of the passive robot after pushes show that the soles tip well before a fall:

- Sole tilt from AHRS + FK (base orientation from the AHRS, soles by forward kinematics) matches the simulator within **0.1°** after removing a constant AHRS bias (≈ −0.49° pitch, −0.11° roll per reset; can be calibrated while standing).  Tilt is visible ≈ 50 ms after the push starts.
- +x 2.5 N·s (survives): both heels lift, 3.4° at 0.3 s, 5.3° peak.
- +y 2.0 N·s (survives): the left sole rolls 4.8° about its outer edge and the right foot leaves the ground for ≈ 0.6 s (right-foot contact force 0 N from the Isaac `ContactSensor`); the foot slip happens in this phase.

Design B's model assumes flat, fixed feet and cannot see any of this.

## Design C — NMPC on a soft-contact floating-base model (prototype)

**Model.**  Floating base (x, y, z, roll, pitch; yaw fixed), independent legs (hip roll, front thigh, ankle pitch, ankle roll per leg) and torso pitch: 14 coordinates.  Mass matrix frozen at the standing pose (Pinocchio CRBA mapped through the coordinates and the parallelogram coupling), exact gravity and contact kinematics, same servo model.  Contact: 4 corners per sole (foot mesh: x +0.068 / −0.107 m from the ankle-roll axis, y ±0.0335 m, 0.056 m below it), each with a one-sided normal spring-damper (smooth max) and smoothed Coulomb friction `F_t = −μ F_z v / sqrt(|v|² + v_s²)`.  Contact is *estimated from the state*, never measured.

- Ground friction: `μ = 0.75` assumed (ground material 1.0 in the env cfg, no material on the feet so PhysX default 0.5, averaged).  A no-slip tangential spring anchored at the initial corner position failed: after a lateral push the foot slides in Isaac, and on landing the spring yanked it back (500 N spikes).
- Contact stiffness sweep against the chirp data (static base height 0.5537 m and joint sag within 0.02° for all):

| normal k (N/m per corner) / damping | x: gain / overshoot, rms | y: gain / overshoot, rms |
|---|---|---|
| 2e4 / 300 | 1.205 / 31 %, 1.47 mm | 1.267 / 47 %, 2.85 mm |
| 5e4 / 500 | 1.179 / 25 %, 0.48 mm | 1.223 / 35 %, 1.12 mm |
| **2e5 / 1000** | **1.167 / 21 %, 0.20 mm** | **1.193 / 27 %, 0.16 mm** |
| 5e5 / 1500 | 1.164 / 20 %, 0.27 mm | 1.188 / 25 %, 0.34 mm |

- Pushes on the passive robot, model vs Isaac (`ContactSensor` enabled only in the validation script): +x 2.5 sole pitch rms difference 0.11°, peak −5.43° vs −5.27°; −x 1.75 rms 0.04°; survive/fall matches in all three cases.  +y 2.0 matches for the first 0.3 s (left foot 75–78 N, right foot 0 N, tilts within ≈ 1°); afterwards the model under-predicts (left roll peak −3.4° vs −4.8°, right foot airborne 0.34 s vs 0.63 s).

**Kinematic base estimator** (no contact sensing): base roll/pitch from the AHRS, joints from the encoders, the lower foot is taken as the stance foot, its lowest corner on the ground and its centre at its reference xy (no slip); base velocity from least squares so the stance-foot corners do not move.  CoM error vs Isaac: 1.3 mm (+x 2.5), 2.6 mm (−x 1.75), **38 mm (+y 2.0)**: the stance foot slides ≈ 40 mm in Isaac, which cannot be observed without foot sensors.

**NMPC.**  State `[q, q̇, u]` (37), control `u̇` (9), horizon 0.5 s / 25 nodes, cost on DCM, CoM velocity, the 8 corner heights (feet flat), torso pitch, `u − u_default`, `u̇`; soft servo-torque bounds.  Offline, with the stiff contact model (k = 2e5) as the plant:

- ERK integration diverges: the foot contact modes (light feet on stiff contact and friction) are too fast for 5 ms explicit steps, so every QP fails.
- IRK, 1-stage Radau IIA, softer contact in the controller model (k = 2e4, v_s = 0.02 m/s): **standing stable, 5.2 ms per solve; +x 2.5 N·s: sole tilt 2.5° vs 15.9° passive**.  But +x ≥ 3.0, −x ≥ 1.75 and +y 2.0 (which the passive robot survives) fall: HPIPM returns MINSTEP (status 4) a few hundred ms after the push, solves take 15–55 ms, and the plan goes bad.  Levenberg-Marquardt 1e-4…1e-1, looser QP tolerances and HPIPM ROBUST mode did not fix it.  Code generation + build takes ≈ 8 min for the IRK version.
- Soft-contact MPC is known to give badly conditioned QPs when contacts switch; this is the reason for design D.

## Design D — NMPC on a rigid-contact model with a contact mode per foot (package)

### Model (`mpc/model.py`)

Same coordinates as design C: floating base (x, y, z, roll, pitch; yaw fixed) and 9 independent joints (hip roll, front thigh, ankle pitch, ankle roll per leg, torso pitch); passive joints from the parallelogram coupling, arms and head at their defaults, mass matrix frozen at the standing pose, exact gravity and kinematics, servos `τ = K (u − q) − D q̇` (`robots/robonionv2.py`: K = 42 N·m/rad, D = 2.4 N·m·s/rad, armature 0.003).

- **Contacts are rigid constraints.**  Each sole has 5 rows: position of a contact point (3), z of the sole y axis (roll), z of the sole x axis (pitch).  An active row is an acceleration-level constraint with Baumgarte stabilisation (α = 20 s⁻¹); an inactive row has λᵢ = 0.  The contact point and the row flags are parameters, so one model covers every **contact mode per foot**: `flat` (sole centre, all rows), `toe` / `heel` (middle of the front / back edge, pitch free), `left_edge` / `right_edge` (middle of a side edge, roll free), `air` (no rows).
- **The contact wrench is not a free variable.**  Position-mode servos leave it no freedom: given the state and the servo targets, λ follows from the KKT system and is eliminated in closed form.
- **Double support has 9 independent rows, not 10.**  Sole pitch equals pelvis pitch through both parallelograms, so the two pitch rows are identical (rank 9, 5 DOF at the standing pose).  The right pitch row is switched off; the left one carries the summed pitch moment.  Only the combined CoP-x of the flat soles is therefore defined; CoP-y is per sole.

Validation against Isaac (1 ms RK4; 5 ms steps give the same result):

| Test | Isaac | Model |
|---|---|---|
| standing: Fz L / R | 38.3 / 38.3 N | 38.9 / 38.9 N (= mg) |
| standing: joint sag | torso −1.145° | torso −1.139°, all joints within 0.02° |
| chirp/step x: gain / overshoot / 50 % time, CoM rms error | 1.160 / 22 % / 145 ms | 1.163 / 20 % / 145 ms, 0.63 mm |
| chirp/step y | 1.183 / 29 % / 155 ms | 1.184 / 24 % / 150 ms, 1.35 mm (same as design B; design C's contact compliance fits y better) |
| +x 2.5, sole pitch over 2–3 s (hybrid simulation, below) | peak −5.27°, toe 2.035–2.490 s, heel from 2.520 s | peak −5.35°, toe 2.015–2.495 s, heel from 2.495 s; rms difference **0.06°** |
| −x 1.75 | heel from 2.030 s, falls | heel from 2.015 s, falls; rms 0.66° (falls slightly slower after 2.4 s) |
| +y 2.0 | right foot unloaded 2.005 s, left sole on its outer edge from 2.050 s | right foot air 2.025 s, left outer edge 2.050 s; CoM y 17.9 vs 18.8 mm at 2.10 s |

The hybrid simulation switches modes the way the rigid model predicts them: flat → toe/heel when the combined CoP of the flat soles reaches the front/back edge, flat → side edge when a sole's CoP-y reaches its side, → air when a normal force would pull, and back to flat (or an edge) with an inelastic impact when a sole is level again or touches down.

Lateral limit: after ≈ 0.1 s the +y 2.0 push diverges (left sole roll −2.8° vs −4.8°, right foot back on the ground at 2.31 s vs 2.635 s).  Design C shows the same pattern, so it is not the rigid contact.  Ruled out: servo torque saturation (peak demand 4.2–4.6 N·m, limit 9.9) and sole geometry (the bottom of `*_foot_visual.stl` is a sharp ±33.5 mm rectangle and the USD collision is its convex hull).  The difference sits in the stance ankle roll (model deflects ≈ 0.6° more, i.e. Isaac sees ≈ 20 % less moment about it); moving the rolling edge inwards in the model made it worse.

The straight-leg default pose is singular for height: a knee bend changes the leg length only to second order, which the linearised contact model barely sees (a small singular value, 0.054, in the double-support contact Jacobian).

### Estimation (`mpc/estimator.py`)

- `AhrsGyro`: pelvis roll/pitch from the AHRS quaternion (yaw removed) and their rates from the gyro, with the torso rate removed through the IMU Jacobian.  The AHRS bias is calibrated during the first 0.5 s of standing (both soles flat, so the joints fix the true pelvis tilt).
- `ContactModeMonitor` (no contact sensing):
  - flat → edge when the sole tilt (AHRS + FK) exceeds 0.2° **or its tilt rate exceeds 3°/s**; a flat sole cannot rotate, so the rate shows the rolling ≈ 15 ms before the angle does (−x 1.75: heel detected at 2.015 s instead of 2.035 s).
  - edge → flat when the tilt crosses zero.
  - a foot whose lowest corner is 3 mm above the other foot's is in the air; one in the air touches down within 0.5 mm.
  - **release**: a foot in contact goes to the air when the normal force the model predicts from the estimated state and the servo targets drops below 10 N, or when the contact points measured by encoders + AHRS drift more than 3 mm apart (both feet on the ground keep their distance).  In the +y 2.0 push the right foot is unloaded but hovers 0.1–2 mm above the ground for 0.6 s, which kinematics alone cannot see.  A foot that has just touched down must carry 20 N, otherwise it is released again (hysteresis against chattering).
- `ContactProjectionEstimator`: weighted least squares of encoders and AHRS roll/pitch onto the manifold of the active contact rows (2 Gauss–Newton steps per tick), velocities projected onto the null space of the contact Jacobian.  The base position comes only from the contacts; the AHRS angles are a weak prior that matters only for rotations the contacts leave free.

Checked on the passive push data with the simulator base orientation as AHRS (2–3 s after the push):

| Push | CoM error | CoM velocity error along the push, max / rms | true velocity peak |
|---|---|---|---|
| +x 2.5 | 1.3 mm | 92 / 11 mm/s (max at the heel landing) | 289 mm/s |
| −x 1.75 | 0.4 mm | 19 / 4 mm/s | 263 mm/s |
| +y 2.0 | 1.0 mm (41 mm before the force-based release) | 53 / 7 mm/s | 240 mm/s |

Foot slip cannot be observed and is not modelled.

### NMPC (`mpc/nmpc.py`)

acados SQP-RTI, one iteration per tick; x = [q, q̇, u] (37), control u̇ (9); 25 nodes over 0.5 s; output: servo targets.

- **Contact plan as per-node parameters** (modes and contact references per node), so modes can change inside the horizon; stage 1 always passes flat/flat, stage 2 will pass the gait schedule.  Edge modes are never planned.
- **Cost:** DCM (`c + ċ/ω`) to the target, CoM velocity, torso pitch, u − u_default (w_u = 0.1), u̇, internal squeeze between the feet, sole tilt and sole height (for feet in the air), **leg length** (sole centre height in the pelvis frame) and **pelvis roll/pitch**.  Without the last two the NMPC folded a knee or rolled the pelvis by 30–40° after pushes (see the straight-leg singularity above); a CoM-height cost instead made it crouch while the soles rolled on their toes, because rolling raises the CoM.
- **Constraints:** soft (L1 + L2 slacks) normal force ≥ 5 N, per-sole CoP-y and combined CoP-x of the flat soles 1 cm inside the edges, friction pyramid μ = 0.5, servo torque ≤ 0.8 × URDF effort; hard servo-target range around the standing pose (hip / ankle roll ±15°, front thigh / ankle pitch ±20°, torso ±15°) and |u̇| ≤ 4.08 rad/s.  Without the range bounds the torso was commanded beyond −100°.
- **Speed:** eliminating λ symbolically gives a 13 k-instruction dynamics function and 245 ms per solve (integrator sensitivities, 385 k instructions per Jacobian).  Instead the contact algebra (Jc, (Jc M⁻¹ Jcᵀ)⁻¹, J̇q̇, contact rows) is evaluated numerically along the previous plan and passed as parameters, so inside a node interval the contact dynamics are linear in the state (servo torques and gravity stay exact).  With a foot in the air the fastest mode is −600 s⁻¹ (RK4 at 5 ms unstable), so the integrator is IRK Radau IIA, 2 stages, one step per node (CoM within 0.005 mm of RK4 at 1 ms over 0.5 s; 1 stage was off by 0.6 mm).  Solve time on the desktop: ≈ 3.8 ms mean, 7–10 ms max in Isaac, plus ≈ 3.3 ms to prepare the parameters in Python — above the 5 ms tick, still open.

### From "acting in every mode" to "only while flat"

The first version ran the NMPC in every contact mode (the phase schedule predicted from the previous plan when an edge-rolling sole levels out or a foot touches down).  It stood stably but fell in all ten Isaac pushes, including +x 2.5 and +y 2.0 which the passive robot survives.  Diagnosis:

1. With the simulator's true state and contact modes instead of the estimator it failed the same way, so the controller, not the estimation, was at fault.
2. Offline, with the validated hybrid model as the plant, it failed too; adding the leg-length and pelvis costs fixed +y 2.0 but not the sagittal pushes.
3. Holding the edge mode over the whole horizon (no schedule) changed nothing.  In toe mode the plan drifted from the exact model within 0.2–0.5 s (e.g. at +0.5 s pelvis pitch 0.07° planned vs 2.7°, CoM 82 vs 101 mm), half from the frozen contact algebra and half from the unconverged RTI iterate; the optimizer then "balanced" the CoM over the toe edge by swinging the thighs (+40°) and torso (−48°).
4. With the exact model and full SQP (5 iterations, ≈ 0.5 s per solve, offline) it still fell (torso commanded to −132°…−285°).  So the formulation itself does not handle a sole rolling on an edge; the passive robot, which does nothing, recovers from +x 2.5 by itself because the CoP at the toe already drives the DCM back.
5. Joint bounds, stronger regularisation and a 0.8 s horizon did not make it match the passive robot.

Final rule: **the NMPC runs only while both soles are flat; in any other mode the servo targets are held**, and the NMPC restarts from the current state once both soles are flat again.  Offline, holding only in toe/heel was better than also holding in side-edge modes, but in Isaac the lateral pushes (right foot in the air, left sole on its outer edge) failed unless the NMPC was also held there.

Isaac, GPU PhysX, one env per process, one push at t = 2 s:

| Push (N·s) | +x 2.5 | +x 3.0 | +x 3.5 | −x 1.75 | −x 2.0 | −x 2.5 | +y 2.0 | +y 2.5 | −y 2.25 | −y 2.5 |
|---|---|---|---|---|---|---|---|---|---|---|
| passive | ok (5.9°) | fall | fall | fall | fall | fall | ok | fall | fall | fall |
| D, acting in every mode | fall | fall | fall | fall | fall | fall | fall | fall | fall | fall |
| **D, only while flat (w_u = 0.1, package)** | ok (5.8°) | fall | fall | **ok (2.7°)** | **ok (4.5°)** | fall | ok (4.3°) | fall | **ok (42.9°, marginal)** | fall |
| D, only while flat, w_u = 10, w_torso = 100 | ok (4.3°) | **ok (10.7°)** | fall | **ok (4.0°)** | fall | fall | ok | fall | fall | fall |

(°: peak sole tilt.)  Rerun after `solver_velocity_iteration_count` went from 4 to 0 in `robots/robonionv2.py` (stage 2, see [`stage2.md`](stage2.md) *Pitfalls*), with seeds 0, 1, 2 of the sensor noise (all three identical):

| Push (N·s), velocity iterations 0 | +x 2.5 | +x 3.0 | +x 3.5 | −x 1.75 | −x 2.0 | −x 2.5 | +y 2.0 | +y 2.5 | −y 2.25 | −y 2.5 |
|---|---|---|---|---|---|---|---|---|---|---|
| passive | ok (14.2°) | fall | fall | fall | fall | fall | ok (4.5°) | fall | fall | fall |
| **D, only while flat (package)** | ok (5.8°) | fall | fall | **ok (3.0°)** | **ok (4.5°)** | fall | ok (4.2°) | fall | fall | fall |

19 of the 20 outcomes are unchanged; −y 2.25, marginal before (42.9°), now falls with the controller as it does passively.  The package is still never worse than passive and better in 2 of 10 pushes (−x 1.75, −x 2.0).

w_u sets how freely the NMPC moves the CoM to the sole centre: w_u = 0.1 does, and is better backwards (the weakest direction, the standing CoM sits near the heel); w_u = 10 keeps the standing pose and is better forwards — the same trade-off as design B.  The package uses w_u = 0.1 (user decision).

## Pitfalls found

- **Multi-environment runs are not trustworthy** for this asset: with `num_envs > 1` PhysX logs `Replication of this type is not supported: ... RobinionSelfCollisionGroup` and outcomes differ from single-env runs even for env 0 (also with `replicate_physics = False`).  Use one env per process.
- Several Isaac processes started in parallel on the GPU can crash during `sim.reset()`; run them sequentially.
- Foot slip along y is a large part of the lateral failures; the foot has no physics material, so its friction is the PhysX default.
- The AHRS noise model adds a per-reset orientation bias (≈ 0.5°), which shifts the estimated CoM by ≈ 4 mm.
- A push of 50 N on the upper body moves the CoP the soles would need far outside them within ≈ 15 ms (+x 2.5: 124 mm at 2.02 s, 248 mm at 2.04 s), so every push in the table tips the soles; no standing controller can keep them flat.
- Feet that are unloaded can hover 0.1–2 mm above the ground for hundreds of ms in Isaac (right foot in +y 2.0 for 0.6 s); a kinematic height threshold alone keeps them "in contact".
- The monitor's kinematic and force tests can disagree within one tick (touch-down by height, release by force); count mode changes on the final mode per tick, not on the intermediate ones.
- An acados solver loaded without rebuilding keeps the cost weights of its last build; set them at run time or delete the build folder.

## Prototype code

Not part of the package; kept for reference in `outputs/stage1_prototypes/` (git-ignored scratch).  `ident.py` and `tilt_check.py` use the design-A estimator/IK, which were removed from the package; run them from the git history.  Scripts import each other and load the `.npz` files by relative path, so run them from that folder with `uv run --project <repo> python <script>`.  acados needs `ACADOS_SOURCE_DIR=~/acados` and `LD_LIBRARY_PATH=~/acados/lib`.

| File | Content |
|---|---|
| `ident.py`, `ident.npz` | Isaac identification run (2 envs: x / y excitation; step + chirp of the IK CoM command, MPC off); logs commands, true CoM, servo targets, joint angles |
| `analyze.py`, `fit2.py` | step / frequency response and the second-order fit |
| `model.py` | design B: `ReducedModel` (4-coordinate double-support model, full dynamics) and `LeanModel` (frozen mass matrix) |
| `validate.py`, `lean_check.py`, `joints.py`, `eig.py` | design-B model validation against `ident.npz` |
| `nmpc_proto.py`, `nmpc_acados.py`, `acados_offline.py` | design B NMPC: ipopt prototype, acados version, offline closed loop |
| `isaac_nmpc.py` | design B in Isaac (`--solver acados|ipopt`, `--hold`, `--push x+2.5`, `--ctrl none` for the passive baseline) |
| `tilt_check.py` | sole tilt from AHRS + FK vs simulator truth |
| `model2.py` | design C: `ContactModel` (floating base, independent legs, soft sole corners) and `KinematicStateEstimator` |
| `push_truth.py`, `truth_*.npz` | passive push ground truth incl. Isaac `ContactSensor` forces |
| `validate2.py`, `push_compare.py`, `est_check.py` | design-C model and estimator validation |
| `nmpc2_acados.py`, `nmpc2_offline.py`, `nmpc2_opts.py` | design C NMPC (acados IRK), offline push tests, solver-option sweep |
| `model3.py` | design D prototype: `RigidContactModel`, contact modes, `ContactModeMonitor`, `ContactProjectionEstimator`, `HybridSim` (hybrid rigid-contact plant); the package `mpc/model.py` is the same model (checked to 1e-10) |
| `validate3.py` | design-D model and estimator validation: static equilibrium, eigenvalues, `ident.npz`, `truth_x+2.5` / `truth_x-1.75` / `truth_y+2.0` with `HybridSim` (`validate3.py [step] [f_off f_on]`) |
| `nmpc3_acados.py` | design D NMPC with every option tried (contact schedule, edge-cost relaxation, exact symbolic contacts, full SQP, joint limits, weights) |
| `nmpc3_offline.py` | offline closed loop of `nmpc3_acados.py` on `HybridSim` (`--exact`, `--sqp`, `--hold_on_edge`, …) |
| `isaac_nmpc3.py` | design D prototype in Isaac (`--truth_state` feeds the simulator state and contact modes instead of the estimator, for diagnosis) |

## Open decisions

1. ~~Code location and push-test mechanism~~ -- resolved: design D in `source/MPC_Humanoid/MPC_Humanoid/mpc/`, `scripts/run_stabilizer.py --push <axis><impulse>`, one push per run, one env per process.
2. ~~Controller model~~ -- resolved: `assets/robonionv2_controller.urdf`, generated from the USD by `tools/asset/export_controller_urdf.py`.
