# Stabilizer (stage 1) — variables and equations

Double-support standing controller that recovers from pushes without stepping.  It runs every control tick (200 Hz) and outputs joint position targets for the Dynamixel servos (position mode 3).  No foot force sensors and no simulator ground truth (`base_lin_vel`, `base_ang_vel`, `projected_gravity`) are used.  Blocks: [A] estimator, [B] DCM MPC, [C] LIPM step, [D] IK, [E] safety.

## Frames

- **S (support frame):** origin at the midpoint of the two sole centres on the ground, z opposite to gravity, x forward, y left.  All horizontal quantities (`c`, `xi`, `p`) are expressed in S.
- **B (base frame):** `base_link` of the controller model.

## Variables

### Inputs (from the env / hardware)

| Symbol | Code name | Dim | Unit | Source |
|---|---|---|---|---|
| `q_a` | `q_act` | 21 | rad | encoders; env obs `joint_pos_rel` + `q_default` |
| `q̇_a` | `qd_act` | 21 | rad/s | encoders; env obs `joint_vel_rel` |
| `quat` | `imu_quat_xyzw` | 4 (x, y, z, w) | — | AHRS orientation of `imu_link` (MTi-630); env obs `imu_orientation`, normalised before use |
| `ω_I` | `gyro` | 3 | rad/s | gyro in the `imu_link` frame (MTi-630); env obs `imu_ang_vel` |

### Internal signals

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

### Output

| Symbol | Code name | Dim | Unit | Destination |
|---|---|---|---|---|
| `q_des − q_default` | `action` | 21 | rad | env action term `joint_pos` (`JointPositionActionCfg`, `use_default_offset=True`) |

### Parameters

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

## Equations

### [A] Estimator

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

### [B] DCM MPC (x and y solved separately)

```text
ξ[k+1] = a ξ[k] + (1 − a) p[k]
ξ      = Φ ξ₀ + Γ p,   Φ[k] = a^k,   Γ[k, j] = a^(k−1−j) (1 − a) for j < k, else 0

min_p   Q Σ_{k=1..N} (ξ[k] − ξ_ref)² + R Σ_{k=0..N−1} (p[k] − p_ref)² + S Σ_k (p[k] − p[k−1])²
s.t.    p_min + m_zmp ≤ p[k] ≤ p_max − m_zmp
        p_min ≤ ξ[N] ≤ p_max
```

Only `p₀ = p[0]` is applied.  `H` is constant; each tick only the linear term and the terminal bounds change.

### [C] LIPM step

```text
c_ref⁺ = p₀ + (c_ref − p₀) cosh(ω dt) + (ċ_ref / ω) sinh(ω dt)
ċ_ref⁺ = (c_ref − p₀) ω sinh(ω dt) + ċ_ref cosh(ω dt)
```

`c_ref` is integrated from its previous value (not from the measured `c`) so the servo targets stay smooth; feedback enters through the measured `ξ` in [B].

### [D] IK

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

### [E] Safety

```text
q_des ← clamp(q_des, soft joint limits)
|q_des − q_des_prev| ≤ q̇_max dt
|roll| or |pitch| > θ_max  → safe_stop
ξ outside [p_min, p_max]   → flag "not recoverable without stepping"
```

## Baseline without stabilizer

Largest push on `upper_body_link` (constant force for 0.05 s) that the robot survives with the servos only holding the default pose, from `scripts/check_estimator.py` (fall = root height well below the standing 0.554 m, or the estimator reporting `fallen`).  The theoretical limit is `J_max = m ω · margin`, with the ZMP jumping straight to the sole edge and the standing CoM 2.6 cm behind the sole centre.

| Direction | Survives | Falls | Theoretical limit |
|---|---|---|---|
| +x (forward) | 2.5 N·s | 3.0 N·s | 4.0 N·s |
| −x (backward) | 1.75 N·s | 2.0 N·s | 2.1 N·s |
| +y (left) | 2.0 N·s | 2.5 N·s | 3.1 N·s |
| −y (right) | 2.25 N·s | 2.5 N·s | 3.1 N·s |

The stabilizer has to beat these numbers; backward is the weakest direction because the standing CoM is close to the heel.

## Open decisions

1. Code location and push-test mechanism.
2. ~~Controller model~~ -- resolved: `assets/robonionv2_controller.urdf`, generated from the USD by `tools/asset/export_controller_urdf.py`.
