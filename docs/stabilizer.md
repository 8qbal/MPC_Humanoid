# Stabilizer (stage 1) — variables and equations

Double-support standing controller that recovers from pushes without stepping.  It runs every control tick (200 Hz) and outputs joint position targets for the Dynamixel servos (position mode 3).  No foot force sensors and no simulator ground truth (`base_lin_vel`, `base_ang_vel`, `projected_gravity`) are used.  Blocks: [A] estimator, [B] DCM MPC, [C] LIPM step, [D] IK, [E] safety.

## Frames

- **S (support frame):** origin at the midpoint of the two sole centres on the ground, z opposite to gravity, x forward, y left.  All horizontal quantities (`c`, `xi`, `p`) are expressed in S.
- **B (base frame):** `base_link` of the controller model.

## Variables

### Inputs (from the env / hardware)

| Symbol | Code name | Dim | Unit | Source |
|---|---|---|---|---|
| `q_a` | `q_act` | 22 | rad | encoders; env obs `joint_pos_rel` + `q_default` |
| `q̇_a` | `qd_act` | 22 | rad/s | encoders; env obs `joint_vel_rel` |
| `quat` | `quat` | 4 (x, y, z, w) | — | AHRS orientation (MTi-630); env obs `imu_orientation`, normalised before use |
| `ω_B` | `gyro` | 3 | rad/s | gyro (MTi-630); env obs `imu_ang_vel` |

### Internal signals

| Symbol | Code name | Dim | Unit | Produced by | Meaning |
|---|---|---|---|---|---|
| `q` | `q_full` | 29 | rad | [A] | all revolute joints incl. passive ones, `q = G q_a` |
| `c` | `c_S` | 2 | m | [A] | measured CoM (x, y) in S |
| `ċ` | `cdot_S` | 2 | m/s | [A] | measured CoM velocity, low-pass filtered finite difference |
| `ξ` | `xi` | 2 | m | [A] | measured DCM, `xi = c + ċ / ω` |
| `p` | `p` | 2 × N | m | [B] | planned ZMP over the horizon (per axis) |
| `p₀` | `p0` | 2 | m | [B] | first planned ZMP, the only one applied |
| `p_prev` | `p_prev` | 2 | m | [B] | `p0` of the previous tick (for the ZMP-rate cost) |
| `c_ref` | `c_ref` | 2 | m | [C] | CoM reference for the next tick |
| `ċ_ref` | `cdot_ref` | 2 | m/s | [C] | CoM velocity reference for the next tick |
| `v` | `v` | 6 + 11 | m/s, rad/s | [D] | IK velocity: base twist + leg (10) and torso (1) joint velocities |
| `q_des` | `q_des` | 22 | rad | [D], [E] | joint position targets |

### Output

| Symbol | Code name | Dim | Unit | Destination |
|---|---|---|---|---|
| `q_des − q_default` | `action` | 22 | rad | env action term `joint_pos` (`JointPositionActionCfg`, `use_default_offset=True`) |

### Parameters

| Symbol | Code name | Initial value | Unit | Source / note |
|---|---|---|---|---|
| `g` | `g` | 9.81 | m/s² | |
| `m` | `mass` | 7.95 | kg | URDF sum of link masses (controller URDF after the USD fixes may differ slightly) |
| `h` | `h` | ≈ 0.50 | m | CoM height above the sole at the standing pose, from FK of the controller model |
| `ω` | `omega` | `sqrt(g / h)` ≈ 4.4 | rad/s | |
| `dt` | `dt` | 0.005 | s | sim dt 1 ms × decimation 5 |
| `Δ` | `dt_mpc` | 0.02 | s | MPC node interval |
| `N` | `horizon` | 25 | — | 0.5 s horizon |
| `a` | `a` | `e^(ω Δ)` ≈ 1.09 | — | |
| `Q`, `R`, `S` | `w_xi`, `w_zmp`, `w_dzmp` | 1, 0.1, 1 | — | DCM tracking, ZMP centring, ZMP rate |
| `m_zmp` | `zmp_margin` | 0.01 | m | safety margin inside the sole edges |
| `ξ_ref`, `p_ref` | `xi_ref`, `p_ref` | undecided | m | standing pose (x ≈ 0.001) or sole centre (x ≈ +0.028), see *Open decisions* |
| `p_min`, `p_max` | `zmp_bounds` | x: [−0.059, +0.116], y: [−0.088, +0.089] | m | hull of both soles at the standing pose; from the foot mesh `*_foot_visual.stl` (CAD) and FK |
| sole size | `sole_*` | length 0.175 (+0.068 / −0.107 from the ankle-roll axis), width 0.067, depth 0.056 below the ankle-roll axis | m | foot mesh `*_foot_visual.stl` (CAD) |
| `f_c` | `vel_cutoff` | 20 | Hz | low-pass cutoff for `ċ` |
| `q̇_max` | `qd_max` | 4.08 (legs) | rad/s | URDF velocity limit |
| `θ_max` | `tilt_limit` | 20 | deg | base roll/pitch that triggers `safe_stop` |

## Equations

### [A] Estimator

Parallelogram coupling (`q = G q_a`):

```text
knee = −front_thigh        back_thigh = front_thigh
front_shin = −ankle_pitch  back_shin  = front_shin
```

Forward kinematics of the controller model with the base orientation set to the IMU roll/pitch (yaw ignored, base position 0) gives the CoM and both sole centres; then

```text
c   = CoM_xy − midpoint(sole_left, sole_right)_xy
ċ   = lowpass((c[t] − c[t−1]) / dt, f_c)
ξ   = c + ċ / ω
```

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
        pelvis height → h, torso upright (torso_pitch); arms and head held at q_default
s.t.    q_min ≤ q + q̇ dt ≤ q_max,   |q̇| ≤ q̇_max
q_des⁺ = q_des + q̇ dt
```

### [E] Safety

```text
q_des ← clamp(q_des, soft joint limits)
|q_des − q_des_prev| ≤ q̇_max dt
|roll| or |pitch| > θ_max  → safe_stop
ξ outside [p_min, p_max]   → flag "not recoverable without stepping"
```

## Open decisions

1. `ξ_ref` / `p_ref`: keep the standing CoM position (6 cm to the heel edge, 11.5 cm to the toe edge) or move to the sole centre (≈ 8.7 cm both ways).
2. Code location and push-test mechanism.
3. Controller model: generated from the USD as a controller URDF (decided); the export tool is not written yet.
