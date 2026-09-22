# Robinion — Joint & Link Reference

Reference sheet for building the Robinion `ArticulationCfg` in `source/MPC_Humanoid`. 

## Sources

| Path | What it is |
|---|---|
| `references/robinion_description/robinion2.urdf` | Generated URDF (from `urdf/robinion2.xacro` + `body/head/left_arm/right_arm/left_leg/right_leg.xacro`). **Primary source for all numbers below.** |
| `assets/robonionv2/` | Raw output of `urdf_usd_converter` (converter version pinned in `robinion.usda`'s `customLayerData.creator`), *before* the hand-authored fixes below: `robinion.usda` (thin root, payloads `Payload/Contents.usda`) + `Payload/{Geometry,Physics,Contents}.usda` + `Payload/GeometryLibrary.usdc` (meshes). Plain tree import: no loop closure, no articulation self-collision override, base welded to the world. Kept as an editable, re-buildable source — do not hand-edit `Payload/`, only `robinion.usda`. |
| `assets/robonionv2.usd` | **The asset actually loaded by `ArticulationCfg`.** Flattened, standalone (`scripts/flatten_usd.py`) copy of `assets/robonionv2/robinion.usda` after the 5 fixes in *USD verification* below: loop-closure joints, self-collision off, floating base, foot inertia, `left_back_thigh_pitch_link` mass. Regenerate it whenever `robonionv2/robinion.usda` changes — never edit it directly. |
| `references/Robinion_InverseKinematic/robinion_description/` | Second copy of the description with a mirrored `leg.xacro` (different axis signs) and `ik_controller_fullbody.py`. The IK script is the only document that states the **parallelogram coupling** between joints (see *Leg mechanism*). |

Notes:
- Actuators are **Dynamixel XH540-W270** (body/legs/arms) and **Dynamixel XH430-W350** (head), 11.1 V bus. The per-joint effort/velocity limits in the URDF (4.1 / 9.9 / 10.6 / 19.8 N·m; 3.14 / 4.08 / 4.82 rad/s) were **confirmed correct by the robot owner** and are used as-is in `ROBINION_CFG` (`URDF_LIMITS` in `robots/robonionv2.py`), including the left/right front-thigh asymmetry. Servo datasheets are only used for stiffness/damping/armature estimates.
- The knee joint (`*_knee_pitch_joint`) has **no actuator** (confirmed by the user).
- **Two separate Isaac Sim installations exist on the dev machine, with two different converter versions** — see *USD verification* for why this matters before you re-run the GUI importer.

## Robot overview

| Item | Value |
|---|---|
| Links | 35 (`base_link`, `imu_link`, `cam_link` are massless frames) |
| Joints | 34 = 29 revolute + 5 fixed (`base_to_lower_body`, `imu`, `cam`, `right_gripper`, `l_gripper`) |
| Total URDF mass | **7.951 kg** (sum of the 32 links that have an `<inertial>`) |
| Root | `base_link` → `lower_body_link` (pelvis); `torso_pitch_joint` sits between pelvis and upper body |

### Kinematic tree

Legend: `LINK` = link (rigid body, plain name) · `( joint_name : type )` = joint. Every joint has exactly one child link directly under it, so the tree alternates link → joint → link, like the Isaac Sim stage view.

```text
base_link
└─ ( base_to_lower_body_fixed_joint : fixed )
   └─ lower_body_link                                   [pelvis]
      ├─ ( torso_pitch_joint : revolute )
      │  └─ upper_body_link
      │     ├─ ( imu_fixed_joint : fixed )
      │     │  └─ imu_link                              [sensor frame, massless]
      │     ├─ ( head_yaw_joint : revolute )
      │     │  └─ head_yaw_link
      │     │     └─ ( head_pitch_joint : revolute )
      │     │        └─ head_pitch_link
      │     │           └─ ( cam_fixed_joint : fixed )
      │     │              └─ cam_link                  [sensor frame, massless]
      │     ├─ ( right_shoulder_pitch_joint : revolute )
      │     │  └─ right_shoulder_pitch_link
      │     │     └─ ( right_shoulder_roll_joint : revolute )
      │     │        └─ right_shoulder_roll_link
      │     │           └─ ( right_elbow_yaw_joint : revolute )
      │     │              └─ right_elbow_yaw_link
      │     │                 └─ ( right_elbow_pitch_joint : revolute )
      │     │                    └─ right_elbow_pitch_link
      │     │                       └─ ( right_gripper_fixed_joint : fixed )
      │     │                          └─ right_gripper
      │     └─ ( left_shoulder_pitch_joint : revolute )
      │        └─ left_shoulder_pitch_link
      │           └─ ( left_shoulder_roll_joint : revolute )
      │              └─ left_shoulder_roll_link
      │                 └─ ( left_elbow_yaw_joint : revolute )
      │                    └─ left_elbow_yaw_link
      │                       └─ ( left_elbow_pitch_joint : revolute )
      │                          └─ left_elbow_pitch_link
      │                             └─ ( l_gripper_fixed_joint : fixed )
      │                                └─ left_gripper
      ├─ ( right_hip_yaw_joint : revolute )
      │  └─ right_hip_yaw_link
      │     └─ ( right_hip_roll_joint : revolute )
      │        └─ right_hip_roll_pitch_link              [thigh-stage ground link]
      │           ├─ ( right_front_thigh_pitch_joint : revolute )
      │           │  └─ right_front_thigh_pitch_link     [bar, 0.200 m]
      │           │     └─ ( right_knee_pitch_joint : revolute, PASSIVE = -front_thigh )
      │           │        └─ right_knee_pitch_link      [thigh-stage coupler / shin-stage ground link]
      │           │           ├─ ( right_front_shin_pitch_joint : revolute, PASSIVE = driven by ankle-pitch motor )
      │           │           │  └─ right_front_shin_pitch_link      [bar, 0.200 m]
      │           │           │     └─ ( right_ankle_pitch_joint : revolute, ACTUATED = -front_shin )
      │           │           │        └─ right_ankle_roll_pitch_link [shin-stage coupler]
      │           │           │           └─ ( right_ankle_roll_joint : revolute )
      │           │           │              └─ right_foot_roll_link
      │           │           └─ ( right_back_shin_pitch_joint : revolute, PASSIVE = front_shin )
      │           │              └─ right_back_shin_pitch_link       [bar, 0.200 m — DANGLING: loop to right_ankle_roll_pitch_link missing]
      │           └─ ( right_back_thigh_pitch_joint : revolute, PASSIVE = front_thigh )
      │              └─ right_back_thigh_pitch_link      [bar, 0.200 m — DANGLING: loop to right_knee_pitch_link missing]
      └─ ( left_hip_yaw_joint : revolute )
         └─ left_hip_yaw_link
            └─ ( left_hip_roll_joint : revolute )
               └─ left_hip_roll_pitch_link               [thigh-stage ground link]
                  ├─ ( left_front_thigh_pitch_joint : revolute )
                  │  └─ left_front_thigh_pitch_link      [bar, 0.200 m]
                  │     └─ ( left_knee_pitch_joint : revolute, PASSIVE = -front_thigh )
                  │        └─ left_knee_pitch_link       [thigh-stage coupler / shin-stage ground link]
                  │           ├─ ( left_front_shin_pitch_joint : revolute, PASSIVE = driven by ankle-pitch motor )
                  │           │  └─ left_front_shin_pitch_link       [bar, 0.200 m]
                  │           │     └─ ( left_ankle_pitch_joint : revolute, ACTUATED = -front_shin )
                  │           │        └─ left_ankle_roll_pitch_link  [shin-stage coupler]
                  │           │           └─ ( left_ankle_roll_joint : revolute )
                  │           │              └─ left_foot_roll_link
                  │           └─ ( left_back_shin_pitch_joint : revolute, PASSIVE = front_shin )
                  │              └─ left_back_shin_pitch_link        [bar, 0.200 m — DANGLING: loop to left_ankle_roll_pitch_link missing]
                  └─ ( left_back_thigh_pitch_joint : revolute, PASSIVE = front_thigh )
                     └─ left_back_thigh_pitch_link       [bar, 0.200 m — DANGLING: loop to left_knee_pitch_link missing]
```

Coordinate conventions: X forward, Y left, Z up (`upAxis = Z` in the USD). Joint origins below are relative to the parent link frame.

## Leg mechanism (parallelogram linkage)

Each leg is a two-stage parallelogram (four-bar) linkage. The URDF can only express a tree, so the two "back" links are left dangling; on the real robot they close the loops:

| Stage | Ground link | Two parallel bars (length 0.200 m each) | Coupler link | Bar spacing on ground/coupler |
|---|---|---|---|---|
| Thigh | `*_hip_roll_pitch_link` | `*_front_thigh_pitch_link` (pivot x = −0.0245) and `*_back_thigh_pitch_link` (pivot x = −0.06575) | `*_knee_pitch_link` | 0.04125 m |
| Shin | `*_knee_pitch_link` | `*_front_shin_pitch_link` (pivot x = 0, z = −0.04775) and `*_back_shin_pitch_link` (pivot x = −0.04125, z = −0.04775) | `*_ankle_roll_pitch_link` | 0.04125 m |

Loop-closure joints (absent from the URDF; **authored by `scripts/fix_robinion_usd.py`, present in `assets/robonionv2.usd`** as `PhysicsSphericalJoint` with `excludeFromArticulation = 1`):
- `*_back_thigh_loop_joint`: `*_back_thigh_pitch_link` @ (0, 0, −0.2) ↔ `*_knee_pitch_link` @ (−0.04125, 0, 0).
- `*_back_shin_loop_joint`: `*_back_shin_pitch_link` @ (0, 0, −0.2) ↔ `*_ankle_roll_pitch_link` @ (−0.04125, 0, 0).

Verified numerically: at q = 0 both anchor points of each loop joint coincide in world space (gap 0.000 mm) and the axes are parallel.

**Why spherical, not revolute.** The three tree joints of each stage are already parallel revolutes, so the closure only needs to pin one point (3 constraints). A revolute closure adds 5 and leaves the planar 4-bar over-constrained by 3 (Grübler in 3D: 6·3 − 4·5 = −2). The closure was originally authored as a revolute; that is stable on omni.physx 110.3.x (this venv's Isaac Sim 6.1.0) but on **omni.physx 110.1.13 (the Isaac Sim 6.0.1 GUI install at `/home/tkuai/isaacsim`) it makes the whole articulation explode to ~1e14 m on first ground contact** (`Invalid PhysX transform detected` on every link, robot vanishes). Reproduced and fixed headlessly with `scripts/drop_test.py`: same asset, same scene, revolute closure → explodes at t ≈ 0.6 s on 110.1.13; spherical closure → stable on both builds, anchor gaps ≤ 0.3 mm after the robot collapses onto the ground. Raising solver iterations, disabling self-collision, timestep, GPU vs CPU dynamics and spawn height made no difference — only the joint type did.

Because both stages are parallelograms, the coupler links **never rotate relative to their ground link**. `ik_controller_fullbody.py` (lines 204-224) publishes the mimic relation for all four passive joints, but the *direction of actuation* it implies (front_thigh and front_shin driven, ankle_pitch passive) is superseded by the user's direct confirmation of the real hardware: each leg has exactly 5 motors -- hip yaw, hip roll, hip pitch (`front_thigh_pitch_joint`), ankle pitch (`ankle_pitch_joint`), ankle roll -- all Dynamixel XH540-W270. The thigh parallelogram is driven from its **proximal** end (hip pitch motor on `front_thigh_pitch_joint`); the shin parallelogram is driven from its **distal** end (ankle pitch motor on `ankle_pitch_joint`), not from the knee end via `front_shin_pitch_joint`. This is the assignment implemented in `robots/robonionv2.py` and `env/robinion_env_cfg.py` (`ankle_pitch` actuated, `front_shin_pitch` passive) -- do not revert it to the IK script's implied direction without re-confirming with the user.

| Joint | Relation | Nature |
|---|---|---|
| `back_thigh_pitch` | = `front_thigh_pitch` | passive, mimic +1 |
| `knee_pitch` | = −`front_thigh_pitch` | **passive, mimic −1** (no actuator) |
| `back_shin_pitch` | = `front_shin_pitch` | passive, mimic +1 |
| `front_shin_pitch` | driven by the ankle-pitch motor through the shin parallelogram | **passive, no actuator of its own** |
| `ankle_pitch` | = −`front_shin_pitch` kinematically, but is the joint the ankle-pitch motor actually drives | **actuated** |

Net effect: the foot sole always stays parallel to the pelvis in the sagittal plane (no independent foot-pitch DOF); the two independently actuated sagittal joints are `front_thigh_pitch` (hip pitch motor) and `ankle_pitch` (ankle pitch motor), plus hip yaw, hip roll and ankle roll. The IK script uses link lengths `L1 = L2 = 0.18 m` whereas the URDF uses 0.20 m for both bars — one of them is wrong; measure the real robot.

Physical motor assignment is **confirmed** by the user ("di ankle ama hip itu ada 2 masing2 terus mereka yang atur roll ama pitch", "iya itu 5 motor per kaki dan telapak sejajar"): `hip_roll_pitch_link` carries the hip roll + hip pitch (front_thigh_pitch) motors, `ankle_roll_pitch_link` carries the ankle roll + ankle pitch motors. The URDF effort/velocity values (`front_thigh_pitch`/`back_thigh_pitch` = 19.8 N·m, `front_shin_pitch`/`ankle_pitch` = 9.9 N·m) are used as the per-joint limits regardless of which joint is actuated — see *Actuator* section below.

## Joint table

Columns: parent → child; origin xyz [m] / rpy [rad] in the parent frame; axis; URDF limits (effort N·m, velocity rad/s, lower/upper rad); URDF `<dynamics>` (damping N·m·s/rad, friction N·m); actuation status.

### Torso and head

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Axis | Effort | Vel | Lower / Upper | Damp / Fric | Status |
|---|---|---|---|---|---|---|---|---|---|---|
| `base_to_lower_body_fixed_joint` | fixed | base_link → lower_body_link | 0 0 0 | 0 0 0 | – | – | – | – | – | – |
| `torso_pitch_joint` | revolute | lower_body_link → upper_body_link | 0.019 0.0245 0.07675 | 0 0 0 | 0 1 0 | 10.6 | 3.14 | −0.52 / +1.5708 | 0.1 / 0 | actuated (motor model unconfirmed) |
| `imu_fixed_joint` | fixed | upper_body_link → imu_link | −0.021 −0.025 0.122 | 0 0 0 | – | – | – | – | – | sensor frame |
| `head_yaw_joint` | revolute | upper_body_link → head_yaw_link | −0.0125 −0.0245 0.183051 | 0 0 0 | 0 0 1 | 4.1 | 4.82 | ±1.5708 | 0.1 / 0 | actuated (motor model unconfirmed) |
| `head_pitch_joint` | revolute | head_yaw_link → head_pitch_link | 0 0.019 0.047 | 0 0 0 | 0 −1 0 | 4.1 | 4.82 | ±1.5708 | 0.1 / 0 | actuated (motor model unconfirmed) |
| `cam_fixed_joint` | fixed | head_pitch_link → cam_link | 0.019 −0.019 0.054 | 0 0 0 | – | – | – | – | – | sensor frame |

### Arms

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Axis | Effort | Vel | Lower / Upper | Damp / Fric | Status |
|---|---|---|---|---|---|---|---|---|---|---|
| `right_shoulder_pitch_joint` | revolute | upper_body_link → right_shoulder_pitch_link | −0.02089 −0.1305 0.1388 | 0 0.0873 0 | 0 −1 0 | 10.6 | 3.14 | ±2.8274 | 0.1 / 0 | actuated |
| `right_shoulder_roll_joint` | revolute | right_shoulder_pitch_link → right_shoulder_roll_link | 0.0245 −0.030017 −0.022 | 0 0 0 | −1 0 0 | 10.6 | 3.14 | ±1.5708 | 0.1 / 0 | actuated |
| `right_elbow_yaw_joint` | revolute | right_shoulder_roll_link → right_elbow_yaw_link | −0.0245 −0.118 0 | −1.5708 0 0 | 0 0 1 | 4.1 | 4.82 | ±2.8274 | 0.1 / 0 | actuated |
| `right_elbow_pitch_joint` | revolute | right_elbow_yaw_link → right_elbow_pitch_link | 0.022 −0.0245 −0.06 | 0 0 0 | 0 1 0 | 10.6 | 3.14 | ±1.5708 | 0.1 / 0 | actuated |
| `right_gripper_fixed_joint` | fixed | right_elbow_pitch_link → right_gripper | 0 0.0245 −0.068 | 0 0 0 | – | – | – | – | – | end-effector frame |
| `left_shoulder_pitch_joint` | revolute | upper_body_link → left_shoulder_pitch_link | −0.02089 0.0815 0.1388 | 0 0.0873 0 | 0 1 0 | 10.6 | 3.14 | ±2.8274 | 0.1 / 0 | actuated |
| `left_shoulder_roll_joint` | revolute | left_shoulder_pitch_link → left_shoulder_roll_link | 0.0245 0.030017 −0.022 | 0 0 0 | 1 0 0 | 10.6 | 3.14 | ±1.5708 | 0.1 / 0 | actuated |
| `left_elbow_yaw_joint` | revolute | left_shoulder_roll_link → left_elbow_yaw_link | −0.0245 0.118 0 | 1.5708 0 0 | 0 0 1 | 4.1 | 4.82 | ±2.8274 | 0.1 / 0 | actuated |
| `left_elbow_pitch_joint` | revolute | left_elbow_yaw_link → left_elbow_pitch_link | 0.022 0.0245 −0.06 | 0 0 0 | 0 1 0 | 10.6 | 3.14 | ±1.5708 | 0.1 / 0 | actuated |
| `l_gripper_fixed_joint` | fixed | left_elbow_pitch_link → left_gripper | 0 −0.0245 −0.068 | 0 0 0 | – | – | – | – | – | end-effector frame (note inconsistent name prefix `l_`) |

Note: right shoulder y-offset is −0.1305 but left is +0.0815 — the shoulders are **not mirrored** about the pelvis mid-plane in this URDF (the pelvis/torso origin itself is offset by 0.0245 in y at `torso_pitch_joint`). Check against the real robot.

### Right leg

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Axis | Effort | Vel | Lower / Upper | Damp / Fric | Status |
|---|---|---|---|---|---|---|---|---|---|---|
| `right_hip_yaw_joint` | revolute | lower_body_link → right_hip_yaw_link | 0 −0.055 0 | 0 0 0 | 0 0 1 | 4.1 | 4.82 | ±1.5708 | 0.1 / 0 | actuated |
| `right_hip_roll_joint` | revolute | right_hip_yaw_link → right_hip_roll_pitch_link | 0.047875 0 −0.05 | 0 0 0 | 1 0 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | actuated |
| `right_front_thigh_pitch_joint` | revolute | right_hip_roll_pitch_link → right_front_thigh_pitch_link | −0.0245 0.0245 0 | 0 0 0 | 0 1 0 | 19.8 | 4.08 | ±1.5708 | 0.1 / 0 | actuated (thigh-stage input) |
| `right_back_thigh_pitch_joint` | revolute | right_hip_roll_pitch_link → right_back_thigh_pitch_link | −0.06575 0.0245 0 | 0 0 0 | 0 1 0 | 19.8 | 4.08 | ±1.5708 | 0.1 / 0 | passive, = front_thigh_pitch |
| `right_knee_pitch_joint` | revolute | right_front_thigh_pitch_link → right_knee_pitch_link | 0 0 −0.2 | 0 0 0 | 0 1 0 | 19.8 | 4.08 | ±1.5708 | 0.1 / 0 | **passive, = −front_thigh_pitch (no motor)** |
| `right_front_shin_pitch_joint` | revolute | right_knee_pitch_link → right_front_shin_pitch_link | 0 0 −0.04775 | 0 0 0 | 0 1 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | passive, no actuator |
| `right_back_shin_pitch_joint` | revolute | right_knee_pitch_link → right_back_shin_pitch_link | −0.04125 0 −0.04775 | 0 0 0 | 0 1 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | passive, = front_shin_pitch |
| `right_ankle_pitch_joint` | revolute | right_front_shin_pitch_link → right_ankle_roll_pitch_link | 0 0 −0.2 | 0 0 0 | 0 1 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | actuated |
| `right_ankle_roll_joint` | revolute | right_ankle_roll_pitch_link → right_foot_roll_link | 0.0245 −0.02425 0 | 0 0 0 | 1 0 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | actuated |

### Left leg

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Axis | Effort | Vel | Lower / Upper | Damp / Fric | Status |
|---|---|---|---|---|---|---|---|---|---|---|
| `left_hip_yaw_joint` | revolute | lower_body_link → left_hip_yaw_link | 0 0.055 0 | 0 0 0 | 0 0 1 | 4.1 | 4.82 | ±1.5708 | 0.1 / 0 | actuated |
| `left_hip_roll_joint` | revolute | left_hip_yaw_link → left_hip_roll_pitch_link | 0.047875 0 −0.05 | 0 0 0 | 1 0 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | actuated |
| `left_front_thigh_pitch_joint` | revolute | left_hip_roll_pitch_link → left_front_thigh_pitch_link | −0.0245 0.0245 0 | 0 0 0 | 0 1 0 | **9.9** (right = 19.8) | 4.08 | ±1.5708 | 0.1 / 0 | actuated (thigh-stage input) |
| `left_back_thigh_pitch_joint` | revolute | left_hip_roll_pitch_link → left_back_thigh_pitch_link | −0.06575 0.0245 0 | 0 0 0 | 0 1 0 | 19.8 | 4.08 | ±1.5708 | 0.1 / 0 | passive, = front_thigh_pitch |
| `left_knee_pitch_joint` | revolute | left_front_thigh_pitch_link → left_knee_pitch_link | 0 0 −0.2 | 0 0 0 | 0 1 0 | 19.8 | 4.08 | ±1.5708 | 0.1 / 0 | **passive, = −front_thigh_pitch (no motor)** |
| `left_front_shin_pitch_joint` | revolute | left_knee_pitch_link → left_front_shin_pitch_link | 0 0 −0.04775 | 0 0 0 | 0 1 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | passive, no actuator |
| `left_back_shin_pitch_joint` | revolute | left_knee_pitch_link → left_back_shin_pitch_link | −0.04125 0 −0.04775 | 0 0 0 | 0 1 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | passive, = front_shin_pitch |
| `left_ankle_pitch_joint` | revolute | left_front_shin_pitch_link → left_ankle_roll_pitch_link | 0 0 −0.2 | 0 0 0 | 0 1 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | actuated |
| `left_ankle_roll_joint` | revolute | left_ankle_roll_pitch_link → left_foot_roll_link | 0.0245 −0.02425 0 | 0 0 0 | 1 0 0 | 9.9 | 4.08 | ±1.5708 | 0.1 / 0 | actuated |

Both legs use identical (non-mirrored) axes and y-offsets in `robinion_description`. The copy in `Robinion_InverseKinematic` instead mirrors the right leg (pitch axes `0 −1 0`, y-offsets negated) but leaves `right_ankle_pitch` at `0 1 0` and `left_ankle_pitch` at `0 −1 0` — internally inconsistent. The canonical URDF must pick one convention deliberately.

## Link table

Columns: mass [kg]; CoM xyz [m] in link frame; inertia at CoM [kg·m²]; visual mesh (`meshes/`); collision geometry (origin in link frame).

### Body and head

| Link | Mass | CoM | ixx / iyy / izz | ixy / ixz / iyz | Visual | Collision |
|---|---|---|---|---|---|---|
| `base_link` | – | – | – | – | – | – |
| `lower_body_link` | 0.960747 | −0.007122 0.000256 0.034914 | 0.002662 / 0.001197 / 0.002578 | 2e-6 / 1.13e-4 / 3e-6 | lower_body_visual.stl (z +0.006) | box 0.114×0.152×0.056 @ (−0.007, 0, 0.032) |
| `upper_body_link` | 1.234139 | −0.029870 −0.024411 0.123492 | 0.006795 / 0.003101 / 0.006100 | 0 / −9.8e-5 / −5e-6 | upper_body_visual.stl | box 0.113×0.172×0.135 @ (−0.027, −0.0245, 0.095) |
| `imu_link` | – | – | – | – | – | – |
| `head_yaw_link` | 0.017180 | 0 0 0.015679 | 9e-6 / 5e-6 / 6e-6 | 0 | head_yaw_visual.stl | none |
| `head_pitch_link` | 0.164500 | 0.000985 −0.018216 0.022305 | 1.15e-4 / 9.2e-5 / 4.7e-5 | 0 / 5e-6 / 1e-6 | head_pitch_visual.stl | box 0.04×0.09×0.028 @ (0, −0.019, 0.054) |
| `cam_link` | – | – | – | – | – | – |

### Arms (left values mirror right in y unless noted)

| Link | Mass | CoM (right) | ixx / iyy / izz | ixy / ixz / iyz (right) | Visual | Collision |
|---|---|---|---|---|---|---|
| `*_shoulder_pitch_link` | 0.044923 | 0 −0.010235 −0.004313 | 1.3e-5 / 2.7e-5 / 2.5e-5 | 0 / 0 / 3e-6 | *_shoulder_pitch_visual.stl | none |
| `*_shoulder_roll_link` | 0.389041 | −0.027884 −0.046707 0 | 6.98e-4 / 1.17e-4 / 7.53e-4 | 7.9e-5 / 0 / 0 | *_shoulder_roll_visual.stl | box 0.05×0.127×0.05 |
| `*_elbow_yaw_link` | 0.051730 | 0.003756 −0.032656 0 | 3.6e-5 / 2.8e-5 / 2.5e-5 | −5e-6 / 0 / 0 | *_elbow_yaw_visual.stl | none |
| `*_elbow_pitch_link` | 0.246669 | 0 −0.018081 −0.023869 | 1.35e-4 / 6.4e-5 / 1.21e-4 | 0 / 0 / 1e-6 | *_elbow_pitch_visual.stl | box 0.04×0.049×0.078 |
| `*_gripper` | 0.216195 | −0.000002 −0.043774 0.003217 | 1.83e-4 / 1.04e-4 / 1.91e-4 | 0 / 0 / −1.5e-5 | *_gripper_visual.stl | box 0.05×0.06×0.09 |

### Legs (values identical for left and right unless noted)

| Link | Mass | CoM | ixx / iyy / izz | ixy / ixz / iyz | Visual | Collision |
|---|---|---|---|---|---|---|
| `*_hip_yaw_link` | 0.048671 | 0.002613 0 −0.016031 | 2.0e-5 / 8.9e-5 / 8.0e-5 | 0 | *_hip_yaw_visual.stl | box 0.096×0.035×0.006 @ (0.002875, 0, −0.003) |
| `*_hip_roll_pitch_link` | 0.467377 | −0.044839 0.000311 −0.015111 | 1.98e-4 / 4.06e-4 / 3.26e-4 | 3e-6 / 1e-6 / 0 | *_hip_roll_pitch_visual.stl | box 0.08×0.0335×0.0645 @ (−0.045, 0, −0.0155) |
| `*_front_thigh_pitch_link` | 0.078 | 0.011621 −0.024250 −0.106919 | 2.55e-4 / 2.20e-4 / 4.3e-5 | 3e-6 / 4e-6 / 0 | *_thigh_visual.stl | box 0.007×0.0545×0.09 @ (0.015, −0.0245, −0.1), rpy (0, −0.093, 0) |
| `right_back_thigh_pitch_link` | **0.06** | −0.010643 −0.0245 −0.094743 | 2.55e-4 / 2.20e-4 / 4.3e-5 | 0 / 4e-6 / 0 | right_back_thigh_visual.stl | box 0.007×0.0545×0.057 @ (−0.015, −0.0245, −0.105) |
| `left_back_thigh_pitch_link` | **0.078** (≠ right) | same as right | same | same | left_back_thigh_visual.stl | same |
| `*_knee_pitch_link` | 0.34 | −0.005325 −0.024024 −0.017387 | 2.42e-4 / 2.10e-4 / 1.90e-4 | 0 | *_knee_visual.stl | box 0.0705×0.047×0.066 @ (−0.01825, −0.0245, −0.01775) |
| `*_front_shin_pitch_link` | 0.079 | 0.012786 −0.024250 −0.104736 | 2.59e-4 / 2.23e-4 / 4.3e-5 | 0 | *_shin_visual.stl | box 0.007×0.0545×0.09 @ (0.015, −0.0245, −0.1), rpy (0, 0.093, 0) |
| `*_back_shin_pitch_link` | 0.06 | −0.010643 −0.0245 −0.105257 | 2.55e-4 / 2.20e-4 / 4.3e-5 | 0 / 4e-6 / 0 | *_back_shin_visual.stl | box 0.007×0.0545×0.057 @ (−0.015, −0.0245, −0.095) |
| `*_ankle_roll_pitch_link` | 0.467377 | −0.044839 −0.000311 0.015111 | 1.98e-4 / 4.06e-4 / 3.26e-4 | 3e-6 / 1e-6 / 0 | *_ankle_roll_pitch_visual.stl (offset 0.0245, −0.0245, 0) | box 0.08×0.0335×0.0645 @ (−0.022, −0.0245, 0.0155) |
| `*_foot_roll_link` | 0.229 | −0.024014 0 −0.048357 | 5.57e-4 / 8.1e-5 / **−2.3e-5** (URDF, invalid — see below) | 3e-6 / 1e-6 / 0 | *_foot_visual.stl | mesh (same STL as visual; `convexHull` in USD) |

Foot inertia in `assets/robonionv2.usd`: the URDF tensor is kept and the negative principal moment is replaced with `5.0e-4` (project decision — do not re-derive inertia from the CAD, which is not trusted): `diagonalInertia = (5.0e-4, 8.098e-5, 5.570e-4)`, `principalAxes` as the converter wrote them. `newton:inertia` gets the same replacement (`izz = 5.0e-4`).

Clamping to exactly `0` (an earlier version of this fix) does not work: PhysX rejects it outright for an articulation link (`PxRigidBody::setMassSpaceInertiaTensor(): components must be > 0 for articulations`). `5.0e-4` is not arbitrary either — every rigid body's principal moments must satisfy the triangle inequality `|Ixx-Iyy| <= Izz <= Ixx+Iyy`, which for this link's `Ixx=5.57e-4, Iyy=8.1e-5` puts the physically valid range at `[4.76e-4, 6.38e-4]`; a uniform-box estimate from the foot mesh's bounding box (0.175 x 0.067 x 0.069 m, mass 0.229 kg) gives `Izz ~= 6.70e-4`, near the top of that range and corroborating it. `5.0e-4` sits inside the valid range, conservative relative to the box estimate.

Observation: `hip_roll_pitch_link` and `ankle_roll_pitch_link` share identical mass/inertia/dimensions (same physical servo bracket used at both ends), and the four thigh/shin bars share identical inertia tensors despite different masses — copy-pasted values, not per-part CAD output.

## Defects found in the URDF (must be fixed in the canonical copy)

1. `*_foot_roll_link` has **izz = −0.000023** — negative principal inertia is physically invalid and will be rejected or silently "fixed" by importers (PhysX rejects it outright for an articulation link, even clamped to exactly 0). *Worked around in `assets/robonionv2.usd` by replacing the negative moment with `5.0e-4`, a value inside the physically valid range for this link (fix 4); the URDF itself is left unchanged.*
2. Left/right asymmetries that look like typos: `left_front_thigh_pitch` effort 9.9 vs right 19.8; `left_back_thigh_pitch_link` mass 0.078 vs right 0.06.
3. Thigh/shin bar inertia tensors are copy-pasted (identical for 0.06 kg and 0.079 kg parts).
4. Loop-closure joints for both parallelograms are absent (URDF limitation) — the passive joints are unconstrained in any simulator that loads the file as-is. *Added in `assets/robonionv2.usd`.*
5. Shoulder mount y-offsets are not symmetric about the pelvis.
6. ~~Effort/velocity limits look like mixed datasheet values~~ — confirmed correct by the robot owner; use them as-is.
7. IK script link length (0.18 m) disagrees with URDF bar length (0.20 m).
8. Joint `<dynamics>` is a uniform placeholder (damping 0.1, friction 0) on every joint.
9. Naming: `l_gripper_fixed_joint` vs `right_gripper_fixed_joint`; link `right_foot_roll_link` vs the mesh name `right_foot`.

## Converting the URDF and building `assets/robonionv2.usd`

Two Isaac Sim installations exist on the dev machine, and **they run different `urdf_usd_converter` versions**:

| Install | How it's launched | `urdf_usd_converter` | Notes |
|---|---|---|---|
| `/home/tkuai/isaacsim` (Isaac Sim Full **6.0.1**, standalone Kit app, **omni.physx 110.1.13**) | desktop/taskbar icon, or headless via `/home/tkuai/isaacsim/python.sh script.py` | **v0.1.3** | Has a quaternion bug in the inertia export (see *Historical bug* below). Do **not** use its GUI importer for this URDF unless you also run the manual fix. Its PhysX build is the one that explodes on a revolute loop closure (see *Leg mechanism*), so every asset change must be drop-tested on it too. |
| this project's `.venv` (`isaacsim` pip package **6.1.0**, **omni.physx 110.3.2**) | `uv run ...` | **v0.3.2** | Bug fixed upstream. No GUI; convert with the CLI (below) and edit the result in the *other* install's GUI, or with the provided scripts. This is the engine Isaac Lab training runs on. |

Check a running Isaac Sim's converter version from its Script Editor: `import urdf_usd_converter; print(urdf_usd_converter.__version__)`. Every USD the converter writes also records its own version in `customLayerData.creator` at the top of the layer — grep for `creator =` before trusting any inertia value in a `.usda` file.

### Pipeline (all commands from the repo root)

```bash
# 1. URDF -> raw USD (converter 0.3.2, via this project's .venv)
uv run urdf_usd_converter \
    references/robinion_description/robinion2.urdf \
    assets/robonionv2 \
    -p robinion_description=$PWD/references/robinion_description

# 2. Author the 5 hand-made physics fixes into assets/robonionv2/robinion.usda
#    (idempotent: safe to re-run, e.g. after further GUI edits)
uv run python scripts/fix_robinion_usd.py assets/robonionv2/robinion.usda

# 3. Flatten into the standalone asset ArticulationCfg actually loads
uv run python scripts/flatten_usd.py assets/robonionv2/robinion.usda assets/robonionv2.usd

# 4. Verify: URDF + STL meshes vs. the final USD, link by link and joint by joint
uv run python scripts/check_urdf_vs_usd.py assets/robonionv2.usd

# 5. Dynamic sanity check: drop the undriven robot on a ground plane headlessly and
#    make sure the articulation does not explode and the loop closures hold.
#    Run it on BOTH engines -- they ship different PhysX builds (see below):
uv run python scripts/drop_test.py                                  # venv Isaac Sim 6.1.0 / physx 110.3.x, 200 Hz
/home/tkuai/isaacsim/python.sh scripts/drop_test.py --dt 0.0166667  # GUI  Isaac Sim 6.0.1 / physx 110.1.13, 60 Hz
```

Step 5 exists because the two installs behave differently: the revolute loop closure that 110.3.x tolerates makes 110.1.13 explode on ground contact (see *Leg mechanism*). `drop_test.py` has `--no-loop-joints` / `--spherical-loops` / `--gpu` / `--open-direct` / `--height` switches for isolating this kind of issue without going through the GUI; it exits non-zero on an explosion and prints the loop-anchor gaps after landing.

`assets/robonionv2/robinion.usda` can also be opened and edited in the Isaac Sim GUI (fixes 1–5 can be done by hand there instead of step 2 — see the fix descriptions below for the exact values); re-run steps 3–4 afterwards. Never hand-edit `assets/robonionv2/Payload/*.usda` — that is pure converter output, kept unmodified so a diff of `robinion.usda` against a fresh conversion shows exactly what was added by hand.

### 5 fixes (only usd can do it)

The URDF only expresses a *tree*, so `urdf_usd_converter` cannot produce these on its own:

1. **4 loop-closure joints**, one pair per leg (thigh stage + shin stage). Each parallelogram's ground/coupler pivots are `0.04125 m` apart and both bars are `0.2 m` long (see *Leg mechanism* above), so for each `*_back_{thigh,shin}_pitch_link` → coupler pair: `PhysicsSphericalJoint`, `body0` = back bar, `body1` = coupler link, `localPos0 = (0, 0, -0.2)`, `localPos1 = (-0.04125, 0, 0)`, `localRot0 = localRot1 = identity`, `excludeFromArticulation = True` (PhysX articulations must be trees; this authors the loop as an ordinary constraint instead). It must be **spherical**, not revolute — a revolute closure over-constrains the planar 4-bar and explodes on contact under omni.physx 110.1.x (see *Leg mechanism*). The script migrates an older revolute closure automatically.
2. **Self-collision off** on the articulation root (`lower_body_link`): `PhysxArticulationAPI` + `physxArticulation:enabledSelfCollisions = False` (and `newton:selfCollisionEnabled = False` for the Newton backend the converter also targets).
3. **Floating base**: deactivate `base_to_lower_body_fixed_joint` (`active = False`). The converter ties `base_link` (massless in the URDF) to the asset root, which PhysX treats as welded to the world; a walking robot needs a free pelvis. Because the joint is `def`-ined in `Payload/Physics.usda`, the root layer can only override it (`active = false`), not truly delete it — that is expected, not a mistake.
4. **Negative inertia fix**: URDF gives `*_foot_roll_link` `izz = -2.3e-5`, a negative principal moment (invalid). PhysX requires every principal moment to be strictly `> 0` for an articulation link, so clamping to `0` is rejected too (`PxRigidBody::setMassSpaceInertiaTensor(): components must be > 0 for articulations`). `fix_robinion_usd.py` scans every prim with `physics:diagonalInertia` and replaces any non-positive component with `NEGATIVE_INERTIA_REPLACEMENT = 5.0e-4` (also in `newton:inertia`), leaving the other moments, CoM and principal axes untouched: `diagonalInertia = (5.0e-4, 8.098e-5, 5.570e-4)`. `5.0e-4` comes from the triangle inequality every rigid body's principal moments must satisfy (`|Ixx-Iyy| <= Izz <= Ixx+Iyy`, giving `[4.76e-4, 6.38e-4]` here) cross-checked against a uniform-box estimate from the foot mesh's bounding box (`~6.70e-4`); see the script for the derivation. The inertia is deliberately *not* recomputed from the STL.
5. **`left_back_thigh_pitch_link` mass** 0.078 → 0.06 kg, to match `right_back_thigh_pitch_link` (URDF asymmetry, presumed typo — see *Defects* item 2).

`scripts/fix_robinion_usd.py` implements all 5; pass `--no-mass-fix` to skip #5 if you'd rather leave the asymmetry in place.

### Historical bug (converter ≤ 0.1.3, no longer relevant to the current pipeline)

An earlier version of this asset was built via the Isaac Sim Full 6.0.1 GUI importer, which runs converter **v0.1.3**. That version has a bug: `physics:principalAxes` is written as the *inverse* of the correct rotation (the eigenvector matrix from `numpy.linalg.eigh` is passed to `Gf.Matrix3d` without the transpose the row-vector convention requires; fixed upstream, see the `extract_inertia` comment in `urdf_usd_converter/_impl/link.py` for versions ≥ 0.3.0). Since PhysX applies the tensor as `R·diag·Rᵀ`, every non-isotropic link's simulated inertia was wrong (e.g. `lower_body_link`'s tensor was off by 39%, `upper_body_link` and `*_elbow_pitch_link` by ~53%). **This is why the pipeline above uses the `.venv`'s converter 0.3.2 instead of the GUI import** — no quaternion patch is needed with 0.3.2. If you ever import via the Isaac Sim Full 6.0.1 GUI again, check `customLayerData.creator` in the result and re-derive/negate every `principalAxes` quaternion's real part if it says v0.1.3.

### Verification method

`scripts/check_urdf_vs_usd.py` parses the URDF and all 31 STL meshes, locates the matching USD prim for every link/joint, and compares: local transform vs. joint `<origin>`; mass and CoM; full inertia tensor (reconstructed as `R·diag·Rᵀ` from `principalAxes`/`diagonalInertia`, flagged separately if the *inverse* rotation would have matched — the v0.1.3 bug signature); visual mesh (triangle count, bounding box, surface area, area-weighted centroid, origin) against the STL; collision shapes; joint type/body0/body1/frame/axis-with-sign/limits/effort/velocity/damping (reading both the pre-0.3 PhysX-drive attributes and the 0.3.x `urdf:limit:effort`/`newton:*` attributes, whichever the converter in use wrote); and a forward-kinematics check that loop-joint anchors coincide at the zero pose.

Expected `assets/robonionv2.usd` SUMMARY — anything beyond this list means an authored value drifted from the URDF or a fix regressed:

```
Counter({'INFO': 5, 'WARN': 1})
[INFO] right_foot_roll_link: URDF inertia has negative principal value ..., expecting it replaced with 0.0005 in USD  (fix 4; the checker compares against the replaced tensor)
[WARN] left_back_thigh_pitch_link: mass urdf 0.078 vs usd 0.06  (fix 5)
[INFO] left_foot_roll_link: URDF inertia has negative principal value ...  (fix 4)
[INFO] joint base_to_lower_body_fixed_joint (fixed) missing/inactive  (fix 3)
[INFO] joint imu_fixed_joint (fixed) missing/inactive  (converter merges massless links)
[INFO] joint cam_fixed_joint (fixed) missing/inactive  (converter merges massless links)
```

Everything else matches the URDF exactly: 35 links, 34 joints (29 revolute + 5 fixed), all transforms/mass/CoM/inertia/mesh/collision data, all 4 loop-joint anchors at 0.000 mm gap, self-collision off, no stray `PhysicsScene`/viewport prims in the flattened file, and `layers used: ['assets/robonionv2.usd']` (standalone, no dependency on `assets/robonionv2/` or `references/`).

### Observations carried over from the URDF (not conversion errors)

- Thigh/shin bar collision boxes cover only 5–8 % of the visual-mesh volume (57–90 mm box on a 210 mm bar); `*_hip_yaw_link` box is 6 mm thick. Contacts with the ground/objects along the bars will not be detected. Self-collision is disabled anyway.
- No physics material is bound anywhere → foot friction is the PhysX default until an `ArticulationCfg`/scene material sets it.
- Joint drives have no stiffness authored; the `ArticulationCfg` actuator models override these.

## Actuator: Dynamixel XH540-W270 (datasheet values — for gain/armature estimates only)

> Body servos are XH540-W270 (confirmed by the robot owner; **not** MX-106), head is XH430-W350, bus 11.1 V. Effort/velocity limits are **not** taken from the datasheet — they come from the URDF (confirmed correct). The table is used only to derive stiffness/damping/armature in `robots/robonionv2.py`.

| Parameter | Value |
|---|---|
| Gear ratio | 272.5 : 1 |
| Stall torque | 9.2 N·m @ 11.1 V (4.5 A) · 9.9 N·m @ 12 V (4.9 A) · 11.7 N·m @ 14.8 V (5.9 A) |
| No-load speed | 36 rpm (3.77 rad/s) @ 11.1 V · 39 rpm (4.08 rad/s) @ 12 V · 46 rpm (4.82 rad/s) @ 14.8 V |
| Operating voltage | 10.0–14.8 V (recommended 12.0 V) |
| Position resolution | 4096 pulses/rev (0.088°) |
| Weight | 165 g |
| Dimensions | 33.5 × 58.5 × 44 mm |
| Control (Protocol 2.0, position mode) | PID on position error, output = PWM (100 % = 885). Default P gain 800 (K_P = 800/128 = 6.25), I = 0, D = 0. Current limit 2047, velocity limit 167 × 0.229 rpm ≈ 38 rpm |
| Rotor inertia, gear friction, backlash | not published — must be identified |

First-principles conversion to an Isaac Lab PD actuator (12 V, default P gain, no I/D):
- PWM saturates (100 %) at a position error of 885 / (6.25 × 651.9 pulses/rad) ≈ **0.217 rad (12.4°)**.
- Effective stiffness below saturation ≈ stall torque / 0.217 rad ≈ **46 N·m/rad** (9.9 N·m @ 12 V).
- Back-EMF speed droop gives an effective damping ≈ stall torque / no-load speed ≈ 9.9 / 4.08 ≈ **2.4 N·m·s/rad**.
- Torque–speed envelope is linear from stall to no-load: Isaac Lab's `DCMotorCfg` (saturation_effort = stall torque, velocity_limit = no-load speed) reproduces this; `ImplicitActuatorCfg` with the stiffness/damping above is the simpler approximation.
- Reflected rotor inertia (armature) for a 272.5:1 coreless motor is expected in the 1e-3 to 5e-3 kg·m² range; treat as a tunable to be identified, not a datasheet value.

These numbers change with supply voltage and with the P/I/D gains actually programmed into the servos — both must be read from the real robot.

