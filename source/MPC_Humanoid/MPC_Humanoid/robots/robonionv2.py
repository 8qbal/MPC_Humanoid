# Robonion TKU humanoid

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

MPC_HUMANOID_ASSETS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../assets"))


# Motor parameters follow references/robonion_description/robonion2.urdf, which is authoritative for
URDF_LIMITS = {  # group: (effort [N·m], velocity [rad/s])
    "yaw": (4.1, 4.82),  # hip yaw, elbow yaw, head
    "torso_arms": (10.6, 3.14),  # torso pitch, shoulder pitch/roll, elbow pitch
    "legs": (9.9, 4.08),  # hip roll, ankle pitch/roll, left front thigh
    "right_thigh": (19.8, 4.08),
}
URDF_FRICTION = 0.0  # N·m, <dynamics friction> on every joint

# Values not defined in the URDF come from the servo datasheets (use 11.1 V bus til i check with jaesik)
XH540_STIFFNESS = 42.0  # N·m/rad
XH430_STIFFNESS = 16.6  # N·m/rad
XH540_DAMPING = 2.4  # N·m·s/rad, 9.2 N·m / 3.77 rad/s
XH430_DAMPING = 1.1  # N·m·s/rad, 3.1 N·m / 2.83 rad/s
XH540_ARMATURE = 0.003
XH430_ARMATURE = 0.002

# 0 rad for standing more than 0 will crouch
_LEG_CROUCH = 0.0  # rad
# set 0 for tpose, -1.4 for arm down initialize
_SHOULDER_ROLL_DOWN = 0.0  # rad

Robonion_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{MPC_HUMANOID_ASSETS_DIR}/robonionv2.usd",
        activate_contact_sensors=False,  # the real robot has no foot contact sensors
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=False,
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.56),
        joint_pos={
            ".*_front_thigh_pitch_joint": -_LEG_CROUCH,
            ".*_back_thigh_pitch_joint": -_LEG_CROUCH,
            ".*_knee_pitch_joint": _LEG_CROUCH,
            ".*_front_shin_pitch_joint": _LEG_CROUCH,
            ".*_back_shin_pitch_joint": _LEG_CROUCH,
            ".*_ankle_pitch_joint": -_LEG_CROUCH,
            ".*_hip_yaw_joint": 0.0,
            ".*_hip_roll_joint": 0.0,
            ".*_ankle_roll_joint": 0.0,
            "torso_pitch_joint": 0.0,
            "head_.*_joint": 0.0,
            ".*_shoulder_pitch_joint": 0.0,
            ".*_shoulder_roll_joint": _SHOULDER_ROLL_DOWN,
            ".*_elbow_.*_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "yaw": DCMotorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_elbow_yaw_joint",
            ],
            saturation_effort=URDF_LIMITS["yaw"][0],
            actuator_effort_limit=URDF_LIMITS["yaw"][0],
            actuator_velocity_limit=URDF_LIMITS["yaw"][1],
            joint_effort_limit=URDF_LIMITS["yaw"][0],
            joint_velocity_limit=URDF_LIMITS["yaw"][1],
            stiffness=XH540_STIFFNESS,
            damping=XH540_DAMPING,
            friction=URDF_FRICTION,
            armature=XH540_ARMATURE,
        ),
        "torso_arms": DCMotorCfg(
            joint_names_expr=[
                "torso_pitch_joint",
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_elbow_pitch_joint",
            ],
            saturation_effort=URDF_LIMITS["torso_arms"][0],
            actuator_effort_limit=URDF_LIMITS["torso_arms"][0],
            actuator_velocity_limit=URDF_LIMITS["torso_arms"][1],
            joint_effort_limit=URDF_LIMITS["torso_arms"][0],
            joint_velocity_limit=URDF_LIMITS["torso_arms"][1],
            stiffness=XH540_STIFFNESS,
            damping=XH540_DAMPING,
            friction=URDF_FRICTION,
            armature=XH540_ARMATURE,
        ),
        "legs": DCMotorCfg(
            joint_names_expr=[
                ".*_hip_roll_joint",
                "left_front_thigh_pitch_joint",
                ".*_ankle_pitch_joint",
                ".*_ankle_roll_joint",
            ],
            saturation_effort=URDF_LIMITS["legs"][0],
            actuator_effort_limit=URDF_LIMITS["legs"][0],
            actuator_velocity_limit=URDF_LIMITS["legs"][1],
            joint_effort_limit=URDF_LIMITS["legs"][0],
            joint_velocity_limit=URDF_LIMITS["legs"][1],
            stiffness=XH540_STIFFNESS,
            damping=XH540_DAMPING,
            friction=URDF_FRICTION,
            armature=XH540_ARMATURE,
        ),
        "right_thigh": DCMotorCfg(
            joint_names_expr=[
                "right_front_thigh_pitch_joint",
            ],
            saturation_effort=URDF_LIMITS["right_thigh"][0],
            actuator_effort_limit=URDF_LIMITS["right_thigh"][0],
            actuator_velocity_limit=URDF_LIMITS["right_thigh"][1],
            joint_effort_limit=URDF_LIMITS["right_thigh"][0],
            joint_velocity_limit=URDF_LIMITS["right_thigh"][1],
            stiffness=XH540_STIFFNESS,
            damping=XH540_DAMPING,
            friction=URDF_FRICTION,
            armature=XH540_ARMATURE,
        ),
        "head": DCMotorCfg(
            joint_names_expr=[
                "head_yaw_joint",
                "head_pitch_joint",
            ],
            saturation_effort=URDF_LIMITS["yaw"][0],
            actuator_effort_limit=URDF_LIMITS["yaw"][0],
            actuator_velocity_limit=URDF_LIMITS["yaw"][1],
            joint_effort_limit=URDF_LIMITS["yaw"][0],
            joint_velocity_limit=URDF_LIMITS["yaw"][1],
            stiffness=XH430_STIFFNESS,
            damping=XH430_DAMPING,
            friction=URDF_FRICTION,
            armature=XH430_ARMATURE,
        ),
        # Passive joints in the knee four-bar linkage.
        "passive": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_knee_pitch_joint",
                ".*_back_thigh_pitch_joint",
                ".*_front_shin_pitch_joint",
                ".*_back_shin_pitch_joint",
            ],
            stiffness=0.0,
            damping=0.0,
            joint_effort_limit=0.0,
        ),
    },
)
