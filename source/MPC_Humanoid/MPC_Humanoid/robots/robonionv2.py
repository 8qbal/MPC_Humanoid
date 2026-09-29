# Robonion TKU humanoid config for isaaclab use

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from .robonion_params import (
    DEFAULT_JOINT_POS,
    URDF_FRICTION,
    URDF_LIMITS,
    XH430_ARMATURE,
    XH430_DAMPING,
    XH430_STIFFNESS,
    XH540_ARMATURE,
    XH540_DAMPING,
    XH540_STIFFNESS,
)

MPC_HUMANOID_ASSETS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../assets"))

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
            # TGS velocity iterations change the joint velocities after the positions are integrated: with 4, a sole
            # lying flat read up to 3.6 deg/s of tilt rate from encoders + gyro (docs/stage2.md); with 0, 0.55 deg/s
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.56),
        joint_pos=DEFAULT_JOINT_POS,
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
