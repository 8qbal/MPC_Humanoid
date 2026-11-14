# Robonion TKU humanoid config for isaaclab use

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from ..actuators import BamMotorCfg
from .robonion_params import (
    DEFAULT_JOINT_POS,
    ERROR_GAIN,
    KP_FW,
    SERVO_JOINTS,
    SERVO_PARAM_FILES,
    SERVOS,
    VIN,
    scaled,
)

# one actuator group per servo type and number of servos per joint
_GROUPS: dict[tuple[str, int], list[str]] = {}
for _name, _joints in SERVO_JOINTS.items():
    for _pattern, _count in _joints.items():
        _GROUPS.setdefault((_name, _count), []).append(_pattern)

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
        # the group name goes into a USD prim name, which cannot hold '-'
        f"{name.replace('-', '_')}_x{count}": BamMotorCfg(
            joint_names_expr=patterns,
            params_json=SERVO_PARAM_FILES[name],
            kp_fw=KP_FW,
            error_gain=ERROR_GAIN,
            vin=VIN,
            parallel_count=count,
            actuator_effort_limit=float("inf"),  # the duty cycle and back-EMF bound the torque
            actuator_velocity_limit=s.velocity,
            joint_effort_limit=s.effort,
            joint_velocity_limit=s.velocity,
        )
        for (name, count), patterns in _GROUPS.items()
        for s in [scaled(SERVOS[name], count)]
    }
    | {
        # The back bars of the two parallelograms carry no servo.
        "passive": ImplicitActuatorCfg(
            joint_names_expr=[".*_back_thigh_pitch_joint", ".*_back_shin_pitch_joint"],
            stiffness=0.0,
            damping=0.0,
            joint_effort_limit=0.0,
        ),
    },
)
