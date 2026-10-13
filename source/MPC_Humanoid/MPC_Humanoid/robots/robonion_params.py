# Ref for mpc model builder and the isaac sim

from __future__ import annotations

import re
from typing import NamedTuple


class Servo(NamedTuple):
    effort: float  # N·m
    velocity: float  # rad/s
    stiffness: float  # N·m/rad
    damping: float  # N·m·s/rad
    armature: float  # kg·m²


# effort, velocity: stall torque and no-load speed at 12 V (ROBOTIS e-Manual); the URDF limits are these same numbers.
# stiffness: 11.1 V stall torque / 0.217 rad, the position error where the default P gain saturates the PWM
# (references/docs/joint_info.md). damping: 11.1 V stall torque / no-load speed. armature: not published, placeholder
# until the servos are identified.
SERVOS = {
    "XH540-W270": Servo(9.9, 4.08, 42.0, 2.4, 0.003),  # 9.2 N·m, 36 rpm at 11.1 V
    "XM540-W270": Servo(10.6, 3.14, 46.0, 3.4, 0.003),  # 10.0 N·m, 28 rpm at 11.1 V
    "XM430-W350": Servo(4.1, 4.82, 17.5, 0.84, 0.002),  # 3.8 N·m, 43 rpm at 11.1 V
}

# User, 2026-10-13. Differs from the URDF limits on hip yaw (URDF: XM430 numbers) and elbow pitch (URDF: XM540).
# Both front thighs are one XH540-W270 (URDF has 19.8 N·m on the right; user decision 2026-10-06).
SERVO_JOINTS = {  # servo -> joint-name regexes
    "XH540-W270": [".*_hip_roll_joint", ".*_front_thigh_pitch_joint", ".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
    "XM540-W270": [".*_hip_yaw_joint", "torso_pitch_joint", ".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
    "XM430-W350": [".*_elbow_yaw_joint", ".*_elbow_pitch_joint", "head_yaw_joint", "head_pitch_joint"],
}
URDF_FRICTION = 0.0  # N·m


def servo_of(joint: str) -> Servo:
    for name, patterns in SERVO_JOINTS.items():
        if any(re.fullmatch(p, joint) for p in patterns):
            return SERVOS[name]
    raise KeyError(f"no servo for joint {joint}")


# 0 = straight legs; > 0 crouches
LEG_CROUCH = 0.0  # rad
# 0 = T-pose; more negative = arms lower
SHOULDER_ROLL_DOWN = -1.0  # rad

DEFAULT_JOINT_POS = {  # joint-name regex -> angle [rad]
    ".*_front_thigh_pitch_joint": -LEG_CROUCH,
    ".*_back_thigh_pitch_joint": -LEG_CROUCH,
    ".*_knee_pitch_joint": LEG_CROUCH,
    ".*_front_shin_pitch_joint": LEG_CROUCH,
    ".*_back_shin_pitch_joint": LEG_CROUCH,
    ".*_ankle_pitch_joint": -LEG_CROUCH,
    ".*_hip_yaw_joint": 0.0,
    ".*_hip_roll_joint": 0.0,
    ".*_ankle_roll_joint": 0.0,
    "torso_pitch_joint": 0.0,
    "head_.*_joint": 0.0,
    ".*_shoulder_pitch_joint": 0.0,
    ".*_shoulder_roll_joint": SHOULDER_ROLL_DOWN,
    ".*_elbow_.*_joint": 0.0,
}
