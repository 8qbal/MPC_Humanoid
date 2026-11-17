# Ref for mpc model builder and the isaac sim

from __future__ import annotations

import json
import math
import os
import re
from typing import NamedTuple


class Servo(NamedTuple):
    effort: float  # N·m
    velocity: float  # rad/s
    stiffness: float  # N·m/rad
    damping: float  # N·m·s/rad
    armature: float  # kg·m²
    friction: float  # N·m, Coulomb


VIN = 14.8  # V
KP_FW = 800.0  # firmware Position P Gain register (X-series default, references/docs/joint_info.md)
# rad -> PWM duty per unit of P gain: 4096 counts/rev, P gain / 128, PWM limit 885 (bam dynamixel/xseries_actuator.py)
ERROR_GAIN = (4096 / (2 * math.pi)) / (128 * 885)

_SERVO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../assets/servos"))
SERVO_PARAM_FILES = {  # BAM m6 identifications (bam.fit), one per servo type
    "XH540-W270": f"{_SERVO_DIR}/xh540_m6.json",
    "XM540-W270": f"{_SERVO_DIR}/xm540_m6.json",
    "XM430-W350": f"{_SERVO_DIR}/xm430_m6.json",
}


def _servo_from_bam(path: str) -> Servo:
    """Linear servo equivalent of a BAM voltage-controlled servo: full-duty stall torque, no-load speed, the P loop
    slope (duty reaches 1 at 1 / (KP_FW * ERROR_GAIN) rad), back-EMF plus viscous damping."""
    with open(path) as f:
        p = json.load(f)
    effort = p["kt"] * VIN / p["R"]
    return Servo(
        effort,
        VIN / p["kt"],
        effort * KP_FW * ERROR_GAIN,
        p["kt"] ** 2 / p["R"] + p["friction_viscous"],
        p["armature"],
        p["friction_base"],
    )


SERVOS = {name: _servo_from_bam(path) for name, path in SERVO_PARAM_FILES.items()}

# servo -> {joint-name regex: number of servos driving the joint in parallel}, as in actuators.yaml (user, 2026-10-15)
SERVO_JOINTS = {
    "XH540-W270": {
        ".*_hip_roll_joint": 1,
        ".*_front_thigh_pitch_joint": 2,
        ".*_knee_pitch_joint": 2,
        ".*_front_shin_pitch_joint": 1,
        ".*_ankle_pitch_joint": 1,
        ".*_ankle_roll_joint": 1,
    },
    "XM540-W270": {
        ".*_hip_yaw_joint": 1,
        "torso_pitch_joint": 1,
        ".*_shoulder_pitch_joint": 1,
        ".*_shoulder_roll_joint": 1,
        ".*_elbow_pitch_joint": 1,
    },
    "XM430-W350": {".*_elbow_yaw_joint": 1, "head_yaw_joint": 1, "head_pitch_joint": 1},
}

# Parallelogram (4-bar) joints that follow an independent joint: suffix -> (independent joint suffix, gain).
# knee and front shin carry servos; back thigh and back shin are bare bars.
PASSIVE_COUPLING = {
    "knee_pitch_joint": ("front_thigh_pitch_joint", -1.0),
    "back_thigh_pitch_joint": ("front_thigh_pitch_joint", 1.0),
    "front_shin_pitch_joint": ("ankle_pitch_joint", -1.0),
    "back_shin_pitch_joint": ("ankle_pitch_joint", -1.0),
}
PASSIVE_JOINTS = {
    f"{side}_{follower}": (f"{side}_{lead}", gain)
    for side in ("left", "right")
    for follower, (lead, gain) in PASSIVE_COUPLING.items()
}


def scaled(servo: Servo, count: int) -> Servo:
    fields = ("effort", "stiffness", "damping", "armature", "friction")
    return servo._replace(**{f: getattr(servo, f) * count for f in fields})


def servo_of(joint: str) -> Servo | None:
    """The servos on one joint (parallel servos summed); None for a joint without a servo."""
    for name, patterns in SERVO_JOINTS.items():
        for pattern, count in patterns.items():
            if re.fullmatch(pattern, joint):
                return scaled(SERVOS[name], count)
    return None


def drive_of(joint: str) -> Servo:
    """The servos acting on an independent joint's coordinate: its own plus those on the parallelogram joints that
    follow it (a follower at gain g adds g² to stiffness, damping and armature, |g| to effort and friction)."""
    s = servo_of(joint)
    if s is None:
        raise KeyError(f"no servo for joint {joint}")
    side = joint.split("_", 1)[0]
    for follower, (lead, g) in PASSIVE_COUPLING.items():
        f = servo_of(f"{side}_{follower}")
        if joint == f"{side}_{lead}" and f is not None:
            s = Servo(
                s.effort + abs(g) * f.effort,
                min(s.velocity, f.velocity),
                s.stiffness + g * g * f.stiffness,
                s.damping + g * g * f.damping,
                s.armature + g * g * f.armature,
                s.friction + abs(g) * f.friction,
            )
    return s


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
