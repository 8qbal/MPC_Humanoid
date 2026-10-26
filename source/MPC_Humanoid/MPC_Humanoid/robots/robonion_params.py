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
    friction: float  # N·m, Coulomb


# From references/robinion_description/config/actuators.yaml (upstream 55ac2c2), at 14.8 V: effort, velocity = stall
# torque and no-load speed (ROBOTIS e-Manual); damping = back-EMF (stall / no-load) + gearbox viscous; armature and
# friction = priors from BAM identifications of MX servos, until the servos are identified.
# stiffness: stall torque / 0.217 rad, the position error where the default P gain saturates the PWM
# (references/docs/joint_info.md).
SERVOS = {
    "XH540-W270": Servo(11.7, 4.817, 11.7 / 0.217, 2.429 + 0.035, 0.038, 0.10),
    "XM540-W270": Servo(12.9, 3.875, 12.9 / 0.217, 3.329 + 0.035, 0.038, 0.10),
    "XM430-W350": Servo(4.8, 5.969, 4.8 / 0.217, 0.804 + 0.012, 0.038, 0.09),
}

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
