# Ref for mpc model builder and the isaac sim

# Motor parameters follow references/robonion_description/robonion2.urdf, which is a dict of
URDF_LIMITS = {  # key <-> value: (effort [N·m], velocity [rad/s])
    "yaw": (4.1, 4.82),  # hip yaw, elbow yaw, head yaw
    "torso_arms": (10.6, 3.14),  # torso pitch, shoulder pitch/roll, elbow pitch
    "legs": (9.9, 4.08),  # hip roll, ankle pitch/roll, left front thigh
    "right_thigh": (19.8, 4.08),
}
URDF_FRICTION = 0.0  # N·m

# Values not defined in the URDF come from the servo datasheets (use 11.1 V for the meantime)
XH540_STIFFNESS = 42.0  # N·m/rad
XH430_STIFFNESS = 16.6  # N·m/rad
XH540_DAMPING = 2.4  # N·m·s/rad, 9.2 N·m / 3.77 rad/s
XH430_DAMPING = 1.1  # N·m·s/rad, 3.1 N·m / 2.83 rad/s
XH540_ARMATURE = 0.003
XH430_ARMATURE = 0.002

# 0 rad for standing more than 0 will crouch
LEG_CROUCH = 0.0  # rad
# set 0 for tpose, -1.4 for arm down initialize
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
