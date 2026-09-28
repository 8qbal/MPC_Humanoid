"""Robonion servo parameters, without Isaac imports: robonionv2.py builds the actuator models from them, and the
controller (mpc/) uses them on machines without Isaac Sim (e.g. the Jetson)."""

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
