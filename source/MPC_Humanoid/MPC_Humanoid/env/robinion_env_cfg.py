# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Manager-based env for the Robinion MPC controller.
The controller reads observations from and writes actions into this env's managers directly,
so it is a plain ``ManagerBasedEnvCfg`` (no reward/termination managers).
"""

from __future__ import annotations

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ImuCfg
from isaaclab.utils.configclass import configclass
from isaaclab_physx.physics import PhysxCfg

from ..robots.robonionv2 import ROBINION_CFG

# Independently actuated joints: legs, torso, arms, and head groups from ROBINION_CFG.
# The four passive parallelogram joints are excluded and must never be commanded directly.
_ACTUATED_JOINTS = [
    ".*_hip_yaw_joint",
    ".*_hip_roll_joint",
    ".*_front_thigh_pitch_joint",
    ".*_ankle_pitch_joint",
    ".*_ankle_roll_joint",
    "torso_pitch_joint",
    ".*_shoulder_pitch_joint",
    ".*_shoulder_roll_joint",
    ".*_elbow_yaw_joint",
    ".*_elbow_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
]

# Scene definition.


@configclass
class MpcHumanoidRobinionSceneCfg(InteractiveSceneCfg):
    """Flat-ground scene holding a single Robinion robot."""

    # Procedural static slab (top face at z = 0) instead of GroundPlaneCfg, whose USD lives on
    # Nucleus and is not available offline.
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.CuboidCfg(
            size=(100.0, 100.0, 0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0, restitution=0.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
    )

    robot: ArticulationCfg = ROBINION_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # MPU6050-class IMU on the real robot (raw gyro + accel, no fused orientation), mounted at
    # the URDF imu_link frame on upper_body_link, axes aligned with that link.
    imu = ImuCfg(prim_path="{ENV_REGEX_NS}/Robot/.*/upper_body_link/imu_link", update_period=0.0)

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


# MDP settings.


@configclass
class ActionsCfg:
    joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=_ACTUATED_JOINTS, scale=1.0)


@configclass
class ObservationsCfg:
    @configclass
    class StateCfg(ObsGroup):
        """Raw state for the MPC controller (no noise, no clipping).

        ``imu_*`` and joint terms are what the hardware can also measure; ``base_*`` are sim
        ground truth for validation.
        """

        imu_ang_vel = ObsTerm(func=mdp.imu_ang_vel, params={"asset_cfg": SceneEntityCfg("imu")})
        imu_lin_acc = ObsTerm(func=mdp.imu_lin_acc, params={"asset_cfg": SceneEntityCfg("imu")})
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    state: StateCfg = StateCfg()


@configclass
class EventsCfg:
    # Also reset the actuator PD targets, otherwise they stay at zero and pull the legs
    # out of the crouch as soon as the sim starts.
    reset_scene = EventTerm(func=mdp.reset_scene_to_default, mode="reset", params={"reset_joint_targets": True})


# Environment configuration.


@configclass
class MpcHumanoidRobinionEnvCfg(ManagerBasedEnvCfg):
    """Single-robot, flat-ground, deterministic PhysX environment for the Robinion MPC controller."""

    scene: MpcHumanoidRobinionSceneCfg = MpcHumanoidRobinionSceneCfg(num_envs=1, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventsCfg = EventsCfg()

    def __post_init__(self) -> None:
        self.seed = 0
        # 1 kHz physics is required by the explicit DCMotorCfg damping (see robots/robonionv2.py);
        # 5x decimation gives a 200 Hz control step.
        self.decimation = 5
        self.sim.dt = 1.0 / 1000.0
        self.sim.render_interval = self.decimation
        # The loop-closure joints (spherical, excludeFromArticulation) and the PhysX-specific
        # articulation props in ROBINION_CFG were validated on PhysX only (diagnosed with a drop test, since removed).
        self.sim.physics = PhysxCfg(
            solver_type=1,
            enable_enhanced_determinism=True,
            enable_stabilization=True,
            bounce_threshold_velocity=0.2,
        )
