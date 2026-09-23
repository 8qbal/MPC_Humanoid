# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Manager-based env for the Robonion MPC controller.
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
# PvaCfg for AHRS, ImuCfg for the IMU
from isaaclab.sensors import ImuCfg, PvaCfg
from isaaclab.utils.configclass import configclass
from isaaclab_physx.physics import PhysxCfg

from ..robots.robonionv2 import Robonion_CFG

# Independently actuated joints: legs, torso, arms, and head groups from Robonion_CFG.
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
class MpcHumanoidRobonionSceneCfg(InteractiveSceneCfg):
    """
    Flat-ground scene holding a single Robonion robot.
    """

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

    robot: ArticulationCfg = Robonion_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Xsens MTi-630 AHRS at imu_link: gyro + accel (ImuCfg) and fused orientation (PvaCfg).
    imu = ImuCfg(prim_path="{ENV_REGEX_NS}/Robot/.*/upper_body_link/imu_link", update_period=0.0)
    ahrs = PvaCfg(prim_path="{ENV_REGEX_NS}/Robot/.*/upper_body_link/imu_link", update_period=0.0)

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


# MDP settings.

# Action sent to the robot
@configclass
class ActionsCfg:
    joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=_ACTUATED_JOINTS, scale=1.0)

# Data that can be observed will be used to feed the controller
@configclass
class ObservationsCfg:
    @configclass
    class StateCfg(ObsGroup):
        """
        State that can be observed and measured.
        """

        imu_ang_vel = ObsTerm(func=mdp.imu_ang_vel, params={"asset_cfg": SceneEntityCfg("imu")})
        imu_lin_acc = ObsTerm(func=mdp.imu_lin_acc, params={"asset_cfg": SceneEntityCfg("imu")})
        imu_orientation = ObsTerm(func=mdp.pva_orientation, params={"asset_cfg": SceneEntityCfg("ahrs")})
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    state: StateCfg = StateCfg()

# Stating the event
@configclass
class EventsCfg:
    reset_scene = EventTerm(func=mdp.reset_scene_to_default, mode="reset", params={"reset_joint_targets": True})


# Environment configuration.


@configclass
class MpcHumanoidRobonionEnvCfg(ManagerBasedEnvCfg):
    """Single-robot, flat-ground, deterministic PhysX environment for the Robonion MPC controller."""

    scene: MpcHumanoidRobonionSceneCfg = MpcHumanoidRobonionSceneCfg(num_envs=1, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventsCfg = EventsCfg()


    # TODO: do more research on the freq (match motor freq with mpc and ahrs (do a double sampling?))
    def __post_init__(self) -> None:
        self.seed = 0
        self.decimation = 5
        self.sim.dt = 1.0 / 1000.0
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysxCfg(
            solver_type=1,
            enable_enhanced_determinism=True,
            enable_stabilization=True,
            bounce_threshold_velocity=0.2,
        )
