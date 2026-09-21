# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Placeholder manager-based env for the Robinion MPC controller.

Not a gym-registered RL task: the MPC controller (``mpc.controller.MpcHumanoidController``)
will read observations from and write actions into this env's managers directly, rather
than through a reward/termination loop, so this stays a plain ``ManagerBasedEnvCfg``
(not ``ManagerBasedRLEnvCfg``). The action/observation terms below are placeholders --
a straight joint-effort passthrough and raw joint/base state -- to be replaced once the
MPC controller and its state adapter (``mpc.state``, not yet implemented) exist; contact
sensors and the model spec are also added in later phases.
"""

from __future__ import annotations

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

from ..robots.robonionv2 import ROBINION_CFG

# Independently actuated joints (legs + torso/arms + head actuator groups in ROBINION_CFG).
# Excludes the 4 passive parallelogram joints -- never command those directly (AGENTS.md).
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

##
# Scene definition
##


@configclass
class MpcHumanoidRobinionSceneCfg(InteractiveSceneCfg):
    """Flat-ground scene holding a single Robinion robot."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot
    robot: ArticulationCfg = ROBINION_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


@configclass
class ActionsCfg:
    joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=_ACTUATED_JOINTS, scale=1.0)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class StateCfg(ObsGroup):
        """Observations for the MPC controller."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    state: StateCfg = StateCfg()


##
# Environment configuration
##


@configclass
class MpcHumanoidRobinionEnvCfg(ManagerBasedEnvCfg):
    """Placeholder manager-based environment configuration for the Robinion MPC controller."""

    # Scene settings
    scene: MpcHumanoidRobinionSceneCfg = MpcHumanoidRobinionSceneCfg(num_envs=1, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()

    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
