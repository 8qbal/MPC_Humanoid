# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Robinion scene/env config driven directly by the MPC controller (no gym/RL task)."""

from .robinion_env_cfg import MpcHumanoidRobinionEnvCfg, MpcHumanoidRobinionSceneCfg

__all__ = ["MpcHumanoidRobinionEnvCfg", "MpcHumanoidRobinionSceneCfg"]
