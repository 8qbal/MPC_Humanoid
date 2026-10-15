"""Joint position action that also drives the servos on the parallelogram follower joints."""

from __future__ import annotations

import torch

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.utils.configclass import configclass

from ..robots.robonion_params import PASSIVE_COUPLING, servo_of


class CoupledJointPositionAction(JointPositionAction):
    """Targets for the independent joints, as JointPositionAction. Each follower joint with a servo (knee, front
    shin) gets gain x the target of the joint it follows, so its servos push along the same coordinate."""

    cfg: CoupledJointPositionActionCfg

    def __init__(self, cfg: CoupledJointPositionActionCfg, env):
        super().__init__(cfg, env)
        followers, columns, gains = [], [], []
        for side in ("left", "right"):
            for follower, (lead, g) in PASSIVE_COUPLING.items():
                name = f"{side}_{follower}"
                if servo_of(name) is not None:
                    followers.append(name)
                    columns.append(self._joint_names.index(f"{side}_{lead}"))
                    gains.append(g)
        ids, _ = self._asset.find_joints(followers, preserve_order=True, as_proxy=True)
        self._follower_ids = ids.torch
        self._lead_columns = torch.tensor(columns, dtype=torch.long, device=self.device)
        self._gains = torch.tensor(gains, device=self.device)

    def apply_actions(self):
        super().apply_actions()
        target = self.processed_actions[:, self._lead_columns] * self._gains
        self._asset.set_joint_position_target_index(target=target, joint_ids=self._follower_ids)


@configclass
class CoupledJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = CoupledJointPositionAction
