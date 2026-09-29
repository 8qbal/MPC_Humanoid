"""Stepping in place without the startup phases: the robot spawns already crouched (no crouch ramp) and
the AHRS bias is not calibrated (the estimator and the contact-mode monitor get the raw AHRS roll/pitch). The gait
starts on the second tick; the initial pose, contact references and gait plan come from the first tick.

Everything else, including the arguments and the report, is scripts/run_stepping.py. Compare against it with the
same --seed, which sets the AHRS bias realization.

Usage:
    uv run python scripts/run_stepping_nocalib.py --viz kit
    uv run python scripts/run_stepping_nocalib.py --steps 20 --seed 1
"""

import os
import runpy

import numpy as np

g = runpy.run_path(os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_stepping.py"), run_name="stepping")
main = g["main"]

from isaaclab.utils import configclass  # noqa: E402

from MPC_Humanoid.env import MpcHumanoidRobonionEnvCfg  # noqa: E402
from MPC_Humanoid.mpc import controller  # noqa: E402

CROUCH = controller.CROUCH
# The crouch shortens the legs by 8.0 mm (sole below pelvis 0.5538 -> 0.5458 m, controller URDF FK), so the
# spawn height drops with it and the soles start 6.2 mm above the ground as at the default 0.56 m.
SPAWN_Z = 0.552
CROUCH_SIGNS = {
    ".*_front_thigh_pitch_joint": -1,
    ".*_back_thigh_pitch_joint": -1,
    ".*_knee_pitch_joint": 1,
    ".*_front_shin_pitch_joint": 1,
    ".*_back_shin_pitch_joint": 1,
    ".*_ankle_pitch_joint": -1,
}


@configclass
class CrouchedEnvCfg(MpcHumanoidRobonionEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        init = self.scene.robot.init_state
        joint_pos = {**init.joint_pos, **{name: sign * CROUCH for name, sign in CROUCH_SIGNS.items()}}
        self.scene.robot.init_state = init.replace(pos=(*init.pos[:2], SPAWN_Z), joint_pos=joint_pos)


class UncalibratedController(controller.RobonionController):
    def _start(self):
        super()._start()
        print(
            "AHRS bias left in (roll, pitch): "
            f"{np.round(np.degrees(self._bias), 3)} deg, first-tick estimate; run_stepping.py removes it"
        )
        self._bias = np.zeros(2)


# The default pose is now the crouch: no second crouch on top of it, and the model keeps the same nominal pose.
controller.CROUCH = 0.0
controller.CALIBRATION_START = 0.0
controller.GAIT_START = 1.5 * 0.005  # one 5 ms tick collects the initial pose, the gait starts on the next
main.__globals__["MpcHumanoidRobonionEnvCfg"] = CrouchedEnvCfg
main.__globals__["RobonionController"] = UncalibratedController

if __name__ == "__main__":
    main()
    g["simulation_app"].close()
