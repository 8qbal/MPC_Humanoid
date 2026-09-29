"""Build the Robonion controller without Isaac: the compiled model, AHRS and estimator functions, the contact-algebra
library (alg.so) and the acados NMPC solver, all into outputs/acados/ under names hashed from what they compute.
scripts/run_stepping.py and scripts/benchmark_stepping.py only load them.

The actuated joints are the revolute joints of assets/robonionv2_controller.urdf without the passive parallelogram
joints, at robots/robonion_params.py DEFAULT_JOINT_POS. Run it again after changing the model, the NMPC or the
default pose; builds of older versions stay in outputs/acados/ until deleted.

acados needs ACADOS_SOURCE_DIR and LD_LIBRARY_PATH:
    export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$HOME/acados/lib:$LD_LIBRARY_PATH

Usage:
    uv run python scripts/build_controller.py
"""

import re

import numpy as np
import pinocchio as pin

from MPC_Humanoid.mpc.controller import RobonionController
from MPC_Humanoid.mpc.model import CONTROLLER_URDF, PASSIVE_COUPLING
from MPC_Humanoid.robots.robonion_params import DEFAULT_JOINT_POS


def default_angle(joint: str) -> float:
    for pattern, angle in DEFAULT_JOINT_POS.items():
        if re.fullmatch(pattern, joint):
            return angle
    raise KeyError(f"{joint} has no entry in DEFAULT_JOINT_POS")


def main() -> None:
    m = pin.buildModelFromUrdf(CONTROLLER_URDF)
    act_names = [
        m.names[j]
        for j in range(1, m.njoints)
        if m.joints[j].shortname().startswith("JointModelR") and not m.names[j].endswith(tuple(PASSIVE_COUPLING))
    ]
    q_default_act = np.array([default_angle(n) for n in act_names])
    RobonionController(act_names, q_default_act, dt=0.005, build=True)
    print(f"built the controller for {len(act_names)} actuated joints")


if __name__ == "__main__":
    main()
