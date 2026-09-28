# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Python package for the generated extension."""

try:
    from .tasks import *
except ImportError:  # no Isaac Lab (e.g. the controller alone on the Jetson): the RL tasks are not registered
    pass

# Kit loads ``ui_extension_example`` through ``extension.toml`` after startup.
