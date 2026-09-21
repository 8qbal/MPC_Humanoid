# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Placeholder for the closed-loop MPC controller described in ``plan.md`` (Phase 4).

This module currently only defines the controller's public shape -- modes and a
per-tick entry point -- so that the task/env side can be wired against a stable
interface before the centroidal MPC (``centroidal.py``), whole-body QP
(``whole_body_qp.py``), and supporting state/contact/reference modules exist.
"""

