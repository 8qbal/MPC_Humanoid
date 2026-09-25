# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
DCM MPC: plans the ZMP over a receding horizon so the measured DCM returns to its
reference while the ZMP stays inside the support polygon. x and y are solved as two
independent QPs (OSQP). ZMP bounds are given per node, so a gait schedule can switch
the support polygon between double and single support.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import osqp
import scipy.sparse as sp


@dataclass
class DcmMpcSolution:
    p: np.ndarray
    # Planned ZMP (x, y) over the horizon in S, shape (2, horizon) [m].
    p0: np.ndarray
    # First planned ZMP, the only one applied [m].
    ok: bool
    # False if either QP failed; p then repeats the previous plan.


class DcmMpc:
    def __init__(
        self,
        omega: float,
        dt_mpc: float = 0.02,
        horizon: int = 25,
        w_xi: float = 1.0,
        w_zmp: float = 0.1,
        w_dzmp: float = 1.0,
        zmp_margin: float = 0.01,
    ):
        self.horizon = horizon
        self.w_xi, self.w_zmp, self.w_dzmp = w_xi, w_zmp, w_dzmp
        self.zmp_margin = zmp_margin

        n = horizon
        a = math.exp(omega * dt_mpc)
        k = np.arange(1, n + 1)
        self._phi = a**k
        # xi[k] = a^k xi0 + sum_{j<k} a^(k-1-j) (1 - a) p[j], rows k = 1..N
        self._gamma = np.tril(a ** np.subtract.outer(k - 1, np.arange(n)).clip(min=0) * (1.0 - a))
        self._diff = np.eye(n) - np.eye(n, k=-1)

        P = 2.0 * (w_xi * self._gamma.T @ self._gamma + w_zmp * np.eye(n) + w_dzmp * self._diff.T @ self._diff)
        A = np.vstack([np.eye(n), self._gamma[-1]])
        self._solvers = []
        for _ in range(2):
            s = osqp.OSQP()
            s.setup(
                P=sp.triu(sp.csc_matrix(P), format="csc"),
                q=np.zeros(n),
                A=sp.csc_matrix(A),
                l=-np.ones(n + 1),
                u=np.ones(n + 1),
                verbose=False,
                eps_abs=1e-6,
                eps_rel=1e-6,
                polish=False,
            )
            self._solvers.append(s)
        self.reset()

    def reset(self, p0: np.ndarray | None = None):
        """Restart the plan (and the ZMP-rate reference) at p0, default the origin of S."""
        p0 = np.zeros(2) if p0 is None else np.asarray(p0, dtype=float)
        self._plan = np.repeat(p0[:, None], self.horizon, axis=1)

    def solve(
        self,
        xi: np.ndarray,
        p_min: np.ndarray,
        p_max: np.ndarray,
        xi_ref: np.ndarray | None = None,
        p_ref: np.ndarray | None = None,
    ) -> DcmMpcSolution:
        """
        p_min, p_max: support-polygon bounds in S, shape (2,) for all nodes or (2, horizon)
        per node; zmp_margin is applied inside. The terminal DCM must lie within the
        bounds of the last node.
        """
        n = self.horizon
        xi_ref = np.zeros(2) if xi_ref is None else xi_ref
        p_ref = np.zeros(2) if p_ref is None else p_ref
        p_min = np.broadcast_to(np.asarray(p_min, dtype=float).reshape(2, -1), (2, n))
        p_max = np.broadcast_to(np.asarray(p_max, dtype=float).reshape(2, -1), (2, n))

        plan, ok = np.empty((2, n)), True
        for i, s in enumerate(self._solvers):
            free = self._phi * xi[i]
            d = np.zeros(n)
            d[0] = self._plan[i, 0]
            q = 2.0 * (
                self.w_xi * self._gamma.T @ (free - xi_ref[i])
                - self.w_zmp * p_ref[i] * np.ones(n)
                - self.w_dzmp * self._diff.T @ d
            )
            l = np.append(p_min[i] + self.zmp_margin, p_min[i, -1] - free[-1])
            u = np.append(p_max[i] - self.zmp_margin, p_max[i, -1] - free[-1])
            s.update(q=q, l=l, u=u)
            res = s.solve()
            if res.info.status_val == osqp.SolverStatus.OSQP_SOLVED:
                plan[i] = res.x
            else:
                plan[i], ok = self._plan[i], False
        self._plan = plan
        return DcmMpcSolution(p=plan.copy(), p0=plan[:, 0].copy(), ok=ok)
