"""BAM (Better Actuator Models) voltage-controlled servo for Isaac Lab on PhysX.

Port of ``bam.mjlab.BamActuator``. Per physics step: fractional command delay, firmware P loop to a PWM duty,
``tau = kt V / R - kt^2 dq / R``, then the m6 friction budget written to the PhysX joint as static = dynamic friction
effort [N.m]. PhysX does the stiction clipping, as MuJoCo's ``dof_frictionloss`` does for BAM. Viscous friction is a
constant joint viscous coefficient. ``q_offset`` of the fit (testbench mounting error) is not modelled.

Needs ``SimulationCfg.use_newton_actuators = False`` and :func:`bind_bam_motors` after ``sim.reset()``
(:func:`bind_bam_motors_event` as a startup event).
"""

from __future__ import annotations

import json
import math
import warnings
from collections.abc import Sequence
from dataclasses import MISSING

import torch
import warp as wp

from isaaclab.actuators import ActuatorBase, ActuatorBaseCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.configclass import configclass
from isaaclab.utils.types import ArticulationActions

_PARAM_KEYS = (
    "kt", "R", "armature", "command_delay", "friction_base", "friction_stribeck", "load_friction_motor",
    "load_friction_external", "load_friction_motor_stribeck", "load_friction_external_stribeck",
    "load_friction_motor_quad", "load_friction_external_quad", "dtheta_stribeck", "alpha", "friction_viscous",
)  # fmt: skip


def load_bam_params(path: str) -> dict:
    with open(path) as f:
        data = json.load(f)
    if data.get("model") != "m6":
        raise ValueError(f"{path}: only BAM model m6 is supported, got {data.get('model')!r}")
    missing = [k for k in _PARAM_KEYS if k not in data]
    if missing:
        raise ValueError(f"{path}: missing BAM parameters {missing}")
    return data


def _to_torch(x) -> torch.Tensor:
    return wp.to_torch(x) if isinstance(x, wp.array) else torch.as_tensor(x)


class BamMotor(ActuatorBase):
    cfg: BamMotorCfg

    def __init__(
        self,
        cfg: BamMotorCfg,
        joint_names: list[str],
        joint_ids: slice | torch.Tensor,
        num_envs: int,
        device: str,
        stiffness: torch.Tensor | float = 0.0,
        damping: torch.Tensor | float = 0.0,
        actuator_effort_limit: torch.Tensor | float | None = None,
        actuator_velocity_limit: torch.Tensor | float | None = None,
        effort_limit: torch.Tensor | float | None = None,
        velocity_limit: torch.Tensor | float | None = None,
    ):
        super().__init__(
            cfg, joint_names, joint_ids, num_envs, device,
            actuator_effort_limit, actuator_velocity_limit, effort_limit, velocity_limit,
        )  # fmt: skip
        p = load_bam_params(cfg.params_json) if cfg.params_json is not None else dict(cfg.params or {})
        self._p = {k: float(p[k]) for k in _PARAM_KEYS}
        self._n = int(cfg.parallel_count)
        self._kp_gain = float(cfg.kp_fw) * cfg.error_gain  # duty per rad
        self._robot = None
        self._dt: float | None = None
        self._ring: torch.Tensor | None = None
        self._delay_steps = 0.0
        self._fresh = torch.ones(num_envs, dtype=torch.bool, device=device)
        self._visc = torch.full((num_envs, self.num_joints), self._n * self._p["friction_viscous"], device=device)
        self.motor_torque = torch.zeros(num_envs, self.num_joints, device=device)  # per servo
        self.friction_budget = torch.zeros_like(self.motor_torque)  # on the joint, all parallel servos
        self.external_torque = torch.zeros_like(self.motor_torque)  # on the joint, all parallel servos

    def bind(self, robot, dt: float) -> None:
        self._robot = robot
        self._dt = float(dt)
        self._delay_steps = self._p["command_delay"] / self._dt if self.cfg.use_command_delay else 0.0
        self._ring = torch.zeros(
            self._num_envs, self.num_joints, int(math.floor(self._delay_steps)) + 2, device=self._device
        )
        self._fresh[:] = True

    def reset(self, env_ids: Sequence[int] | slice | None = None) -> None:
        self._fresh[slice(None) if env_ids is None else env_ids] = True

    def _delayed_target(self, target: torch.Tensor) -> torch.Tensor:
        if self._delay_steps <= 0.0:
            return target
        if bool(self._fresh.any()):
            self._ring[self._fresh] = target[self._fresh].unsqueeze(-1)  # BAM holds the first sample
            self._fresh[:] = False
        self._ring = torch.roll(self._ring, 1, dims=-1)
        self._ring[..., 0] = target
        d0 = int(math.floor(self._delay_steps))
        frac = self._delay_steps - d0
        return (1.0 - frac) * self._ring[..., d0] + frac * self._ring[..., d0 + 1]

    def _group_cols(self, buf: torch.Tensor) -> torch.Tensor:
        """Backend-ordered per-joint buffer -> this group's columns in user order."""
        data = self._robot.data
        if data.has_joint_ordering:
            buf = buf[:, list(data.joint_ordering.user_to_backend_indices)]
        return buf if isinstance(self._joint_indices, slice) else buf[:, self._joint_indices.long()]

    def _external_torque(self) -> torch.Tensor:
        mode = self.cfg.external_torque_mode
        if mode == "motor_only":
            return torch.zeros_like(self.motor_torque)
        view = self._robot.root_view
        if mode == "gravity":
            g = _to_torch(view.get_gravity_compensation_forces())
            c = _to_torch(view.get_coriolis_and_centrifugal_compensation_forces())
            if g.shape[1] != self._robot.num_joints:
                raise ValueError("external_torque_mode 'gravity' supports fixed-base articulations only")
            return -self._group_cols(g + c)
        if mode == "projected":
            return -self._group_cols(_to_torch(view.get_dof_projected_joint_forces()))
        raise ValueError(f"unknown external_torque_mode {mode!r}")

    def _friction_budget(self, tau_m: torch.Tensor, tau_e: torch.Tensor, dq: torch.Tensor) -> torch.Tensor:
        p = self._p
        stribeck = torch.exp(-torch.pow(torch.abs(dq) / p["dtheta_stribeck"], p["alpha"]))
        budget = p["friction_base"] + stribeck * p["friction_stribeck"]
        budget = budget + torch.abs(p["load_friction_external"] * tau_e - p["load_friction_motor"] * tau_m)
        budget = budget + stribeck * torch.abs(
            p["load_friction_external_stribeck"] * tau_e - p["load_friction_motor_stribeck"] * tau_m
        )
        drive = (torch.abs(tau_m) > torch.abs(tau_e)).to(tau_m.dtype)
        quad = (
            drive * p["load_friction_external_quad"] * tau_e**2
            + (1.0 - drive) * p["load_friction_motor_quad"] * tau_m**2
        )
        return budget + stribeck * quad

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        if self._robot is None:
            raise RuntimeError("BamMotor is not bound: call bind_bam_motors(robot, dt) after sim.reset().")
        p, n = self._p, self._n
        dq = joint_vel
        target = self._delayed_target(control_action.joint_positions)

        duty = (target - joint_pos) * self._kp_gain
        if self.cfg.max_current is not None:
            center = p["kt"] * dq / self.cfg.vin
            span = p["R"] * self.cfg.max_current / self.cfg.vin
            duty = torch.clamp(duty, center - span, center + span)
        duty = torch.clamp(duty, -self.cfg.max_pwm, self.cfg.max_pwm)
        tau_m = p["kt"] * (self.cfg.vin * duty) / p["R"] - p["kt"] ** 2 * dq / p["R"]

        tau_e_total = self._external_torque()
        budget = n * self.cfg.friction_scale * self._friction_budget(tau_m, tau_e_total / n, dq)
        self._robot.write_joint_friction_coefficient_to_sim_index(
            joint_friction_coeff=budget,
            joint_dynamic_friction_coeff=budget,
            joint_viscous_friction_coeff=self._visc,
            joint_ids=None if isinstance(self._joint_indices, slice) else self._joint_indices,
        )

        self.motor_torque, self.friction_budget, self.external_torque = tau_m, budget, tau_e_total
        self.computed_effort = n * tau_m
        self.applied_effort = self._clip_effort(self.computed_effort)
        control_action.joint_efforts = self.applied_effort
        control_action.joint_positions = None
        control_action.joint_velocities = None
        return control_action


@configclass
class BamMotorCfg(ActuatorBaseCfg):
    """One group of identical servos; ``parallel_count`` servos drive each joint."""

    class_type: type = BamMotor

    stiffness: dict[str, float] | float | None = None
    damping: dict[str, float] | float | None = None

    params_json: str | None = None
    params: dict | None = None
    kp_fw: float = MISSING
    """Firmware Position P Gain register."""
    error_gain: float = MISSING
    """Duty per rad per unit of P gain."""
    vin: float = MISSING
    """Supply voltage [V]."""
    max_pwm: float = 1.0
    max_current: float | None = None
    """Firmware current limit [A], modelled as a duty window; None = off."""
    parallel_count: int = 1
    external_torque_mode: str = "projected"
    """Load on the gearbox for the load-dependent friction: ``gravity`` (-(gravity + Coriolis), no contacts),
    ``projected`` (-(torque the joint transmits), quasi-static, includes contacts) or ``motor_only``."""
    friction_scale: float = 1.0
    use_command_delay: bool = True

    def __post_init__(self):
        if self.params_json is None and not self.params:
            raise ValueError("BamMotorCfg needs params_json or params")
        p = load_bam_params(self.params_json) if self.params_json is not None else self.params
        n = self.parallel_count
        # build-time seeds; the actuator overwrites the friction every step
        if self.armature is None:
            self.armature = n * float(p["armature"])
        if self.viscous_friction is None:
            self.viscous_friction = n * float(p["friction_viscous"])
        if self.friction is None:
            self.friction = n * float(p["friction_base"])
        if self.dynamic_friction is None:
            self.dynamic_friction = n * float(p["friction_base"])


def bind_bam_motors(robot, dt: float) -> int:
    count = 0
    for actuator in robot.actuators.values():
        if isinstance(actuator, BamMotor):
            actuator.bind(robot, dt)
            count += 1
    if count == 0:
        warnings.warn("bind_bam_motors: no BamMotor group on the articulation", stacklevel=2)
    return count


def bind_bam_motors_event(env, env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> None:
    bind_bam_motors(env.scene[asset_cfg.name], env.physics_dt)
