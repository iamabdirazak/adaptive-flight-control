"""The baseline: cascaded PID, as PX4 and Betaflight fly a quadrotor.

Position sets a velocity, velocity sets an acceleration, the acceleration
sets a thrust direction and so an attitude, the attitude sets a body rate,
and the rate loop sets the torques the mixer turns into motor speeds.
Gains are tuned once on the nominal drone and then left alone, which is
how the baseline is compared in every scenario.
"""
from dataclasses import dataclass
import torch

from ..plant import rotations
from ..plant.allocation import allocation_matrix
from ..plant.vehicle import Vehicle
from .hover_thrust import HoverThrust


@dataclass
class Gains:
    """Tuned on the nominal drone by python -m src.evaluation.tune."""
    position: float = 2.17  # 1/s
    velocity: float = 6.96  # 1/s
    velocity_integral: float = 0.51  # 1/s²
    attitude: float = 11.39  # 1/s
    rate: float = 24.29  # 1/s
    rate_integral: float = 51.79  # 1/s², trims out steady torque errors, as PX4 does
    max_speed: float = 3.0  # m/s
    max_tilt: float = 0.6  # rad


class Cascade:
    def __init__(self, vehicle: Vehicle, count: int, gains: Gains = Gains(), estimate_mass=True, dtype=torch.float32):
        self.v, self.g = vehicle, gains
        kw = dict(dtype=dtype)
        self.mass = torch.full((count,), vehicle.mass, **kw)  # what the controller believes
        self.inertia = torch.tensor(vehicle.inertia, **kw).repeat(count, 1)
        self.max_thrust = vehicle.max_thrust
        self.mixer = torch.linalg.inv(allocation_matrix(vehicle, dtype))
        self.integral = torch.zeros(count, 3, **kw)
        self.rate_integral = torch.zeros(count, 3, **kw)
        self.hover = HoverThrust(self.mass, vehicle.gravity) if estimate_mass else None
        self.last_thrust = self.mass * vehicle.gravity

    def __call__(self, state, target, heading, dt):
        """target is a position, or (position, velocity, acceleration) to fly a path."""
        g = self.g
        p, v, q, w = state
        p_ref, v_ff, a_ff = target if isinstance(target, tuple) else (target, 0.0, 0.0)
        R = rotations.to_matrix(q)
        if self.hover is not None:
            self.mass = self.hover.update(v[:, 2], self.last_thrust, R[:, 2, 2], dt)
        # Position → velocity → acceleration (NED, gravity along +z), with feedforward
        v_sp = v_ff + (g.position * (p_ref - p)).clamp(-g.max_speed, g.max_speed)
        err = v_sp - v
        self.integral = (self.integral + err * dt).clamp(-2.0, 2.0)
        a_sp = a_ff + g.velocity * err + g.velocity_integral * self.integral
        force = self.mass[:, None] * (a_sp - torch.tensor([0.0, 0.0, self.v.gravity]))
        # Limit tilt by keeping enough of the force vertical
        horizontal = force[:, :2].norm(dim=-1, keepdim=True)
        limit = (-force[:, 2:]).clamp(min=1e-3) * torch.tan(torch.tensor(g.max_tilt))
        force = torch.cat((force[:, :2] * (limit / horizontal.clamp(min=1e-6)).clamp(max=1.0), force[:, 2:]), -1)
        # Force direction and heading → desired attitude
        z_d = -force / force.norm(dim=-1, keepdim=True)
        x_c = torch.stack((torch.cos(heading), torch.sin(heading), torch.zeros_like(heading)), -1)
        y_d = torch.cross(z_d, x_c, dim=-1)
        y_d = y_d / y_d.norm(dim=-1, keepdim=True)
        x_d = torch.cross(y_d, z_d, dim=-1)
        R_d = torch.stack((x_d, y_d, z_d), -1)
        thrust = -(force * R[..., 2]).sum(-1)  # collective thrust along current body z
        # Attitude error on SO(3) → body rate setpoint → torques
        E = 0.5 * (R_d.transpose(-1, -2) @ R - R.transpose(-1, -2) @ R_d)
        e_R = torch.stack((E[:, 2, 1], E[:, 0, 2], E[:, 1, 0]), -1)
        w_sp = -g.attitude * e_R
        rate_error = w_sp - w
        self.rate_integral = (self.rate_integral + rate_error * dt).clamp(-1.5, 1.5)  # about 30% of torque authority, as PX4 allows
        torque = self.inertia * (g.rate * rate_error + g.rate_integral * self.rate_integral) \
            + torch.cross(w, self.inertia * w, dim=-1)
        # Mixer: thrust and torques → per-motor thrust → normalised speed
        motor = self.mix(thrust, torque)
        self.last_thrust = motor.sum(-1)
        return (motor / self.max_thrust).sqrt()

    def mix(self, thrust, torque):
        """Motor thrusts, keeping roll and pitch whole and giving up yaw first, as PX4 does."""
        zero_yaw = torch.cat((thrust[:, None], torque[:, :2], torch.zeros_like(thrust)[:, None]), -1)
        base = (zero_yaw @ self.mixer.T).clamp(0.0, self.max_thrust)
        yaw = torque[:, 2:] * self.mixer[:, 3]  # each motor's share of the yaw torque
        room = torch.where(yaw > 0, (self.max_thrust - base) / yaw.clamp(min=1e-9),
                           torch.where(yaw < 0, -base / yaw.clamp(max=-1e-9), torch.full_like(base, 1e9)))
        scale = room.min(-1, keepdim=True).values.clamp(0.0, 1.0)
        return (base + scale * yaw).clamp(0.0, self.max_thrust)

