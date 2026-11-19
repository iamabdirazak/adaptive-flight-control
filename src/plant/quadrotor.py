"""The quadrotor as physics: many drones stepped together, one row each.

Frames follow PX4: north-east-down for the world, forward-right-down for
the body. Commands are normalised motor speeds between 0 and 1.
"""
import torch

from . import rotations
from .allocation import allocation_matrix
from .vehicle import Vehicle


class Quadrotor:
    def __init__(self, vehicle: Vehicle, count: int = 1, dtype=torch.float32):
        self.vehicle = vehicle
        self.n = count
        kw = dict(dtype=dtype)
        full = lambda v, *shape: torch.full((count, *shape), float(v), **kw)
        # Physical properties, one row per drone, so each can differ.
        self.mass = full(vehicle.mass)
        self.inertia = torch.tensor(vehicle.inertia, **kw).repeat(count, 1)
        self.max_thrust = full(vehicle.max_thrust, 4)  # health scales this per motor
        self.time_constant = full(vehicle.motor_time_constant)
        self.drag = full(vehicle.drag)
        self.wind = torch.zeros(count, 3, **kw)
        self.allocation = allocation_matrix(vehicle, dtype)
        self.reset()

    def reset(self, position=None):
        kw = dict(dtype=self.mass.dtype)
        self.position = torch.zeros(self.n, 3, **kw) if position is None else position.clone()
        self.velocity = torch.zeros(self.n, 3, **kw)
        self.attitude = rotations.identity(self.n, **kw)
        self.rate = torch.zeros(self.n, 3, **kw)
        hover = (self.mass * self.vehicle.gravity / 4 / self.max_thrust.mean(-1)).sqrt()
        self.motor = hover[:, None].repeat(1, 4)  # normalised speed, starts at hover
        self.landed = torch.zeros(self.n, dtype=torch.bool)
        self.touchdown_speed = torch.zeros(self.n, **kw)  # vertical speed at first contact
        self.touchdown_tilt = torch.zeros(self.n, **kw)  # body tilt at first contact, rad

    def thrusts(self):
        return self.max_thrust * self.motor**2

    def step(self, command, dt):
        v = self.vehicle
        command = command.clamp(0.0, 1.0)
        self.motor = self.motor + (command - self.motor) * (dt / self.time_constant)[:, None]
        wrench = self.thrusts() @ self.allocation.T  # (n, 4): thrust, roll, pitch, yaw
        R = rotations.to_matrix(self.attitude)
        thrust_body = torch.zeros_like(self.velocity)
        thrust_body[:, 2] = -wrench[:, 0]
        air = self.velocity - self.wind
        accel = (R @ thrust_body[..., None]).squeeze(-1) - self.drag[:, None] * air
        accel = accel / self.mass[:, None]
        accel[:, 2] += v.gravity
        torque = wrench[:, 1:]
        spin = torch.cross(self.rate, self.inertia * self.rate, dim=-1)
        rate = self.rate + (torque - spin) / self.inertia * dt
        velocity = self.velocity + accel * dt
        position = self.position + velocity * dt
        attitude = rotations.integrate(self.attitude, rate, dt)
        # The ground is z = 0. The first contact is recorded and the drone stays there.
        contact = (position[:, 2] >= 0) & ~self.landed
        self.touchdown_speed = torch.where(contact, velocity[:, 2], self.touchdown_speed)
        self.touchdown_tilt = torch.where(contact, torch.acos(R[:, 2, 2].clamp(-1, 1)), self.touchdown_tilt)
        self.landed = self.landed | contact
        keep = self.landed[:, None]
        self.rate = torch.where(keep, torch.zeros_like(rate), rate)
        self.velocity = torch.where(keep, torch.zeros_like(velocity), velocity)
        position = torch.cat((position[:, :2], position[:, 2:].clamp(max=0.0)), -1)
        self.position = torch.where(keep & ~contact[:, None], self.position, position)
        self.attitude = torch.where(keep, self.attitude, attitude)
        return accel
