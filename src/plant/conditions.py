"""What changes between drones and during flight: the body, the air, the motors.

Each drone gets its own draw. Ranges come from docs/requirements.md, with a
training range and a wider test range so the envelope can be found.
"""
from dataclasses import dataclass
import math
import torch

from .quadrotor import Quadrotor

TRAINING = dict(mass=0.30, inertia=0.40, motor=0.20, wind=8.0, time_constant=(0.010, 0.040))
TEST = dict(mass=0.45, inertia=0.60, motor=0.30, wind=12.0, time_constant=(0.010, 0.060))


def _spread(n, half_width, generator):
    return 1.0 + (2 * torch.rand(n, generator=generator) - 1) * half_width


@dataclass
class Airframe:
    """Draw a body for each drone: mass, inertia, motor strength, motor lag."""
    ranges: dict

    def apply(self, drone: Quadrotor, generator):
        n, r, v = drone.n, self.ranges, drone.vehicle
        drone.mass = v.mass * _spread(n, r["mass"], generator)
        drone.inertia = torch.tensor(v.inertia) * _spread(n * 3, r["inertia"], generator).reshape(n, 3)
        drone.max_thrust = v.max_thrust * _spread(n, r["motor"], generator)[:, None].repeat(1, 4)
        lo, hi = r["time_constant"]
        drone.time_constant = lo + (hi - lo) * torch.rand(n, generator=generator)


class Wind:
    """A steady wind of random speed and direction, with gusts on top.

    Gusts follow a first-order random process: they change over about a
    second and their strength grows with the mean wind.
    """

    def __init__(self, n, max_speed, generator, correlation=1.0):
        speed = max_speed * torch.rand(n, generator=generator)
        heading = 2 * math.pi * torch.rand(n, generator=generator)
        self.mean = torch.stack((speed * torch.cos(heading), speed * torch.sin(heading), torch.zeros(n)), -1)
        self.sigma = (0.15 * speed)[:, None]  # gust strength, 15% of the mean
        self.tau = correlation
        self.gust = torch.zeros(n, 3)
        self.generator = generator

    def step(self, dt):
        noise = torch.randn(self.gust.shape, generator=self.generator)
        self.gust = self.gust - self.gust * dt / self.tau + self.sigma * math.sqrt(2 * dt / self.tau) * noise
        self.gust[:, 2] *= 0.3  # vertical gusts are weaker
        return self.mean + self.gust
