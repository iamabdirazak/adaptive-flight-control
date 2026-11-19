"""The training world: many drones, each with its own body, air and damage.

Every episode draws a body within the training ranges, a steady wind with
gusts, a point to hold, and a change partway through: added payload, a
motor losing some or all of its thrust, or nothing. The teacher sees the
truth about all of it; the student will not.
"""
import torch

from ..paths import CONTROL_PERIOD, SIMULATION_STEP
from ..plant import rotations
from ..plant.conditions import TRAINING, Wind
from ..plant.quadrotor import Quadrotor
from ..plant.vehicle import Vehicle

SUBSTEPS = round(CONTROL_PERIOD / SIMULATION_STEP)
EPISODE = 2000  # ticks, 4 s


def observe(drone, reference, velocity_reference, last_action, wind, privileged=True):
    """What the controller sees. Privileged terms are the truth only simulation knows."""
    v = drone.vehicle
    parts = [
        (reference - drone.position).clamp(-2, 2),
        (velocity_reference - drone.velocity).clamp(-4, 4) / 2,
        rotations.to_matrix(drone.attitude).flatten(1),
        drone.rate / 5,
        last_action * 2 - 1,
    ]
    if privileged:
        parts += [
            drone.mass[:, None] / v.mass - 1,
            drone.inertia / torch.tensor(v.inertia) - 1,
            drone.max_thrust / v.max_thrust - 1,
            (drone.time_constant[:, None] - 0.025) / 0.015,
            wind / 8,
        ]
    return torch.cat(parts, -1)


class Environment:
    def __init__(self, count, seed=0, vehicle=Vehicle()):
        self.n, self.vehicle = count, vehicle
        self.generator = torch.Generator().manual_seed(seed)
        self.drone = Quadrotor(vehicle, count)
        self.reset(torch.arange(count))

    def _uniform(self, k, lo, hi):
        return lo + (hi - lo) * torch.rand(k, generator=self.generator)

    def reset(self, idx):
        d, v, k, r = self.drone, self.vehicle, len(idx), TRAINING
        if k == 0:
            return
        d.mass[idx] = v.mass * self._uniform(k, 1 - r["mass"], 1 + r["mass"])
        d.inertia[idx] = torch.tensor(v.inertia) * self._uniform(k * 3, 1 - r["inertia"], 1 + r["inertia"]).reshape(k, 3)
        d.max_thrust[idx] = v.max_thrust * self._uniform(k, 1 - r["motor"], 1 + r["motor"])[:, None].repeat(1, 4)
        d.time_constant[idx] = self._uniform(k, *r["time_constant"])
        start = torch.zeros(k, 3)
        start[:, 2] = -self._uniform(k, 3.0, 8.0)
        d.position[idx], d.velocity[idx], d.rate[idx] = start, 0.0, 0.0
        d.attitude[idx] = rotations.identity(k)
        d.motor[idx] = (d.mass[idx] * v.gravity / 4 / d.max_thrust[idx].mean(-1)).sqrt()[:, None].repeat(1, 4)
        d.landed[idx] = False
        if not hasattr(self, "time"):
            self.time, self.event = torch.zeros(self.n, dtype=torch.long), torch.zeros(self.n, dtype=torch.long)
            self.kind, self.last = torch.zeros(self.n, dtype=torch.long), torch.zeros(self.n, 4)
            self.wind = Wind(self.n, r["wind"], self.generator)
            self.motor_hit, self.severity = torch.zeros(self.n, dtype=torch.long), torch.zeros(self.n)
            self.centre, self.moving, self.rate_path = torch.zeros(self.n, 3), torch.zeros(self.n), torch.zeros(self.n)
        self.centre[idx] = start + torch.cat((self._uniform(k * 2, -1, 1).reshape(k, 2), self._uniform(k, -0.5, 0.5)[:, None]), -1)
        self.moving[idx] = (torch.rand(k, generator=self.generator) < 0.5).float()  # half hold a point, half fly a figure-eight
        self.rate_path[idx] = self._uniform(k, 0.3, 0.75)
        self.time[idx] = 0
        self.event[idx] = self._uniform(k, 250, 1500).long()
        self.kind[idx] = torch.multinomial(torch.tensor([0.25, 0.25, 0.5]), k, replacement=True, generator=self.generator)
        self.motor_hit[idx] = torch.randint(0, 4, (k,), generator=self.generator)
        self.severity[idx] = self._uniform(k, 0.0, 1.0)  # health left in the damaged motor
        speed = r["wind"] * torch.rand(k, generator=self.generator)
        heading = 6.2832 * torch.rand(k, generator=self.generator)
        self.wind.mean[idx] = torch.stack((speed * heading.cos(), speed * heading.sin(), torch.zeros(k)), -1)
        self.wind.sigma[idx] = 0.15 * speed[:, None]
        self.wind.gust[idx] = 0.0
        self.last[idx] = d.motor[idx]

    def reference(self):
        t, w, m = self.time * CONTROL_PERIOD, self.rate_path, self.moving
        s, c, s2, c2 = (w * t).sin(), (w * t).cos(), (2 * w * t).sin(), (2 * w * t).cos()
        zero = torch.zeros_like(t, dtype=torch.float32)
        p = self.centre + m[:, None] * torch.stack((2 * s, s2, zero), -1)
        v = m[:, None] * torch.stack((2 * w * c, 2 * w * c2, zero), -1)
        a = m[:, None] * torch.stack((-2 * w * w * s, -4 * w * w * s2, zero), -1)
        return p, v, a

    def observation(self, privileged=True):
        p, v, _ = self.reference()
        return observe(self.drone, p, v, self.last, self.wind.mean + self.wind.gust, privileged)

    def step(self, action):
        d = self.drone
        hit = self.time == self.event
        payload = hit & (self.kind == 1)
        d.mass = torch.where(payload, d.mass * 1.3, d.mass)
        damage = hit & (self.kind == 2)
        rows = damage.nonzero().squeeze(-1)
        d.max_thrust[rows, self.motor_hit[rows]] *= self.severity[rows]
        for _ in range(SUBSTEPS):
            d.wind = self.wind.step(SIMULATION_STEP)
            d.step(action, SIMULATION_STEP)
        self.time += 1
        error = (self.reference()[0] - d.position).norm(dim=-1)
        tilt = rotations.to_matrix(d.attitude)[:, 2, 2]
        reward = (-error - 0.05 * d.velocity.norm(dim=-1) - 0.02 * d.rate[:, :2].norm(dim=-1)
                  - 0.1 * (action - self.last).abs().sum(-1) + 0.5)
        crashed = d.landed | (error > 3.0) | (tilt < -0.2)
        reward = torch.where(crashed, torch.full_like(reward, -20.0), reward)
        done = crashed | (self.time >= EPISODE)
        self.last = action.clone()
        info = dict(crashed=crashed, timeout=(self.time >= EPISODE) & ~crashed)
        self.reset(done.nonzero().squeeze(-1))
        return reward, done, info
