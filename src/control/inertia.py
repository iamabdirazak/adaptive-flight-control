"""Estimate how hard each axis is to turn, motor by motor, from RPM and the gyro.

Each motor's thrust follows from its measured speed, and its share of each
axis's torque from where it sits. The gyro shows the angular acceleration
that results. For every axis a least-squares fit with forgetting finds how
strongly each of the four motors turns it, plus a steady offset. A weak
motor only lowers its own value, so the axis's inertia is read from the
median of the four, which the healthy majority decides.
"""
import torch


class Inertia:
    def __init__(self, count, nominal, allocation, max_thrust, memory=2.0, prior=1e-3):
        self.memory, self.allocation, self.max_thrust = memory, allocation, max_thrust
        self.nominal = nominal.clone()
        self.estimate = nominal.clone()
        self.prior = prior  # pulls the fit towards the datasheet when the drone is barely moving
        self.gram = torch.zeros(count, 3, 5, 5)
        self.cross = torch.zeros(count, 3, 5)
        self.effect = (1 / nominal)[..., None].repeat(1, 1, 4)  # angular acceleration per newton-metre, per motor
        self.rate = None

    def update(self, speed, rate, dt):
        if self.rate is not None:
            accel = (rate - self.rate) / dt + torch.cross(rate, self.estimate * rate, dim=-1) / self.estimate
            share = (self.max_thrust * speed.pow(2))[:, None, :] * self.allocation[1:][None]  # (n, axis, motor) torque
            x = torch.cat((share, torch.ones_like(share[..., :1])), -1)  # four motors and an offset
            fade = 1 - dt / self.memory
            self.gram = fade * self.gram + x[..., :, None] * x[..., None, :]
            self.cross = fade * self.cross + x * accel[..., None]
            target = torch.cat((self.effect, torch.zeros_like(self.effect[..., :1])), -1)
            ridge = self.prior * torch.eye(5)
            fit = torch.linalg.solve(self.gram + ridge, self.cross + (ridge @ target[..., None]).squeeze(-1))
            sound = torch.isfinite(fit).all(-1, keepdim=True)
            self.effect = torch.where(sound, fit[..., :4], self.effect)  # keep the last good fit if this one is not
            lost = ~sound[..., 0]
            self.gram[lost], self.cross[lost] = 0.0, 0.0  # and start that axis's fit afresh
            middle = self.effect.sort(-1).values[..., 1:3].mean(-1)  # median of four: the healthy majority
            self.estimate = (1 / middle.clamp(min=50.0)).clamp(3e-4, 1e-2)
        self.rate = rate.clone()
        return self.estimate
