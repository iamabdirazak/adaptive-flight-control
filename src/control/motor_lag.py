"""Estimate how slowly each drone's motors respond, from their RPM telemetry.

A motor follows its command with a first-order lag. Every tick, how fast
the measured speed moves towards the command reveals the lag. A
least-squares fit with forgetting keeps the estimate current.
"""
import torch


class MotorLag:
    def __init__(self, count, initial=0.025, memory=2.0):
        self.memory = memory  # s, how long old measurements count
        self.estimate = torch.full((count,), initial)
        self.num = torch.zeros(count)  # running sums for the least-squares fit of 1/lag
        self.den = torch.zeros(count)
        self.speed = None

    def update(self, command, speed, dt):
        if self.speed is not None:
            rate = (speed - self.speed) / dt  # how fast each motor's speed is changing
            gap = command - self.speed  # how far it still had to go
            fade = 1 - dt / self.memory
            self.num = fade * self.num + (rate * gap).sum(-1)
            self.den = fade * self.den + gap.pow(2).sum(-1)
            informed = self.den > 1e-4
            self.estimate = torch.where(informed, (self.den / self.num.clamp(min=1e-6)).clamp(0.005, 0.2), self.estimate)
        self.speed = speed.clone()
        return self.estimate
