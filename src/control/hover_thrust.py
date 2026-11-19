"""Hover-thrust estimation, as PX4 does it, so the baseline learns its own weight.

PX4 runs a small filter that keeps estimating the thrust needed to hover.
Here the same idea estimates the drone's mass from the thrust it commands
and the vertical acceleration that follows. It only updates while the
drone is near level and not manoeuvring hard, where the estimate is reliable.
"""
import torch


class HoverThrust:
    def __init__(self, mass, gravity, time_constant=0.6, filter_time=0.05):
        self.mass = mass.clone()  # starts at what the datasheet says
        self.gravity = gravity
        self.tau = time_constant  # how fast the estimate follows, s
        self.filter_time = filter_time  # smoothing of the measured acceleration, s
        self.velocity = None
        self.accel = torch.zeros_like(mass)

    def update(self, velocity_z, thrust, tilt_cos, dt):
        if self.velocity is not None:
            raw = (velocity_z - self.velocity) / dt
            self.accel = self.accel + (raw - self.accel) * dt / self.filter_time
        self.velocity = velocity_z.clone()
        lift = self.gravity - self.accel  # NED: upward acceleration needed against gravity
        valid = (tilt_cos > 0.9) & (lift > 4.0) & (lift < 16.0) & (thrust > 0)
        measured = thrust * tilt_cos / lift.clamp(min=1e-3)
        step = torch.where(valid, (measured - self.mass) * dt / self.tau, 0.0)
        self.mass = self.mass + step
        return self.mass
