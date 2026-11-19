"""Find a failing motor from torque that goes missing.

From the measured motor speeds the controller knows the roll and pitch
torque the motors should produce. From the gyro it sees the rotation they
actually produce. A steady gap between the two is torque that is missing.
Each motor sits on its own diagonal, so the direction of the gap names the
motor and its size says how much thrust that motor has lost.
"""
import torch


class MotorFault:
    def __init__(self, count, inertia, allocation, max_thrust, smoothing=0.15):
        self.inertia, self.max_thrust = inertia, max_thrust
        self.arms = allocation[1:3].T  # (motor, 2): roll and pitch torque per newton of each motor
        self.smoothing = smoothing  # s, averages out manoeuvres so only a steady gap remains
        self.missing = torch.zeros(count, 2)
        self.health = torch.ones(count, 4)
        self.rate = None

    def update(self, speed, rate, dt):
        if self.rate is not None:
            accel = (rate - self.rate) / dt
            actual = (self.inertia * accel + torch.cross(rate, self.inertia * rate, dim=-1))[:, :2]
            thrust = self.max_thrust * speed.pow(2)
            expected = thrust @ self.arms
            self.missing += (expected - actual - self.missing) * dt / self.smoothing
            # Thrust each motor would have to have lost to leave this gap on its own
            lost = (self.missing @ self.arms.T) / (self.arms.pow(2).sum(-1) * thrust.clamp(min=0.05))
            self.health = (1 - lost).clamp(0.0, 1.0)
        self.rate = rate.clone()
        return self.health
