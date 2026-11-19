"""The adaptive flight controller: the tuned cascade, adapting where fixed gains fail.

Weight is adapted through hover-thrust estimation, already part of the
cascade. Dynamics are adapted here. The effective inertia of each axis is
estimated from RPM and the gyro, so torque commands match the real body.
The motor lag is estimated from RPM, and once motors are slower than the
gains were tuned for, the attitude and rate loops are slowed to match,
keeping the margin the tuning had.

A failing motor is the third adaptation. Once a motor is confirmed to have
lost most of its thrust, it is switched off. The other three keep lift, roll
and pitch, yaw is given up so the drone spins, and it descends straight down
to land.
"""
from dataclasses import asdict

import torch

from .cascade import Cascade, Gains
from .inertia import Inertia
from .motor_lag import MotorLag
from .motor_fault import MotorFault

TUNED_LAG = 0.045  # s, the slowest motors the tuned gains keep a full margin for
FAILED = 0.6  # a motor below this share of its thrust is treated as failed
CONFIRM = 0.1  # s, how long it must stay there before the drone acts
DESCENT = 0.7  # m/s, landing speed after a failure
# Off until a spinning-flight controller exists: with a motor off, its diagonal partner must
# idle too, and only the spin itself can keep the drone upright. The cascade cannot do that.
FAILURE_MODE = False


class Adaptive(Cascade):
    def __init__(self, vehicle, count, gains: Gains = Gains()):
        super().__init__(vehicle, count, gains)
        self.base = gains
        self.lag = MotorLag(count, vehicle.motor_time_constant)
        self.body = Inertia(count, self.inertia, self.mixer.inverse(), vehicle.max_thrust)
        self.fault = MotorFault(count, self.inertia, self.mixer.inverse(), vehicle.max_thrust)
        self.drone = None  # set by the host, the source of RPM telemetry
        self.command = None
        self.failed = torch.zeros(count, dtype=torch.bool)
        self.failed_motor = torch.zeros(count, dtype=torch.long)
        self.below = torch.zeros(count)  # how long the weakest motor has stayed below FAILED
        self.anchor = torch.zeros(count, 3)
        self.descent = torch.zeros(count)
        A = self.mixer.inverse()[:3]  # thrust, roll and pitch from each motor; yaw is let go
        self.three = torch.stack([torch.linalg.inv(A[:, [m for m in range(4) if m != k]]) for k in range(4)])

    def __call__(self, state, target, heading, dt):
        if self.command is not None:
            lag = self.lag.update(self.command, self.drone.motor, dt)
            self.inertia = self.body.update(self.drone.motor, state[3], dt)
            self.fault.inertia = self.inertia
            self.fault.update(self.drone.motor, state[3], dt)
            scale = (TUNED_LAG / lag).clamp(0.4, 1.0)[:, None]
            self.g = Gains(**{**asdict(self.base),
                              "attitude": self.base.attitude * scale,
                              "rate": self.base.rate * scale,
                              "rate_integral": self.base.rate_integral * scale})
            if FAILURE_MODE:
                self.detect(state, dt)
        if self.failed.any():
            target, heading = self.land(state, target, heading, dt)
        self.command = super().__call__(state, target, heading, dt)
        return self.command

    def detect(self, state, dt):
        health = self.fault.health
        weakest, motor = health.min(-1)
        self.below = torch.where(weakest < FAILED, self.below + dt, torch.zeros_like(self.below))
        new = (self.below >= CONFIRM) & ~self.failed
        self.failed_motor = torch.where(new, motor, self.failed_motor)
        self.anchor = torch.where(new[:, None], state[0], self.anchor)
        self.failed = self.failed | new

    def land(self, state, target, heading, dt):
        """Hold the point where the motor failed and descend from it; let the heading spin."""
        p, v, a = target if isinstance(target, tuple) else (target, torch.zeros_like(target), torch.zeros_like(target))
        f = self.failed[:, None]
        self.descent = torch.where(self.failed, self.descent + DESCENT * dt, self.descent)
        depth = (self.anchor[:, 2] + self.descent).clamp(max=0.0)
        down = torch.stack((self.anchor[:, 0], self.anchor[:, 1], depth), -1)
        sink = torch.zeros_like(v)
        sink[:, 2] = torch.where(depth < 0, DESCENT, 0.0)
        R = state[2]
        w, x, y, z = R.unbind(-1)
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        heading = torch.where(self.failed, yaw, heading)
        return (torch.where(f, down, p), torch.where(f, sink, v), torch.where(f, torch.zeros_like(a), a)), heading

    def mix(self, thrust, torque):
        motors = super().mix(thrust, torque)
        if not self.failed.any():
            return motors
        rows = self.failed.nonzero().squeeze(-1)
        k = self.failed_motor[rows]
        wanted = torch.cat((thrust[rows, None], torque[rows, :2]), -1)
        three = (self.three[k] @ wanted[..., None]).squeeze(-1).clamp(0.0, self.max_thrust)
        keep = torch.ones(len(rows), 4, dtype=torch.bool)
        keep[torch.arange(len(rows)), k] = False
        out = torch.zeros(len(rows), 4)
        out[keep] = three.flatten()
        motors = motors.clone()
        motors[rows] = out
        return motors
