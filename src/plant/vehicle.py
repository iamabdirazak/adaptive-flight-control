"""The reference drone: a compact 5-inch quadrotor of about 500 g.

The values are first estimates for that class of drone. They are replaced
by datasheet values once the components are chosen.
"""
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class Vehicle:
    mass: float = 0.50  # kg
    inertia: tuple = (1.6e-3, 1.6e-3, 2.8e-3)  # kg m², about the body axes
    arm: float = 0.110  # m, centre to motor (220 mm diagonal)
    max_speed: float = 3000.0  # rad/s, motor at full command
    thrust_coefficient: float = 5.45e-7  # N per (rad/s)², 4.9 N per motor at full speed
    torque_ratio: float = 0.016  # m, yaw torque per newton of thrust
    motor_time_constant: float = 0.020  # s, first-order lag of a motor
    drag: float = 0.10  # N per m/s, linear body drag
    gravity: float = 9.81  # m/s²
    # PX4 quad X order: front-right, rear-left, front-left, rear-right.
    # +1 spins counter-clockwise seen from above, -1 clockwise.
    spin: tuple = (1.0, 1.0, -1.0, -1.0)
    layout: tuple = field(default=((1, 1), (-1, -1), (1, -1), (-1, 1)))

    @property
    def max_thrust(self) -> float:
        return self.thrust_coefficient * self.max_speed**2

    @property
    def thrust_to_weight(self) -> float:
        return 4 * self.max_thrust / (self.mass * self.gravity)

    def motor_positions(self):
        """Motor positions in the body frame (forward, right), metres."""
        d = self.arm / math.sqrt(2)
        return [(fx * d, ry * d) for fx, ry in self.layout]
