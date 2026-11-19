"""The scenarios of docs/requirements.md, each a flight with a pass criterion.

A scenario says where the drone starts, what it is asked to fly, what
changes about it before and during flight, and how the flight is judged.
"""
from dataclasses import dataclass, field
from typing import Callable

import torch

from ..plant import trajectories
from ..plant.conditions import TEST, TRAINING, Airframe

EVENT = 2.0  # s, when an in-flight change happens


@dataclass
class Scenario:
    key: str
    name: str
    seconds: float
    metric: str  # what is measured, see metrics.py
    target: float  # pass if the metric is at or below this
    unit: str
    path: Callable = trajectories.hover
    airframe: dict | None = None  # ranges to draw each drone's body from
    wind: float = 0.0  # m/s, the strongest steady wind drawn
    event: Callable | None = None  # change applied once at EVENT
    land: bool = False  # after the event, the reference descends to the ground
    notes: str = ""
    extra: dict = field(default_factory=dict)


def add_payload(drone, generator):
    drone.mass = drone.mass * 1.3


def weaken_motor(drone, generator):
    drone.max_thrust[:, 0] *= 0.6


def damage_motor(drone, generator):  # one motor left with 20 to 40% of its thrust
    drone.max_thrust[:, 0] *= 0.2 + 0.2 * torch.rand(drone.n, generator=generator)


def payload_and_damage(drone, generator):
    drone.mass = drone.mass * 1.3
    drone.max_thrust[:, 0] *= 0.4


def fail_motor(drone, generator):
    drone.max_thrust[:, 0] = 0.0


# Everyday scenarios: tuned PID already handles these, so the adaptive
# controller is held to the fair PID's measured numbers (no regression).
# Hard scenarios: past what fixed gains can handle, the reason this exists.
SCENARIOS = [
    Scenario("S1", "Nominal flight", 14.0, "rms", 0.10, "m", path=trajectories.figure_eight),
    Scenario("S2", "Unknown airframe", 14.0, "rms", 0.15, "m", path=trajectories.figure_eight,
             airframe=TRAINING, notes="target is 1.5 × the S1 target"),
    Scenario("S3", "Payload added", 6.0, "drop", 0.30, "m", event=add_payload),
    Scenario("S4", "Wind and gusts", 10.0, "deviation", 0.50, "m", wind=TRAINING["wind"]),
    Scenario("S5", "Weak motor", 6.0, "deviation", 0.30, "m", event=weaken_motor),
    Scenario("S6", "Motor damage", 6.0, "deviation", 0.30, "m", event=damage_motor),
    Scenario("S7", "Combined changes", 8.0, "deviation", 0.30, "m", event=payload_and_damage,
             wind=TRAINING["wind"]),
    Scenario("S8", "Extreme airframe", 14.0, "rms", 0.15, "m", path=trajectories.figure_eight, airframe=TEST),
    Scenario("S9", "Motor failure", 12.0, "landing", 2.0, "m", event=fail_motor, land=True,
             extra=dict(altitude=8.0)),
]
EVERYDAY = ("S1", "S2", "S3", "S4", "S5")


def draw(scenario: Scenario, drone, generator):
    """Prepare each drone's body for this scenario."""
    if scenario.airframe:
        Airframe(scenario.airframe).apply(drone, generator)
    return drone
