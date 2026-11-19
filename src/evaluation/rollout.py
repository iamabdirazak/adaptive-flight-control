"""Fly one scenario for many drones at once and record what happened."""
import torch

from ..paths import CONTROL_PERIOD, SIMULATION_STEP
from ..plant.conditions import Wind
from ..plant.quadrotor import Quadrotor
from .scenarios import EVENT, Scenario, draw

SUBSTEPS = round(CONTROL_PERIOD / SIMULATION_STEP)
DESCENT = 0.7  # m/s, the landing speed asked for after a failure


def rollout(scenario: Scenario, make_controller, vehicle, trials: int, seed: int = 0):
    generator = torch.Generator().manual_seed(seed)
    drone = draw(scenario, Quadrotor(vehicle, trials), generator)
    altitude = scenario.extra.get("altitude", 2.0)
    start, _, _ = scenario.path(trials, 0.0, altitude=altitude)
    drone.reset(start)
    controller = make_controller(trials)
    if hasattr(controller, "drone"):  # learned controllers read the drone they fly
        controller.drone = drone
    wind = Wind(trials, scenario.wind, generator) if scenario.wind else None
    heading = torch.zeros(trials)
    steps = round(scenario.seconds / CONTROL_PERIOD)
    log = dict(time=torch.arange(steps) * CONTROL_PERIOD, position=[], reference=[])
    anchor = None
    for k in range(steps):
        t = k * CONTROL_PERIOD
        if scenario.event and abs(t - EVENT) < CONTROL_PERIOD / 2:
            scenario.event(drone, generator)
            anchor = drone.position.clone()
        p, v, a = scenario.path(trials, t, altitude=altitude)
        if scenario.land and anchor is not None:  # descend straight down from where the change happened
            depth = (anchor[:, 2] + DESCENT * (t - EVENT)).clamp(max=0.0)
            p = torch.stack((anchor[:, 0], anchor[:, 1], depth), -1)
            v = torch.zeros_like(p)
            v[:, 2] = torch.where(depth < 0, DESCENT, 0.0)
            a = torch.zeros_like(p)
        command = controller((drone.position, drone.velocity, drone.attitude, drone.rate), (p, v, a), heading, CONTROL_PERIOD)
        for _ in range(SUBSTEPS):
            if wind is not None:
                drone.wind = wind.step(SIMULATION_STEP)
            drone.step(command, SIMULATION_STEP)
        log["position"].append(drone.position.clone())
        log["reference"].append(p.clone())
    log["position"] = torch.stack(log["position"], 1)  # (trials, steps, 3)
    log["reference"] = torch.stack(log["reference"], 1)
    log.update(landed=drone.landed, touchdown_speed=drone.touchdown_speed,
               touchdown_tilt=drone.touchdown_tilt, anchor=anchor)
    return log
