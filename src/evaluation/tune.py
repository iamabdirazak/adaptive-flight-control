"""Tune the baseline the way a PID controller is tuned: once, on the nominal drone.

Hundreds of gain sets fly side by side, one per drone, on the nominal
figure-eight and a nominal step. As in any careful tuning, a set must keep
a margin: it has to fly the step cleanly with motors twice as slow as
nominal. The best set becomes the baseline's defaults. Nothing about
payloads, wind or motor faults is seen here.

    python -m src.evaluation.tune
"""
import json
from dataclasses import asdict, fields

import torch

from ..control.cascade import Cascade, Gains
from ..paths import REPORTS
from ..plant.vehicle import Vehicle
from .metrics import measure
from .rollout import rollout
from .scenarios import SCENARIOS, Scenario

CENTRE = dict(position=1.2, velocity=3.0, velocity_integral=1.0, attitude=7.0, rate=24.0, rate_integral=60.0)
TUNED = ("position", "velocity", "velocity_integral", "attitude", "rate", "rate_integral")


def candidates(n, generator):
    columns = {}
    for name in TUNED:  # log-uniform from a third to three times the hand-tuned value
        columns[name] = CENTRE[name] * 3 ** (2 * torch.rand(n, 1, generator=generator) - 1)
        columns[name][0] = CENTRE[name]  # keep the hand-tuned set as candidate 0
    return Gains(**{**asdict(Gains()), **columns})


def step(n, t, altitude=2.0):
    p = torch.zeros(n, 3)
    p[:, 2] = -altitude
    if t >= 1.0:
        p += torch.tensor([1.0, 1.0, -1.0])
    return p, torch.zeros(n, 3), torch.zeros(n, 3)


def main(count=512):
    vehicle = Vehicle()
    gains = candidates(count, torch.Generator().manual_seed(0))
    make = lambda n: Cascade(vehicle, n, gains)
    nominal = SCENARIOS[0]
    tracking = measure("rms", rollout(nominal, make, vehicle, count))
    def step_error(airframe=None):
        log = rollout(Scenario("step", "Step", 5.0, "rms", 0, "m", path=step, airframe=airframe), make, vehicle, count)
        return (log["position"] - log["reference"]).norm(dim=-1)[:, log["time"] >= 2.5].max(-1).values

    overshoot = step_error()
    slow = step_error(dict(mass=0.0, inertia=0.0, motor=0.0, time_constant=(0.04, 0.04)))  # the delay margin
    cost = tracking + 0.5 * overshoot + torch.where(slow < 0.1, 0.0, 1e3)
    cost = torch.where(torch.isfinite(cost), cost, torch.full_like(cost, 1e9))
    best = cost.argmin().item()
    chosen = {name: round(getattr(gains, name)[best].item(), 2) for name in TUNED}
    print(f"hand-tuned: tracking {tracking[0]:.3f} m, step error {overshoot[0]:.3f} m, slow motors {slow[0]:.3f} m")
    print(f"tuned:      tracking {tracking[best]:.3f} m, step error {overshoot[best]:.3f} m, slow motors {slow[best]:.3f} m")
    print(chosen)
    REPORTS.mkdir(exist_ok=True)
    (REPORTS / "tuning.json").write_text(json.dumps(dict(gains=chosen, tracking=tracking[best].item(),
                                                          step_error=overshoot[best].item(), slow_motor_step_error=slow[best].item(), candidates=count), indent=2))


if __name__ == "__main__":
    main()
