"""What the adaptive controller believes about the drone, against the truth.

The estimators already run inside the controller. This reads them at the
end of each flight and compares them with the drone the simulator flew.

    python -m src.evaluation.readout
"""
import json

import torch

from ..control.adaptive import Adaptive
from ..paths import REPORTS
from ..plant.vehicle import Vehicle
from .baseline import TRIALS
from .rollout import rollout
from .scenarios import SCENARIOS

SCENARIO = {s.key: s for s in SCENARIOS}


def fly(key, vehicle, trials=TRIALS):
    """Fly one scenario with the same seed as the study and return the controller and the drone."""
    made = {}

    def make(n):
        made["controller"] = Adaptive(vehicle, n)
        return made["controller"]

    rollout(SCENARIO[key], make, vehicle, trials, seed=int(key[1:]))
    controller = made["controller"]
    return controller, controller.drone


def summary(error, within):
    error = error.flatten().abs()
    return dict(median=error.median().item(), p95=error.quantile(0.95).item(),
                within=(error <= within).float().mean().item())


def main():
    vehicle = Vehicle()
    report = {}

    # Mass, motor lag and inertia: an unknown airframe, read at the end of the flight
    controller, drone = fly("S2", vehicle)
    strength = drone.max_thrust[:, 0] / vehicle.max_thrust  # motor constants the controller cannot see
    report["mass"] = dict(unit="%", target=10.0, scenario="S2",
                          **summary(100 * (controller.mass / drone.mass - 1), 10.0))
    # Hover thrust is read through the motors, so what it measures is mass in units of their thrust
    report["mass_thrust_referenced"] = dict(unit="%", scenario="S2",
                                            **summary(100 * (controller.mass * strength / drone.mass - 1), 10.0))
    report["motor_lag"] = dict(unit="ms", scenario="S2",
                               **summary(1000 * (controller.lag.estimate - drone.time_constant), 5.0))
    for axis, name in enumerate(("roll", "pitch", "yaw")):
        report[f"inertia_{name}"] = dict(unit="%", scenario="S2", **summary(
            100 * (controller.inertia[:, axis] / drone.inertia[:, axis] - 1), 10.0))
    controller, drone = fly("S3", vehicle)
    report["mass_payload"] = dict(unit="%", target=10.0, scenario="S3",
                                  **summary(100 * (controller.mass / drone.mass - 1), 10.0))

    # Motor health of the weakened motor: 60 % in S5, 20 to 40 % in S6
    for key in ("S5", "S6"):
        controller, drone = fly(key, vehicle)
        truth = drone.max_thrust[:, 0] / vehicle.max_thrust
        report[f"motor_health_{key}"] = dict(unit="pp", target=10.0, scenario=key,
                                             **summary(100 * (controller.fault.health[:, 0] - truth), 10.0))

    for name, r in report.items():
        print(f"{name:<24} {r['scenario']}  median {r['median']:.2f} {r['unit']}  "
              f"p95 {r['p95']:.2f}  within {r['within']:.0%}")
    REPORTS.mkdir(exist_ok=True)
    (REPORTS / "readout.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
