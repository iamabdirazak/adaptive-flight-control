"""First check that the ground is solid: the drone flies under the baseline,
and the simulator runs thousands of drones fast enough to train on.

    python -m src.evaluation.validation
"""
import time

import matplotlib.pyplot as plt
import torch

from ..control.cascade import Cascade
from ..paths import CONTROL_PERIOD, FIGURES, SIMULATION_STEP
from ..plant.quadrotor import Quadrotor
from ..plant.vehicle import Vehicle
from ..visualization import style

SUBSTEPS = round(CONTROL_PERIOD / SIMULATION_STEP)


def fly(drone, controller, target, seconds, step_at=0.5):
    """Hover, then step to the target. Returns time, positions and motor commands."""
    heading = torch.zeros(drone.n)
    start = torch.tensor([[0.0, 0.0, -2.0]]).repeat(drone.n, 1)  # hovering 2 m up
    drone.reset(start)
    times, positions, commands = [], [], []
    for k in range(round(seconds / CONTROL_PERIOD)):
        t = k * CONTROL_PERIOD
        goal = target if t >= step_at else start
        state = (drone.position, drone.velocity, drone.attitude, drone.rate)
        command = controller(state, goal, heading, CONTROL_PERIOD)
        for _ in range(SUBSTEPS):
            drone.step(command, SIMULATION_STEP)
        times.append(t)
        positions.append(drone.position[0].clone())
        commands.append(command[0].clone())
    return torch.tensor(times), torch.stack(positions), torch.stack(commands)


def throughput(count=4096, seconds=1.0):
    """Simulated drone-seconds per wall-clock second, all drones stepped together."""
    vehicle = Vehicle()
    drone, controller = Quadrotor(vehicle, count), Cascade(vehicle, count)
    drone.reset(torch.tensor([[0.0, 0.0, -2.0]]).repeat(count, 1))
    target, heading = torch.ones(count, 3) * torch.tensor([1.0, 1.0, -3.0]), torch.zeros(count)
    start = time.perf_counter()
    for _ in range(round(seconds / CONTROL_PERIOD)):
        command = controller((drone.position, drone.velocity, drone.attitude, drone.rate), target, heading, CONTROL_PERIOD)
        for _ in range(SUBSTEPS):
            drone.step(command, SIMULATION_STEP)
    return count * seconds / (time.perf_counter() - start)


def figure(times, positions, commands, target):
    style.apply()
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True,
                                      gridspec_kw=dict(height_ratios=(3, 2), hspace=0.45))
    colors = (style.SHADES["aqua"], style.SHADES["sky"], style.SHADES["violet"])
    for axis, name, color in zip(range(3), ("North", "East", "Down"), colors):
        top.plot(times, positions[:, axis], color=color, linewidth=1.2, label=name)
        top.plot(times, torch.where(times >= 0.5, target[0, axis], 0.0 if axis < 2 else -2.0), color=color,
                 linestyle=(0, (2, 3)), linewidth=0.8, alpha=0.6)
    top.set_title("Position")
    top.set_ylabel("m")
    for motor in range(4):
        bottom.plot(times, commands[:, motor], color=style.GREY, linewidth=0.8, alpha=0.4 + 0.15 * motor)
    bottom.set_title("Motor commands")
    bottom.set_xlabel("Time (s)")
    bottom.set_ylim(0, 1)
    style.legend(fig, [("swatch", c, n) for c, n in zip(colors, ("North", "East", "Down"))], anchor=(0.92, 0.94))
    style.save(fig, FIGURES, "validation")


def main():
    vehicle = Vehicle()
    target = torch.tensor([[2.0, 1.0, -3.5]])
    drone, controller = Quadrotor(vehicle), Cascade(vehicle, 1)
    times, positions, commands = fly(drone, controller, target, seconds=5.0)
    error = (positions[-1] - target[0]).norm().item()
    print(f"Thrust-to-weight {vehicle.thrust_to_weight:.1f}, hover command {commands[100].mean():.2f}")
    print(f"Final position error {error * 100:.1f} cm")
    print(f"Simulator {throughput():,.0f} drone-seconds per second")
    figure(times, positions, commands, target)
    print(f"Figure saved to {FIGURES / 'validation.png'}")


if __name__ == "__main__":
    main()
