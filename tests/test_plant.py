import torch

from src.control.cascade import Cascade
from src.plant.quadrotor import Quadrotor
from src.plant.vehicle import Vehicle

STEP = 0.0005


def test_hover_is_an_equilibrium():
    drone = Quadrotor(Vehicle(), 3)
    drone.reset(torch.tensor([[0.0, 0.0, -5.0]]).repeat(3, 1))
    for _ in range(2000):
        drone.step(drone.motor.clone(), STEP)
    assert (drone.position[:, 2] + 5.0).abs().max() < 1e-4 and not drone.landed.any()


def test_motors_off_means_free_fall():
    drone = Quadrotor(Vehicle(), 1)
    drone.reset(torch.tensor([[0.0, 0.0, -10.0]]))
    drone.motor.zero_()
    drone.drag.zero_()  # gravity alone
    for _ in range(2000):  # one second
        drone.step(torch.zeros(1, 4), STEP)
    assert abs(drone.position[0, 2].item() + 10.0 - 0.5 * 9.81) < 0.1  # NED: falling is +z


def test_baseline_reaches_target():
    vehicle = Vehicle()
    drone, controller = Quadrotor(vehicle, 1), Cascade(vehicle, 1)
    drone.reset(torch.tensor([[0.0, 0.0, -2.0]]))
    target, heading = torch.tensor([[1.0, -1.0, -3.0]]), torch.zeros(1)
    for _ in range(2500):
        command = controller((drone.position, drone.velocity, drone.attitude, drone.rate), target, heading, 0.002)
        for _ in range(4):
            drone.step(command, STEP)
    assert (drone.position - target).norm() < 0.05


def test_ground_stops_a_falling_drone():
    drone = Quadrotor(Vehicle(), 1)
    drone.reset(torch.tensor([[0.0, 0.0, -1.0]]))
    for _ in range(2000):
        drone.step(torch.zeros(1, 4), STEP)
    assert drone.landed.all() and drone.position[0, 2] == 0 and drone.touchdown_speed[0] > 3.0
