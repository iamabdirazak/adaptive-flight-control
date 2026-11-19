"""How four motor thrusts become one collective thrust and three body torques."""
import torch

from .vehicle import Vehicle


def allocation_matrix(vehicle: Vehicle, dtype=torch.float32):
    """Rows: collective thrust, roll, pitch and yaw torque. Columns: motors.

    Thrust acts along -z of the FRD body frame, so a motor at (x, y)
    produces a roll torque of -y·T and a pitch torque of x·T.
    """
    rows = [[1.0] * 4, [], [], []]
    for (x, y), s in zip(vehicle.motor_positions(), vehicle.spin):
        rows[1].append(-y)
        rows[2].append(x)
        rows[3].append(s * vehicle.torque_ratio)
    return torch.tensor(rows, dtype=dtype)
