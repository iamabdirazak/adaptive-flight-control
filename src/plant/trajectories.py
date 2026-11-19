"""Where the drone is asked to be: position, velocity and acceleration over time."""
import math
import torch


def hover(n, t, altitude=2.0):
    p = torch.zeros(n, 3)
    p[:, 2] = -altitude
    return p, torch.zeros(n, 3), torch.zeros(n, 3)


def figure_eight(n, t, speed=2.0, size=2.0, altitude=2.0):
    """A horizontal figure-eight whose fastest point is at the given speed."""
    w = speed / (size * math.sqrt(2))
    s, c, s2, c2 = math.sin(w * t), math.cos(w * t), math.sin(2 * w * t), math.cos(2 * w * t)
    p = torch.tensor([size * s, size / 2 * s2, -altitude])
    v = torch.tensor([size * w * c, size * w * c2, 0.0])
    a = torch.tensor([-size * w * w * s, -2 * size * w * w * s2, 0.0])
    return p.repeat(n, 1), v.repeat(n, 1), a.repeat(n, 1)
