"""Quaternion maths, batched. Quaternions are (w, x, y, z), body to world."""
import torch


def multiply(a, b):
    aw, ax, ay, az = a.unbind(-1)
    bw, bx, by, bz = b.unbind(-1)
    return torch.stack((
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ), -1)


def to_matrix(q):
    w, x, y, z = q.unbind(-1)
    return torch.stack((
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
    ), -1).reshape(*q.shape[:-1], 3, 3)


def integrate(q, rate, dt):
    """Advance an attitude by a body angular rate over dt."""
    half = torch.cat((torch.zeros_like(rate[..., :1]), rate), -1) * 0.5 * dt
    q = q + multiply(q, half)
    return q / q.norm(dim=-1, keepdim=True)


def identity(n, **kw):
    q = torch.zeros(n, 4, **kw)
    q[:, 0] = 1.0
    return q
