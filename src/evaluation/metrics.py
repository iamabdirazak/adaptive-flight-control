"""How each flight is judged. Every metric is per trial, lower is better."""
import torch

from .scenarios import EVENT

SETTLE = 2.0  # s, the start of a flight is not judged
SAFE_SPEED = 1.5  # m/s, the fastest touchdown that counts as a landing
SAFE_TILT = 0.5  # rad, about 30°


def measure(metric: str, log) -> torch.Tensor:
    t = log["time"]
    error = log["position"] - log["reference"]
    if metric == "rms":  # tracking error along the path
        judged = error[:, t >= SETTLE]
        return judged.norm(dim=-1).pow(2).mean(-1).sqrt()
    if metric == "drop":  # how far the drone sinks after the change
        after = error[:, t >= EVENT, 2]  # NED: sinking is +z
        return after.max(-1).values.clamp(min=0.0)
    if metric == "deviation":  # the furthest the drone strays from where it should be
        judged = error[:, t >= SETTLE]
        return judged.norm(dim=-1).max(-1).values
    if metric == "landing":  # distance from the failure point, infinite if it crashed
        offset = (log["position"][:, -1, :2] - log["anchor"][:, :2]).norm(dim=-1)
        safe = log["landed"] & (log["touchdown_speed"] < SAFE_SPEED) & (log["touchdown_tilt"] < SAFE_TILT)
        return torch.where(safe, offset, torch.full_like(offset, float("inf")))
    raise ValueError(metric)
