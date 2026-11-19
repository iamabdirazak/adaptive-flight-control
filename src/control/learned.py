"""A trained policy behind the same interface as the baseline."""
import torch

from ..training.environment import observe


class Learned:
    def __init__(self, policy, count, privileged=True):
        self.policy, self.privileged = policy, privileged
        self.drone, self.last = None, None

    @torch.no_grad()
    def __call__(self, state, target, heading, dt):
        p, v, _ = target
        if self.last is None:
            self.last = self.drone.motor.clone()
        obs = observe(self.drone, p, v, self.last, self.drone.wind, self.privileged)
        self.last = self.policy.command(obs)
        return self.last
