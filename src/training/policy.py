"""The flight policy: a small network from what it sees to four motor commands."""
import torch
from torch import nn


class Policy(nn.Module):
    def __init__(self, inputs, hidden=128):
        super().__init__()
        self.actor = nn.Sequential(nn.Linear(inputs, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(), nn.Linear(hidden, 4))
        self.critic = nn.Sequential(nn.Linear(inputs, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(), nn.Linear(256, 1))
        self.register_buffer("log_std", torch.full((4,), -3.5))  # fixed exploration noise, about 0.03

    def command(self, obs):
        """Mean motor command in [0, 1]."""
        return torch.sigmoid(self.actor(obs))

    def distribution(self, obs):
        return torch.distributions.Normal(self.command(obs), self.log_std.exp())
