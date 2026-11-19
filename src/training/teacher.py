"""Train the teacher: first copy the PID baseline, then improve on it with PPO.

Copying first gives the policy stable flight from the start, so
reinforcement learning only has to find what the PID cannot do.

    python -m src.training.teacher
"""
import time

import torch

from ..control.cascade import Cascade
from ..paths import CONTROL_PERIOD, MODELS
from .environment import Environment
from .policy import Policy

ENVS, HORIZON, GAMMA, LAMBDA = 1024, 128, 0.995, 0.95


def imitate(env, policy, ticks=3000, epochs=1500):
    """Fly the PID, record what it commands, and fit the policy to it."""
    pid = Cascade(env.vehicle, env.n)
    obs, actions = [], []
    for k in range(ticks):
        p, v, a = env.reference()
        state = (env.drone.position, env.drone.velocity, env.drone.attitude, env.drone.rate)
        command = pid(state, (p, v, a), torch.zeros(env.n), CONTROL_PERIOD)
        if k % 10 == 0:
            obs.append(env.observation()); actions.append(command)
        _, done, _ = env.step(command)
        pid.reset(done)
    obs, actions = torch.cat(obs), torch.cat(actions)
    optimiser = torch.optim.Adam(policy.actor.parameters(), lr=1e-3)
    for epoch in range(epochs):
        idx = torch.randint(0, len(obs), (8192,))
        loss = (policy.command(obs[idx]) - actions[idx]).pow(2).mean()
        optimiser.zero_grad(); loss.backward(); optimiser.step()
    return loss.item()


def train(iterations=120, seed=0):
    torch.manual_seed(seed)
    env = Environment(ENVS, seed)
    policy = Policy(env.observation().shape[1])
    print(f"imitation loss {imitate(env, policy):.5f}")
    optimiser = torch.optim.Adam(policy.parameters(), lr=1e-4)
    WARMUP = 20  # iterations that train only the critic, so the copied flight is not undone
    start = time.time()
    for it in range(iterations):
        buf = dict(obs=[], act=[], logp=[], rew=[], done=[], val=[])
        crashes = 0
        with torch.no_grad():
            for _ in range(HORIZON):
                obs = env.observation()
                dist = policy.distribution(obs)
                act = dist.sample().clamp(0, 1)
                reward, done, info = env.step(act)
                crashes += info["crashed"].sum().item()
                for key, val in zip(buf, (obs, act, dist.log_prob(act).sum(-1), reward, done.float(), policy.critic(obs).squeeze(-1))):
                    buf[key].append(val)
            last = policy.critic(env.observation()).squeeze(-1)
        rew, done, val = (torch.stack(buf[k]) for k in ("rew", "done", "val"))
        adv, gae = torch.zeros_like(rew), torch.zeros(ENVS)
        for t in reversed(range(HORIZON)):
            nxt = last if t == HORIZON - 1 else val[t + 1]
            delta = rew[t] + GAMMA * nxt * (1 - done[t]) - val[t]
            gae = delta + GAMMA * LAMBDA * (1 - done[t]) * gae
            adv[t] = gae
        ret = (adv + val).flatten()
        obs, act, logp = (torch.cat(buf[k]) for k in ("obs", "act", "logp"))
        adv = adv.flatten(); adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        for _ in range(4):
            for idx in torch.randperm(len(obs)).split(16384):
                dist = policy.distribution(obs[idx])
                ratio = (dist.log_prob(act[idx]).sum(-1) - logp[idx]).exp()
                actor = -torch.min(ratio * adv[idx], ratio.clamp(0.8, 1.2) * adv[idx]).mean()
                critic = (policy.critic(obs[idx]).squeeze(-1) - ret[idx]).pow(2).mean()
                loss = 0.5 * critic if it < WARMUP else actor + 0.5 * critic
                optimiser.zero_grad(); loss.backward()
                torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0); optimiser.step()
        if it % 10 == 0:
            print(f"{it:4d}  reward {rew.mean():.3f}  crashes {crashes:4d}  {time.time() - start:.0f}s", flush=True)
    MODELS.mkdir(exist_ok=True)
    torch.save(policy.state_dict(), MODELS / "teacher.pt")
    return policy


if __name__ == "__main__":
    train()
