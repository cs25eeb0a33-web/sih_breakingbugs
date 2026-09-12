"""
Shared-weight decentralized policy, pure numpy.

Trained by IMITATION of WHCA* (behaviour cloning), not by RL from scratch.

Why imitation first, and why it is the honest starting point:
  * The expert (WHCA*) sees the whole map and every peer's plan. The student
    sees a 9x9 window and six scalars. The gap between them measures exactly
    one thing -- how much coordination survives when you remove the global
    view. That is the decentralization question, stated numerically.
  * It trains in seconds on a CPU with no framework, matching the repo's
    numpy-only constraint and the existing train_policy.py precedent.
  * REINFORCE/PPO from scratch on 32+ agents needs GPU-hours and a reward
    that is hard to get right. Behaviour cloning gives a defensible number
    today and is the standard warm-start for MARL fine-tuning anyway
    (PRIMAL does exactly this).

All agents share one network -- this is CTDE with a centralized expert at
train time and fully decentralized execution at run time.
"""

from __future__ import annotations

import numpy as np


class MLP:
    def __init__(self, dims, seed=0):
        rng = np.random.default_rng(seed)
        self.W, self.b = [], []
        for a, b_ in zip(dims[:-1], dims[1:]):
            self.W.append(rng.normal(0, np.sqrt(2.0 / a), (a, b_)).astype(np.float32))
            self.b.append(np.zeros(b_, dtype=np.float32))
        self.mW = [np.zeros_like(w) for w in self.W]
        self.vW = [np.zeros_like(w) for w in self.W]
        self.mb = [np.zeros_like(x) for x in self.b]
        self.vb = [np.zeros_like(x) for x in self.b]
        self.t = 0

    def forward(self, x, cache=None):
        a = x
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            z = a @ W + b
            if i < len(self.W) - 1:
                a = np.maximum(z, 0.0)
            else:
                a = z
            if cache is not None:
                cache.append((z, a))
        return a

    def act(self, x, greedy=True):
        z = self.forward(x)
        return np.argmax(z, axis=1) if greedy else self._sample(z)

    @staticmethod
    def _sample(z):
        z = z - z.max(1, keepdims=True)
        p = np.exp(z); p /= p.sum(1, keepdims=True)
        c = p.cumsum(1)
        u = np.random.random((len(z), 1))
        return (u > c).sum(1)

    def train_step(self, x, y, lr=1e-3):
        """Cross-entropy on expert actions. Returns (loss, accuracy)."""
        acts = [x]
        a = x
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            z = a @ W + b
            a = np.maximum(z, 0.0) if i < len(self.W) - 1 else z
            acts.append(a)
        logits = acts[-1]
        m = logits.max(1, keepdims=True)
        p = np.exp(logits - m); p /= p.sum(1, keepdims=True)
        n = len(x)
        loss = float(-np.log(p[np.arange(n), y] + 1e-9).mean())
        acc = float((logits.argmax(1) == y).mean())

        g = p.copy(); g[np.arange(n), y] -= 1.0; g /= n
        self.t += 1
        for i in reversed(range(len(self.W))):
            gW = acts[i].T @ g
            gb = g.sum(0)
            if i > 0:
                g = (g @ self.W[i].T) * (acts[i] > 0)
            b1, b2, eps = 0.9, 0.999, 1e-8
            self.mW[i] = b1 * self.mW[i] + (1 - b1) * gW
            self.vW[i] = b2 * self.vW[i] + (1 - b2) * gW * gW
            self.mb[i] = b1 * self.mb[i] + (1 - b1) * gb
            self.vb[i] = b2 * self.vb[i] + (1 - b2) * gb * gb
            mhW = self.mW[i] / (1 - b1 ** self.t); vhW = self.vW[i] / (1 - b2 ** self.t)
            mhb = self.mb[i] / (1 - b1 ** self.t); vhb = self.vb[i] / (1 - b2 ** self.t)
            self.W[i] -= lr * mhW / (np.sqrt(vhW) + eps)
            self.b[i] -= lr * mhb / (np.sqrt(vhb) + eps)
        return loss, acc

    @property
    def n_params(self):
        return sum(w.size for w in self.W) + sum(b.size for b in self.b)

    def save(self, path):
        d = {f"W{i}": w for i, w in enumerate(self.W)}
        d.update({f"b{i}": b for i, b in enumerate(self.b)})
        np.savez(path, **d)

    @staticmethod
    def load(path):
        z = np.load(path)
        nW = sum(1 for k in z.files if k.startswith("W"))
        m = MLP([1, 1])
        m.W = [z[f"W{i}"] for i in range(nW)]
        m.b = [z[f"b{i}"] for i in range(nW)]
        return m


def collect(grid, n_agents, episodes, steps, seed=0, window=8):
    """Roll out WHCA* and record (local observation, expert action) pairs."""
    from env import LifelongMAPF
    from baselines import WHCAStar
    X, Y = [], []
    for ep in range(episodes):
        e = LifelongMAPF(grid, n_agents, seed=seed + ep)
        pol = WHCAStar(e, window=window)
        rng = np.random.default_rng(seed + ep)
        obs = e.reset()
        for _ in range(steps):
            a = np.asarray(pol(e, rng))
            X.append(obs.copy()); Y.append(a.copy())
            obs, _, _ = e.step(a)
    return np.concatenate(X).astype(np.float32), np.concatenate(Y).astype(np.int64)
