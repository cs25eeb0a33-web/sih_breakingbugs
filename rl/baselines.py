"""
Classical baselines for the scaling study.

These are what a learned policy must beat. If it cannot beat WHCA*, the
honest conclusion is that MARL is not warranted at that fleet size -- which
is exactly the question this folder exists to answer.
"""

from __future__ import annotations

import heapq
import numpy as np

from env import ACTIONS, N_ACTIONS


def greedy(env, rng) -> np.ndarray:
    """
    Move to whichever neighbour most reduces Manhattan distance.

    The naive decentralized baseline: no peer awareness at all. The
    environment rejects colliding moves, so this degrades into livelock at
    density rather than crashing.
    """
    d = env.goal - env.pos
    act = np.zeros(env.n, dtype=np.int64)
    for i in range(env.n):
        dx, dy = d[i]
        if dx == 0 and dy == 0:
            continue
        act[i] = (4 if dx > 0 else 3) if abs(dx) >= abs(dy) else (2 if dy > 0 else 1)
    return act


def greedy_jitter(env, rng, p: float = 0.25) -> np.ndarray:
    """Greedy plus random jitter when stalled -- the classic livelock escape."""
    act = greedy(env, rng)
    stuck = env.steps_since_progress > 3
    if stuck.any():
        n = int(stuck.sum())
        act[stuck] = np.where(rng.random(n) < p,
                              rng.integers(0, N_ACTIONS, n), act[stuck])
    return act


class WHCAStar:
    """
    Windowed Hierarchical Cooperative A* (Silver 2005) -- the strong classical
    baseline, and essentially what sim2d.py does in continuous space.

    Agents are planned in priority order over a short time window. Each
    agent reserves (cell, t) and later agents plan around those reservations.
    Priority = distance to goal, recomputed each replan, which is a decent
    proxy for urgency and avoids the starvation a fixed order causes.
    """

    def __init__(self, env, window: int = 8, replan_every: int = 4):
        self.env = env
        self.window = window
        self.replan_every = replan_every
        self.plans = [[] for _ in range(env.n)]
        self._k = 0

    def _astar(self, start, goal, reserved, t0):
        """Space-time A*. State is (x, y, t) -- time IS in the state here."""
        env, w_, h_ = self.env, self.env.w, self.env.h
        W = self.window
        sx, sy = int(start[0]), int(start[1])
        gx, gy = int(goal[0]), int(goal[1])
        h = lambda x, y: abs(gx - x) + abs(gy - y)
        openh = [(h(sx, sy), 0, sx, sy, 0)]
        came, best = {}, {(sx, sy, 0): 0}
        goal_state = None
        while openh:
            f, g, x, y, t = heapq.heappop(openh)
            if (x, y) == (gx, gy) or t >= W:
                goal_state = (x, y, t)
                break
            for a in range(N_ACTIONS):
                dx, dy = ACTIONS[a]
                nx, ny, nt = x + dx, y + dy, t + 1
                if not (0 <= nx < w_ and 0 <= ny < h_):
                    continue
                if env.grid[ny, nx] == 1:
                    continue
                if (nx, ny, nt) in reserved:
                    continue
                if (nx, ny, x, y, nt) in reserved:      # edge / swap conflict
                    continue
                ng = g + 1
                if ng < best.get((nx, ny, nt), 1 << 30):
                    best[(nx, ny, nt)] = ng
                    came[(nx, ny, nt)] = (x, y, t, a)
                    heapq.heappush(openh, (ng + h(nx, ny), ng, nx, ny, nt))
        if goal_state is None:
            return [], []
        path, acts, cur = [], [], goal_state
        while cur in came:
            px, py, pt, a = came[cur]
            path.append(cur); acts.append(a)
            cur = (px, py, pt)
        return path[::-1], acts[::-1]

    def __call__(self, env, rng) -> np.ndarray:
        if self._k % self.replan_every == 0:
            dist = np.abs(env.goal - env.pos).sum(1)
            order = np.argsort(dist)                     # nearest-goal first
            reserved = set()
            self.plans = [[] for _ in range(env.n)]
            for i in order:
                path, acts = self._astar(env.pos[i], env.goal[i], reserved, 0)
                self.plans[i] = acts
                px, py = int(env.pos[i][0]), int(env.pos[i][1])
                for t, (x, y, tt) in enumerate(path, start=1):
                    reserved.add((x, y, t))
                    reserved.add((px, py, x, y, t))      # block the swap
                    px, py = x, y
                for t in range(len(path) + 1, self.window + 1):
                    reserved.add((px, py, t))            # hold final cell
        self._k += 1
        act = np.zeros(env.n, dtype=np.int64)
        for i in range(env.n):
            if self.plans[i]:
                act[i] = self.plans[i].pop(0)
        return act
