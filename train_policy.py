"""
Train the lightweight coordination policy by IMITATION.

Expert  = full space-time A* planner with peer reservations (global info)
Student = 7x7 local observation window -> action  (local info ONLY)

The student never sees the full map. That information gap is the entire
point: if it can reproduce the expert's decisions from a local window, the
coordination has genuinely been decentralised.

Run:  python3 train_policy.py
"""

from __future__ import annotations

import random
import time

import numpy as np

from learned import (ACTION_NAMES, PolicyNet, build_observation,
                     expert_action)
from planner import ReservationTable, path_to_reservations, plan
from warehouse_map import WarehouseMap


def generate_dataset(n_scenarios: int = 400, seed: int = 0):
    """
    Roll out the expert planner on random scenarios and record
    (local observation -> action the expert took) pairs.

    Labels are EXACT -- no reward shaping, no exploration, no instability.
    That is why this trains in seconds where RL would take days.
    """
    rng = random.Random(seed)
    wmap = WarehouseMap()
    free = [(x, y) for y in range(wmap.h) for x in range(wmap.w)
            if wmap.is_free(x, y)]

    X, Y = [], []
    for i in range(n_scenarios):
        table = ReservationTable()

        # populate a few synthetic peers so the observation has traffic in it
        for pid in range(2, 5):
            ps = rng.choice(free)
            pg = rng.choice(free)
            pp = plan(wmap, ps, pg, table, pid, 0.0, 0.0)
            if pp:
                table.update(pid, path_to_reservations(pp, 0.0, 0.6), 0.0)

        start, goal = rng.choice(free), rng.choice(free)
        path = plan(wmap, start, goal, table, 1, 0.0, 0.0)
        if not path or len(path) < 3:
            continue

        for (cx, cy) in path[:-1]:
            a = expert_action(path, cx, cy)
            obs = build_observation(wmap, cx, cy, goal, table, 0.0, 1,
                                    battery_soc=rng.uniform(0.3, 1.0),
                                    priority_rank=rng.uniform(0, 1))
            X.append(obs)
            Y.append(a)

    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.int64)


def main() -> None:
    print("Generating imitation dataset from planner expert ...")
    t0 = time.time()
    X, Y = generate_dataset(400)
    print(f"  {len(X)} samples in {time.time()-t0:.1f}s")

    counts = np.bincount(Y, minlength=5)
    print("  action distribution:",
          {ACTION_NAMES[i]: int(c) for i, c in enumerate(counts)})

    # held-out split
    n = len(X)
    idx = np.random.default_rng(0).permutation(n)
    cut = int(0.85 * n)
    Xtr, Ytr = X[idx[:cut]], Y[idx[:cut]]
    Xte, Yte = X[idx[cut:]], Y[idx[cut:]]

    net = PolicyNet()
    print(f"\nTraining policy ({net.n_params()} parameters) ...")
    t0 = time.time()
    net.train(Xtr, Ytr, epochs=40, lr=0.05)
    train_s = time.time() - t0

    tr = net.accuracy(Xtr, Ytr)
    te = net.accuracy(Xte, Yte)
    print(f"\n  train acc {tr:.3f}   HELD-OUT acc {te:.3f}")
    print(f"  trained in {train_s:.1f}s on CPU")

    # inference latency -- the edge-deployment claim
    t0 = time.time()
    for i in range(2000):
        net.predict(Xte[i % len(Xte)])
    per_call_ms = (time.time() - t0) / 2000 * 1000
    print(f"  inference {per_call_ms:.3f} ms/call  "
          f"({1000/per_call_ms:.0f} Hz capable)")

    net.save("policy.npz")
    print("\nsaved policy.npz")
    print("\nClaim you can defend:")
    print(f"  '{net.n_params()} parameter policy distilled from a centralised")
    print(f"   planner, {te:.0%} held-out action agreement, {per_call_ms:.2f} ms")
    print( "   inference on CPU, acting on a 7x7 local window only.'")


if __name__ == "__main__":
    main()
