# `rl/` — MARL scaling study for fleets > 32

**This folder is isolated from the SIH deliverable.** It shares no code with
`sim2d.py`, `coordination.py` or `planner.py`, and nothing in the production
path imports it. The 3–5 robot system stays exactly as it is.

Purpose: answer one question with measurements instead of intuition —
**at what fleet size does a learned policy beat classical planning?**

---

## 1. The answer

**~200+ agents.** Not 16, not 32. On this map, classical prioritized planning
does not degrade until roughly **17–22% agent density**, and below that there
is no headroom for a learned method to exploit.

```
    N  density    WHCA*  blocked  stalled  plan_ms
   32     2.7%    3.208        0        0      0.8
   64     5.5%    3.214        0        0      1.5
   96     8.2%    3.076        5        0      2.3
  128    11.0%    3.029       16        0      3.2
  160    13.7%    2.935      272        0      4.0
  192    16.4%    2.885      328        0      4.8
  256    21.9%    1.693    12712      116      8.1   <-- degrading
  320    27.4%    0.835    27663      243     10.0   <-- collapsed
```

WHCA\* holds **within 10% of peak from 32 to 192 agents.** It only breaks past
~200. That is the crossover, and it is where MARL first has something to offer.

Caveat: this is map-dependent. A tighter layout with 1-wide corridors and more
contention would break classical earlier. But the order of magnitude is the
finding — the threshold is hundreds of agents, not tens.

## 2. Formulation

**Dec-POMDP, lifelong (online) MAPF.** Agents are homogeneous and share one
policy network — centralized training, decentralized execution.

| element | definition |
|---|---|
| **State** | grid occupancy + every agent's (position, goal) |
| **Observation** `o_i` | 9×9 egocentric window × 4 channels *(obstacles, peers, peer goals, self)* + 6 scalars → **330-dim**. Nothing global. |
| **Action** `a_i` | 5 discrete: stay, ↑, ↓, ←, → |
| **Transition** | simultaneous; **vertex and edge (swap) conflicts are rejected**, the agent stays put |
| **Reward** `r_i` | `0.3·Δmanhattan + 5.0·delivered − 0.05·step − 0.25·stalled(>12)` |
| **Horizon** | infinite; on reaching a goal a new one is drawn immediately |

The six scalars are deliberately the same quantities the real fleet already
broadcasts (see `docs/COMMS_INTERFACE.md`): normalised goal bearing (×2),
normalised distance to goal, stall counter, fleet size, local peer density.
Anything a policy is trained on must be something a real AMR can actually
receive.

**Why lifelong rather than one-shot MAPF:** one-shot rewards makespan, which a
degenerate policy games by parking. Lifelong measures a steady-state delivery
rate — the same reason `test_tasks_actually_complete` exists in the main repo.

**Why conflicts are rejected rather than penalised:** a policy cannot buy
throughput by accepting collisions. Congestion appears as lost throughput, so
all methods are compared at identical (zero) collision cost.

## 3. Baselines

| policy | description |
|---|---|
| `random` | uniform actions — sanity floor |
| `greedy` | move to reduce Manhattan distance; no peer awareness |
| `greedy_jitter` | greedy + random escape when stalled — the naive decentralized baseline |
| **`WHCAStar`** | **Windowed Hierarchical Cooperative A\*** (Silver 2005), window 8, replan every 4, priority = distance-to-goal. Space-time A\* with `(x,y,t)` in the state. The strong classical baseline, and the discrete analogue of what `planner.py` does in continuous space. |

```
  random         throughput=0.021  blocked= 1346
  greedy         throughput=0.073  blocked= 9261     <- livelocks
  greedy_jitter  throughput=0.333  blocked= 7080
  WHCA*          throughput=3.188  blocked=    0     <- 10x greedy
```

Note `WHCAStar._astar` keys its closed set on `(x, y, t)`. That is exactly the
fix still outstanding as flaw #2 in the main repo's `planner.py`, which keys on
`(x, y)` only — this folder is a working reference for it.

## 4. Learned policy — current status

Trained by **behaviour cloning** of WHCA\*. The expert sees the whole map; the
student sees a 9×9 window. The gap measures how much coordination survives
removing the global view.

```
118,277 params · 330-dim obs · 23,040 samples from N∈{16,32,48}
held-out action agreement with WHCA*: 87.9%
inference: 0.7 us per agent per step (numpy, single core)
```

**87.9% agreement, and it still loses badly in closed loop:**

```
   N  density   greedy+j    WHCA*  learned  learned/WHCA*
   8     0.7%      0.229    3.375    1.510           45%
  16     1.4%      0.229    3.182    0.719           23%
  32     2.7%      0.245    3.279    0.349           11%
  48     4.1%      0.238    3.234    0.224            7%
  64     5.5%      0.253    3.220    0.134            4%
```

This is the honest result and it is worth stating plainly: **high action
agreement did not transfer.** At N=64 the learned policy is beaten by
`greedy_jitter`. The cause is textbook behaviour-cloning failure — compounding
error under distribution shift. One wrong action puts the agent in a state the
expert never visited, and errors accumulate faster the more agents there are to
get in each other's way.

Do not present this as a working MARL system. It is a correctly-built
scaffold with a measured, negative result.

## 5. What it would take to do properly

1. **DAgger** — re-query the expert on states the *student* visits. This is the
   standard fix for exactly the failure above and usually recovers most of it.
2. **RL fine-tuning** (PPO/IMPALA) from the cloned weights, as PRIMAL does.
   Needs a framework and GPU-hours; out of scope for numpy.
3. **Communication learning** — let agents exchange a learned vector. This is
   where MARL genuinely beats classical, and it maps onto the real peer link.
4. **A harder map** — the crossover is at ~200 agents *here*. Narrow corridors
   would lower it and make the study cheaper.
5. **Recurrence** — an MLP on a 9×9 window cannot represent "I already tried
   that". The stall counter is a crude substitute for memory.

## 6. Files

```
env.py        lifelong MAPF environment, vectorised numpy
baselines.py  greedy, greedy+jitter, WHCA* (space-time A*)
policy.py     numpy MLP + Adam + behaviour-cloning trainer
train.py      distil WHCA* -> policy_marl.npz
run_study.py  scaling table -> results.json
```

```bash
python3 train.py       # ~15 s
python3 run_study.py   # ~60 s
```

## 7. Bottom line for the SIH presentation

Use this folder as **evidence of rigour, not as a feature.** The defensible
claim is:

> We tested whether multi-agent RL would help. On our layout, classical
> prioritized planning stays within 10% of peak throughput from 32 to 192
> agents and only degrades past ~200 — roughly 20% agent density. At our
> deployment size of 3–5 AMRs, a learned policy has no headroom to exploit,
> and our behaviour-cloned baseline confirms it: 87.9% action agreement
> converts to 4–45% of classical throughput. We therefore keep coordination
> deterministic and apply learning where it actually pays — congestion and
> ETA prediction, which feeds a cost function and never touches safety.

That answers "why no deep RL?" with a measurement instead of an opinion, which
is a stronger position than having shipped a policy that underperforms A\*.
