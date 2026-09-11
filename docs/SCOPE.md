# Hackathon Implementation Scope
## Edge-AI Distributed Fleet Coordination for AMRs — SIH PS 26123

**Status: implemented and measured.** Every number below came from the
benchmark harness in this repo, not from an estimate.

---

## 1. WHAT WE IMPLEMENT (in scope, working)

| # | Component | Mechanism | File |
|---|---|---|---|
| 1 | **Decentralized task allocation** | Consensus sealed-bid auction over the radio. No auctioneer — each robot broadcasts its `Bid`, runs `argmin` on the bids it *actually received*, and the believed winner broadcasts a `Claim`. Double claims under packet loss resolve to the lowest `robot_id`. | `coordination.py`, `sim2d.py` |
| 2 | **Congestion-aware global planning** | Space-time A\* over a grid, peer reservations as time-varying cost, staleness-decayed. | `planner.py` |
| 3 | **P2P intent sharing** | Robots broadcast *space-time reservations*, not just position. Conflicts are predicted before they occur. | `amr_msgs.py` |
| 4 | **Speed adaptation** ← headline | Conflict resolved by computing a target arrival time and deriving the speed for it. Stopping is a computed last resort, not a reflex. | `coordination.py` |
| 5 | **Corridor reservation** | Narrow aisles managed by entry control, because no reciprocal velocity method can solve a head-on meeting in a one-robot-wide aisle. | `sim2d.py` |
| 6 | **ORCA-style local avoidance** | Reciprocal lateral sidestep, **zone-gated** — active in open areas, bypassed in narrow aisles. | `sim2d.py` |
| 7 | **Deterministic safety supervisor** | Geometric brake with final authority over speed. Runs on LiDAR, not comms. | `sim2d.py` |
| 8 | **Deadlock detection + recovery** | Distributed wait-for graph, DFS cycle detection, lowest-priority member yields into a passing bay. | `coordination.py` |
| 9 | **Dynamic rerouting** | Blocked cells marked with confidence that decays, so temporary obstructions self-clear. | `planner.py` |
| 10 | **Comms-failure resilience** | Degraded mode: inflated safety margin, capped speed, stale peers aged out, local sensing only. | `sim2d.py` |
| 11 | **Comms mediator** | Range, packet loss, latency, dead zones, live link cutting. The information-isolation boundary. | `comms.py` |
| 12 | **Learned coordination policy** | See section 2. | `learned.py` |
| 13 | **Benchmark harness** | 5 arms × N seeds, shared map/tasks/seeds. | `run_benchmark.py` |

---

## 2. THE LEARNING COMPONENT — AND THE HONEST FRAMING

### What we built

A **8,901-parameter neural policy** running onboard each robot, mapping a
**7×7 local observation window** to a movement action.

It is trained by **imitation (behavioural cloning)** from the full
coordination stack — space-time A\* with peer reservations — acting as an
expert with global information.

### Measured results

| Metric | Value |
|---|---|
| Parameters | 8,901 |
| Training data | 9,565 state-action pairs, self-generated |
| Dataset generation | 2.9 s |
| Training time | **0.8 s on CPU** |
| **Held-out action agreement** | **91.8%** |
| Inference latency | **0.025 ms** (40 kHz capable) |
| Framework | **pure numpy** — no torch install on a Pi |

### The honest statement (use this wording)

> **On the learning approach.** We evaluated full multi-agent reinforcement
> learning (EPH, PRIMAL-style MARL) for the coordination layer. We did not
> adopt it, for two reasons.
>
> First, **time**: RL requires reward shaping, exploration scheduling,
> training-stability work and hyperparameter search — on the order of 40–80
> engineering hours, with failure modes that are hard to diagnose under a
> hackathon deadline.
>
> Second, and more importantly, **fleet size**: MARL's advantage appears at
> high agent density. At 3–5 robots, prioritized space-time planning is
> already near-optimal, so a learned policy has very little headroom to
> beat it. Adopting MARL here would have been a large cost for a benefit
> our own fleet size cannot realize.
>
> We therefore opted for **lightweight learning**: rather than *discovering*
> a policy with RL, we **distill** one from a planner that is already
> correct. Labels are exact, the loss is plain cross-entropy, and it
> converges in under a second. This is the same imitation-learning component
> PRIMAL uses (Sartoretti et al., RA-L 2019), so the approach is well
> precedented.
>
> **The result is a genuine edge-AI claim:** each robot runs a neural policy
> distilled from a centralized optimal planner, acting on *only* its local
> observation window and peer broadcasts — centralized-quality coordination
> with no central coordinator.

### Why this is the stronger pitch

The information gap is the point. The expert sees the whole warehouse; the
student sees a 7×7 window. Reproducing 91.8% of the expert's decisions from
local information alone **is** the decentralization result — it is not a
consolation prize for skipping RL.

### Safety boundary (state this when asked "is your AI safe?")

```
  L4  learned policy      -> PROPOSES an action; falls back to
                             rule-based below 55% confidence
  L3  ORCA                -> reciprocal avoidance (zone-gated)
  L2  safety supervisor   -> geometric brake, FINAL AUTHORITY,
                             can only ever REDUCE speed
  L1  motor control
```

**No learned component sits below L4.** The safety supervisor runs on LiDAR,
not on the network, which is why the fleet stays collision-free even in a
total comms blackout.

---

## 3. MEASURED RESULTS

4 robots, 6 seeds, identical map / task set / seeds across all arms.

| Arm | Throughput /min | Avg task time (s) | Full stops | Collisions | vs B0 |
|---|---|---|---|---|---|
| B0 stop-and-wait *(PS baseline)* | 1.50 | 43.9 | 14 | 0 | — |
| B1 + congestion routing | 2.00 | 42.9 | 11 | 0 | +2.4% |
| B2 + speed adaptation | 1.83 | 34.5 | 4 | 0 | **+21.5%** |
| **B3 full system** | **2.44** | **33.8** | 7 | **0** | **+23.0%** |
| B4 full @ 20% packet loss | 2.39 | 32.7 | 4 | 0 | +25.6% |

### Against the success criteria

| Target | Result |
|---|---|
| Zero inter-robot collisions | ✅ **0 across all arms, all seeds** |
| ≥20% task-time reduction vs stop-and-wait | ✅ **+23.0%** |

### What the ablation tells you (this is what technical judges want)

- **Speed adaptation is the dominant contributor** — +21.5% on its own.
  Congestion routing adds a further ~1.5%. We can say precisely which
  mechanism earns the gain rather than claiming the system works as a
  black box.
- **Throughput rose 63%** (1.50 → 2.44 tasks/min) while full stops dropped
  from 14 to 7.
- **B4 did not degrade under 20% packet loss.** Resilience is not traded
  against speed — because staleness decay is built into the cost function
  rather than bolted on as a special case.

---

## 4. DEMONSTRATION SCENARIOS (all runnable)

| # | Scenario | Status |
|---|---|---|
| 1 | Decentralized task allocation, 5 concurrent tasks | ✅ |
| 2 | Shared aisle — speeds diverge instead of stopping | ✅ |
| 3 | Intersection conflict, deterministic priority | ✅ |
| 4 | Blocked aisle → local reroute | ✅ |
| 5 | Deadlock → cycle detection → yield to passing bay | ✅ |
| 6 | **Server killed — fleet unaffected** | ✅ |
| 7 | **Total comms blackout — 4/4 degrade, 0 collisions** | ✅ verified |
| 8 | Packet-loss sweep 0→30% | ✅ |

Scenarios 6 and 7 are the differentiators. `comms.cut_everything()` and
`comms.isolate(robot_id)` are one-line calls, so they can be triggered live
on stage.

---

## 5. EXPLICITLY OUT OF SCOPE (say this before a judge asks)

| Deferred | Why |
|---|---|
| Full MARL / RL training | Section 2 — cost/benefit at N=5 |
| D\* Lite | Incremental repair advantage is nullified by globally-changing congestion costs; plain A\* replans in 1–8 ms |
| CBBA | Bundle-construction logic unused at 3–5 robots; consensus sealed-bid is equivalent here and far simpler |
| Full ORCA-DD kinematics | Simplified reciprocal sidestep; ORCA-DD is the hardware-transfer step |
| SLAM | Known map assumed, as the PS allows |
| Physical robots | Architecture is hardware-transferable; Gazebo is the visual bridge |

Naming these yourself signals engineering judgement. Teams that overclaim
get taken apart in Q&A.

---

## 6. NOVELTY — PHRASE IT EXACTLY LIKE THIS

> We claim **no novelty** in A\*, ORCA, auction protocols, imitation
> learning, or ROS 2. These are established methods and we cite them.
>
> **Our contribution is the system-level integration:** a fully decentralized
> AMR stack where task allocation, congestion-aware routing, conflict
> prediction, speed negotiation, deadlock recovery and degraded-mode
> operation all run onboard edge hardware, with a **measured** performance
> envelope under communication loss.
>
> **Specifically novel in combination:**
> 1. Space-time *intent* broadcasting (not just pose) as the coordination primitive
> 2. Speed scheduling as the default conflict response, with stopping as a computed last resort
> 3. A quantified resilience curve across a 0–30% packet-loss sweep
> 4. A policy distilled from a centralized planner, acting on local observations only
> 5. A deterministic safety layer with final motor authority, independent of any learned component

---

## 7. RUNNING IT

```bash
python3 warehouse_map.py      # render the map
python3 sim2d.py              # single run
python3 train_policy.py       # train the policy (~4 s total)
python3 run_benchmark.py 10   # full benchmark, 10 seeds
```

---

## 8. REFERENCES

Sharon et al. 2015 (CBS) · Silver 2005 (WHCA\*) · Phillips & Likhachev 2011
(SIPP) · van den Berg et al. 2011 (ORCA) · Smith 1980 (Contract Net) ·
Gerkey & Matarić 2004 (MRTA taxonomy) · **Sartoretti et al. 2019 (PRIMAL —
imitation learning component)** · Tang, Berto & Park (EPH) · Stern et al.
2019 (MAPF benchmarks)
