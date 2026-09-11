# Decentralized Edge-AI Fleet Coordination for Warehouse AMRs
## Complete Technical Architecture — SIH PS 26123

**Scope:** 3–5 AMRs, simulated first, hardware-transferable.
**Design principle preserved throughout:** the server broadcasts and observes; the robots decide.

---

# 0. EXECUTIVE SUMMARY — WHAT I'M CHANGING IN YOUR PLAN

You asked me to be critical rather than agree. Five substantive disagreements, in order of importance:

### 0.1 ❌ Drop D* Lite. Use A* replanning.

D* Lite's entire advantage is cheap incremental repair when **a few edge costs change locally**. Your cost function includes congestion, which changes **globally every time a peer broadcasts** (5–10 Hz). Under global cost churn, D* Lite's repair cost approaches or exceeds a fresh A* search, and you've paid 5× the implementation complexity and a much harder debugging story for nothing.

On a 200×200 grid, A* completes in **1–8 ms on a Pi 5**. You are replanning at ≤2 Hz. There is no performance problem to solve.

**Verdict:** plain A* over a space-time graph, replanned on event. Delete D* Lite from the design entirely.

### 0.2 ⚠️ EPH/MARL must come off the critical path — but you can keep real Edge-AI

Honest assessment of EPH for your case:

| EPH assumption | Your system | Gap |
|---|---|---|
| Discrete grid, synchronous timesteps | Continuous space, async, ORCA velocities | Severe |
| Unit-speed agents, discrete actions | Variable speed, diff-drive kinematics | Severe |
| Trained on dense grids, 8–128+ agents | 3–5 agents, sparse | Benefit ≈ 0 |
| Needs training infrastructure + time | You have hackathon timelines | Blocking |

**MARL's advantage appears at high agent density.** At 3–5 agents in a warehouse, prioritized space-time planning is *near-optimal* and a learned policy cannot meaningfully beat it. You would spend 60% of your effort for a component that doesn't move your metric, and which you then can't explain crisply to judges.

**But you shouldn't drop AI — the PS says "Edge-AI."** Here is the honest, defensible substitution:

> **Replace full MARL policy control with a small supervised *learned congestion/ETA predictor* running onboard, plus an optional EPH-style deadlock-escape policy behind a rule-based fallback.**

Why this is strictly better for you:
- **Trains supervised in minutes, not GPU-days.** You self-generate data by logging your own sim: (broadcast peer intents + map region) → actual traversal time. Standard regression.
- **It's genuinely useful.** Your planner's congestion cost is otherwise a hand-tuned guess. A learned predictor of "how long will this corridor *actually* take given these peer intents" directly improves the metric you're graded on.
- **~30k-parameter MLP/GRU → <1 ms inference on a Pi 5.** Real edge inference, honestly claimed.
- **It never touches safety.** It feeds a cost function. ORCA and the safety supervisor remain deterministic.
- **It's explainable to judges in one sentence**, which MARL is not.

Keep EPH as a **pluggable policy module** with a rule-based default, so you can demo A/B ("here's our system with learned escape vs. rule-based escape") if time permits. Cite EPH properly (Tang, Berto, Park — KAIST OMELET). Do not claim you invented it.

### 0.3 ❌ CBBA is over-engineered for 5 robots. Use a consensus sealed-bid auction.

CBBA exists to solve **multi-task bundle assignment under inconsistent information** across many agents. Its complexity is the bundle-construction and consensus-conflict-resolution rulebook.

For 3–5 robots and one task at a time, this collapses to something you can write in ~80 lines:

1. Server broadcasts task.
2. Every robot computes its own bid, broadcasts it.
3. After a fixed window (`T_bid = 300 ms`), **every robot independently computes `argmin` over all received bids.**
4. Winner broadcasts `CLAIM`. Others record it.
5. If no `CLAIM` within `T_claim`, re-run excluding the presumed winner.

There is no auctioneer. Every robot runs the same deterministic selection over the same bid set. That *is* decentralized consensus, and it's far easier to demo and defend than CBBA's bundle logic.

**Verdict:** consensus sealed-bid auction with deterministic tie-break. Mention CBBA in "future work / scaling."

### 0.4 ⚠️ ORCA will fail in your narrow aisles — you must gate it

This is the failure mode nobody anticipates. ORCA works by finding a velocity in the intersection of half-planes. **In a corridor narrower than ~2.5 robot diameters, that feasible set is empty or near-empty for head-on encounters.** Result: ORCA outputs zero velocity or oscillates. Your "no unnecessary stops" requirement breaks precisely where you most want to demo it.

**Fix — spatial gating:**

| Zone | Coordination mechanism |
|---|---|
| Open areas, intersections, wide aisles | **ORCA active** (its actual design domain) |
| Narrow aisles (width < 2.5·D) | **ORCA bypassed.** Corridor reservation protocol + speed adaptation handles it. ORCA reduced to emergency-brake only. |

Head-on conflict in a one-wide aisle is a **routing and reservation problem, not a local-avoidance problem.** No reciprocal velocity method can solve it. Design your map with **passing bays** at aisle midpoints and reserve corridor direction.

Also: ORCA assumes holonomic agents. For differential drive use **ORCA-DD** (Alonso-Mora et al., 2013) or the standard effective-radius inflation trick (inflate radius by `r_eff = r + v_max·τ_turn`).

### 0.5 ✅ Your multi-robot ROS 2 setup will accidentally cheat — you must enforce information isolation

If all 5 robots run in one DDS domain, every robot can subscribe to every other robot's ground-truth topics. Your "decentralized" system is then quietly reading a shared global blackboard, and your comms-failure demo is meaningless.

**You must insert a comms-mediator node** between peers (detailed in §H). Every peer message goes `robot → /link_out → comms_sim → /amr_N/link_in → robot`. This gives you:
- A single place to inject packet loss, latency, range limits, and dropouts
- A **literal switch to cut links live on stage** for Scenario 8
- Honest decentralization you can prove to a judge who asks

This single node is the difference between a real result and a demo that falls apart under questioning.

---

# A. FINAL ARCHITECTURE

```
                          ┌─────────────┐
                          │  OPERATOR   │
                          │ "Fetch A17  │
                          │  → B03"     │
                          └──────┬──────┘
                                 │
                    ┌────────────▼─────────────┐
                    │     CENTRAL SERVER       │
                    │  (NOT a controller)      │
                    │  • Task broadcast        │
                    │  • Telemetry sink        │
                    │  • Dashboard / logging   │
                    │  • Replay & metrics      │
                    └────────────┬─────────────┘
                        broadcast │ telemetry
             ┌───────────┬────────┼────────┬───────────┐
             ▼           ▼        ▼        ▼           ▼
          ┌─────┐    ┌─────┐  ┌─────┐  ┌─────┐    ┌─────┐
          │AMR 1│◄──►│AMR 2│◄►│AMR 3│◄►│AMR 4│◄──►│AMR 5│
          └─────┘    └─────┘  └─────┘  └─────┘    └─────┘
             ▲──────────▲────────▲────────▲──────────▲
                   PEER-TO-PEER MESH (AMR-Link)
              intents · bids · claims · reroutes · waits
```

**Per-AMR internal stack** (identical software on every robot):

```
┌──────────────────────────────────────────────────────────┐
│  L6  TASK LAYER        Auction · Claim · Release          │  event
│      └─ bid function, feasibility gate                    │  driven
├──────────────────────────────────────────────────────────┤
│  L5  ROUTE LAYER       Space-time A* · congestion cost    │  0.5–2 Hz
│      └─ learned ETA/congestion predictor (Edge-AI)        │
├──────────────────────────────────────────────────────────┤
│  L4  COORDINATION      Conflict prediction · priority     │  5 Hz
│      └─ speed scheduling · deadlock cycle detection       │
│      └─ [pluggable: rule-based | EPH escape policy]       │
├──────────────────────────────────────────────────────────┤
│  L3  LOCAL AVOIDANCE   ORCA (gated by zone)               │  10–20 Hz
├──────────────────────────────────────────────────────────┤
│  L2  SAFETY SUPERVISOR Deterministic. Geometric. Final.   │  30–50 Hz
│      └─ can only ever REDUCE speed. Never increases.      │
├──────────────────────────────────────────────────────────┤
│  L1  CONTROL           Diff-drive velocity controller     │  20–50 Hz
├──────────────────────────────────────────────────────────┤
│  L0  PERCEPTION        LiDAR · odom · IMU · localization  │  10–20 Hz
└──────────────────────────────────────────────────────────┘
      ║                                              ║
   AMR-Link P2P                              Telemetry → server
```

**Critical invariant:** L2 is the only layer with authority to reach the motors, and it can only ever *reduce* the commanded speed. No learned component sits below L4. Say this sentence to judges — it's the answer to "is your AI safe?"

---

# B. COMPONENT RESPONSIBILITIES

| Component | Responsibility | Explicitly NOT responsible for |
|---|---|---|
| **LiDAR** | Raw scan → obstacle points, free space | Classification, semantics |
| **Localization** | AMCL (sim) / pose from encoders+IMU+scan-match | Global map building |
| **Local world model** | Rolling occupancy grid (~10×10 m), dynamic obstacle tracks, peer poses w/ staleness | Being authoritative about remote regions |
| **AMR-Link (P2P)** | Serialize/broadcast state+intent, receive, deduplicate, age-out stale entries | Deciding anything |
| **Task allocation** | Feasibility gate → bid → consensus argmin → claim/release | Taking orders from server |
| **A\* (space-time)** | Route to pickup/dropoff over grid, congestion-weighted, peer-reservation-aware | Local obstacle reaction |
| **Congestion model** | Estimate occupancy + traversal time per cell/time window; learned predictor refines this | Hard safety |
| **Learned ETA predictor** | Regression: peer intents + region → expected traversal time. Feeds L5 cost. | Ever touching velocity directly |
| **Coordination layer** | Predict conflicts on intent trajectories, compute deterministic priority, schedule speeds, detect deadlock cycles | Emergency reaction |
| **EPH module (optional)** | Deadlock-escape action proposal when rule-based fallback stalls | Normal-operation control |
| **ORCA** | Reciprocal velocity avoidance vs. *known* agents, in *open* zones only | Unknown obstacles, narrow aisles |
| **Safety supervisor** | Geometric time-to-collision brake on *any* LiDAR return. Deterministic. | Smooth behavior |
| **Rerouting** | Mark blocked edge, decay belief over time, A* replan, rebroadcast | Waiting for server |
| **Task reassignment** | Detect infeasibility → RELEASE → re-auction | Direct handoff to a chosen peer |
| **Motion control** | Diff-drive (v, ω) tracking of target velocity | Any planning |
| **Dashboard** | Visualize, log, compute metrics | Any control decision |

---

# C. ALGORITHM SELECTION — RECOMMENDATIONS

### C.1 Task allocation

| Option | Verdict |
|---|---|
| CBBA | ❌ Over-engineered. Bundle logic unused at N=5. |
| CBAA | ⚠️ Close, but consensus rulebook still heavier than needed |
| **Consensus sealed-bid auction** | ✅ **RECOMMENDED.** ~80 LOC, deterministic, fully decentralized, trivially explainable |

### C.2 Global planning

| Option | Verdict |
|---|---|
| D* Lite | ❌ Advantage nullified by global congestion churn; 5× complexity |
| Plain A* (distance only) | ❌ Ignores fleet — your explicit requirement |
| **Space-time A\* + congestion cost + prioritized peer reservations** | ✅ **RECOMMENDED** (Silver WHCA*-style, windowed) |
| Full CBS | ❌ Centralized by construction. Use only as **offline oracle** to generate training labels and an upper-bound baseline. |

> **Use CBS cleverly:** run it offline, centralized, to produce near-optimal solutions. Report your decentralized system's gap to CBS. "We achieve 94% of centralized-optimal throughput with zero central coordination" is a *far* stronger claim than beating stop-and-wait.

### C.3 Coordination layer

| Option | Verdict |
|---|---|
| Full EPH / trained MARL | ❌ Not on critical path. Grid-to-continuous gap + training cost + no benefit at N=5 |
| Custom MARL from scratch | ❌ Absolutely not in your timeline |
| **Rule-based prioritized coordination + learned congestion predictor** | ✅ **RECOMMENDED** |
| EPH-inspired escape policy, behind fallback | ⚠️ Stretch goal only, after everything else works |

### C.4 Local avoidance

| Option | Verdict |
|---|---|
| **ORCA (zone-gated) + deterministic safety supervisor** | ✅ **RECOMMENDED** |
| ORCA alone | ❌ Insufficient. Blind to unknown obstacles; fails in narrow aisles |
| DWA only | ⚠️ Non-reciprocal → both robots evade the same way, oscillation |
| ORCA-DD or radius inflation | ✅ Required for diff-drive |

### C.5 Communication

| Option | Verdict |
|---|---|
| ROS 2 DDS raw | ⚠️ Genuinely P2P, but invisible to judges and lets you accidentally cheat |
| Custom UDP from scratch | ⚠️ Real work, real bugs, no extra credit |
| **DDS/Zenoh transport + explicit AMR-Link protocol layer + comms-mediator node** | ✅ **RECOMMENDED** |

Zenoh (`rmw_zenoh`) is worth 30 minutes if it installs cleanly — it's purpose-built for lossy wireless multi-robot networks and strengthens your resilience narrative. If it fights you, use Fast DDS and move on. **The comms-mediator node matters more than the transport choice.**

---

# D. DATA STRUCTURES

All messages carry a common header:

```python
Header:
    robot_id:    uint8
    seq:         uint32        # monotonic, for dedup + loss detection
    stamp:       float64       # sender clock
    ttl_ms:      uint16        # after this, receivers treat as stale
```

### D.1 RobotState — broadcast 5–10 Hz
```python
RobotState:
    header
    pose:           (x, y, theta)
    velocity:       (vx, vy, omega)
    state:          enum{IDLE, BIDDING, TO_PICKUP, LOADED, TO_DROPOFF,
                         CHARGING, BLOCKED, DEGRADED, FAULT}
    battery_soc:    float       # 0..1
    payload_kg:     float
    capacity_kg:    float
    current_task:   uint16      # 0 = none
    health:         uint8       # bitfield: lidar_ok, comms_ok, motors_ok
```

### D.2 Intent — broadcast on change + 2 Hz heartbeat
This is the heart of the system. **Not just position — reserved space-time.**
```python
Intent:
    header
    task_id:        uint16
    path_cells:     [ (cx, cy) ]          # remaining route, grid coords
    reservations:   [ (cx, cy, t_enter, t_exit) ]   # space-time, next H=10s only
    next_waypoint:  (x, y)
    eta_goal:       float64
    nominal_speed:  float
    priority:       uint32                # see §E.5, computed deterministically
```

### D.3 Task — server → all
```python
Task:
    task_id, pickup_node, dropoff_node
    payload_kg, priority_class, deadline (optional)
    announced_at
```

### D.4 Bid — P2P broadcast
```python
Bid:
    header, task_id
    cost:        float32        # lower wins; INF = infeasible
    feasible:    bool
    eta:         float32
```

### D.5 Claim / Release
```python
Claim:    header, task_id, winning_cost
Release:  header, task_id, reason:enum{BATTERY, BLOCKED, FAULT, PREEMPTED}
```

### D.6 Obstacle
```python
Obstacle:
    header
    cells:       [(cx,cy)]
    kind:        enum{STATIC, DYNAMIC, UNKNOWN}
    velocity:    (vx, vy)       # if DYNAMIC
    confidence:  float          # 0..1
    observed_at: float64
```

### D.7 Reroute
```python
Reroute:
    header, task_id
    blocked_cells: [(cx,cy)]
    new_path:      [(cx,cy)]
    new_reservations: [...]
    reason:        enum{BLOCKAGE, CONGESTION, CONFLICT, DEADLOCK}
```

### D.8 Coordination (speed negotiation)
```python
Coordination:
    header
    conflict_with:  uint8       # peer robot_id
    conflict_cell:  (cx, cy)
    conflict_time:  float64
    resolution:     enum{I_YIELD, I_PROCEED, I_SLOW}
    my_new_speed:   float
    my_priority:    uint32
```

### D.9 WaitFor (deadlock graph edge)
```python
WaitFor:
    header
    waiting_for:    uint8       # robot_id I am blocked by, 0 = none
    blocked_since:  float64
    blocking_cell:  (cx, cy)
```

Every robot assembles the fleet-wide wait-for graph locally from these broadcasts.

---

# E. DECISION EQUATIONS

### E.1 Feasibility gate (hard — evaluated before bidding)

A robot bids only if **all** hold:

1. **Capacity:** `m_task ≤ cap_i − payload_i`
2. **Energy:** `E_task(i) + E_return_to_charger + E_margin ≤ E_i`
   where `E_i = SoC_i · E_max`, `E_margin = 0.10 · E_max`
3. **State:** `state_i ∈ {IDLE, TO_DROPOFF with queue slot free}`
4. **Health:** `lidar_ok ∧ motors_ok`

Fail any → broadcast `Bid(cost=INF, feasible=False)`. Broadcasting infeasibility explicitly is important — silence is indistinguishable from packet loss.

### E.2 Bid cost (minimize)

$$b_i(t) = \alpha \cdot \hat{T}_i(t) + \beta \cdot C_{cong}(\pi_i) + \gamma \cdot \frac{E_{task}(i)}{E_i} + \delta \cdot Q_i + \varepsilon \cdot R_{conf}(\pi_i)$$

| Term | Meaning | Suggested weight |
|---|---|---|
| `T̂_i(t)` | Predicted completion time (s) — **from the learned predictor** | α = 1.0 |
| `C_cong(π_i)` | Congestion cost of route (§E.3) | β = 0.5 |
| `E_task/E_i` | **Fraction of *remaining* energy consumed** — self-normalizing | γ = 40.0 |
| `Q_i` | Tasks already queued | δ = 15.0 |
| `R_conf(π_i)` | Predicted conflict risk (§E.4) | ε = 8.0 |

> The `E_task/E_i` form is the key detail. It automatically solves your "5 m away at 8% battery vs 15 m at 80%" case without a special rule: the same absolute energy is a large *fraction* for a depleted robot, so its bid inflates naturally.

**Tie-break:** lowest `robot_id`. Deterministic, so all robots agree without further messages.

### E.3 Congestion cost

For cell `c` entered at predicted time `t`:

$$occ(c,t) = \sum_{j \neq i} \rho^{\,age_j} \cdot \mathbb{1}\big[[t-\tau,\,t+\tau] \cap [t^j_{enter}(c),\, t^j_{exit}(c)] \neq \emptyset\big]$$

$$C_{cong}(\pi) = \kappa \sum_{c \in \pi} occ(c, t_{arr}(c))^2$$

- `ρ = 0.9` per second of age → **stale peer information automatically decays**, which is how you handle degraded comms gracefully rather than with a special case
- Squared term is deliberate: superlinear penalty discourages pile-ups far more effectively than linear
- `τ = 1.5 s` temporal buffer, `κ = 2.0`

### E.4 Conflict risk

Over horizon `H = 10 s`, sampling intent trajectories at `Δ = 0.5 s`:

$$R_{conf}(\pi_i) = \sum_{j\neq i} \sum_{k=0}^{H/\Delta} \exp\!\left(-\frac{k\Delta}{\tau_c}\right) \cdot \mathbb{1}\big[\|p_i(k\Delta) - p_j(k\Delta)\| < d_{safe}\big]$$

`τ_c = 4 s` — imminent conflicts dominate distant ones. `d_safe = 0.8 m` for TurtleBot-scale.

### E.5 Priority (deterministic, lexicographic)

Every robot must compute **identical** priorities from broadcast state alone, or coordination diverges:

$$P_i = \big(\underbrace{c_i}_{\text{task class}},\; \underbrace{-s_i}_{\text{neg. slack}},\; \underbrace{-u_i}_{\text{neg. battery urgency}},\; \underbrace{id_i}_{\text{tiebreak}}\big)$$

- `s_i = deadline_i − ETA_i` (slack; less slack ⇒ higher priority)
- `u_i = 1 − SoC_i` (a robot near depletion gets priority to finish and recharge)
- Compared lexicographically, **lower tuple = higher priority**

Because all four fields ride in `RobotState`/`Intent`, every robot derives the same ordering with no negotiation round-trip. This is what makes prioritized planning work decentrally.

### E.6 Speed adaptation — the anti-stop-and-wait mechanism

This is your headline behavior. Given predicted conflict at cell `c`, where robot `j` has higher priority and occupies `c` over `[t_j^enter, t_j^exit]`:

Lower-priority robot `i` computes a **target arrival time** rather than stopping:

$$t_i^{target} = t_j^{exit} + t_{buffer}, \qquad t_{buffer} = 1.0\ \text{s}$$

$$v_i^{new} = \text{clip}\left(\frac{d_i(c)}{t_i^{target} - t_{now}},\; v_{min},\; v_{max}\right)$$

where `d_i(c)` = remaining path distance to `c`.

**Decision rule:**
```
if v_new ≥ v_min:          → SLOW to v_new          (preferred: keep moving)
elif reroute cost < wait:  → REROUTE
elif passing bay nearby:   → YIELD into bay
else:                      → STOP  (last resort only)
```

Higher-priority robot `j` may **accelerate** toward `v_max` if doing so clears `c` sooner and no downstream conflict exists. This is what produces your "1.0 → 0.5 / 0.7 m/s" behavior, computed rather than hand-tuned.

Apply a **hysteresis band** (only change speed if `|v_new − v_cur| > 0.08 m/s`) to prevent chatter.

### E.7 Deadlock detection

Build directed graph `G = (R, E)` where edge `i → j` exists iff:
- `‖v_i‖ < 0.05 m/s` for `> T_stall = 3.0 s`, **and**
- robot `i`'s next required cell is occupied or reserved by `j`

Every robot builds `G` locally from broadcast `WaitFor` messages. Run **DFS cycle detection at 2 Hz** (trivial at N=5).

On cycle `C` detected:
1. Identify **lowest-priority member** `i* = argmax_{i∈C} P_i` — all robots compute the same `i*`
2. `i*` executes escape, in order of preference:
   a. Reverse to nearest passing bay
   b. Replan with all cycle members' cells marked blocked
   c. Reverse a fixed 1.5 m and re-plan
3. `i*` broadcasts `Reroute(reason=DEADLOCK)`
4. If unresolved after 5 s → escalate: `i*` **and** next-lowest both yield
5. **[Optional EPH hook]** query escape policy before step 2; fall back to deterministic rules if it returns low confidence

Deterministic resolution is essential — a learned policy that resolves deadlocks *sometimes* is worse than a rule that resolves them *always*.

---

# F. CONTROL LOOP FREQUENCIES

Targets for **Raspberry Pi 5** (Jetson Orin Nano has ~3× headroom):

| Layer | Rate | Est. budget | Notes |
|---|---|---|---|
| LiDAR ingest + filter | 10–15 Hz | 8 ms | Sensor-limited |
| Localization (AMCL) | 10 Hz | 12 ms | Or 20 Hz with fewer particles |
| **Safety supervisor** | **30–50 Hz** | **<1 ms** | Pure geometry. Highest rate, lowest cost. |
| Motor control | 20–50 Hz | <1 ms | Diff-drive velocity tracking |
| ORCA | 10–20 Hz | 2–4 ms | N=5 neighbors is trivial for RVO2 |
| Dynamic obstacle tracking | 10 Hz | 5 ms | Cluster + track |
| P2P state broadcast | 5–10 Hz | 1 ms | |
| Intent broadcast | 2 Hz + on-change | 2 ms | Event-driven is what matters |
| Conflict prediction | 5 Hz | 3 ms | |
| Learned ETA inference | 2 Hz | **<1 ms** | ~30k-param MLP |
| Deadlock cycle check | 2 Hz | <1 ms | |
| Global A* replan | 0.5–2 Hz, **event-driven** | 3–8 ms | Don't poll; trigger on blockage/congestion/release |
| Task auction | Event-driven | — | `T_bid = 300 ms` window |
| Telemetry → server | 2–5 Hz | — | Decouple from control entirely |

**Total sustained CPU: roughly 25–40% of one Pi 5 core.** Comfortable. Run L0–L3 in one process with a real-time-ish thread; L4–L6 in a separate lower-priority process so planning can never stall safety.

---

# G. FAILURE HANDLING

| Failure | Behavior |
|---|---|
| **Central server fails** | Nothing changes operationally. Robots complete current tasks, continue P2P coordination, buffer telemetry locally (ring buffer), resync on reconnect. **No new tasks enter the system** — this is the honest limitation; state it openly. |
| **P2P comms lost (one robot)** | Enter `DEGRADED`: inflate `d_safe` 0.8 → 1.5 m, cap speed at `0.4·v_max`, stop broadcasting stale intents, **rely on LiDAR only**. Continue current task if path is clear. If any conflict-prone zone (intersection) is on the route → hold at nearest safe bay and wait for comms. |
| **P2P comms lost (all)** | Every robot independently degrades as above. Fleet continues at reduced throughput with zero collisions. **This is your money demo.** |
| **Peer info goes stale** | Automatic via `ρ^age` decay in §E.3 — no special-case code. After `2·ttl`, drop the peer from the world model and treat its last known region as uncertain (inflate cost, don't treat as free). |
| **One AMR fails (fault)** | Broadcasts `Release(FAULT)` if able. If silent: peers age it out, mark its last known cell as a static obstacle, re-auction its task after `T_orphan = 10 s`. |
| **LiDAR detects blockage** | Mark cells blocked with `confidence`; decay confidence at 0.05/s so temporary obstructions self-clear; A* replan; broadcast `Reroute`. |
| **Dynamic obstacle (human/forklift)** | Track velocity → predict trajectory → if TTC < 3 s, ORCA adjusts; if TTC < 1.2 s, safety supervisor brakes. **Humans always get maximum clearance** — never treat a human as a reciprocal ORCA agent (they don't reciprocate). Treat as a moving obstacle with inflated radius. |
| **Two AMRs choose same route** | Not a problem by itself. Space-time reservations + speed adaptation (§E.6) let both proceed at different speeds. Reroute only if `C_cong` exceeds alternative route cost. |
| **Three AMRs at intersection** | Priority ordering (§E.5) is total and globally consistent → deterministic sequencing. Highest priority proceeds at `v_max`; others compute target arrival times and slow. No round-trip negotiation needed. |
| **Deadlock** | §E.7. Detection ≤ 2 s (3 s stall + cycle check), resolution target < 5 s. |
| **Battery low** | At `SoC < 0.25`: stop bidding on new tasks. At `SoC < 0.15`: `Release(BATTERY)` current task, route to charger. At `SoC < 0.08`: abandon everything, shortest path to charger at reduced speed. |
| **Task impossible** | `Release(reason)` → re-auction. If *no* robot bids feasibly → task goes to `PENDING`, retried every 15 s, and flagged red on the dashboard for operator attention. |

---

# H. ROS 2 IMPLEMENTATION

### H.1 Node graph

```
Per robot (namespace /amr_N):
  amr_N/perception_node        lidar → obstacles, tracks
  amr_N/localization_node      AMCL
  amr_N/world_model_node       occupancy + peer registry + staleness
  amr_N/task_node              feasibility, bid, claim, release
  amr_N/planner_node           space-time A* + congestion + learned ETA
  amr_N/coordination_node      conflict, priority, speed sched, deadlock
  amr_N/orca_node              zone-gated ORCA
  amr_N/safety_node            deterministic supervisor  ← final authority
  amr_N/control_node           diff-drive (v, ω)
  amr_N/link_node              AMR-Link serialize/deserialize

Fleet-level:
  comms_sim_node               ★ THE ISOLATION BOUNDARY
  server_node                  task broadcast + telemetry sink
  dashboard (web)              visualization only
  metrics_node                 logging, benchmark harness
```

### H.2 Topic design — the isolation boundary

```
# --- P2P (mediated — this is what makes it honest) ---
/amr_N/link_out        amr_msgs/LinkPacket    →  comms_sim_node
/amr_N/link_in         amr_msgs/LinkPacket    ←  comms_sim_node

# comms_sim_node holds a link matrix L[i][j] ∈ {0,1} + loss rate + latency.
# Cutting a link on stage = one service call. Nothing else changes.

/comms/set_link        amr_msgs/srv/SetLink   (demo control)
/comms/link_state      amr_msgs/LinkMatrix    (dashboard viz)

# --- Server → fleet (broadcast only, never commands) ---
/fleet/task_announce   amr_msgs/Task

# --- Fleet → server (telemetry only, never acknowledged as control) ---
/amr_N/telemetry       amr_msgs/Telemetry

# --- Strictly robot-internal (never crosses robots) ---
/amr_N/scan, /amr_N/odom, /amr_N/cmd_vel, /amr_N/pose, ...
```

**Enforcement rule for your team:** no node may subscribe to any topic under another robot's namespace. Everything peer-related arrives on `link_in`. Add a CI check or a code-review rule for this — it's the single easiest thing to violate accidentally, and it invalidates your entire result.

**Why `LinkPacket` wraps everything:** one packet type carrying a `type` discriminator + payload means the comms mediator drops *all* peer traffic uniformly. If you use separate topics per message type, you'll forget one and leak information.

---

# I. SIMULATION ARCHITECTURE

### I.1 Use TWO simulators — this is important

| Simulator | Purpose | Why |
|---|---|---|
| **Fast 2D headless sim** (Python, custom, ~600 LOC) | Algorithm development, **all benchmarking** | Runs 100–1000× real-time. You need ~5 baselines × 20 seeds × 10 min scenarios = **~17 hours of wall-clock in Gazebo**. Infeasible. In 2D sim: minutes. |
| **Gazebo Harmonic** (3 robots) | Visual demo, kinematic realism, sim-to-real path | Judges need to *see* robots. 5 robots + Nav2 + LiDAR in Gazebo will struggle on a laptop. |

Share the **same coordination code** between both — only the sensor/actuator interface differs. This is the highest-leverage architectural decision in the whole document. Teams that skip it get no benchmark numbers.

### I.2 Warehouse map

```
Grid: 40 × 30 cells @ 0.5 m = 20 m × 15 m
- 6 shelf blocks, aisles 1.2 m (narrow — forces the hard cases)
- 2 cross-aisles 2.5 m (wide — ORCA active zone)
- 4 intersections (deliberate choke points)
- 3 passing bays at aisle midpoints  ← REQUIRED for deadlock escape
- 4 pickup nodes, 3 delivery stations, 2 chargers
- Designed so ≥2 tasks share an aisle → guarantees the overlapping-path condition
```

Design the map so conflicts are **inevitable, not accidental.** A map where robots rarely meet produces a meaningless 20% number.

### I.3 Robot model
- TurtleBot3 Waffle-class diff drive, `v_max = 0.8 m/s`, `ω_max = 1.8 rad/s`, radius 0.22 m
- 2D LiDAR: 360°, 8 m range, 1° resolution, Gaussian noise σ = 0.02 m
- Battery model: `dE/dt = P_idle + P_move·v + P_load·m` (linear, sufficient)

### I.4 Communication simulation
Model in `comms_sim_node`:
- Range-based: link drops beyond `R_comm = 25 m`
- Packet loss: Bernoulli, configurable 0–30%
- Latency: Gaussian, μ = 20 ms, σ = 10 ms
- **Dead zones:** polygonal regions where link probability → 0.1 (mirrors real warehouse Wi-Fi holes)
- Scriptable outage schedule for repeatable demos

### I.5 Logging
Every robot logs to Parquet/CSV: pose, velocity, state, task, path, decisions, conflicts, stops, deadlocks, CPU, msgs sent/received. `metrics_node` aggregates into the metric table in §K.

---

# J. IMPLEMENTATION ROADMAP

Reordered from your list, with critical-path marked. **Your phases 5 and 6 were in the wrong order** — ORCA must exist before you attempt any learned coordination, or you'll be debugging a neural net and a safety layer simultaneously.

| Phase | Deliverable | Critical? | Notes |
|---|---|---|---|
| **0** | 2D headless sim + map + metrics harness | 🔴 **DO FIRST** | Everything downstream depends on this. Skipping it is the #1 project-killer. |
| **1** | Single AMR: A* + waypoint following + LiDAR stop | 🔴 Yes | |
| **2** | 3 AMRs + AMR-Link + comms_sim + **information isolation** | 🔴 Yes | Verify isolation *now*, not later |
| **3** | Consensus sealed-bid auction | 🔴 Yes | Scenario 1, 2 |
| **4** | Space-time reservations + congestion cost + prioritized A* | 🔴 Yes | Core of your throughput claim |
| **5** | **Speed adaptation (§E.6)** | 🔴 Yes | **This is your headline result.** Scenario 3, 4 |
| **6** | ORCA (zone-gated) + safety supervisor | 🔴 Yes | Scenario 4, 5 |
| **7** | Blockage detection + dynamic rerouting | 🔴 Yes | Scenario 6 |
| **8** | Deadlock detection + deterministic resolution | 🔴 Yes | Scenario 7 |
| **9** | Comms failure + degraded mode | 🔴 **Yes — your differentiator** | Scenario 8 |
| **10** | Task release + reassignment | 🟡 Important | Scenario 9 |
| **11** | Dashboard | 🟡 Important | Judges see this; don't leave to the last night |
| **12** | Benchmarking, all baselines, seeds | 🔴 Yes | This produces your 20% number |
| **13** | Gazebo port, 3 robots | 🟡 Visual demo | |
| **14** | Learned ETA/congestion predictor | 🟢 Stretch | The "Edge-AI" claim |
| **15** | EPH-style escape policy + A/B | 🟢 Stretch | Only if 0–14 are solid |

**If you run short on time, cut in this order:** 15 → 14 → 13 → 10. Never cut 9 or 12.

### Simplifications explicitly permitted
- Perfect localization in 2D sim (AMCL only in Gazebo)
- Grid-discretized motion in 2D sim; continuous only in Gazebo
- Linear battery model
- Humans as scripted moving obstacles, not learned agents

---

# K. TESTING PLAN

| # | Test | Setup | Expected outcome | Pass criterion |
|---|---|---|---|---|
| T1 | Auction correctness | 5 robots, varied battery/capacity/distance | Lowest-cost feasible robot wins | Winner = offline argmin, 100/100 trials |
| T2 | Capacity gate | Task 18 kg, only 2 robots capable | Only capable robots bid | Zero infeasible wins |
| T3 | Battery-aware bid | Robot 5 m @ 8% vs 15 m @ 80% | Far/charged robot wins | Matches §E.2 by hand-calc |
| T4 | Concurrent tasks | 5 tasks announced together | 5 distinct assignments | No double-assignment, no orphan |
| T5 | **Shared aisle** | 2 robots, same corridor, same direction | **Both keep moving at different speeds** | **Zero full stops**; min separation ≥ `d_safe` |
| T6 | Head-on narrow aisle | 2 robots opposed, 1.2 m aisle | One diverts to passing bay or reroutes | No deadlock; resolution < 8 s |
| T7 | 3-way intersection | 3 robots converge | Deterministic priority sequencing | Zero collisions; total wait < stop-and-wait |
| T8 | Dynamic obstacle | Human crosses at 1.2 m/s | Slow → avoid → resume | Min clearance ≥ 1.0 m; no emergency stop |
| T9 | Blocked aisle | Inject obstacle on active path | Local reroute + broadcast | Replan < 500 ms; peers update < 1 s |
| T10 | **Forced deadlock** | Construct R1→R2→R3→R1 | Cycle detected, lowest priority yields | Detection < 2 s, resolution < 5 s, 20/20 trials |
| T11 | Server kill | Kill server mid-run | Fleet continues, telemetry buffers | 100% of in-flight tasks complete |
| T12 | **Single-robot comms loss** | Cut one robot's links | DEGRADED mode, safe continuation | Zero collisions; speed ≤ 0.4·v_max |
| T13 | **Total comms loss** | Cut all P2P | All degrade, fleet still completes | Zero collisions; throughput ≥ 40% nominal |
| T14 | Packet loss sweep | 0/10/20/30% loss | Graceful degradation | No collisions at any level; plot throughput curve |
| T15 | Task reassignment | Drain robot battery mid-task | Release → re-auction → completion | Handover < 3 s |
| T16 | Robot death | Hard-kill one robot | Peers age out, re-auction, avoid its cell | Orphan task recovered < 10 s |
| T17 | Compute budget | Full 5-robot run | Measure CPU/RAM per robot | < 60% of one core sustained |
| T18 | **Benchmark** | All 5 baselines, same map/tasks/seeds | Proposed system fastest | **≥20% vs stop-and-wait, p < 0.05 over ≥20 seeds** |

### K.1 Baseline design (for T18 — get this right)

Hold constant across all arms: map, task set, task arrival times, initial poses, `v_max`, random seeds, obstacle scripts.

| Arm | Config | Purpose |
|---|---|---|
| B0 | A* + stop-and-wait | **The PS baseline.** Full stop on any predicted conflict. |
| B1 | A* + ORCA | Isolates value of reactive avoidance |
| B2 | + congestion-aware routing | Isolates value of route-level coordination |
| B3 | + speed adaptation, no learning | Isolates your headline mechanism |
| B4 | **Full system** | |
| B5 | *(Optional)* Offline CBS oracle | **Upper bound.** Report your gap to centralized-optimal. |

Report mean ± std over ≥20 seeds with a paired test. An ablation table is worth more to technical judges than any single number, because it shows you know *which part* of your system produces the gain.

> **Honest expectation:** ≥20% vs stop-and-wait is **conservative** on a congested map. Stop-and-wait wastes the entire duration of every conflict. You may see 35–60%. If you see <20%, your map isn't congested enough — increase task density or narrow the aisles, and say so transparently in the methodology.

---

# L. SIH PRESENTATION

### L.1 30-second version
> "Warehouse robot fleets today are run by a central server that plans every path. If the network hiccups or the server goes down, the fleet stops. We removed the central planner entirely. Our robots broadcast where they're *going*, not just where they *are*, negotiate directly with each other, and each one decides its own task, route, and speed onboard a Raspberry Pi. Cut the network mid-run and they keep working."

### L.2 1-minute version
> "The operator says *what* needs doing. The robots decide *who* and *how*.
>
> A task is broadcast to all robots. Each one computes its own cost — distance, battery, load, congestion — and bids. Every robot independently picks the same winner from the same bids. No auctioneer.
>
> The winner plans its own route, and here's the key part: it broadcasts its **intended space-time reservations** — not just its position, but which cells it will occupy and when. Peers use that to predict conflicts *before* they happen.
>
> When two robots want the same aisle, conventional systems stop one. We instead compute a target arrival time and **adjust speeds** — 1.0 and 0.7 m/s instead of 1.0 and 0. That's where our 20%+ throughput gain comes from.
>
> Underneath, ORCA handles reciprocal avoidance and a deterministic geometric safety layer has final authority over the motors — no neural network ever commands a wheel.
>
> And because nothing depends on the server, we can kill it live and the fleet doesn't notice."

### L.3 Technical version (for the deep-dive question)
Walk the 7-layer stack in §A top-down, then state the invariant: *"Learned components influence cost functions. Deterministic components control motion. The boundary is L4."*

### L.4 Slide set

**Slide — Architecture.** The §A diagram. One sentence: *"Note what's missing from the server box: a planner."*

**Slide — Novelty (phrase it exactly like this):**
> We do not claim novelty in A*, ORCA, auction protocols, MARL, or EPH — these are established methods and we cite them.
>
> **Our contribution is the system-level integration:** a fully decentralized AMR stack in which task allocation, congestion-aware routing, conflict prediction, speed negotiation, deadlock recovery, and task reassignment all execute onboard edge hardware, with *measured* graceful degradation under partial and total communication loss.
>
> **Specifically novel in combination:**
> 1. Space-time *intent* broadcasting (not just pose) as the coordination primitive
> 2. Speed scheduling as the default conflict response, with stopping as last resort
> 3. Quantified performance envelope across a comms-degradation sweep (0–100% loss)
> 4. Deterministic safety layer with final motor authority, independent of any learned component

This framing is defensible because it is **true**, it is **measurable**, and it **pre-empts** the "isn't this just CBBA + ORCA?" question. Judges respect a team that draws this line themselves. Teams that overclaim get dismantled in Q&A.

**Slide — Workflow.** Task announce → bid → consensus → claim → plan → broadcast intent → detect conflict → adapt speed → execute → (blockage → reroute) → complete.

**Slide — Comparison.** The B0–B5 ablation table. Bar chart: completion time by arm. Second chart: throughput vs. packet loss (this one wins the room — nobody else will have it).

**Slide — Metrics.** The full §24 table, not just collisions. **Explicitly call out:** *"Zero collisions alone is a trivially gameable metric — five parked robots achieve it. We report throughput, unnecessary-stop count, and deadlock recovery time alongside it."* Saying this yourself signals rigor.

### L.5 Demo sequence (rehearse in this exact order)

| # | Scenario | Duration | Judge takeaway |
|---|---|---|---|
| 1 | 5 concurrent tasks auctioned | 45 s | Decentralized allocation works |
| 2 | Shared aisle, speeds diverge on-screen | 30 s | **Not stop-and-wait** |
| 3 | 3-way intersection | 30 s | Deterministic priority, no collision |
| 4 | Human crosses aisle | 20 s | Safety layer |
| 5 | Blocked aisle → live reroute | 30 s | Local autonomy |
| 6 | Forced deadlock → recovery | 30 s | Cycle detection works |
| 7 | **Kill the server** | 45 s | **Fleet doesn't care** |
| 8 | **Cut P2P for one robot** | 45 s | Graceful degradation |
| 9 | Side-by-side vs stop-and-wait | 60 s | The number, visually |

**Put a live comms-state panel on the dashboard** showing the link matrix with links going red as you cut them. Judges remember what they see, and this makes an abstract claim physical.

**Rehearsal note:** scenarios 7 and 8 are your differentiators. Rehearse them until the recovery timing is predictable. A demo that fails at the climactic moment is worse than not attempting it.

---

# M. REFERENCES (verify venue/year before the slide)

| Component | Citation |
|---|---|
| CBS | Sharon, Stern, Felner, Sturtevant — *Artificial Intelligence* 219 (2015) |
| ECBS | Barer, Sharon, Stern, Felner — SoCS 2014 |
| Priority-Based Search | Ma, Harabor, Stuckey, Li, Koenig — AAAI 2019 |
| SIPP | Phillips & Likhachev — ICRA 2011 |
| WHCA* (windowed hierarchical cooperative A*) | Silver — AIIDE 2005 |
| ORCA | van den Berg, Guy, Lin, Manocha — ISRR 2011 |
| ORCA-DD (differential drive) | Alonso-Mora et al. — 2013 |
| DWA | Fox, Burgard, Thrun — *IEEE R&A Magazine* 1997 |
| Contract Net | Smith — *IEEE Trans. Computers* 1980 |
| CBBA | Choi, Brunet, How — *IEEE Trans. Robotics* 2009 |
| MRTA taxonomy | Gerkey & Matarić — *IJRR* 2004 |
| Lifelong MAPF (RHCR) | Li et al. — AAAI 2021 |
| Token Passing | Ma, Li, Kumar, Koenig — AAMAS 2017 |
| PRIMAL | Sartoretti et al. — *IEEE RA-L* 2019 |
| **EPH** | **Tang, Berto, Park — KAIST OMELET** |
| MAPF benchmarks | Stern et al. — SoCS 2019 |
| Grid pathfinding benchmarks | Sturtevant — *IEEE T-CIAIG* 2012 |

---

# N. THE FOUR THINGS MOST LIKELY TO SINK THIS PROJECT

1. **Skipping the fast 2D simulator.** You will not get benchmark numbers from Gazebo in time. Build Phase 0 first.
2. **Accidental information sharing through DDS.** Enforce the `link_in`/`link_out` boundary from day one. Retrofitting isolation is painful and you'll never be sure you got it all.
3. **Spending your best weeks on MARL.** It won't move your metric at N=5, and it competes for time with the comms-resilience demo that actually differentiates you.
4. **Building a map where conflicts are rare.** Your 20% claim requires genuine congestion. Design the map adversarially, then report the task density honestly.
