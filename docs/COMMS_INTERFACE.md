# Robot-to-Robot Feature Specification — for the comms workstream

**Audience:** whoever implements the peer link (DDS topics / radio framing).
**Contract:** `amr_msgs.py` is frozen. Anything marked **NEW** below is a
proposed addition — raise it before editing, do not just add the field.

Every peer message travels inside one `LinkPacket`. There is deliberately no
topic-per-message-type: the mediator must be able to drop *all* peer traffic
uniformly, or a forgotten channel silently leaks state between robots and the
decentralisation claim dies quietly.

---

## 1. Measured bandwidth budget (4 robots, 180 s, real run)

| msg_type | rate /robot/s | mean B | max B | B/s/robot |
|---|---|---|---|---|
| `INTENT` | 3.81 | 1042 | 1629 | 3967 |
| `ROBOT_STATE` | 3.81 | 448 | 478 | 1705 |
| `BID` | 2.27 | 224 | 225 | 507 |
| `CLAIM` | 0.02 | 205 | 208 | 4 |

```
per robot outbound : 6.2 kB/s  =  49.5 kbit/s
fleet of 4         : 198 kbit/s
fleet of 32        : 1.58 Mbit/s          <-- size the radio for this
inbound per robot  : 148 kbit/s at N=4  (broadcast: N-1 copies)
```

Inbound scales as **N−1 per robot**, so a 32-robot fleet means ~1.5 Mbit/s
*into every robot*. `INTENT` is 64% of the budget and is the field to attack
first if you need headroom — `path_cells` and `reservations` are largely
redundant (one is derivable from the other plus `nominal_speed`).

**Hard requirement:** cap `reservations` and `path_cells` on receive. Today
they are unbounded and one oversized Intent stalls every peer's planner for
~80 s (see `SECURITY_AUDIT.md` V4). 64 entries is generous for a 10 s horizon.

---

## 2. Field inventory

Status: **LIVE** = populated and sent today · **DEAD** = declared in
`amr_msgs.py` but never populated · **NEW** = proposed.

### 2.1 Pose and motion — `RobotState`, 5–10 Hz

| field | type | unit | range | status | why a peer needs it |
|---|---|---|---|---|---|
| `x`, `y` | f32 | m | map | LIVE | position for conflict prediction |
| `theta` | f32 | rad | −π..π | LIVE | facing; a diff-drive robot cannot move sideways |
| `vx`, `vy` | f32 | m/s | ±0.8 | LIVE | extrapolate short-horizon position |
| **`omega`** | f32 | rad/s | ±1.5 | **DEAD** | turn rate — without it peers cannot predict a turning robot's swept area |
| `mode` | enum | — | `RobotMode` | LIVE | intent class (IDLE/TO_PICKUP/…) |
| **`pose_cov`** | f32 | m | 0..2 | **NEW** | 1σ localisation error. A peer must know *how much to trust* the pose; at 0.5 m uncertainty a 0.5 m safety radius is meaningless |

### 2.2 Load and energy — `RobotState`

| field | type | unit | range | status | why a peer needs it |
|---|---|---|---|---|---|
| `battery_soc` | f32 | frac | 0..1 | LIVE | bid weighting; low-SoC peers should be yielded to |
| `payload_kg` | f32 | kg | 0..20 | LIVE | feasibility gate + **braking distance scales with mass** |
| `capacity_kg` | f32 | kg | fixed | LIVE | feasibility gate |
| **`energy_reserve_m`** | f32 | m | ≥0 | **NEW** | metres the robot can still travel. Far more actionable than raw SoC, which peers cannot convert without knowing its efficiency curve |
| **`braking_dist_m`** | f32 | m | 0..3 | **NEW** | stopping distance at current v and mass. This is what a peer actually needs for safe following — not mass, not speed, the derived number |

### 2.3 Turn and manoeuvre — **NEW block**, the "ext turn" ask

None of this is transmitted today, and its absence is why broadcast ETAs are
optimistic (`CLAUDE.md` flaw 3). A robot that must stop, rotate 90° and drive
on arrives seconds later than its own Intent claims.

| field | type | unit | range | status | why a peer needs it |
|---|---|---|---|---|---|
| **`next_turn_angle`** | f32 | rad | −π..π | **NEW** | signed heading change at the next waypoint |
| **`dist_to_next_turn`** | f32 | m | ≥0 | **NEW** | where it happens — lets a peer time its own approach |
| **`turn_duration_est`** | f32 | s | ≥0 | **NEW** | `abs(angle)/omega_max`. The missing term in every ETA |
| **`turning_in_place`** | bool | — | — | **NEW** | in-place rotation sweeps a 1.40 m circle; a peer 0.8 m away is inside it even though both are "stopped" |
| **`swept_radius`** | f32 | m | 0.70 | **NEW** | `hypot(len/2, width/2)`. Constant today, but must be on the wire once the fleet is mixed-model |

`swept_radius` matters more than it looks: peers currently assume a shared
hard-coded radius. The moment one robot carries an overhanging pallet, that
assumption is wrong and nothing detects it.

### 2.4 Path and timing — `Intent`, on change + 2 Hz

| field | type | unit | status | why a peer needs it |
|---|---|---|---|---|
| `path_cells` | list | cell | LIVE | route for congestion cost |
| `reservations` | list | cell+t | LIVE | **the core primitive** — space-time claims |
| `nominal_speed` | f32 | m/s | LIVE | converts cells to times |
| `priority` | tuple | — | LIVE | deterministic right-of-way; both sides must compute the same answer |
| **`eta_goal`** | f32 | s | **DEAD** | when it clears the shared region |
| **`straight_dist_to_goal`** | f32 | m | **NEW** | your "straight distance" — euclidean, no path. Cheap congestion-independent urgency signal, and a sanity check on a corrupted path |
| **`remaining_path_dist`** | f32 | m | **NEW** | actual remaining route length. `remaining/straight` is a detour ratio — a good congestion indicator in one float |
| **`direction_of_travel`** | i8 | — | **NEW** | ±1 along the aisle axis. Corridor reservation needs this explicitly; today it is re-derived from the path by every peer |
| **`aisle_id`** | i16 | — | **NEW** | which corridor segment it occupies — mutual exclusion key |

### 2.5 Inter-robot geometry — **compute locally, transmit the prediction**

Do **not** put "distance between AMRs" on the wire. Each robot derives it from
peer `x,y`, so transmitting it wastes bandwidth and introduces disagreement.
What *is* worth transmitting is the agreed prediction:

| field | type | unit | status | why |
|---|---|---|---|---|
| `conflict_with` | i16 | — | DEAD (`Coordination`) | which peer |
| `conflict_cx/cy`, `conflict_time` | — | cell, s | DEAD | where and when |
| `resolution` | enum | — | DEAD | `I_YIELD` / `I_PROCEED` / `I_SLOW` |
| **`cpa_dist`** | f32 | m | **NEW** | closest point of approach, metres |
| **`cpa_time`** | f32 | s | **NEW** | time to that point — **the single best scalar for "how urgent is this"** |

Derived locally, never transmitted: `d_ij = hypot(xi−xj, yi−yj)`, and the
clearance test `d_ij < swept_i + swept_j + margin`.

### 2.6 Health and degradation — `RobotState`

| field | status | why a peer needs it |
|---|---|---|
| `lidar_ok`, `motors_ok`, `comms_ok` | **DEAD** | all three declared, none populated. A peer with a dead LiDAR is invisible to itself and must be given a **wider** berth, not the standard one |
| **`degraded`** | **NEW** | already tracked internally as `Robot.degraded`, never broadcast — peers cannot tell a degraded robot from a healthy one |
| **`localisation_ok`** | **NEW** | if a robot knows its pose is bad, peers must stop trusting its reservations |

### 2.7 Deadlock — `WaitFor`, DEAD

Declared and never sent. The detector currently builds its graph from
**locally inferred** edges, which is why it detects 2801 cycles and resolves
none. If the wait-for edge were broadcast, every robot would build the same
graph and pick the same yielder.

| field | unit | why |
|---|---|---|
| `waiting_for` | robot_id | who blocks me (0 = nobody) |
| `blocked_since` | s | how long — drives yield priority |
| `blocking_cx/cy` | cell | where the block is |

---

## 3. Situation matrix — what actually matters when

`!` = decisive · `+` = used · blank = irrelevant

| field | open floor | narrow aisle head-on | blind corner | auction | deadlock | comms degraded | low SoC |
|---|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| `x,y,theta` | ! | ! | ! | + | + | | |
| `vx,vy`, `omega` | ! | + | ! | | | | |
| `pose_cov` | + | ! | ! | | | ! | |
| `reservations` | ! | ! | + | + | + | | |
| `priority` | + | ! | + | | ! | | + |
| `eta_goal`, `turn_duration_est` | + | ! | + | ! | | | |
| `straight_dist_to_goal` | | | | ! | | + | + |
| `remaining_path_dist` | + | + | | ! | + | | ! |
| `direction_of_travel`, `aisle_id` | | ! | + | | ! | | |
| `swept_radius`, `turning_in_place` | + | ! | ! | | + | | |
| `braking_dist_m`, `payload_kg` | + | ! | ! | + | | | |
| `battery_soc`, `energy_reserve_m` | | | | ! | + | | ! |
| `cpa_dist`, `cpa_time` | ! | ! | ! | | + | | |
| `waiting_for`, `blocked_since` | | + | | | ! | | |
| `lidar_ok`, `degraded` | + | ! | ! | + | + | ! | |

Read the columns, not the rows. **Narrow-aisle head-on** and **blind corner**
need the most fields and are where the current message set is thinnest —
neither `swept_radius` nor `turning_in_place` nor `braking_dist_m` exists
today, and all three are decisive there.

---

## 4. Validation rules — mandatory, not optional

From `SECURITY_AUDIT.md`. Implement these **in the comms layer**, before any
payload reaches coordination code.

1. **Never raise into the control loop.** Today a packet missing `header`
   throws `KeyError` and kills the robot. Parse defensively, drop on failure,
   count the drop.
2. **Whitelist fields and check types.** `cost=None` and `task_id="x"` are
   currently accepted silently and corrupt the auction.
3. **Range-check every numeric.** Reject non-finite values explicitly —
   `c < INF_COST` does not catch `NaN`, and a `-inf` bid wins every task
   in the fleet forever.
4. **Cap list lengths.** `reservations` ≤ 64, `path_cells` ≤ 128.
5. **Bind `src` to transport identity.** Reject any packet whose
   `header.robot_id` disagrees with the transport `src` — one forged `CLAIM`
   from a lower id currently steals a task.
6. **Enforce `seq` monotonicity per sender.** `header.seq` exists and is read
   nowhere; replayed Intents are accepted as current.
7. **Honour `header.ttl_ms`.** `Header.is_stale()` is written and never called.

---

## 5. Priorities for the comms workstream

1. Validation + caps (§4 items 1–4) — closes two total-failure modes.
2. Populate the DEAD fields already in the contract: `omega`, `lidar_ok`,
   `comms_ok`, `motors_ok`, `eta_goal`. No contract change needed.
3. Start sending `WaitFor`. It is fully specified and unused, and it is the
   missing input to distributed deadlock resolution.
4. Propose the §2.3 turn block and `pose_cov` as a contract amendment.
5. `src` binding and `seq` checks.
