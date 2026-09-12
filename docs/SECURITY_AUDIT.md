# Vulnerability Audit — peer message path and coordination layer

Scope: the 3–5 robot production configuration. Every finding below was
**reproduced empirically**, not inferred from reading. Line numbers are from
the commit that introduced the networked auction.

The threat model that matters here is not a hacker. It is **one faulty robot**
— bad IMU, stuck serialiser, half-flashed firmware, a node restarting mid-run.
Every finding below is reachable by a peer that is merely broken. That the same
vectors are also open to a malicious peer is secondary, because a warehouse
DDS domain has no authentication today.

| # | Finding | Severity | Trigger |
|---|---|---|---|
| V1 | Malformed packet crashes the receive loop | **HIGH** | 1 packet |
| V2 | No sanity bounds on bid cost | **HIGH** | 1 packet |
| V3 | `Claim` / `Release` spoofing | **HIGH** | 1 packet |
| V4 | Reservation-table poisoning (planner DoS) | **HIGH** | 1 packet |
| V5 | No replay protection — `header.seq` unused | MED | replayed packet |
| V6 | Unbounded per-robot state growth | MED | time |
| V7 | Battery is monotonic; no recharge path | MED | time |
| V8 | Fleet size hard-capped at 5 | LOW | config |
| V9 | `comms.stats()["loss_pct"]` overstates by (n−1)× | LOW | reporting |

---

## V1 — a single malformed packet crashes the robot. **HIGH**

`Robot.receive()` constructs dataclasses straight from wire payloads:

```python
d["header"] = Header(**d["header"])     # KeyError if absent
tasks.append(Task(**d))                 # TypeError on unknown field
self._on_claim(pkt.src, pkt.payload["task_id"], now)   # KeyError
```

Measured:

```
missing header   -> KeyError: 'header'     *** robot dies ***
empty payload    -> KeyError: 'task_id'    *** robot dies ***
extra field      -> accepted silently
wrong types      -> accepted silently  (task_id="x", cost=None, feasible="yes")
```

There is no try/except anywhere in the receive path, so the exception
propagates out of `Simulation.step()` and takes down the whole tick. On
hardware this is a remotely-triggerable node crash. Worse, the two *silent*
cases are arguably more dangerous: a `cost=None` bid enters `pending_bids`
and corrupts the auction without any signal.

**Fix.** Validate before construct. One `parse(msg_type, payload)` helper that
whitelists fields, checks types and ranges, and drops the packet on failure —
never raises into the control loop. Log and count drops so a flapping peer is
visible.

## V2 — no sanity bounds on bid cost. **HIGH**

`resolve_auction` filters only `feasible and c < INF_COST`:

```
bids {1:(10.0,True), 2:(-1e9,True)}         -> winner 2
bids {1:(10.0,True), 2:(-inf,True)}         -> winner 2
bids {1:(10.0,True), 2:(nan ,True)}         -> winner 1   (NaN compares False)
```

A robot with a divide-by-zero in its cost function bids `-inf` and **wins every
task in the fleet, permanently**, while being physically unable to do any of
them. This needs no malice — it is one unchecked division away.

**Fix.** Reject any bid that is not finite and within `[0, INF_COST)`. Treat a
non-finite bid as infeasible and count it. NaN must be excluded explicitly:
`not (0.0 <= c < INF_COST)` catches it, `c < INF_COST` does not.

## V3 — `Claim` and `Release` are unauthenticated. **HIGH**

`_on_claim` trusts `pkt.src` and applies the lowest-id tie-break:

```
robot2 holds task 77
forged CLAIM(src=1, task=77)  ->  task=None, owner=1     *** STOLEN ***
```

The lowest-id rule that correctly resolves genuine split-brain also means
**any peer claiming a low id wins every contest.** Asserting `src=1` is enough.
A forged `Release` is equally cheap:

```
robot1 believes task 88 owned by robot2
forged RELEASE(src=2, task=88) -> claimed cleared, task re-auctioned
```

Note the asymmetry: a forged claim from a *higher* id is harmless
(`min(1,99)=1`), so the exposure is entirely to low-id impersonation.

**Fix.** For the SIH demo, bind `src` to the transport identity in
`CommsMediator` and reject any packet whose `header.robot_id` disagrees with
its transport `src`. For hardware, this is what DDS Security (authentication +
message signing) is for — do not hand-roll it.

## V4 — reservation-table poisoning stalls every planner. **HIGH**

`Intent.reservations` and `Intent.path_cells` have no length cap, and
`ReservationTable.occupancy()` scans a peer's list linearly on every cell
query. One peer sending 200 000 reservations:

```
occupancy() worst case (cache miss): 1.32 ms per call
A* expands up to 60 000 nodes      -> 79.5 s for ONE plan
```

The fleet does not crash; it goes catatonic. Every robot's replan blocks, and
because the safety supervisor still runs, they brake and sit. This is
indistinguishable from the deadlock failure at the metric level, which makes
it hard to diagnose in the field.

**Fix.** Cap `reservations` and `path_cells` on receive (the 10 s horizon is
already ~24 cells; a cap of 64 is generous). Drop over-length Intents and count
them. Index the table by `(cx,cy)` instead of scanning per peer.

## V5 — `header.seq` is never checked. MED

`seq` is populated by `wrap()` and read nowhere — `'seq' in
inspect.getsource(Robot.receive)` is `False`. A replayed Intent is accepted as
current. The only defence is staleness decay, and that keys off **local arrival
time**, not the sender's stamp, so a replayed packet looks brand new.

**Fix.** Track `last_seq[src]` and drop non-increasing sequences. Cheap, and it
also catches duplicate delivery.

## V6 — per-robot state grows without bound. MED

After 120 s with 3 robots: `known_tasks=15 pending_bids=15 claimed=12`. Nothing
is ever pruned. Over an 8-hour shift at one task per 8 s that is ~3 600 entries
per dict per robot, plus every peer's reservation list retained in
`ReservationTable._res`.

**Fix.** Evict completed/claimed task state after a TTL. `ReservationTable`
already decays weights — drop the entry when `w < 0.05` instead of skipping it.

## V7 — battery is monotonic; there is no recharge. MED

`Robot.step` only ever subtracts (`self.battery -= move * 0.0008`). There is a
`RobotMode.CHARGING` and chargers exist as map nodes, but no code path routes
to one or restores charge. `evaluate_task` refuses to bid below 0.25 SoC, so a
long-running fleet monotonically degrades until **every robot is permanently
ineligible** and throughput goes to zero. Not visible in a 180 s benchmark
(battery ends ≈0.93); guaranteed over a real shift.

**Fix.** Charging behaviour, plus a benchmark long enough to expose it.

## V8 — fleet size hard-capped at 5. LOW

`Simulation.__init__` holds five literal start cells; `n_robots=6` raises
`IndexError: list index out of range`. Blocks the N>32 scaling work.

**Fix.** Generate starts from free cells when the literal list is exhausted.

## V9 — reported packet loss is wrong. LOW

`dropped` counts per-destination, `sent` counts per-packet, so broadcast
inflates the ratio by (n_robots − 1):

```
n=2 configured 20.0% -> reported 20.44%   (1x)
n=4 configured 20.0% -> reported 62.51%   (2.99x)
n=5 configured 20.0% -> reported 82.42%   (3.99x)
```

Injection is correct; only the report is wrong. It errs *flatteringly* —
it implies the fleet survives 62% loss when it survives 20%.

**Fix.** Denominator should be `delivered + dropped`.

---

## Not in this audit

Already tracked in `CLAUDE.md` / `ENGINEERING_LOG.md`: the terminal fleet
deadlock (13/20 seeds, never recovers), 1-D zone classification on
`warehouse_map.py`, omniscient LiDAR, instant heading snap, space-time A\*
missing its time dimension, and `run_sih_layout.py` raising `TypeError`
before it runs.

## Suggested order

1. **V1 + V2** — together they are ~40 lines of validation and they close the
   two cheapest total-failure modes. Do these before any hardware bring-up.
2. **V4** — cap on receive. Another ~10 lines.
3. **V3** — transport-identity binding for the demo; DDS Security for hardware.
4. **V5, V6, V7** — before any multi-hour run.
5. **V8, V9** — housekeeping; V8 blocks the scaling study.
