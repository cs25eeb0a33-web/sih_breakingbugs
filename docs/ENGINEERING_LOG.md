# Engineering Log — Failures, Root Causes, Fixes

Every entry below is a real failure hit during development, with the actual
measurements that exposed it. Kept in the repo deliberately: the debugging
path is evidence of engineering, and several of these bugs are traps any
team building multi-robot coordination will hit.

Each has a regression test in `tests/test_all.py`.

---

## BUG 1 — No safety layer: 105 collisions

**Symptom.** First integrated run completed 17 tasks but logged 105 collisions.

```
tasks_completed: 17     collisions: 105     near_misses: 240
```

**Root cause.** Coordination layers (L4–L6) were implemented, but L2 — the
deterministic geometric safety supervisor — was not. Everything relied on
*predicted* conflicts from broadcast intents. Prediction is not safety:
it fails on timing error, stale data, and anything not broadcast at all.

**Fix.** Added `Robot.safety_supervisor()`. Two properties matter:

- It can only ever **reduce** commanded speed, never raise it.
- It runs on **LiDAR returns, not comms** — so it keeps working when the
  network is dead.

**Result.** 105 → **0 collisions.**

**Lesson.** Prediction and safety are different layers. Safety must depend
on local sensing only, or a comms failure becomes a safety failure.

**Regression:** `test_zero_collisions_nominal`, `test_T13_zero_collisions_under_blackout`

---

## BUG 2 — Braking-only livelock: fleet frozen at 0.56 m

**Symptom.** Collisions hit zero, but throughput collapsed. Robots froze
permanently in pairs.

```
t=150  R1 (16,14) v=0.00   R3 (17,15) v=0.01     <- 0.56 m apart, forever
t=150  R2 (15,22) v=0.00   R4 (15,23) v=0.00
tasks_completed: 7    time_stopped: 320s of 720 robot-seconds (44%)
```

**Root cause.** The only avoidance mechanism was **braking along the path**.
Two robots approaching head-on both slow, both stop just outside the
hard-stop radius, and neither has any mechanism to move *around* the other.
Classic reciprocal livelock. Deadlock detection fired, but the escape
maneuver had nowhere to go because there was no lateral motion primitive at all.

**Fix.** Added `Robot.orca_adjust()` — reciprocal lateral sidestep (L3).
Reciprocity comes from a shared convention: **everyone sidesteps to their own
right**, so both deviate in opposite world directions and slide past.

**Result.** tasks 7 → 9, deadlocks 9 → 4.

**Lesson.** Speed control alone cannot resolve a head-on encounter. You need
a lateral degree of freedom, and the tie-break convention must be shared or
both robots dodge the same way.

**Regression:** `test_T5_slows_instead_of_stopping`, `test_robots_stay_on_free_cells`

---

## BUG 3 — Yield maneuver never executed

**Symptom.** Deadlocks were being *detected* (metric incrementing) but robots
stayed frozen.

**Root cause.** Logic error in `resolve_deadlocks()`:

```python
saved = r.goal
r.goal = bay
if r.replan(self.t):
    r.goal = saved        # <-- BUG: original goal restored immediately
```

The goal was restored on the same tick, so the next replan routed straight
back into the conflict. The robot had no *state* representing "I am
currently yielding."

**Fix.** Added an explicit yield state machine: `yielding`, `saved_goal`,
`yield_deadline`. The original goal is restored only on arrival at the bay
or on timeout.

**Lesson.** A multi-tick maneuver needs persistent state. Setting and
reverting a goal within one tick is a no-op.

**Regression:** `test_T10_detects_3cycle`, `test_yielder_is_lowest_priority_and_agreed`

---

## BUG 4 — Corridor reservation silently never fired

**Symptom.** Implemented corridor reservation to stop robots entering an
occupied narrow aisle. Benchmark output was **byte-identical** before and
after — always a red flag.

Instrumentation:

```
corridor_blocked called: 6662    entering: 166    blocked: 0
```

**Root cause.** Aisle segmentation grouped narrow cells **per column**:

```
aisle_at(14,21) = 19
aisle_at(15,21) = 23      <- same physical aisle, different segment id
aisle_at(16,21) = ...
```

The aisle is 3 cells wide. Treating each column as its own corridor meant
two robots in the same physical aisle never shared a segment id, so the
opposing-direction check could never trigger.

**Fix.** Rewrote `_segment_aisles()` as a 2D flood fill over connected narrow
cells, so a corridor is one segment across its full width.

```
segments: 52 -> 18
aisle_at(14,21) = aisle_at(15,21) = aisle_at(16,21) = 15   OK
```

**Lesson.** A safety rule that never fires is worse than no rule — it creates
false confidence. **Always instrument a new guard to confirm it triggers.**
Identical output after a behavioural change means the code path is dead.

**Regression:** `test_aisle_segments_span_width`

---

## BUG 5 — Peer heading always computed as zero

**Symptom.** After fixing BUG 4, peers were correctly *found* in the target
aisle (36 times) but entry was still never blocked.

```
entering: 166    peers_in: 36    blocked: 0
```

**Root cause.** Peer direction was inferred as:

```python
their_dir = it.path_cells[0][1] - pc[1]
```

But a robot broadcasts `path[path_idx:]`, and `path_cells[0]` is its
**current cell**. So `their_dir` was almost always `0`, and
`my_dir * their_dir < 0` could never be true.

**Fix.** Infer heading from broadcast velocity when moving, otherwise scan
ahead to the first path cell that actually differs in `y`. Also added a
corridor **capacity limit** (≥2 occupants blocks entry) as a second line of
defence that does not depend on heading inference at all.

**Result.** near_misses 1954 → 1142.

**Lesson.** Know exactly what your own message contains. "Next waypoint"
and "current cell" being the same value is easy to miss and silently
disables downstream logic.

---

## BUG 6 — Idle robots parked on dropoff points

**Symptom.** Persistent congestion around delivery stations.

```
top stuck cells: [(14,21): 725, (15,21): 677, (15,22): 614, (15,23): 431]
```

**Root cause.** On task completion a robot stopped where it was — on the
dropoff node, inside a narrow aisle. Every subsequent robot routed to that
same dropoff was blocked by an idle peer that had no reason to move.

**Fix.** On completion, idle robots route to the nearest charger to clear
the aisle.

**Lesson.** Idle behaviour is part of the coordination problem. A robot with
nothing to do is still occupying space.

---

## BUG 7 — Stop metric counted ticks, not events

**Symptom.** `full_stops: 1689` over a 180 s run with 4 robots — implausible.

**Root cause.** The counter incremented every timestep the robot was stopped
(10 Hz), not on each *transition* into a stop. A single 5 s stop counted as 50.

**Fix.** Count only transitions (`if action == "STOP" and self.v > 0.01`).

**Result.** 1689 → **8**, which matches observed behaviour.

**Lesson.** Rate-based counters inflate by the control frequency. This one
mattered because `full_stops` is a headline metric against the stop-and-wait
baseline — the bug would have made our own system look *worse* than it is.

---

## Summary

| # | Bug | Metric before → after |
|---|---|---|
| 1 | No safety layer | 105 → **0** collisions |
| 2 | Braking-only livelock | 7 → 9 tasks, deadlocks 9 → 4 |
| 3 | Yield never executed | deadlocks detected but unresolved → resolved |
| 4 | Per-column aisle segmentation | rule fired 0 times → active |
| 5 | Peer heading always 0 | near-misses 1954 → 1142 |
| 6 | Idle robots blocking dropoffs | removed a 725-tick hotspot |
| 7 | Per-tick stop counting | 1689 → 8 |

**Final:** 0 collisions, +23.0% vs stop-and-wait, 51/51 tests passing.

### The two most transferable lessons

1. **A guard that never fires is worse than no guard.** Bugs 4 and 5 both
   produced silent no-ops that looked like working code. Instrument every
   new rule and confirm it triggers before trusting it.
2. **Identical output after a behavioural change means the code path is
   dead.** Byte-identical benchmark results were the clue that cracked BUG 4.
