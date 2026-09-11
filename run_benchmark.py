"""
Benchmark harness.  This produces the number you defend to judges.

Arms (architecture doc K.1) -- all share map, tasks, seeds, speeds:

  B0  stop-and-wait           <- the problem statement's baseline
  B1  + congestion-aware routing
  B2  + speed adaptation      <- the headline mechanism
  B3  full system
  B4  full system @ 20% packet loss   (resilience, not speed)

Run:  python3 run_benchmark.py [n_seeds]
"""

from __future__ import annotations

import json
import statistics as stats
import sys

from sim2d import Simulation

ARMS = {
    "B0_stop_and_wait":  dict(congestion=False, speed_adapt=False, loss_rate=0.0),
    "B1_congestion":     dict(congestion=True,  speed_adapt=False, loss_rate=0.0),
    "B2_speed_adapt":    dict(congestion=False, speed_adapt=True,  loss_rate=0.0),
    "B3_full":           dict(congestion=True,  speed_adapt=True,  loss_rate=0.0),
    "B4_full_20pct_loss": dict(congestion=True, speed_adapt=True,  loss_rate=0.20),
}


def run_arm(name: str, cfg: dict, seeds: list[int],
            n_robots: int = 4, duration: float = 180.0) -> dict:
    rows = []
    for s in seeds:
        sim = Simulation(n_robots=n_robots, seed=s, **cfg)
        rows.append(sim.run(duration=duration))

    def agg(key):
        vals = [r[key] for r in rows]
        return (round(stats.mean(vals), 2),
                round(stats.pstdev(vals), 2) if len(vals) > 1 else 0.0)

    tp_m, tp_s = agg("throughput_per_min")
    tt_m, tt_s = agg("avg_task_time")
    return {
        "arm": name,
        "throughput_per_min": tp_m, "throughput_std": tp_s,
        "avg_task_time": tt_m, "task_time_std": tt_s,
        "tasks_completed": agg("tasks_completed")[0],
        "collisions": sum(r["collisions"] for r in rows),
        "deadlocks": agg("deadlocks")[0],
        "full_stops": agg("full_stops")[0],
        "time_stopped": agg("time_stopped")[0],
        "seeds": len(seeds),
    }


def main() -> None:
    n_seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    seeds = list(range(1, n_seeds + 1))
    print(f"Running {len(ARMS)} arms x {n_seeds} seeds, 4 robots\n")

    results = {}
    for name, cfg in ARMS.items():
        print(f"  {name} ...", flush=True)
        results[name] = run_arm(name, cfg, seeds)

    base = results["B0_stop_and_wait"]
    print("\n" + "=" * 86)
    print(f"{'arm':<22}{'thru/min':>10}{'task_s':>9}{'stops':>8}"
          f"{'stopped_s':>11}{'coll':>6}{'dlk':>6}{'vs B0':>12}")
    print("-" * 86)
    for name, r in results.items():
        if r["avg_task_time"] > 0 and base["avg_task_time"] > 0:
            impr = 100.0 * (base["avg_task_time"] - r["avg_task_time"]) \
                / base["avg_task_time"]
        else:
            impr = 0.0
        tag = "baseline" if name == "B0_stop_and_wait" else f"{impr:+.1f}%"
        print(f"{name:<22}{r['throughput_per_min']:>10.2f}"
              f"{r['avg_task_time']:>9.1f}{r['full_stops']:>8.0f}"
              f"{r['time_stopped']:>11.1f}{r['collisions']:>6}"
              f"{r['deadlocks']:>6.1f}{tag:>12}")
    print("=" * 86)

    full = results["B3_full"]
    if base["avg_task_time"] > 0:
        gain = 100.0 * (base["avg_task_time"] - full["avg_task_time"]) \
            / base["avg_task_time"]
        print(f"\nHEADLINE: task-completion time {gain:+.1f}% vs stop-and-wait")
        print(f"          collisions across ALL runs: {full['collisions']}")
        print(f"          target is >=20% reduction + zero collisions")
        print("\nPASS" if gain >= 20 and full["collisions"] == 0
              else "\nNOT YET AT TARGET")

    with open("benchmark_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("\nwrote benchmark_results.json")


if __name__ == "__main__":
    main()
