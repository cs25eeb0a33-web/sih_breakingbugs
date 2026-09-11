"""
Run the coordination stack on the SIH layout from the drawing.

Two configurations compared:
    AS DRAWN        exactly the layout as given
    + PASSING BAYS  rack notches added at x = +/-5

The comparison exists because the central aisle measures 2.00 m while two
of these robots need 2.36 m to pass. A head-on meeting there has no
resolution unless one robot can pull aside.
"""

from __future__ import annotations

import json

from sim2d import Simulation
from warehouse_map_sih import (JUNCTIONS, ROBOT_OVERALL_W, ROBOT_RADIUS,
                               SPAWNS, TARGET, Node, SIHWarehouseMap)


def build_nodes(m: SIHWarehouseMap):
    """Junctions act as pickups, TARGET as the dropoff."""
    picks, drops = [], []
    for name, n in m.nodes.items():
        if n.kind == "junction":
            picks.append(n)
        elif n.kind == "dropoff":
            drops.append(n)
    # a second drop point so routes actually cross
    cx, cy = m.to_cell(7.0, 0.0)
    drops.append(Node("DROP_E", cx, cy, "dropoff"))
    return picks, drops


def make_sim(bays: bool, seed: int, n_robots: int, loss: float = 0.0,
             speed_adapt: bool = True, congestion: bool = True):
    m = SIHWarehouseMap()
    if bays:
        m.add_passing_bays()
    starts = []
    for (sx, sy) in SPAWNS:
        starts.append(m.to_cell(sx, sy))
    # extra spawns for 3-5 robots, placed in open floor
    for extra in ((-8.0, 5.0), (8.0, 5.0), (-8.0, -5.0)):
        starts.append(m.to_cell(*extra))
    return Simulation(n_robots=n_robots, seed=seed, wmap=m, starts=starts,
                      robot_radius=ROBOT_RADIUS, task_nodes=build_nodes(m),
                      loss_rate=loss, speed_adapt=speed_adapt,
                      congestion=congestion)


def run_cfg(label: str, bays: bool, seeds, n_robots=3, **kw):
    rows = [make_sim(bays, s, n_robots, **kw).run(duration=180, task_interval=10)
            for s in seeds]
    n = len(rows)
    done = sum(r["tasks_completed"] for r in rows) / n
    tt = [r["avg_task_time"] for r in rows if r["avg_task_time"] > 0]
    return {
        "config": label,
        "robots": n_robots,
        "tasks_completed": round(done, 1),
        "avg_task_time": round(sum(tt) / len(tt), 1) if tt else 0.0,
        "collisions": sum(r["collisions"] for r in rows),
        "deadlocks": round(sum(r["deadlocks"] for r in rows) / n, 1),
        "full_stops": round(sum(r["full_stops"] for r in rows) / n, 1),
        "time_stopped": round(sum(r["time_stopped"] for r in rows) / n, 1),
    }


def main():
    seeds = [1, 2, 3, 4, 5]

    print("=" * 72)
    print("SIH LAYOUT  --  geometry check against the ACTUAL robot footprint")
    print("=" * 72)
    m = SIHWarehouseMap()
    rep = m.aisle_report()
    print(json.dumps(rep, indent=2))
    print(f"\n  rack labels inside their racks: {m.verify_labels()}")

    print("\n  VERDICT")
    print(f"    one robot in the central aisle : "
          f"{'OK' if rep['vert_fits_one'] else 'FAIL'}")
    print(f"    two robots passing             : "
          f"{'OK' if rep['vert_fits_two'] else 'FAIL  <-- design issue'}")

    print("\n" + "=" * 72)
    print("FLEET RUN  --  3 robots, 5 seeds, 180 s")
    print("=" * 72)

    results = [run_cfg("as drawn", False, seeds, 3),
               run_cfg("+ passing bays", True, seeds, 3)]

    hdr = f"{'config':<18}{'tasks':>7}{'task_s':>9}{'stops':>8}{'stopped_s':>11}{'dlk':>7}{'coll':>6}"
    print("\n" + hdr)
    print("-" * len(hdr))
    for r in results:
        print(f"{r['config']:<18}{r['tasks_completed']:>7.1f}"
              f"{r['avg_task_time']:>9.1f}{r['full_stops']:>8.1f}"
              f"{r['time_stopped']:>11.1f}{r['deadlocks']:>7.1f}"
              f"{r['collisions']:>6}")

    a, b = results
    if a["tasks_completed"] > 0:
        gain = 100 * (b["tasks_completed"] - a["tasks_completed"]) \
            / a["tasks_completed"]
        print(f"\n  passing bays change throughput by {gain:+.0f}%")

    print("\n" + "=" * 72)
    print("BASELINE COMPARISON on this layout (3 robots, with bays)")
    print("=" * 72)
    base = run_cfg("stop-and-wait", True, seeds, 3,
                   speed_adapt=False, congestion=False)
    full = run_cfg("full system", True, seeds, 3)
    print(f"\n{'arm':<18}{'tasks':>7}{'task_s':>9}{'stops':>8}{'coll':>6}")
    print("-" * 48)
    for r in (base, full):
        print(f"{r['config']:<18}{r['tasks_completed']:>7.1f}"
              f"{r['avg_task_time']:>9.1f}{r['full_stops']:>8.1f}"
              f"{r['collisions']:>6}")
    if base["avg_task_time"] > 0 and full["avg_task_time"] > 0:
        g = 100 * (base["avg_task_time"] - full["avg_task_time"]) \
            / base["avg_task_time"]
        print(f"\n  task time {g:+.1f}% vs stop-and-wait on the SIH layout")

    print("\n" + "=" * 72)
    print("COMMS BLACKOUT on the SIH layout")
    print("=" * 72)
    sim = make_sim(True, 3, 3)
    for _ in range(500):
        sim.step()
    print(f"\n  t={sim.t:.0f}s  degraded={sum(r.degraded for r in sim.robots)}/3"
          f"  collisions={sim.metrics.collisions}")
    sim.comms.cut_everything()
    for _ in range(700):
        sim.step()
    print(f"  >>> ALL LINKS CUT <<<")
    print(f"  t={sim.t:.0f}s  degraded={sum(r.degraded for r in sim.robots)}/3"
          f"  collisions={sim.metrics.collisions}")

    with open("results/sih_layout_results.json", "w") as f:
        json.dump({"geometry": rep, "configs": results,
                   "baseline": base, "full": full}, f, indent=2)
    print("\nwrote results/sih_layout_results.json")


if __name__ == "__main__":
    main()
