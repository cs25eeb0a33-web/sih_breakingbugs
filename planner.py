"""
Space-time A* with congestion cost and peer reservations.

Architecture doc sections E.3 (congestion) and C.2 (planner choice).

Key correctness point: congestion goes into g() ONLY. The heuristic h()
stays pure distance/v_max so it remains admissible and A* stays correct.
Putting congestion into h is the classic way to silently break optimality.
"""

from __future__ import annotations

import heapq
import math

from amr_msgs import Reservation
from warehouse_map import WarehouseMap, CELL_SIZE

# cost weights (architecture doc E.3)
KAPPA = 2.0        # congestion weight
RHO = 0.9          # staleness decay per second
TAU = 1.5          # temporal buffer, seconds
LAMBDA_BLOCK = 50.0  # blocked-cell penalty


class ReservationTable:
    """Peer space-time claims, with automatic staleness decay.

    Stale peer info decays smoothly instead of being special-cased --
    that is how the system degrades gracefully when comms get patchy.
    """

    def __init__(self) -> None:
        self._res: dict[int, tuple[list[Reservation], float]] = {}
        self._blocked: dict[tuple[int, int], tuple[float, float]] = {}

    def update(self, robot_id: int, reservations: list[Reservation],
               stamp: float) -> None:
        self._res[robot_id] = (reservations, stamp)

    def drop(self, robot_id: int) -> None:
        self._res.pop(robot_id, None)

    def mark_blocked(self, cx: int, cy: int, now: float,
                     confidence: float = 1.0) -> None:
        self._blocked[(cx, cy)] = (confidence, now)

    def blocked_belief(self, cx: int, cy: int, now: float) -> float:
        """Confidence decays 0.05/s so temporary obstructions self-clear."""
        e = self._blocked.get((cx, cy))
        if not e:
            return 0.0
        conf, t = e
        return max(0.0, conf - 0.05 * (now - t))

    def occupancy(self, cx: int, cy: int, t: float, now: float,
                  exclude: int = 0) -> float:
        """Weighted count of peers reserving this cell near time t."""
        total = 0.0
        for rid, (res_list, stamp) in self._res.items():
            if rid == exclude:
                continue
            age = max(0.0, now - stamp)
            w = RHO ** age                      # staleness decay
            if w < 0.05:
                continue
            for r in res_list:
                if r.cx == cx and r.cy == cy:
                    if not (r.t_exit + TAU < t or t + TAU < r.t_enter):
                        total += w
                        break
        return total


def heuristic(cx: int, cy: int, gx: int, gy: int, v_max: float) -> float:
    """Admissible: straight-line distance at max speed. No congestion here."""
    d = math.hypot(gx - cx, gy - cy) * CELL_SIZE
    return d / v_max


def plan(wmap: WarehouseMap, start: tuple[int, int], goal: tuple[int, int],
         table: ReservationTable, robot_id: int, t_start: float, now: float,
         v_nom: float = 0.6, v_max: float = 0.8,
         congestion_aware: bool = True,
         max_expansions: int = 60000) -> list[tuple[int, int]]:
    """
    Returns a list of grid cells from start to goal, or [] if unreachable.

    Set congestion_aware=False to get the plain-A* baseline (B0/B1).
    """
    sx, sy = start
    gx, gy = goal
    if not wmap.is_free(sx, sy) or not wmap.is_free(gx, gy):
        return []
    if start == goal:
        return [start]

    step_time = CELL_SIZE / v_nom
    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (0.0, counter, start))
    came: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}
    closed: set[tuple[int, int]] = set()
    expansions = 0

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expansions += 1
        if expansions > max_expansions:
            break

        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]

        cx, cy = cur
        g_cur = g_score[cur]
        t_here = t_start + g_cur

        for nx, ny in wmap.neighbors(cx, cy):
            if (nx, ny) in closed:
                continue
            t_arrive = t_here + step_time
            cost = step_time

            if congestion_aware:
                occ = table.occupancy(nx, ny, t_arrive, now, exclude=robot_id)
                cost += KAPPA * (occ ** 2)          # superlinear, doc E.3
                cost += LAMBDA_BLOCK * table.blocked_belief(nx, ny, now)

            tentative = g_cur + cost
            if tentative < g_score.get((nx, ny), float("inf")):
                g_score[(nx, ny)] = tentative
                came[(nx, ny)] = cur
                f = tentative + heuristic(nx, ny, gx, gy, v_max)
                counter += 1
                heapq.heappush(open_heap, (f, counter, (nx, ny)))

    return []


def path_to_reservations(path: list[tuple[int, int]], t_start: float,
                         v_nom: float, horizon: float = 10.0
                         ) -> list[Reservation]:
    """
    Convert a path into space-time claims for the next `horizon` seconds.

    This is what gets broadcast in Intent. Publishing only the near horizon
    keeps messages small and avoids over-committing to a distant future that
    will be replanned anyway.
    """
    out: list[Reservation] = []
    step_time = CELL_SIZE / max(0.05, v_nom)
    t = t_start
    for (cx, cy) in path:
        if t - t_start > horizon:
            break
        out.append(Reservation(cx=cx, cy=cy, t_enter=t, t_exit=t + step_time))
        t += step_time
    return out


def path_length_m(path: list[tuple[int, int]]) -> float:
    return max(0, len(path) - 1) * CELL_SIZE


def congestion_cost(path: list[tuple[int, int]], table: ReservationTable,
                    robot_id: int, t_start: float, now: float,
                    v_nom: float) -> float:
    """C_cong for the bid function (doc E.2/E.3)."""
    step_time = CELL_SIZE / max(0.05, v_nom)
    total = 0.0
    t = t_start
    for (cx, cy) in path:
        occ = table.occupancy(cx, cy, t, now, exclude=robot_id)
        total += KAPPA * (occ ** 2)
        t += step_time
    return total
