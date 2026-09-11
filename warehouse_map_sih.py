"""
SIH warehouse map  --  built from the provided layout drawing.

WORLD FRAME (as drawn)
    origin at warehouse centre
    x in [-10, +10]   (20 m, West -> East)
    y in [-7.5, +7.5] (15 m, South -> North)

FIXTURES (from the drawing)
    Rack 1  (-6,  2)      Rack 2  ( 2,  2)
    Rack 3  (-6, -2)      Rack 4  ( 2, -2)
    Target  (-6,  0)
    J2 ( 2,  0)   J3 (-3,  4)   J4 ( 5, -4)
    AMR spawns: (0, 4) and (0, 0.25)
    Aisles: vertical x=0, horizontal y=0

ROBOT (from the provided spec)
    body            1.00 m (X) x 0.70 m (Y) x 0.30 m (Z)
    wheel radius    0.18 m      wheel width 0.12 m
    wheel centres   Y = +/-0.43  =>  separation 0.86 m
    OVERALL WIDTH   0.43 + 0.12/2 = 0.49 per side  ->  0.98 m
    circumscribed radius = hypot(1.00/2, 0.98/2) = 0.70 m

ASSUMPTIONS (flagged -- correct these if the drawing says otherwise)
  A1. Rack labels are CENTRE points.
  A2. Rack footprint 5.0 m (X) x 1.0 m (Y). The drawing gives no rack
      dimensions; this is the largest size that leaves the x=0 aisle clear
      given a centre at x=+/-6 and +/-2.  <-- VERIFY
  A3. J1 is not labelled in the drawing. Assumed to be the main crossing
      at (0, 0) where the two aisles meet.  <-- VERIFY
"""

from __future__ import annotations

import math
from dataclasses import dataclass

CELL = 0.25                      # m per grid cell
X_MIN, X_MAX = -10.0, 10.0
Y_MIN, Y_MAX = -7.5, 7.5
W = int((X_MAX - X_MIN) / CELL)  # 80
H = int((Y_MAX - Y_MIN) / CELL)  # 60

FREE, WALL, RACK = 0, 1, 2
ZONE_NARROW, ZONE_OPEN = 0, 1

# ---- robot spec (given) --------------------------------------------------
ROBOT_LEN = 1.00
ROBOT_BODY_W = 0.70
WHEEL_SEP = 0.86
WHEEL_W = 0.12
ROBOT_OVERALL_W = WHEEL_SEP + WHEEL_W          # 0.98 m, wheels included
ROBOT_RADIUS = math.hypot(ROBOT_LEN / 2, ROBOT_OVERALL_W / 2)   # 0.70 m

# ---- fixtures ------------------------------------------------------------
# Rack EXTENTS, derived from constraints rather than assumed.
#
# The drawing shows the x=0 aisle open, so the labelled rack points cannot
# be centres of 5 m racks -- that puts Rack 2 on top of the aisle
# (measured clearance 0.25 m). We instead solve for extents that satisfy:
#   (a) the labelled point lies inside its rack        (-6,2) etc.
#   (b) the x=0 vertical aisle stays clear
#   (c) the y=0 horizontal aisle stays clear
#   (d) racks stop 1 m short of the side walls
AISLE_HALF_X = 1.20              # -> 2.40 m vertical aisle
WALL_MARGIN = 1.00

RACK_X_WEST = (-10.0 + WALL_MARGIN, -AISLE_HALF_X)   # [-9.0, -1.2]
RACK_X_EAST = (AISLE_HALF_X, 10.0 - WALL_MARGIN)     # [ 1.2,  9.0]
RACK_Y_NORTH = (1.5, 2.5)                            # centred on y=+2
RACK_Y_SOUTH = (-2.5, -1.5)                          # centred on y=-2

RACKS = {
    "Rack1": (RACK_X_WEST, RACK_Y_NORTH),    # label (-6,  2) lies inside
    "Rack2": (RACK_X_EAST, RACK_Y_NORTH),    # label ( 2,  2) lies inside
    "Rack3": (RACK_X_WEST, RACK_Y_SOUTH),    # label (-6, -2) lies inside
    "Rack4": (RACK_X_EAST, RACK_Y_SOUTH),    # label ( 2, -2) lies inside
}
RACK_LABELS = {"Rack1": (-6.0, 2.0), "Rack2": (2.0, 2.0),
               "Rack3": (-6.0, -2.0), "Rack4": (2.0, -2.0)}

JUNCTIONS = {"J1": (0.0, 0.0),   # ASSUMPTION A3
             "J2": (2.0, 0.0),
             "J3": (-3.0, 4.0),
             "J4": (5.0, -4.0)}

TARGET = (-6.0, 0.0)
SPAWNS = [(0.0, 4.0), (0.0, 0.25)]


@dataclass(frozen=True)
class Node:
    name: str
    cx: int
    cy: int
    kind: str


class SIHWarehouseMap:
    def __init__(self):
        self.w, self.h = W, H
        self.cell = CELL
        self.grid = [[FREE] * W for _ in range(H)]
        self.zone = [[ZONE_OPEN] * W for _ in range(H)]
        self.nodes: dict[str, Node] = {}
        self.passing_bays: list[tuple[int, int]] = []
        self._build()

    # -- frame conversion --------------------------------------------------

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int((x - X_MIN) / CELL), int((y - Y_MIN) / CELL)

    def to_world(self, cx: int, cy: int) -> tuple[float, float]:
        return X_MIN + (cx + 0.5) * CELL, Y_MIN + (cy + 0.5) * CELL

    # -- construction ------------------------------------------------------

    def _build(self) -> None:
        for x in range(W):
            self.grid[0][x] = self.grid[H - 1][x] = WALL
        for y in range(H):
            self.grid[y][0] = self.grid[y][W - 1] = WALL

        for name, ((x0, x1), (y0, y1)) in RACKS.items():
            cx0, cy0 = self.to_cell(x0, y0)
            cx1, cy1 = self.to_cell(x1, y1)
            for cy in range(max(1, cy0), min(H - 1, cy1 + 1)):
                for cx in range(max(1, cx0), min(W - 1, cx1 + 1)):
                    self.grid[cy][cx] = RACK

        self._classify_zones()
        self.segment_aisles()
        self._place_nodes()

    def _free_run_width_x(self, cx: int, cy: int) -> float:
        n = 1
        x = cx - 1
        while x >= 0 and self.grid[cy][x] == FREE:
            n += 1; x -= 1
        x = cx + 1
        while x < W and self.grid[cy][x] == FREE:
            n += 1; x += 1
        return n * CELL

    def _classify_zones(self) -> None:
        """Narrow = free width below what two robots need to pass."""
        need_two = 2 * ROBOT_OVERALL_W + 0.40      # 2.36 m
        for cy in range(H):
            for cx in range(W):
                if self.grid[cy][cx] != FREE:
                    continue
                self.zone[cy][cx] = (
                    ZONE_NARROW if self._free_run_width_x(cx, cy) < need_two
                    else ZONE_OPEN)

    def _place_nodes(self) -> None:
        defs = [("TARGET", TARGET, "dropoff")]
        for jn, (jx, jy) in JUNCTIONS.items():
            defs.append((jn, (jx, jy), "junction"))
        for name, (wx, wy), kind in defs:
            cx, cy = self.to_cell(wx, wy)
            if self.grid[cy][cx] != FREE:
                cx, cy = self._nearest_free(cx, cy)
            self.nodes[name] = Node(name, cx, cy, kind)

    def _nearest_free(self, cx: int, cy: int) -> tuple[int, int]:
        for r in range(1, 40):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if self.is_free(cx + dx, cy + dy):
                        return cx + dx, cy + dy
        raise RuntimeError("no free cell")

    # -- queries -----------------------------------------------------------

    def is_free(self, cx: int, cy: int) -> bool:
        return 0 <= cx < W and 0 <= cy < H and self.grid[cy][cx] == FREE

    def is_narrow(self, cx: int, cy: int) -> bool:
        return self.is_free(cx, cy) and self.zone[cy][cx] == ZONE_NARROW

    def neighbors(self, cx: int, cy: int):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if self.is_free(cx + dx, cy + dy):
                yield cx + dx, cy + dy

    def segment_aisles(self) -> None:
        """Flood-fill connected narrow cells into corridor segments."""
        self.aisle_id = [[-1] * W for _ in range(H)]
        seg = 0
        for sy in range(H):
            for sx in range(W):
                if not self.is_narrow(sx, sy) or self.aisle_id[sy][sx] != -1:
                    continue
                stack, cells = [(sx, sy)], []
                self.aisle_id[sy][sx] = seg
                while stack:
                    cx, cy = stack.pop()
                    cells.append((cx, cy))
                    for nx, ny in self.neighbors(cx, cy):
                        if self.is_narrow(nx, ny) and self.aisle_id[ny][nx] == -1:
                            self.aisle_id[ny][nx] = seg
                            stack.append((nx, ny))
                if len(cells) < 3:
                    for (cx, cy) in cells:
                        self.aisle_id[cy][cx] = -1
                else:
                    seg += 1
        self.n_aisles = seg

    def aisle_at(self, cx: int, cy: int) -> int:
        if 0 <= cx < W and 0 <= cy < H:
            return self.aisle_id[cy][cx]
        return -1

    def add_passing_bays(self, xs=(-5.0, 5.0)) -> None:
        """
        Notch the racks to create passing bays.

        REQUIRED for this layout: the 2.0 m central aisle cannot fit two
        robots abreast (they need 2.36 m), so a head-on meeting there has
        no resolution unless one robot has somewhere to pull aside into.
        """
        depth = int(round(1.2 / CELL))
        width = int(round(1.4 / CELL))
        for wx in xs:
            for wy, sign in ((2.0, +1), (-2.0, -1)):
                cx, cy = self.to_cell(wx, wy)
                for dx in range(-width // 2, width // 2 + 1):
                    for dd in range(depth):
                        ny = cy + sign * dd
                        if 0 < ny < H - 1 and 0 < cx + dx < W - 1:
                            if self.grid[ny][cx + dx] == RACK:
                                self.grid[ny][cx + dx] = FREE
                self.passing_bays.append((cx, cy))
        self._classify_zones()
        self.segment_aisles()

    def nearest_bay(self, cx: int, cy: int):
        if not self.passing_bays:
            return None
        return min(self.passing_bays,
                   key=lambda b: abs(b[0] - cx) + abs(b[1] - cy))

    def clearance_at(self, wx: float, wy: float) -> float:
        cx, cy = self.to_cell(wx, wy)
        return self._free_run_width_x(cx, cy)

    # -- feasibility analysis ---------------------------------------------

    def aisle_report(self) -> dict:
        """
        Measure the corridors that matter and check them against the
        ACTUAL robot footprint. This is the check that decides whether the
        layout is even drivable before any coordination logic runs.
        """
        one = ROBOT_OVERALL_W + 0.20        # one robot + margin
        two = 2 * ROBOT_OVERALL_W + 0.40    # two passing + margin

        # vertical aisle at x=0, measured between the rack bands
        vert = self.clearance_at(0.0, 2.0)
        # horizontal aisle at y=0, measured vertically at x=-6 (where the
        # racks actually flank it -- measuring at x=0 samples the vertical
        # corridor instead and reports a meaningless 14.5 m)
        cx, cy = self.to_cell(-6.0, 0.0)
        n = 1
        y = cy - 1
        while y >= 0 and self.grid[y][cx] == FREE:
            n += 1; y -= 1
        y = cy + 1
        while y < H and self.grid[y][cx] == FREE:
            n += 1; y += 1
        horiz = n * CELL

        return {
            "robot_overall_width_m": round(ROBOT_OVERALL_W, 2),
            "robot_radius_m": round(ROBOT_RADIUS, 2),
            "need_one_robot_m": round(one, 2),
            "need_two_passing_m": round(two, 2),
            "vertical_aisle_x0_m": round(vert, 2),
            "horizontal_aisle_y0_m": round(horiz, 2),
            "vert_fits_one": vert >= one,
            "vert_fits_two": vert >= two,
            "horiz_fits_one": horiz >= one,
            "horiz_fits_two": horiz >= two,
        }

    def verify_labels(self) -> dict:
        """Every labelled rack point must fall inside its own rack."""
        out = {}
        for name, (lx, ly) in RACK_LABELS.items():
            (x0, x1), (y0, y1) = RACKS[name]
            out[name] = bool(x0 <= lx <= x1 and y0 <= ly <= y1)
        return out

    def stats(self) -> dict:
        free = sum(r.count(FREE) for r in self.grid)
        narrow = sum(1 for y in range(H) for x in range(W)
                     if self.is_narrow(x, y))
        return {"grid": f"{W}x{H} @ {CELL} m",
                "free_cells": free,
                "narrow_cells": narrow,
                "narrow_fraction": round(narrow / max(1, free), 3)}

    def render_ascii(self) -> str:
        sym = {FREE: ".", WALL: "#", RACK: "="}
        out = [[sym[self.grid[y][x]] for x in range(W)] for y in range(H)]
        for n in self.nodes.values():
            out[n.cy][n.cx] = "T" if n.kind == "dropoff" else n.name[1]
        for (sx, sy) in SPAWNS:
            cx, cy = self.to_cell(sx, sy)
            out[cy][cx] = "A"
        return "\n".join("".join(r) for r in reversed(out))   # north up


if __name__ == "__main__":
    import json
    m = SIHWarehouseMap()
    print(m.render_ascii())
    print("\n", json.dumps(m.stats(), indent=2))
    print("\nAISLE FEASIBILITY vs ACTUAL ROBOT FOOTPRINT")
    print(json.dumps(m.aisle_report(), indent=2))
