"""
Warehouse grid map.

Designed ADVERSARIALLY on purpose: narrow aisles, few cross-aisles, and
task nodes placed so routes overlap. A map where robots rarely meet
produces a meaningless throughput number -- if your paths don't conflict,
you are not measuring coordination, you are measuring A*.

Geometry
--------
    40 x 30 cells @ 0.5 m  =  20 m x 15 m
    narrow aisles  1.5 m  (3 cells)  -> ORCA gated OFF here
    cross aisles   2.5 m  (5 cells)  -> ORCA active
    passing bays   notches in shelf blocks, needed for deadlock escape
"""

from __future__ import annotations

from dataclasses import dataclass

CELL_SIZE = 0.5
W, H = 40, 30

FREE, WALL, SHELF = 0, 1, 2

# zone classification, drives the ORCA gate
ZONE_NARROW, ZONE_OPEN, ZONE_BAY = 0, 1, 2


@dataclass(frozen=True)
class Node:
    name: str
    cx: int
    cy: int
    kind: str          # "pickup" | "dropoff" | "charger"


class WarehouseMap:
    def __init__(self) -> None:
        self.w, self.h = W, H
        self.grid = [[FREE] * W for _ in range(H)]
        self.zone = [[ZONE_OPEN] * W for _ in range(H)]
        self.passing_bays: list[tuple[int, int]] = []
        self.nodes: dict[str, Node] = {}
        self._build()

    # -- construction ------------------------------------------------------

    def _build(self) -> None:
        # outer walls
        for x in range(W):
            self.grid[0][x] = self.grid[H - 1][x] = WALL
        for y in range(H):
            self.grid[y][0] = self.grid[y][W - 1] = WALL

        # Shelf blocks. Vertical racks separated by 3-cell (1.5 m) aisles.
        # Two bands of shelving with a mid cross-aisle between them.
        shelf_x = [(3, 6), (10, 13), (17, 20), (24, 27), (31, 34)]
        bands = [(7, 13), (17, 23)]

        for (x0, x1) in shelf_x:
            for (y0, y1) in bands:
                for y in range(y0, y1 + 1):
                    for x in range(x0, x1 + 1):
                        self.grid[y][x] = SHELF

        # Passing bays: notch the middle of each shelf block so a yielding
        # robot has somewhere to go. Without these, head-on conflicts in a
        # narrow aisle are UNRESOLVABLE and deadlock recovery cannot work.
        for (x0, x1) in shelf_x:
            for (y0, y1) in bands:
                my = (y0 + y1) // 2
                self.grid[my][x0] = FREE           # left-side notch
                self.grid[my][x1] = FREE           # right-side notch
                self.passing_bays.append((x0, my))
                self.passing_bays.append((x1, my))

        self._classify_zones()
        self._segment_aisles()
        self._place_nodes()

    def _segment_aisles(self) -> None:
        """
        Group contiguous narrow cells into AISLE SEGMENTS.

        Narrow aisles are managed by reservation, not by reciprocal
        avoidance: a head-on meeting inside a one-robot-wide corridor has no
        velocity solution, so the only fix is to stop the second robot from
        ENTERING while the first is inside. Segment ids make that checkable.
        """
        self.aisle_id = [[-1] * W for _ in range(H)]
        seg = 0
        for sy in range(H):
            for sx in range(W):
                if not self.is_narrow(sx, sy) or self.aisle_id[sy][sx] != -1:
                    continue
                # flood fill the whole connected corridor, across its WIDTH
                stack, cells = [(sx, sy)], []
                self.aisle_id[sy][sx] = seg
                while stack:
                    cx, cy = stack.pop()
                    cells.append((cx, cy))
                    for nx, ny in self.neighbors(cx, cy):
                        if self.is_narrow(nx, ny) and self.aisle_id[ny][nx] == -1:
                            self.aisle_id[ny][nx] = seg
                            stack.append((nx, ny))
                if len(cells) < 3:                  # too small to matter
                    for (cx, cy) in cells:
                        self.aisle_id[cy][cx] = -1
                else:
                    seg += 1
        self.n_aisles = seg

    def aisle_at(self, cx: int, cy: int) -> int:
        if 0 <= cx < W and 0 <= cy < H:
            return self.aisle_id[cy][cx]
        return -1

    def _classify_zones(self) -> None:
        """Narrow = free cell with blocked cells on both left and right."""
        for y in range(H):
            for x in range(W):
                if self.grid[y][x] != FREE:
                    continue
                if (x, y) in self.passing_bays:
                    self.zone[y][x] = ZONE_BAY
                    continue
                left = self.grid[y][x - 1] != FREE if x > 0 else True
                right = self.grid[y][x + 1] != FREE if x < W - 1 else True
                width = self._free_run_width(x, y)
                self.zone[y][x] = ZONE_NARROW if (left and right) or width <= 3 \
                    else ZONE_OPEN

    def _free_run_width(self, x: int, y: int) -> int:
        n = 1
        xx = x - 1
        while xx >= 0 and self.grid[y][xx] == FREE:
            n += 1
            xx -= 1
        xx = x + 1
        while xx < W and self.grid[y][xx] == FREE:
            n += 1
            xx += 1
        return n

    def _place_nodes(self) -> None:
        # Pickups deep in the aisles, dropoffs on the far side.
        # Deliberately arranged so common routes share the mid cross-aisle.
        defs = [
            Node("P1", 8, 9, "pickup"),
            Node("P2", 15, 21, "pickup"),
            Node("P3", 22, 9, "pickup"),
            Node("P4", 29, 21, "pickup"),
            Node("D1", 2, 27, "dropoff"),
            Node("D2", 20, 27, "dropoff"),
            Node("D3", 37, 27, "dropoff"),
            Node("C1", 2, 2, "charger"),
            Node("C2", 37, 2, "charger"),
        ]
        for n in defs:
            assert self.grid[n.cy][n.cx] == FREE, f"node {n.name} inside obstacle"
            self.nodes[n.name] = n

    # -- queries -----------------------------------------------------------

    def is_free(self, cx: int, cy: int) -> bool:
        return 0 <= cx < W and 0 <= cy < H and self.grid[cy][cx] == FREE

    def is_narrow(self, cx: int, cy: int) -> bool:
        return self.is_free(cx, cy) and self.zone[cy][cx] == ZONE_NARROW

    def neighbors(self, cx: int, cy: int):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if self.is_free(nx, ny):
                yield nx, ny

    def to_world(self, cx: int, cy: int) -> tuple[float, float]:
        return (cx + 0.5) * CELL_SIZE, (cy + 0.5) * CELL_SIZE

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int(x / CELL_SIZE), int(y / CELL_SIZE)

    def nearest_bay(self, cx: int, cy: int) -> tuple[int, int] | None:
        if not self.passing_bays:
            return None
        return min(self.passing_bays,
                   key=lambda b: abs(b[0] - cx) + abs(b[1] - cy))

    # -- debug -------------------------------------------------------------

    def render_ascii(self) -> str:
        sym = {FREE: ".", WALL: "#", SHELF: "="}
        out = [[sym[self.grid[y][x]] for x in range(W)] for y in range(H)]
        for (bx, by) in self.passing_bays:
            out[by][bx] = "o"
        for n in self.nodes.values():
            out[n.cy][n.cx] = n.name[0]
        return "\n".join("".join(r) for r in out)

    def stats(self) -> dict:
        free = sum(r.count(FREE) for r in self.grid)
        narrow = sum(1 for y in range(H) for x in range(W)
                     if self.is_free(x, y) and self.zone[y][x] == ZONE_NARROW)
        return {"free_cells": free,
                "narrow_cells": narrow,
                "narrow_fraction": round(narrow / free, 3),
                "passing_bays": len(self.passing_bays),
                "nodes": len(self.nodes)}


if __name__ == "__main__":
    m = WarehouseMap()
    print(m.render_ascii())
    print()
    print(m.stats())
