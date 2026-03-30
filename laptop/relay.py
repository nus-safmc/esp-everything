"""
Relay path planner.

Given a density grid (CrumbStore) and a set of candidate bonus-victim
positions, finds the shortest traversable path from the start area to each
candidate, then computes relay-drone landing positions spaced at most 1 m
apart with mutual line of sight.

Usage:
    from relay import plan_best_relay

    result = plan_best_relay(
        store,
        start=(0.0, 0.0),
        targets={"A": (2.5, 3.0), "B": (1.0, 2.0)},
    )
    if result is not None:
        print(result.target_id, result.path_length_m,
              result.n_relay_drones, result.relay_positions)
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from crumb_store import CrumbStore


@dataclass
class RelayPlan:
    target_id: str
    path_length_m: float
    n_relay_drones: int
    relay_positions: list[tuple[float, float]]  # ordered start → target


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

def _traversable_set(store: CrumbStore, min_visits: int = 1) -> set[tuple[int, int]]:
    """Return the set of grid cells with at least *min_visits* hits."""
    grid = store.grid_snapshot()
    return {k for k, v in grid.items() if v >= min_visits}


def _cell_centre(ix: int, iy: int, cell: float) -> tuple[float, float]:
    return ((ix + 0.5) * cell, (iy + 0.5) * cell)


def _pos_to_cell(x: float, y: float, inv_cell: float) -> tuple[int, int]:
    return (int(math.floor(x * inv_cell)), int(math.floor(y * inv_cell)))


# ---------------------------------------------------------------------------
# Line-of-sight (Bresenham on the grid)
# ---------------------------------------------------------------------------

def _line_of_sight(
    ix0: int, iy0: int, ix1: int, iy1: int,
    traversable: set[tuple[int, int]],
) -> bool:
    """True if every grid cell along the straight line is traversable."""
    dx = abs(ix1 - ix0)
    dy = abs(iy1 - iy0)
    sx = 1 if ix1 > ix0 else -1
    sy = 1 if iy1 > iy0 else -1
    err = dx - dy
    x, y = ix0, iy0

    while True:
        if (x, y) not in traversable:
            return False
        if x == ix1 and y == iy1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return True


# ---------------------------------------------------------------------------
# A* on the grid
# ---------------------------------------------------------------------------

_NEIGHBOURS = [
    (-1, -1), (-1, 0), (-1, 1),
    ( 0, -1),          ( 0, 1),
    ( 1, -1), ( 1, 0), ( 1, 1),
]
_SQRT2 = math.sqrt(2.0)


def _astar(
    start: tuple[int, int],
    goal: tuple[int, int],
    traversable: set[tuple[int, int]],
    cell: float,
) -> list[tuple[int, int]] | None:
    """A* search on the 8-connected grid.  Returns cell path or None."""
    if start not in traversable or goal not in traversable:
        return None

    open_set: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))

    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}

    gx, gy = goal
    def h(node: tuple[int, int]) -> float:
        return math.hypot(node[0] - gx, node[1] - gy) * cell

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cx, cy = current
        cur_g = g_score[current]

        for dx, dy in _NEIGHBOURS:
            nb = (cx + dx, cy + dy)
            if nb not in traversable:
                continue
            step = (_SQRT2 if dx != 0 and dy != 0 else 1.0) * cell
            tentative = cur_g + step
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                came_from[nb] = current
                counter += 1
                heapq.heappush(open_set, (tentative + h(nb), counter, nb))

    return None  # no path


# ---------------------------------------------------------------------------
# Path simplification (greedy line-of-sight removal)
# ---------------------------------------------------------------------------

def _simplify_path(
    cell_path: list[tuple[int, int]],
    traversable: set[tuple[int, int]],
) -> list[tuple[int, int]]:
    """Remove intermediate waypoints where direct line of sight exists."""
    if len(cell_path) <= 2:
        return list(cell_path)

    simplified = [cell_path[0]]
    i = 0
    while i < len(cell_path) - 1:
        # Greedily find the farthest point visible from cell_path[i]
        farthest = i + 1
        for j in range(len(cell_path) - 1, i + 1, -1):
            if _line_of_sight(*cell_path[i], *cell_path[j], traversable):
                farthest = j
                break
        simplified.append(cell_path[farthest])
        i = farthest

    return simplified


# ---------------------------------------------------------------------------
# Relay point placement
# ---------------------------------------------------------------------------

def _place_relay_drones(
    waypoints: list[tuple[float, float]],
    max_spacing: float = 1.0,
) -> list[tuple[float, float]]:
    """
    Given a simplified path (world coords), place relay positions so that
    consecutive drones are at most *max_spacing* metres apart.

    Returns positions including the first and last waypoint.
    """
    if len(waypoints) <= 1:
        return list(waypoints)

    positions = [waypoints[0]]

    for k in range(1, len(waypoints)):
        ax, ay = positions[-1]
        bx, by = waypoints[k]
        seg_len = math.hypot(bx - ax, by - ay)

        if seg_len <= max_spacing:
            positions.append((bx, by))
        else:
            n_segments = math.ceil(seg_len / max_spacing)
            for s in range(1, n_segments + 1):
                t = s / n_segments
                positions.append((ax + t * (bx - ax), ay + t * (by - ay)))

    return positions


def _path_length(waypoints: list[tuple[float, float]]) -> float:
    total = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i - 1][0]
        dy = waypoints[i][1] - waypoints[i - 1][1]
        total += math.hypot(dx, dy)
    return total


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def find_relay_path(
    store: CrumbStore,
    start: tuple[float, float],
    target: tuple[float, float],
    max_spacing: float = 1.0,
    min_visits: int = 1,
) -> tuple[float, list[tuple[float, float]]] | None:
    """
    Find a traversable relay path from *start* to *target*.

    Returns ``(path_length_m, relay_positions)`` or ``None`` if no path exists.
    *relay_positions* are ordered start → target, spaced ≤ max_spacing metres
    apart with mutual line of sight through traversed cells.
    """
    cell = store._cell_size
    inv_cell = store._inv_cell

    traversable = _traversable_set(store, min_visits)
    sc = _pos_to_cell(*start, inv_cell)
    gc = _pos_to_cell(*target, inv_cell)

    cell_path = _astar(sc, gc, traversable, cell)
    if cell_path is None:
        return None

    # Simplify: remove waypoints where line of sight exists
    simple_cells = _simplify_path(cell_path, traversable)

    # Convert to world coordinates
    waypoints = [_cell_centre(ix, iy, cell) for ix, iy in simple_cells]

    # Place relay drones
    relay = _place_relay_drones(waypoints, max_spacing)
    length = _path_length(relay)

    return (length, relay)


def plan_best_relay(
    store: CrumbStore,
    start: tuple[float, float],
    targets: dict[str, tuple[float, float]],
    max_spacing: float = 1.0,
    min_visits: int = 1,
) -> RelayPlan | None:
    """
    Evaluate all candidate bonus victims and return the relay plan with the
    shortest path.  Returns ``None`` if no target is reachable.
    """
    best: RelayPlan | None = None

    for tid, tpos in targets.items():
        result = find_relay_path(store, start, tpos, max_spacing, min_visits)
        if result is None:
            continue
        length, positions = result
        # Relay drones exclude the endpoints (start drone + victim drone)
        # — only the intermediate drones that form the chain.
        n_relay = max(0, len(positions) - 2)
        plan = RelayPlan(
            target_id=tid,
            path_length_m=length,
            n_relay_drones=n_relay,
            relay_positions=positions,
        )
        if best is None or length < best.path_length_m:
            best = plan

    return best
