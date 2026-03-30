"""
CrumbStore — grid-based density map built from telemetry positions.

Instead of storing individual breadcrumb points relayed from the ESP32,
positions are recorded directly from every telemetry packet (10 Hz).
A 2D grid accumulates visit counts per cell, giving O(1) insertion and
efficient spatial density queries without a KDTree.

Usage:
    store = CrumbStore()
    store.add_position(drone_id=0, x=1.0, y=2.0)   # call every telem tick
    density = store.cone_density(origin=(3.0, 1.0), direction_rad=0.5,
                                 radius=2.0, half_angle_rad=0.35)
    trail   = store.get_trail(drone_id=0)
"""

import math
import threading


class CrumbStore:
    def __init__(self, cell_size: float = 0.1):
        self._cell_size = cell_size
        self._inv_cell = 1.0 / cell_size

        # Grid: (ix, iy) -> visit count
        self._grid: dict[tuple[int, int], int] = {}

        # Per-drone ordered trail (for approach-bearing estimation)
        self._trails: dict[int, list[tuple[float, float]]] = {}

        # Counts
        self._total: int = 0
        self._per_drone: dict[int, int] = {}

        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Ingestion
    # ------------------------------------------------------------------

    def add_position(self, drone_id: int, x: float, y: float) -> None:
        """Record a single telemetry position into the density grid."""
        ix = int(math.floor(x * self._inv_cell))
        iy = int(math.floor(y * self._inv_cell))
        key = (ix, iy)

        with self._lock:
            self._grid[key] = self._grid.get(key, 0) + 1

            if drone_id not in self._trails:
                self._trails[drone_id] = []
                self._per_drone[drone_id] = 0
            self._trails[drone_id].append((x, y))
            self._per_drone[drone_id] += 1
            self._total += 1

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_trail(self, drone_id: int) -> list[tuple[float, float]]:
        """Return the full ordered position trail for one drone (map frame)."""
        with self._lock:
            return list(self._trails.get(drone_id, []))

    def cone_density(
        self,
        origin: tuple[float, float],
        direction_rad: float,
        radius: float = 2.0,
        half_angle_rad: float = math.radians(30),
    ) -> float:
        """
        Weighted visit density inside a forward cone.

        origin:         (x, y) centre of the cone in map frame
        direction_rad:  NED bearing of the cone axis (rad, CW from north)
        radius:         reach of the cone (m)
        half_angle_rad: half-width of the cone (rad)

        Returns a float score — higher means more explored.
        Grid cells closer to the origin are weighted more heavily
        (weight = 1 - dist/radius), scaled by the cell's visit count.
        """
        with self._lock:
            grid = self._grid
        if not grid:
            return 0.0

        ox, oy = origin
        cell = self._cell_size
        inv_cell = self._inv_cell

        # Bounding box in grid coordinates
        r_cells = int(math.ceil(radius * inv_cell))
        cx = int(math.floor(ox * inv_cell))
        cy = int(math.floor(oy * inv_cell))

        total = 0.0
        for ix in range(cx - r_cells, cx + r_cells + 1):
            for iy in range(cy - r_cells, cy + r_cells + 1):
                count = grid.get((ix, iy), 0)
                if count == 0:
                    continue

                # Cell centre position
                px = (ix + 0.5) * cell
                py = (iy + 0.5) * cell

                dx = px - ox
                dy = py - oy
                dist = math.sqrt(dx * dx + dy * dy)
                if dist > radius:
                    continue

                # Bearing from origin to cell centre
                bearing = math.atan2(dy, dx)
                diff = (bearing - direction_rad + math.pi) % (2.0 * math.pi) - math.pi
                if abs(diff) > half_angle_rad:
                    continue

                weight = 1.0 - dist / radius
                total += weight * count

        return total

    def total_crumbs(self) -> int:
        """Total number of recorded positions across all drones."""
        with self._lock:
            return self._total

    def crumb_count(self, drone_id: int) -> int:
        """Number of recorded positions for a specific drone."""
        with self._lock:
            return self._per_drone.get(drone_id, 0)

    def grid_snapshot(self) -> dict[tuple[int, int], int]:
        """Return a shallow copy of the density grid, safe to read off-lock."""
        with self._lock:
            return dict(self._grid)
