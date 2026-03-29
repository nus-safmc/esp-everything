"""
CrumbStore — aggregates breadcrumb trails from all drones in the shared
map frame, with a KDTree for spatial density queries.

Crumb positions arrive already in map frame from the ESP32 firmware
(converted via odom_to_map before transmission).

Usage:
    store = CrumbStore()
    store.add_crumbs(drone_id=0, crumbs=[(1.0, 2.0), (1.4, 2.1)],
                     start_index=0)
    density = store.cone_density(origin=(3.0, 1.0), direction_rad=0.5,
                                 radius=2.0, half_angle_rad=0.35)
    trail   = store.get_trail(drone_id=0)
"""

import math
import threading

import numpy as np
from scipy.spatial import cKDTree


class CrumbStore:
    def __init__(self):
        # Per-drone ordered trail (map frame)
        self._trails: dict[int, list[tuple[float, float]]] = {}
        # Expected next index per drone (for dedup / ordering)
        self._next_idx: dict[int, int] = {}

        # Combined array + KDTree (rebuilt lazily)
        self._all_pts: np.ndarray = np.empty((0, 2), dtype=np.float32)
        self._tree: cKDTree | None = None
        self._dirty_count: int = 0         # crumbs added since last rebuild
        self._rebuild_threshold: int = 20  # rebuild tree every N new crumbs

        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Ingestion
    # ------------------------------------------------------------------

    def add_crumbs(
        self,
        drone_id: int,
        crumbs: list[tuple[float, float]],
        start_index: int,
    ) -> None:
        """
        Append crumbs from a telemetry packet.  Crumbs arrive already in
        the shared map frame (converted on the ESP32 via odom_to_map).

        start_index: the crumb_batch_start from the telemetry packet, used
                     to place crumbs at the correct trail position.
        """
        if not crumbs:
            return

        with self._lock:
            if drone_id not in self._trails:
                self._trails[drone_id] = []
                self._next_idx[drone_id] = 0

            expected = self._next_idx[drone_id]

            # Only accept crumbs starting at or after expected index
            skip = max(0, expected - start_index)
            new_crumbs = crumbs[skip:]
            if not new_crumbs:
                return

            self._trails[drone_id].extend(new_crumbs)
            self._next_idx[drone_id] = start_index + len(crumbs)

            # Append to combined array
            pts = np.array(new_crumbs, dtype=np.float32)
            self._all_pts = np.vstack([self._all_pts, pts])
            self._dirty_count += len(mapped)

            # Rebuild tree if enough new crumbs accumulated
            if self._dirty_count >= self._rebuild_threshold:
                self._rebuild_tree()

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_trail(self, drone_id: int) -> list[tuple[float, float]]:
        """Return the full ordered crumb trail for one drone (map frame)."""
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
        Weighted crumb density inside a forward cone.

        origin:         (x, y) centre of the cone in map frame
        direction_rad:  NED bearing of the cone axis (rad, CW from north)
        radius:         reach of the cone (m)
        half_angle_rad: half-width of the cone (rad)

        Returns a float score — higher means more explored.
        Crumbs closer to the origin are weighted more heavily
        (weight = 1 - dist/radius).
        """
        with self._lock:
            if self._all_pts.shape[0] == 0:
                return 0.0
            self._ensure_tree()
            tree = self._tree
            pts = self._all_pts

        # Ball query: all crumbs within radius of origin
        indices = tree.query_ball_point(origin, radius)
        if not indices:
            return 0.0

        candidates = pts[indices]
        dx = candidates[:, 0] - origin[0]
        dy = candidates[:, 1] - origin[1]
        dists = np.sqrt(dx * dx + dy * dy)

        # Bearing from origin to each candidate (NED: atan2(east, north))
        bearings = np.arctan2(dy, dx)

        # Angular difference wrapped to (-pi, pi]
        diff = bearings - direction_rad
        diff = (diff + math.pi) % (2 * math.pi) - math.pi

        # Filter by cone half-angle
        in_cone = np.abs(diff) <= half_angle_rad

        # Weighted sum: closer crumbs count more
        weights = 1.0 - dists[in_cone] / radius
        return float(np.sum(weights))

    def total_crumbs(self) -> int:
        """Total number of crumbs across all drones."""
        with self._lock:
            return self._all_pts.shape[0]

    def crumb_count(self, drone_id: int) -> int:
        """Number of crumbs for a specific drone."""
        with self._lock:
            return len(self._trails.get(drone_id, []))

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _ensure_tree(self) -> None:
        """Build the tree if it doesn't exist yet (caller holds lock)."""
        if self._tree is None and self._all_pts.shape[0] > 0:
            self._rebuild_tree()

    def _rebuild_tree(self) -> None:
        """Rebuild the KDTree from the current combined array (caller holds lock)."""
        self._tree = cKDTree(self._all_pts)
        self._dirty_count = 0
