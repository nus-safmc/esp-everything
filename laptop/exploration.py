"""
ExplorationDirector — picks the least-explored navigable direction for a
drone and projects a goal in that direction, clamped to the arena bounds.

Usage:
    director = ExplorationDirector(crumb_store, "setup.yaml")
    goal = director.compute_goal(
        drone_id=0, map_x=1.0, map_y=2.0,
        heading_rad=0.3, vfh_blocked=[False]*32)
    if goal:
        send CMD_GOTO to drone at goal
"""

import math
from pathlib import Path

import yaml

from crumb_store import CrumbStore
from protocol import VFH_BINS, NavTag

_BIN_WIDTH_RAD = 2.0 * math.pi / VFH_BINS   # 11.25 deg


# ------------------------------------------------------------------
# Gap extraction from a circular blocked array
# ------------------------------------------------------------------

def extract_gaps(blocked: list[bool]) -> list[tuple[int, int]]:
    """
    Find contiguous runs of free (False) bins in the circular VFH array.

    Returns a list of (center_bin, width) tuples.
    center_bin is the middle bin index of the gap (mod VFH_BINS).
    """
    n = len(blocked)

    # Find any blocked bin to start from (avoids splitting a gap)
    start = None
    for i in range(n):
        if blocked[i]:
            start = i
            break

    if start is None:
        # Entire field clear — one gap spanning all bins
        return [(0, n)]

    gaps = []
    gap_start = None
    gap_len = 0

    for step in range(n):
        b = (start + 1 + step) % n
        if not blocked[b]:
            if gap_start is None:
                gap_start = b
            gap_len += 1
        else:
            if gap_start is not None:
                center = (gap_start + gap_len // 2) % n
                gaps.append((center, gap_len))
                gap_start = None
                gap_len = 0

    # Close a gap that wraps past the starting point
    if gap_start is not None:
        center = (gap_start + gap_len // 2) % n
        gaps.append((center, gap_len))

    return gaps


# ------------------------------------------------------------------
# Director
# ------------------------------------------------------------------

def load_nav_tags(config_path: str = "setup.yaml") -> list[NavTag]:
    """Load navigation AprilTag positions from setup.yaml."""
    cfg = yaml.safe_load(Path(config_path).read_text())
    tags = []
    for tag_id, pos in cfg.get("nav_tags", {}).items():
        tags.append(NavTag(id=int(tag_id), map_x=pos["map_x"], map_y=pos["map_y"]))
    return tags


def load_drone_starts(config_path: str = "setup.yaml") -> dict[int, tuple[float, float]]:
    """Load per-drone start positions (map frame) from setup.yaml.

    Returns {drone_id: (start_x, start_y)}.
    """
    cfg = yaml.safe_load(Path(config_path).read_text())
    return {
        int(did): (props["start_x"], props["start_y"])
        for did, props in cfg.get("drones", {}).items()
    }


class ExplorationDirector:
    def __init__(self, crumb_store: CrumbStore, config_path: str = "setup.yaml"):
        cfg = yaml.safe_load(Path(config_path).read_text())

        arena = cfg["arena"]
        self._min_x = arena["min_x"]
        self._max_x = arena["max_x"]
        self._min_y = arena["min_y"]
        self._max_y = arena["max_y"]

        exp = cfg.get("exploration", {})
        self._goal_dist      = exp.get("goal_distance", 2.0)
        self._cone_radius    = exp.get("cone_radius", 2.0)
        self._cone_half_angle = math.radians(exp.get("cone_half_angle_deg", 30))
        self._min_gap_bins   = exp.get("min_gap_bins", 2)

        self._store = crumb_store

        # Load nav tags for callers that need them
        self.nav_tags = load_nav_tags(config_path)

    def compute_goal(
        self,
        drone_id: int,
        map_x: float,
        map_y: float,
        heading_rad: float,
        vfh_blocked: list[bool],
        stuck_zones: list[tuple[float, float, float]] | None = None,
    ) -> tuple[float, float] | None:
        """
        Pick the least-explored navigable direction and return a goal
        in map-frame coordinates, clamped to arena bounds.

        Returns None if no navigable gap exists.
        """
        gaps = extract_gaps(vfh_blocked)
        if not gaps:
            return None

        best_bearing = None
        best_score   = float("inf")

        for center_bin, width in gaps:
            if width < self._min_gap_bins:
                continue

            # Body-frame angle of gap centre (CW from forward, radians)
            body_rad = (center_bin + 0.5) * _BIN_WIDTH_RAD
            map_bearing = heading_rad + body_rad

            density = self._store.cone_density(
                origin=(map_x, map_y),
                direction_rad=map_bearing,
                radius=self._cone_radius,
                half_angle_rad=self._cone_half_angle,
            )

            # Add penalty for goals inside stuck zones
            stuck_penalty = 0.0
            if stuck_zones:
                gx = map_x + self._goal_dist * math.cos(map_bearing)
                gy = map_y + self._goal_dist * math.sin(map_bearing)
                for zx, zy, zr in stuck_zones:
                    if math.hypot(gx - zx, gy - zy) < zr:
                        stuck_penalty = 1e6   # effectively block this direction
                        break

            score = density + stuck_penalty
            if score < best_score:
                best_score   = score
                best_bearing = map_bearing

        if best_bearing is None:
            return None

        gx = map_x + self._goal_dist * math.cos(best_bearing)
        gy = map_y + self._goal_dist * math.sin(best_bearing)
        gx = max(self._min_x, min(self._max_x, gx))
        gy = max(self._min_y, min(self._max_y, gy))

        return (gx, gy)
