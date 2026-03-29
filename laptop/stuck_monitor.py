"""
StuckMonitor — detects stuck drones and computes escape goals.

A drone is considered stuck if it has not made progress toward its goal
for stuck_timeout_s seconds (positional progress < progress_threshold_m).

The is_stuck flag (all VFH bins blocked) is NOT used as a trigger: if the
drone is fully surrounded there is genuinely nowhere to go horizontally,
and any retreat would be equally blocked.  The timeout-based detection
handles the more realistic stuck scenario: the drone oscillates (e.g.
entering and exiting a U-shaped obstacle) without making net progress.

Escape strategy:
  Look at the drone's crumb trail to determine the direction it came from
  (approach direction), then retreat in the opposite direction.  This
  reliably gets the drone back to the last known clear area regardless of
  how deep it penetrated the obstacle.

  The exploration director is NOT used for escape because crumbs are
  dense behind the drone (the way it came in), so the director would
  pick the low-density direction ahead — back into the trap.

Stuck locations are recorded as zones (x, y, radius) passed to the
director when selecting normal exploration goals, so no drone is sent
back toward a known trap.

Usage:
    monitor = StuckMonitor(director, store, "setup.yaml")

    # Call on every telemetry packet:
    if monitor.update(drone_id, map_x, map_y, heading, vfh_blocked):
        goal = monitor.escape_goal(drone_id)
        if goal:
            comms.send_command(drone_id, CommandPacket(CMD_GOTO, *goal))
        monitor.clear_stuck(drone_id)

    # Pass zones to director for normal goal selection:
    goal = director.compute_goal(..., stuck_zones=monitor.stuck_zones())
"""

import math
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from crumb_store import CrumbStore
from exploration import ExplorationDirector
from protocol import VFH_BINS

# How many recent crumbs to average for approach-direction estimation
_APPROACH_CRUMBS = 5


@dataclass
class StuckZone:
    x:          float
    y:          float
    radius:     float
    expires_at: float   # time.monotonic()


class StuckMonitor:
    def __init__(
        self,
        director: ExplorationDirector,
        store: CrumbStore,
        config_path: str = "setup.yaml",
    ):
        self._director = director
        self._store    = store

        cfg = yaml.safe_load(Path(config_path).read_text())
        stuck_cfg = cfg.get("stuck", {})
        self._progress_threshold_m = stuck_cfg.get("progress_threshold_m", 0.3)
        self._stuck_timeout_s      = stuck_cfg.get("stuck_timeout_s",      5.0)
        self._stuck_zone_radius    = stuck_cfg.get("stuck_zone_radius",     0.5)
        self._stuck_zone_ttl_s     = stuck_cfg.get("stuck_zone_ttl_s",     60.0)
        self._retreat_dist_m       = stuck_cfg.get("retreat_dist_m",        1.5)

        # Per-drone state
        self._last_pos:           dict[int, tuple[float, float]] = {}
        self._last_progress_time: dict[int, float]               = {}
        self._last_heading:       dict[int, float]               = {}

        self._stuck_drones: set[int]        = set()
        self._zones:        list[StuckZone] = []

    # ------------------------------------------------------------------
    # Main update — call on every telemetry packet
    # ------------------------------------------------------------------

    def update(
        self,
        drone_id: int,
        map_x: float,
        map_y: float,
        heading_rad: float,
    ) -> bool:
        """
        Update position state for one drone.
        Returns True the moment the drone is newly detected as stuck.
        """
        now = time.monotonic()

        prev = self._last_pos.get(drone_id)
        if prev is None:
            self._last_progress_time[drone_id] = now
        elif math.hypot(map_x - prev[0], map_y - prev[1]) > self._progress_threshold_m:
            self._last_progress_time[drone_id] = now
            self._stuck_drones.discard(drone_id)

        self._last_pos[drone_id]     = (map_x, map_y)
        self._last_heading[drone_id] = heading_rad

        if drone_id in self._stuck_drones:
            return False   # already known stuck, don't re-trigger

        elapsed = now - self._last_progress_time.get(drone_id, now)
        if elapsed <= self._stuck_timeout_s:
            return False

        # Newly stuck
        self._stuck_drones.add(drone_id)
        self._add_zone(map_x, map_y, now)
        return True

    # ------------------------------------------------------------------
    # Escape goal
    # ------------------------------------------------------------------

    def escape_goal(self, drone_id: int) -> tuple[float, float] | None:
        """
        Compute a retreat goal by reversing the drone's approach direction.

        Uses the last few crumbs on the trail to estimate how the drone
        entered the current area, then projects a goal in the opposite
        direction.  Falls back to reversing the current heading if the
        trail is too short.
        """
        pos = self._last_pos.get(drone_id)
        if pos is None:
            return None

        map_x, map_y = pos
        retreat_bearing = self._approach_bearing(drone_id) + math.pi

        gx = map_x + self._retreat_dist_m * math.cos(retreat_bearing)
        gy = map_y + self._retreat_dist_m * math.sin(retreat_bearing)
        gx = max(self._director._min_x, min(self._director._max_x, gx))
        gy = max(self._director._min_y, min(self._director._max_y, gy))
        return (gx, gy)

    def _approach_bearing(self, drone_id: int) -> float:
        """
        Estimate the bearing the drone was travelling when it got stuck,
        using the last few crumbs of its trail.  Falls back to heading.
        """
        trail = self._store.get_trail(drone_id)
        if len(trail) >= 2:
            n   = min(_APPROACH_CRUMBS, len(trail))
            dx  = trail[-1][0] - trail[-n][0]
            dy  = trail[-1][1] - trail[-n][1]
            if math.hypot(dx, dy) > 0.05:          # enough displacement to be meaningful
                return math.atan2(dy, dx)
        return self._last_heading.get(drone_id, 0.0)

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------

    def is_stuck(self, drone_id: int) -> bool:
        return drone_id in self._stuck_drones

    def clear_stuck(self, drone_id: int) -> None:
        """
        Call after sending an escape goal so the drone isn't immediately
        re-detected as stuck while it starts moving.
        """
        self._stuck_drones.discard(drone_id)
        self._last_progress_time[drone_id] = time.monotonic()

    def stuck_zones(self) -> list[tuple[float, float, float]]:
        """Return [(x, y, radius)] of active stuck zones, expiring stale ones."""
        now = time.monotonic()
        self._zones = [z for z in self._zones if z.expires_at > now]
        return [(z.x, z.y, z.radius) for z in self._zones]

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _add_zone(self, x: float, y: float, now: float) -> None:
        for z in self._zones:
            if math.hypot(z.x - x, z.y - y) < z.radius:
                z.expires_at = now + self._stuck_zone_ttl_s
                return
        self._zones.append(StuckZone(
            x=x, y=y,
            radius=self._stuck_zone_radius,
            expires_at=now + self._stuck_zone_ttl_s,
        ))
