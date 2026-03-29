"""
Live crumb trail visualiser.

Shows a 2D map-frame plot with:
  - Per-drone crumb trails (coloured lines)
  - Current drone positions (large markers)
  - Arena bounds
  - Nav-tag positions

Designed to be driven by a CommsNode telemetry callback — no separate
socket, no packet stealing.

Usage:
    from visualise import Visualiser

    vis = Visualiser("setup.yaml")
    comms.on_telemetry(vis.feed)   # register as callback
    vis.run()                       # blocks (runs matplotlib main loop)
"""

import logging
import threading
from pathlib import Path

import yaml
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from protocol import TelemetryPacket

log = logging.getLogger("vis")

DRONE_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3",
                "#ff7f00", "#a65628", "#f781bf", "#999999"]


class Visualiser:
    """Non-blocking telemetry visualiser.  Call feed() from any thread."""

    def __init__(self, config_path: str):
        cfg = yaml.safe_load(Path(config_path).read_text())
        self._arena = cfg["arena"]
        self._drone_ids = sorted(int(d) for d in cfg["drones"])
        self._nav_tags = cfg.get("nav_tags", {})

        self._lock = threading.Lock()
        self._trails: dict[int, list[tuple[float, float]]] = {d: [] for d in self._drone_ids}
        self._positions: dict[int, tuple[float, float] | None] = {d: None for d in self._drone_ids}
        self._nav_states: dict[int, str] = {d: "?" for d in self._drone_ids}

    def feed(self, pkt: TelemetryPacket, _src_ip: str) -> None:
        """Telemetry callback — safe to call from any thread."""
        if pkt.drone_id not in self._positions:
            return
        did = pkt.drone_id
        with self._lock:
            self._positions[did] = (pkt.ned_x, pkt.ned_y)
            self._nav_states[did] = pkt.nav_state_name
            if pkt.crumbs:
                self._trails[did].extend(pkt.crumbs)

    def run(self) -> None:
        """Set up the plot and enter the matplotlib main loop (blocks)."""
        arena = self._arena
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_aspect("equal")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title("Fleet Exploration")

        # Arena bounds
        w = arena["max_y"] - arena["min_y"]
        h = arena["max_x"] - arena["min_x"]
        rect = patches.Rectangle(
            (arena["min_y"], arena["min_x"]), w, h,
            linewidth=2, edgecolor="black", facecolor="none", linestyle="--",
        )
        ax.add_patch(rect)
        margin = 0.5
        ax.set_xlim(arena["min_y"] - margin, arena["max_y"] + margin)
        ax.set_ylim(arena["min_x"] - margin, arena["max_x"] + margin)

        # Nav tags
        for tag_id, pos in self._nav_tags.items():
            ax.plot(pos["map_y"], pos["map_x"], "k^", markersize=10)
            ax.annotate(f"T{tag_id}", (pos["map_y"], pos["map_x"]),
                        textcoords="offset points", xytext=(5, 5), fontsize=8)

        # Per-drone plot elements
        trail_lines = {}
        pos_markers = {}
        label_texts = {}
        for i, did in enumerate(self._drone_ids):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            line, = ax.plot([], [], "-", color=color, linewidth=1, alpha=0.5,
                            label=f"Drone {did}")
            marker, = ax.plot([], [], "o", color=color, markersize=10)
            txt = ax.text(0, 0, "", fontsize=8, color=color)
            trail_lines[did] = line
            pos_markers[did] = marker
            label_texts[did] = txt

        ax.legend(loc="upper right", fontsize=8)

        def update(_):
            with self._lock:
                for did in self._drone_ids:
                    trail = self._trails[did]
                    if trail:
                        ys = [c[1] for c in trail]  # east
                        xs = [c[0] for c in trail]  # north
                        trail_lines[did].set_data(ys, xs)

                    pos = self._positions[did]
                    if pos is not None:
                        pos_markers[did].set_data([pos[1]], [pos[0]])
                        label_texts[did].set_position((pos[1] + 0.08, pos[0] + 0.08))
                        label_texts[did].set_text(f"D{did} {self._nav_states[did]}")

            fig.canvas.draw_idle()

        timer = fig.canvas.new_timer(interval=500)
        timer.add_callback(update, None)
        timer.start()

        log.info("Visualiser running. Close the window to stop.")
        plt.show()
