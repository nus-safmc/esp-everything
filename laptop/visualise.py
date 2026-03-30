"""
Live fleet visualiser.

Layout
------
Top row  : 2D map — density heatmap, drone markers, nav-tag markers.
Bottom row: one polar VFH subplot per drone — blocked bins in red,
            free bins in green, an arrow showing current heading.

The density grid is normalised to the current maximum cell count on every
refresh so early crumbs stay visible while late-flight density is still
readable.

Usage:
    from visualise import Visualiser

    vis = Visualiser(crumb_store, "setup.yaml")
    comms.on_telemetry(vis.feed)
    vis.run()           # blocks — must be called from the main thread
"""

import logging
import math
import threading
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec

from crumb_store import CrumbStore
from protocol import TelemetryPacket, VFH_BINS

log = logging.getLogger("vis")

DRONE_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3",
                "#ff7f00", "#a65628", "#f781bf", "#999999"]

# Bin angular width (radians) — 32 bins over 2π
_BIN_WIDTH  = 2.0 * math.pi / VFH_BINS
# Centre angles for each bin (bin 0 = forward = 0 rad)
_BIN_ANGLES = np.array([i * _BIN_WIDTH for i in range(VFH_BINS)])


class Visualiser:
    """Non-blocking telemetry visualiser.  Call feed() from any thread."""

    def __init__(self, store: CrumbStore, config_path: str):
        self._store = store

        cfg = yaml.safe_load(Path(config_path).read_text())
        self._arena    = cfg["arena"]
        self._drone_ids = sorted(int(d) for d in cfg["drones"])
        self._nav_tags  = cfg.get("nav_tags", {})
        n_drones = len(self._drone_ids)

        self._lock = threading.Lock()
        self._positions:  dict[int, tuple[float, float] | None] = {d: None for d in self._drone_ids}
        self._nav_states: dict[int, str]                        = {d: "?"  for d in self._drone_ids}
        self._headings:   dict[int, float]                      = {d: 0.0  for d in self._drone_ids}
        self._vfh:        dict[int, list[bool]]                 = {
            d: [False] * VFH_BINS for d in self._drone_ids
        }

    def feed(self, pkt: TelemetryPacket, _src_ip: str) -> None:
        """Telemetry callback — safe to call from any thread."""
        if pkt.drone_id not in self._positions:
            return
        did = pkt.drone_id
        with self._lock:
            self._positions[did]  = (pkt.ned_x, pkt.ned_y)
            self._nav_states[did] = pkt.nav_state_name
            self._headings[did]   = pkt.heading_rad
            self._vfh[did]        = list(pkt.vfh_blocked)

    def run(self) -> None:
        """Set up the plot and enter the matplotlib main loop (blocks)."""
        arena    = self._arena
        cell     = self._store._cell_size
        n_drones = len(self._drone_ids)

        # Grid cell-aligned extents
        x_min, x_max = arena["min_x"], arena["max_x"]
        y_min, y_max = arena["min_y"], arena["max_y"]
        ix0 = math.floor(x_min / cell)
        ix1 = math.ceil(x_max  / cell)
        iy0 = math.floor(y_min / cell)
        iy1 = math.ceil(y_max  / cell)
        n_x = ix1 - ix0
        n_y = iy1 - iy0
        x_edges = np.linspace(ix0 * cell, ix1 * cell, n_x + 1)
        y_edges = np.linspace(iy0 * cell, iy1 * cell, n_y + 1)

        # ---------------------------------------------------------------
        # Figure layout: map on top, polar histograms on bottom
        # ---------------------------------------------------------------
        fig_w = max(8, n_drones * 2.5)
        fig   = plt.figure(figsize=(fig_w, fig_w + 2.5))
        gs    = gridspec.GridSpec(
            2, n_drones,
            height_ratios=[fig_w, 2.5],
            hspace=0.35, wspace=0.4,
        )

        # --- Map axes (spans all columns) ---
        ax_map = fig.add_subplot(gs[0, :])
        ax_map.set_aspect("equal")
        ax_map.set_xlabel("East (m)")
        ax_map.set_ylabel("North (m)")
        ax_map.set_title("Fleet Exploration — density grid")
        margin = 0.5
        ax_map.set_xlim(y_min - margin, y_max + margin)
        ax_map.set_ylim(x_min - margin, x_max + margin)

        # Density mesh
        Z    = np.zeros((n_x, n_y), dtype=np.float32)
        mesh = ax_map.pcolormesh(
            y_edges, x_edges, Z,
            cmap="hot", vmin=0.0, vmax=1.0, shading="flat", zorder=1,
        )
        fig.colorbar(mesh, ax=ax_map, label="Relative visit density")

        # Arena boundary
        rect = patches.Rectangle(
            (y_min, x_min), y_max - y_min, x_max - x_min,
            linewidth=2, edgecolor="cyan", facecolor="none",
            linestyle="--", zorder=3,
        )
        ax_map.add_patch(rect)

        # Nav tags
        for tag_id, pos in self._nav_tags.items():
            ax_map.plot(pos["map_y"], pos["map_x"], "c^", markersize=10, zorder=4)
            ax_map.annotate(
                f"T{tag_id}", (pos["map_y"], pos["map_x"]),
                textcoords="offset points", xytext=(5, 5),
                fontsize=8, color="cyan", zorder=4,
            )

        # Drone markers on map
        pos_markers = {}
        label_texts = {}
        for i, did in enumerate(self._drone_ids):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            m, = ax_map.plot([], [], "o", color=color, markersize=12,
                             markeredgecolor="white", markeredgewidth=1.5,
                             zorder=5, label=f"Drone {did}")
            t = ax_map.text(0, 0, "", fontsize=8, color=color, zorder=5,
                            fontweight="bold")
            pos_markers[did] = m
            label_texts[did] = t
        ax_map.legend(loc="upper right", fontsize=8)

        # ---------------------------------------------------------------
        # Polar VFH subplots — one per drone
        # ---------------------------------------------------------------
        ax_vfh: dict[int, plt.Axes] = {}
        vfh_bars: dict[int, any]    = {}   # BarContainer
        heading_arrows: dict[int, any] = {}

        for i, did in enumerate(self._drone_ids):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            ax = fig.add_subplot(gs[1, i], projection="polar")
            ax.set_theta_zero_location("N")   # 0 rad = top = forward
            ax.set_theta_direction(-1)         # clockwise (matches NED yaw)
            ax.set_ylim(0, 1)
            ax.set_yticks([])
            ax.set_title(f"D{did} VFH", fontsize=8, pad=3)
            ax.tick_params(labelsize=6)

            # Initial bars — all green (free)
            bars = ax.bar(
                _BIN_ANGLES, np.ones(VFH_BINS),
                width=_BIN_WIDTH, align="edge",
                color="green", alpha=0.7, linewidth=0,
            )
            # Heading arrow (starts pointing up)
            arrow = ax.annotate(
                "", xy=(0, 0.9), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color=color, lw=2),
            )
            ax_vfh[did]         = ax
            vfh_bars[did]       = bars
            heading_arrows[did] = arrow

        # ---------------------------------------------------------------
        # Timer callback — updates all artists
        # ---------------------------------------------------------------
        def update(_):
            # --- Density grid ---
            grid = self._store.grid_snapshot()
            Z[:] = 0.0
            for (gx, gy), count in grid.items():
                xi = gx - ix0
                yi = gy - iy0
                if 0 <= xi < n_x and 0 <= yi < n_y:
                    Z[xi, yi] = count
            max_val = Z.max()
            mesh.set_array((Z / max_val).ravel() if max_val > 0 else Z.ravel())

            # --- Snapshot per-drone state ---
            with self._lock:
                positions  = dict(self._positions)
                nav_states = dict(self._nav_states)
                headings   = dict(self._headings)
                vfh_data   = {d: list(v) for d, v in self._vfh.items()}

            for did in self._drone_ids:
                # Map marker
                pos = positions[did]
                if pos is not None:
                    pos_markers[did].set_data([pos[1]], [pos[0]])
                    label_texts[did].set_position((pos[1] + 0.08, pos[0] + 0.08))
                    label_texts[did].set_text(f"D{did} {nav_states[did]}")

                # VFH polar bars
                blocked = vfh_data[did]
                for bar, is_blocked in zip(vfh_bars[did], blocked):
                    bar.set_facecolor("#e41a1c" if is_blocked else "#4daf4a")

                # Heading arrow: heading_rad is map-frame yaw (CW from north).
                # The polar plot also uses CW-from-north, so use it directly.
                h = headings[did]
                heading_arrows[did].xy     = (h, 0.9)
                heading_arrows[did].xytext = (h, 0.0)

            fig.canvas.draw_idle()

        timer = fig.canvas.new_timer(interval=250)
        timer.add_callback(update, None)
        timer.start()

        log.info("Visualiser running.  Close the window to stop.")
        plt.show()
