#!/usr/bin/env python3
"""
Live crumb trail visualiser.

Shows a 2D map-frame plot with:
  - Per-drone crumb trails (coloured lines)
  - Current drone positions (large markers)
  - Arena bounds
  - Nav-tag positions

Runs alongside the exploration scripts — attaches to the same
telemetry port and passively listens.

Usage:
    python visualise.py [--config setup.yaml] [--telem-port 5005]
"""

import argparse
import logging
import threading
import time
from pathlib import Path

import yaml
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from protocol import parse_telemetry, TelemetryPacket, NAV_STATE_NAMES
import socket

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("vis")

DRONE_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3",
                "#ff7f00", "#a65628", "#f781bf", "#999999"]


def main():
    parser = argparse.ArgumentParser(description="Live crumb trail visualiser")
    parser.add_argument("--config", default="setup.yaml")
    parser.add_argument("--telem-port", type=int, default=5005)
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    arena = cfg["arena"]
    drone_ids = sorted(int(d) for d in cfg["drones"])
    nav_tags = cfg.get("nav_tags", {})

    # Shared state (written by recv thread, read by plot loop)
    lock = threading.Lock()
    trails: dict[int, list[tuple[float, float]]] = {d: [] for d in drone_ids}
    positions: dict[int, tuple[float, float] | None] = {d: None for d in drone_ids}
    headings: dict[int, float] = {d: 0.0 for d in drone_ids}
    nav_states: dict[int, str] = {d: "?" for d in drone_ids}

    running = True

    def recv_loop():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind(("", args.telem_port))
        sock.settimeout(1.0)

        while running:
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            pkt = parse_telemetry(data)
            if pkt is None or pkt.drone_id not in positions:
                continue

            did = pkt.drone_id
            with lock:
                positions[did] = (pkt.ned_x, pkt.ned_y)
                headings[did] = pkt.heading_rad
                nav_states[did] = pkt.nav_state_name
                if pkt.crumbs:
                    trails[did].extend(pkt.crumbs)

        sock.close()

    thread = threading.Thread(target=recv_loop, daemon=True)
    thread.start()

    # --- Set up plot ---
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
    for tag_id, pos in nav_tags.items():
        ax.plot(pos["map_y"], pos["map_x"], "k^", markersize=10)
        ax.annotate(f"T{tag_id}", (pos["map_y"], pos["map_x"]),
                    textcoords="offset points", xytext=(5, 5), fontsize=8)

    # Per-drone plot elements
    trail_lines = {}
    pos_markers = {}
    label_texts = {}
    for i, did in enumerate(drone_ids):
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
        with lock:
            for did in drone_ids:
                trail = trails[did]
                if trail:
                    ys = [c[1] for c in trail]  # east
                    xs = [c[0] for c in trail]  # north
                    trail_lines[did].set_data(ys, xs)

                pos = positions[did]
                if pos is not None:
                    pos_markers[did].set_data([pos[1]], [pos[0]])
                    label_texts[did].set_position((pos[1] + 0.08, pos[0] + 0.08))
                    label_texts[did].set_text(f"D{did} {nav_states[did]}")

        fig.canvas.draw_idle()

    # Use a timer for updates instead of FuncAnimation for simplicity
    timer = fig.canvas.new_timer(interval=500)
    timer.add_callback(update, None)
    timer.start()

    log.info("Visualiser running. Close the window to stop.")
    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    running = False
    thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
