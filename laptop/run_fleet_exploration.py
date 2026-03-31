#!/usr/bin/env python3
"""
Multi-drone exploration test.

Flow:
  1. Starts receiving telemetry from all drones listed in setup.yaml.
  2. Waits for user to press Enter -> sends CMD_START to all drones (arm + takeoff).
  3. Waits for user to press Enter again -> starts sending exploration goals.
  4. Ctrl+C at any point -> sends CMD_LAND to all drones and exits.

Usage:
    python run_fleet_exploration.py [--config setup.yaml]
"""

import argparse
import logging
import math
import signal
import sys
import threading
import time
from pathlib import Path

import yaml

from comms import CommsNode
from crumb_store import CrumbStore
from exploration import ExplorationDirector
from visualise import Visualiser
# from vfh_logger import VfhLogger
from protocol import (
    CMD_GOTO, CMD_LAND, CMD_START, CommandPacket,
    NAV_IDLE, NAV_ARRIVED, NAV_STUCK,
    TelemetryPacket,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("fleet")


def main():
    parser = argparse.ArgumentParser(description="Multi-drone exploration test")
    parser.add_argument("--config", default="setup.yaml", help="Path to setup.yaml")
    parser.add_argument("--telem-port", type=int, default=5005)
    parser.add_argument("--cmd-port", type=int, default=5006)
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    drone_ids = sorted(int(did) for did in cfg["drones"])
    log.info("Fleet: drones %s", drone_ids)

    # Start positions from config (used for staggered launch)
    drone_starts: dict[int, tuple[float, float]] = {}
    for did in drone_ids:
        dcfg = cfg["drones"][did]
        drone_starts[did] = (dcfg["start_x"], dcfg["start_y"])

    # Arena bounds for density normalisation
    arena = cfg["arena"]
    arena_bounds = (arena["min_x"], arena["max_x"], arena["min_y"], arena["max_y"])

    # --- Initialise components ---
    store    = CrumbStore(start_positions=list(drone_starts.values()),
                          arena_bounds=arena_bounds)
    director = ExplorationDirector(store, args.config)
    # vfh_log  = VfhLogger()
    comms    = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
                         config_path=args.config)

    GOAL_INTERVAL = 2.5 # seconds between goal updates per drone
    RELOC_AGE_THRESHOLD_S = 120  # send drone to nav tag after this many seconds without a fix

    # Per-drone state
    last_goal_time: dict[int, float]              = {d: 0.0 for d in drone_ids}
    goal_counts:    dict[int, int]                = {d: 0 for d in drone_ids}
    drone_state:    dict[int, TelemetryPacket | None] = {d: None for d in drone_ids}
    reloc_sent:     dict[int, bool]               = {d: False for d in drone_ids}
    last_log_time = 0.0

    exploring = False
    exploring_drones: set[int] = set()  # drones allowed to explore (staggered)

    def on_telemetry(pkt: TelemetryPacket, src_ip: str):
        nonlocal last_log_time

        if pkt.drone_id not in drone_ids:
            return

        did = pkt.drone_id

        drone_state[did] = pkt
        map_x, map_y = pkt.ned_x, pkt.ned_y

        # --- Record position into density grid ---
        store.add_position(did, map_x, map_y)
        now = time.monotonic()

        # --- Periodic status log (every 2 s, all drones) ---
        if now - last_log_time > 2.0:
            last_log_time = now
            for d in drone_ids:
                st = drone_state[d]
                if st is not None:
                    log.info(
                        "drone=%d  pos=(%.2f, %.2f)  nav=%s  goals=%d  crumbs=%d",
                        d, st.ned_x, st.ned_y,
                        st.nav_state_name,
                        goal_counts[d],
                        store.crumb_count(d),
                    )

        if not exploring or did not in exploring_drones:
            return

        # --- Relocalisation: send drone to nearest nav tag if odom is stale ---
        # Skip if drone has found a target tag — don't pull it away from landing
        needs_reloc = (pkt.tag_id < 0
                       and pkt.reloc_age_s != 0xFFFF
                       and pkt.reloc_age_s > RELOC_AGE_THRESHOLD_S)

        if needs_reloc and not reloc_sent[did]:
            # Find closest nav tag to minimise travel time
            best_tag = None
            best_dist = float("inf")
            for tag in director.nav_tags:
                d = math.hypot(tag.map_x - map_x, tag.map_y - map_y)
                if d < best_dist:
                    best_dist = d
                    best_tag = tag
            if best_tag is not None:
                log.warning("drone=%d  reloc_age=%ds > %ds — sending to nav tag %d "
                            "at (%.2f, %.2f)  dist=%.1fm",
                            did, pkt.reloc_age_s, RELOC_AGE_THRESHOLD_S,
                            best_tag.id, best_tag.map_x, best_tag.map_y, best_dist)
                cmd = CommandPacket(CMD_GOTO, goal_x=best_tag.map_x,
                                    goal_y=best_tag.map_y)
                comms.send_command(did, cmd)
                reloc_sent[did] = True
                last_goal_time[did] = now
                return

        # Clear reloc flag once the drone has relocalised
        if reloc_sent[did] and not needs_reloc:
            log.info("drone=%d  relocalised (reloc_age=%ds) — resuming exploration",
                     did, pkt.reloc_age_s)
            reloc_sent[did] = False

        # --- Send exploration goal when this drone is ready (rate-limited) ---
        if pkt.nav_state in (NAV_IDLE, NAV_ARRIVED, NAV_STUCK) \
                and (now - last_goal_time[did]) >= GOAL_INTERVAL:
            goal = director.compute_goal(
                drone_id=did,
                map_x=map_x,
                map_y=map_y,
                heading_rad=pkt.heading_rad,
                vfh_blocked=pkt.vfh_blocked,
            )
            if goal:
                log.info("drone=%d  goal #%d -> (%.2f, %.2f)",
                         did, goal_counts[did] + 1, goal[0], goal[1])
                cmd = CommandPacket(CMD_GOTO, goal_x=goal[0], goal_y=goal[1])
                comms.send_command(did, cmd)
                goal_counts[did] += 1
                last_goal_time[did] = now

    comms.on_telemetry(on_telemetry)
    # comms.on_telemetry(vfh_log.feed)

    # --- Visualiser (fed via callback, no separate socket) ---
    vis = Visualiser(store, args.config)
    comms.on_telemetry(vis.feed)

    # --- Ctrl+C handler ---
    shutdown = False

    def handle_sigint(sig, frame):
        nonlocal shutdown
        if shutdown:
            sys.exit(1)
        shutdown = True

    signal.signal(signal.SIGINT, handle_sigint)

    # --- Start listening ---
    comms.start()
    log.info("Listening for %d drones on port %d...", len(drone_ids), args.telem_port)

    # --- Interactive triggers + run loop (background thread) ---
    # matplotlib needs the main thread, so we run the trigger/wait logic
    # in a daemon thread and give the main thread to vis.run().
    STAGGER_DELAY_S = 5.0  # seconds between enabling exploration per drone

    def control_loop():
        nonlocal exploring, shutdown

        # Trigger 1: arm + takeoff all drones simultaneously
        print(f"\nPress Enter to send CMD_START to all {len(drone_ids)} drones, "
              "or Ctrl+C to abort.")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            shutdown = True

        if not shutdown:
            log.info("Sending CMD_START to all drones")
            comms.send_all(CommandPacket(CMD_START))

        # Trigger 2: start exploration with staggered goals
        if not shutdown:
            print(f"\nPress Enter to start fleet exploration "
                  f"(staggered {STAGGER_DELAY_S}s apart), or Ctrl+C to land all.")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                shutdown = True

        if not shutdown:
            exploring = True
            for i, did in enumerate(drone_ids):
                if shutdown:
                    break
                exploring_drones.add(did)
                log.info("Exploration enabled → drone %d (%d/%d)",
                         did, i + 1, len(drone_ids))
                if i < len(drone_ids) - 1:
                    time.sleep(STAGGER_DELAY_S)

        # Run until Ctrl+C
        try:
            while not shutdown:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

        # Land all
        log.info("Sending CMD_LAND to all drones")
        comms.send_all(CommandPacket(CMD_LAND))
        time.sleep(1.0)
        # vfh_log.stop()
        comms.stop()

        total_goals = sum(goal_counts.values())
        log.info("Done. %d total goals, %d total crumbs.", total_goals, store.total_crumbs())

    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()

    # Main thread: matplotlib event loop (blocks until window closed)
    try:
        vis.run()
    except KeyboardInterrupt:
        pass

    # If the window is closed, trigger shutdown so control_loop exits
    shutdown = True
    ctrl_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
