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
import signal
import sys
import time
from pathlib import Path

import yaml

from comms import CommsNode
from crumb_store import CrumbStore
from exploration import ExplorationDirector
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

    # --- Initialise components ---
    store    = CrumbStore()
    director = ExplorationDirector(store, args.config)
    comms    = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
                         config_path=args.config)

    GOAL_INTERVAL = 0.5  # seconds between goal updates per drone

    # Per-drone state
    last_goal_time: dict[int, float]              = {d: 0.0 for d in drone_ids}
    goal_counts:    dict[int, int]                = {d: 0 for d in drone_ids}
    drone_state:    dict[int, TelemetryPacket | None] = {d: None for d in drone_ids}
    last_log_time = 0.0

    exploring = False

    def on_telemetry(pkt: TelemetryPacket, src_ip: str):
        nonlocal last_log_time

        if pkt.drone_id not in drone_ids:
            return

        did = pkt.drone_id

        # --- Ingest crumbs ---
        if pkt.crumbs:
            store.add_crumbs(
                drone_id=did,
                crumbs=pkt.crumbs,
                start_index=pkt.crumb_batch_start,
            )

        map_x, map_y = pkt.ned_x, pkt.ned_y
        drone_state[did] = pkt
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

        if not exploring:
            return

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

    # --- Trigger 1: arm + takeoff all drones ---
    print(f"\nPress Enter to send CMD_START to all {len(drone_ids)} drones, "
          "or Ctrl+C to abort.")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        shutdown = True

    if not shutdown:
        log.info("Sending CMD_START to all drones")
        comms.send_all(CommandPacket(CMD_START))

    # --- Trigger 2: start exploration ---
    if not shutdown:
        print("\nPress Enter to start fleet exploration, or Ctrl+C to land all.")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            shutdown = True

    if not shutdown:
        log.info("Starting fleet exploration")
        exploring = True

    # --- Run until Ctrl+C ---
    try:
        while not shutdown:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    # --- Land all ---
    log.info("Sending CMD_LAND to all drones")
    comms.send_all(CommandPacket(CMD_LAND))
    time.sleep(1.0)
    comms.stop()

    total_goals = sum(goal_counts.values())
    log.info("Done. %d total goals, %d total crumbs.", total_goals, store.total_crumbs())


if __name__ == "__main__":
    main()
