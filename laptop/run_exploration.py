#!/usr/bin/env python3
"""
Single-drone exploration test.

Flow:
  1. Starts receiving telemetry from drone.
  2. Waits for user to press Enter → sends CMD_START (drone arms + takes off).
  3. Waits for user to press Enter again → starts sending exploration goals.
  4. Ctrl+C at any point → sends CMD_LAND and exits.

Usage:
    python run_exploration.py [--config setup.yaml] [--drone-id 0]
"""

import argparse
import logging
import signal
import sys
import time

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
log = logging.getLogger("explore")


def main():
    parser = argparse.ArgumentParser(description="Single-drone exploration test")
    parser.add_argument("--config", default="setup.yaml", help="Path to setup.yaml")
    parser.add_argument("--drone-id", type=int, default=0, help="Drone ID to command")
    parser.add_argument("--telem-port", type=int, default=5005, help="UDP port for telemetry")
    parser.add_argument("--cmd-port", type=int, default=5006, help="UDP port for commands")
    args = parser.parse_args()

    drone_id = args.drone_id

    # --- Initialise components ---
    store    = CrumbStore()
    director = ExplorationDirector(store, args.config)
    comms    = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
                         config_path=args.config)

    goal_count    = 0
    last_log_time = 0.0
    last_goal_time = 0.0
    GOAL_INTERVAL  = 0.5   # seconds between exploration goal updates

    # Exploration gated by this flag — set after second user trigger
    exploring = False

    def on_telemetry(pkt: TelemetryPacket, src_ip: str):
        nonlocal goal_count, last_log_time, last_goal_time

        if pkt.drone_id != drone_id:
            return

        # Telemetry position is already in map frame (converted on ESP32)
        map_x, map_y = pkt.ned_x, pkt.ned_y

        # --- Record position into density grid ---
        store.add_position(pkt.drone_id, map_x, map_y)

        # --- Periodic status log (every 2 s) ---
        now = time.monotonic()
        if now - last_log_time > 2.0:
            last_log_time = now
            log.info(
                "drone=%d  pos=(%.2f, %.2f)  nav=%s  crumbs=%d",
                pkt.drone_id, map_x, map_y,
                pkt.nav_state_name,
                store.total_crumbs(),
            )

        if not exploring:
            return

        # --- Send exploration goal when drone is ready (rate-limited) ---
        if pkt.nav_state in (NAV_IDLE, NAV_ARRIVED, NAV_STUCK) \
                and (now - last_goal_time) >= GOAL_INTERVAL:
            goal = director.compute_goal(
                drone_id=pkt.drone_id,
                map_x=map_x,
                map_y=map_y,
                heading_rad=pkt.heading_rad,
                vfh_blocked=pkt.vfh_blocked,
            )
            if goal:
                log.info("Goal #%d → (%.2f, %.2f)", goal_count + 1, goal[0], goal[1])
                cmd = CommandPacket(CMD_GOTO, goal_x=goal[0], goal_y=goal[1])
                comms.send_command(pkt.drone_id, cmd)
                goal_count += 1
                last_goal_time = now

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
    log.info("Listening for drone %d telemetry on port %d...", drone_id, args.telem_port)

    # --- Trigger 1: arm + takeoff ---
    print("\nPress Enter to send CMD_START (arm + takeoff), or Ctrl+C to abort.")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        shutdown = True

    if not shutdown:
        log.info("Sending CMD_START → drone arming and taking off")
        comms.send_command(drone_id, CommandPacket(CMD_START))

    # --- Trigger 2: start exploration ---
    if not shutdown:
        print("\nPress Enter to start sending exploration goals, or Ctrl+C to land.")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            shutdown = True

    if not shutdown:
        log.info("Starting exploration")
        exploring = True

    # --- Run until Ctrl+C ---
    try:
        while not shutdown:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    # --- Land ---
    log.info("Sending CMD_LAND to drone %d", drone_id)
    comms.send_command(drone_id, CommandPacket(CMD_LAND))
    time.sleep(1.0)
    comms.stop()

    log.info("Done. Sent %d goals, collected %d crumbs.", goal_count, store.total_crumbs())


if __name__ == "__main__":
    main()
