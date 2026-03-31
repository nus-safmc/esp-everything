#!/usr/bin/env python3
"""
SAFMC Mission Controller — full autonomous search-and-relay mission.

Phase flow (each phase advanced by pressing Enter):
  1. ARM exploration drones (batch 1 simultaneous takeoff)
  2. BEGIN exploration — autonomous search with stuck recovery
  3. END exploration → plan relay → ARM relay drones (batch 2) → deploy
  4. Ctrl+C at any point → land all and exit

Usage:
    python run_mission.py [--config setup.yaml]

Requires `mission` section in setup.yaml — see setup.yaml for schema.
"""

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path

import yaml

from comms import CommsNode
from crumb_store import CrumbStore
from exploration import ExplorationDirector
from relay import plan_best_relay
from stuck_monitor import StuckMonitor
from visualise import Visualiser
from vfh_logger import VfhLogger
from protocol import (
    CMD_GOTO, CMD_HOLD, CMD_LAND, CMD_START, CommandPacket,
    NAV_IDLE, NAV_ARRIVED, NAV_STUCK,
    TelemetryPacket,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("mission")

# Tuning constants
EXPLORE_GOAL_INTERVAL_S = 2.5   # min seconds between explore goals per drone
RELAY_RESEND_INTERVAL_S = 3.0   # retry interval for stuck/idle relay drones
STATUS_LOG_INTERVAL_S   = 5.0   # periodic fleet status log


def main():
    ap = argparse.ArgumentParser(description="SAFMC mission controller")
    ap.add_argument("--config", default="setup.yaml")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()

    cfg  = yaml.safe_load(Path(args.config).read_text())
    mcfg = cfg.get("mission", {})

    # Drone assignments — default explore_ids = all drones in config
    all_drone_keys = sorted(int(d) for d in cfg["drones"])
    explore_ids   = sorted(mcfg.get("explore_ids", all_drone_keys))
    relay_ids     = sorted(mcfg.get("relay_ids", []))
    all_ids       = sorted(set(explore_ids + relay_ids))

    relay_start   = tuple(mcfg.get("relay_start", [0.0, 0.0]))
    bonus_tag_ids = set(mcfg.get("bonus_tag_ids", []))
    relay_spacing = mcfg.get("relay_spacing_m", 0.9)

    log.info("Explorers : %s (%d drones)", explore_ids, len(explore_ids))
    log.info("Relay     : %s (%d drones)", relay_ids, len(relay_ids))
    if bonus_tag_ids:
        log.info("Bonus tags: %s", bonus_tag_ids)

    # ------------------------------------------------------------------
    # Components
    # ------------------------------------------------------------------
    arena = cfg["arena"]
    arena_bounds = (arena["min_x"], arena["max_x"], arena["min_y"], arena["max_y"])
    store    = CrumbStore(arena_bounds=arena_bounds)
    director = ExplorationDirector(store, args.config)
    monitor  = StuckMonitor(director, store, args.config)
    vfh_log  = VfhLogger()
    comms    = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
                         config_path=args.config)

    # ------------------------------------------------------------------
    # Shared state (written by telemetry callback, read by control loop)
    # ------------------------------------------------------------------
    drone_telem:   dict[int, TelemetryPacket | None] = {d: None for d in all_ids}
    last_goal_time: dict[int, float] = {d: 0.0 for d in all_ids}
    goal_counts:    dict[int, int]   = {d: 0   for d in all_ids}

    # Tag discovery tracking
    landed_tags:  dict[int, int]                = {}   # drone_id → tag_id
    bonus_found:  dict[int, tuple[float, float]] = {}   # tag_id → (map_x, map_y)

    # Relay execution
    relay_assignments: dict[int, tuple[float, float]] = {}   # drone_id → position
    relay_landed:      set[int] = set()

    # Phase flags
    exploring = False
    relaying  = False
    shutdown  = False
    last_log  = 0.0

    # ------------------------------------------------------------------
    # Telemetry callback (runs in CommsNode recv thread)
    # ------------------------------------------------------------------
    def on_telemetry(pkt: TelemetryPacket, src_ip: str):
        nonlocal last_log

        did = pkt.drone_id
        if did not in all_ids:
            return

        drone_telem[did] = pkt
        store.add_position(did, pkt.ned_x, pkt.ned_y)
        now = time.monotonic()

        # ---- Track tag discoveries from exploration drones ----
        if did in explore_ids and pkt.tag_id >= 0 and did not in landed_tags:
            landed_tags[did] = pkt.tag_id
            log.info(">> Drone %d found tag %d at (%.2f, %.2f)",
                     did, pkt.tag_id, pkt.ned_x, pkt.ned_y)
            if pkt.tag_id in bonus_tag_ids:
                bonus_found[pkt.tag_id] = (pkt.ned_x, pkt.ned_y)
                log.info(">> BONUS VICTIM tag %d discovered!", pkt.tag_id)

        # ---- Periodic status log ----
        if now - last_log > STATUS_LOG_INTERVAL_S:
            last_log = now
            n_active = sum(1 for d in explore_ids if d not in landed_tags)
            log.info("active=%d/%d  tags=%s  bonus=%s  crumbs=%d",
                     n_active, len(explore_ids),
                     list(landed_tags.values()) or "none",
                     list(bonus_found.keys()) or "none",
                     store.total_crumbs())

        # ---- Exploration goal logic ----
        if exploring and did in explore_ids and did not in landed_tags:
            _exploration_tick(did, pkt, now)

        # ---- Relay arrival / resend logic ----
        if relaying and did in relay_assignments and did not in relay_landed:
            _relay_tick(did, pkt, now)

    def _exploration_tick(did: int, pkt: TelemetryPacket, now: float):
        """Handle one explorer drone's telemetry: stuck check then goal."""
        # Stuck monitor — fires once when newly stuck
        if monitor.update(did, pkt.ned_x, pkt.ned_y, pkt.heading_rad):
            esc = monitor.escape_goal(did)
            if esc:
                log.info("Drone %d STUCK — escape → (%.2f, %.2f)", did, esc[0], esc[1])
                comms.send_command(did, CommandPacket(CMD_GOTO,
                                                     goal_x=esc[0], goal_y=esc[1]))
                last_goal_time[did] = now
                goal_counts[did] += 1
            monitor.clear_stuck(did)
            return   # don't send a normal goal on the same tick

        # Normal exploration goal (rate-limited)
        if pkt.nav_state in (NAV_IDLE, NAV_ARRIVED, NAV_STUCK) \
                and (now - last_goal_time[did]) >= EXPLORE_GOAL_INTERVAL_S:
            goal = director.compute_goal(
                drone_id=did,
                map_x=pkt.ned_x,
                map_y=pkt.ned_y,
                heading_rad=pkt.heading_rad,
                vfh_blocked=pkt.vfh_blocked,
                stuck_zones=monitor.stuck_zones(),
            )
            if goal:
                comms.send_command(did, CommandPacket(CMD_GOTO,
                                                     goal_x=goal[0], goal_y=goal[1]))
                goal_counts[did] += 1
                last_goal_time[did] = now

    def _relay_tick(did: int, pkt: TelemetryPacket, now: float):
        """Handle one relay drone's telemetry: land on arrival, resend if stuck."""
        if pkt.nav_state == NAV_ARRIVED:
            log.info("Relay drone %d ARRIVED — landing", did)
            comms.send_command(did, CommandPacket(CMD_LAND))
            relay_landed.add(did)
        elif pkt.nav_state in (NAV_STUCK, NAV_IDLE) \
                and (now - last_goal_time.get(did, 0)) > RELAY_RESEND_INTERVAL_S:
            pos = relay_assignments[did]
            log.info("Relay drone %d resending goal (%.2f, %.2f)", did, pos[0], pos[1])
            comms.send_command(did, CommandPacket(CMD_GOTO,
                                                 goal_x=pos[0], goal_y=pos[1]))
            last_goal_time[did] = now

    # ------------------------------------------------------------------
    # Register callbacks & start comms
    # ------------------------------------------------------------------
    comms.on_telemetry(on_telemetry)
    comms.on_telemetry(vfh_log.feed)

    vis = Visualiser(store, args.config)
    comms.on_telemetry(vis.feed)

    comms.start()
    log.info("Comms started on port %d", args.telem_port)

    # ------------------------------------------------------------------
    # Signal handler
    # ------------------------------------------------------------------
    def sigint_handler(sig, frame):
        nonlocal shutdown
        if shutdown:
            sys.exit(1)       # second Ctrl+C = force quit
        shutdown = True

    signal.signal(signal.SIGINT, sigint_handler)

    # ------------------------------------------------------------------
    # Control loop (daemon thread — main thread runs matplotlib)
    # ------------------------------------------------------------------
    def control_loop():
        nonlocal exploring, relaying, shutdown

        # ==============================================================
        # PHASE 1 — Arm & take off exploration drones
        # ==============================================================
        print(f"\n{'='*60}")
        print(f"  PHASE 1: Press Enter to ARM {len(explore_ids)} exploration drones")
        print(f"{'='*60}")
        if _wait_enter():
            return _land_all()

        log.info("CMD_START → explorers %s", explore_ids)
        for did in explore_ids:
            comms.send_command(did, CommandPacket(CMD_START))

        # ==============================================================
        # PHASE 2 — Exploration
        # ==============================================================
        print(f"\n{'='*60}")
        print(f"  PHASE 2: Press Enter to BEGIN autonomous exploration")
        print(f"{'='*60}")
        if _wait_enter():
            return _land_all()

        exploring = True
        log.info("=== EXPLORATION STARTED ===")

        if relay_ids:
            print(f"\n  Exploring with {len(explore_ids)} drones...")
            print(f"  Press Enter to END exploration and start relay phase")
            print(f"  ({len(relay_ids)} relay drones available)")
        else:
            print(f"\n  Exploring with {len(explore_ids)} drones...")
            print(f"  No relay drones configured. Ctrl+C to land all.")

        if _wait_enter():
            return _land_all()

        # ==============================================================
        # PHASE 3 — Transition: stop exploration, plan relay
        # ==============================================================
        exploring = False
        log.info("=== EXPLORATION ENDED ===")

        # Hold remaining active explorers in place
        for did in explore_ids:
            if did not in landed_tags:
                comms.send_command(did, CommandPacket(CMD_HOLD))
                log.info("Drone %d holding position", did)

        if not relay_ids:
            log.info("No relay drones — waiting for Ctrl+C")
            _wait_shutdown()
            return _land_all()

        # ---- Plan relay ----
        targets = {str(tid): pos for tid, pos in bonus_found.items()}
        if not targets:
            log.warning("No bonus victims found — falling back to all discovered tags")
            for did, tid in landed_tags.items():
                st = drone_telem[did]
                if st is not None:
                    targets[str(tid)] = (st.ned_x, st.ned_y)

        if not targets:
            log.error("No relay targets available at all!")
            print("\n  No targets found. Ctrl+C to land all.")
            _wait_shutdown()
            return _land_all()

        log.info("Planning relay from (%.1f, %.1f) to %d candidate targets...",
                 relay_start[0], relay_start[1], len(targets))
        plan = plan_best_relay(store, relay_start, targets,
                               max_spacing=relay_spacing)
        if plan is None:
            log.error("No traversable path to any target!")
            print("\n  No relay path found (need more explored area?). Ctrl+C to land.")
            _wait_shutdown()
            return _land_all()

        # relay_positions: [start_area, ..., victim_position]
        # Exclude last position — the exploration drone already landed there.
        relay_positions = plan.relay_positions[:-1]
        n_needed = len(relay_positions)

        log.info("Relay plan: target=tag %s | %.1fm path | %d positions",
                 plan.target_id, plan.path_length_m, n_needed)
        for i, pos in enumerate(relay_positions):
            log.info("  Position %d: (%.2f, %.2f)", i, pos[0], pos[1])

        if n_needed > len(relay_ids):
            log.error("Need %d relay drones but only %d available!",
                      n_needed, len(relay_ids))
            print(f"\n  Not enough relay drones ({n_needed} needed, "
                  f"{len(relay_ids)} available). Ctrl+C to land.")
            _wait_shutdown()
            return _land_all()

        # Assign drones to positions
        used_relay_ids = relay_ids[:n_needed]
        for i, pos in enumerate(relay_positions):
            relay_assignments[used_relay_ids[i]] = pos
            log.info("  Drone %d → (%.2f, %.2f)", used_relay_ids[i], pos[0], pos[1])

        # ==============================================================
        # PHASE 3b — Arm relay drones
        # ==============================================================
        print(f"\n{'='*60}")
        print(f"  PHASE 3: Press Enter to ARM {n_needed} relay drones")
        print(f"  Plan: {n_needed} drones, {plan.path_length_m:.1f}m path to tag {plan.target_id}")
        print(f"{'='*60}")
        if _wait_enter():
            return _land_all()

        log.info("CMD_START → relay drones %s", used_relay_ids)
        for did in used_relay_ids:
            comms.send_command(did, CommandPacket(CMD_START))

        # ==============================================================
        # PHASE 4 — Deploy relay drones
        # ==============================================================
        print(f"\n{'='*60}")
        print(f"  PHASE 4: Press Enter to DEPLOY relay drones to positions")
        print(f"{'='*60}")
        if _wait_enter():
            return _land_all()

        relaying = True
        for did, pos in relay_assignments.items():
            log.info("CMD_GOTO drone %d → (%.2f, %.2f)", did, pos[0], pos[1])
            comms.send_command(did, CommandPacket(CMD_GOTO,
                                                 goal_x=pos[0], goal_y=pos[1]))
            last_goal_time[did] = time.monotonic()

        log.info("Waiting for %d relay drones to arrive and land...", n_needed)

        # Wait for all relay drones to land (or shutdown)
        while not shutdown:
            if len(relay_landed) >= len(relay_assignments):
                break
            remaining = [d for d in relay_assignments if d not in relay_landed]
            time.sleep(1.0)

        if len(relay_landed) >= len(relay_assignments):
            log.info("=== RELAY COMPLETE — all %d drones landed ===",
                     len(relay_landed))
            print(f"\n  RELAY COMPLETE! Chain of {len(relay_landed)} drones formed.")
            print(f"  Ctrl+C to finish mission.")
            _wait_shutdown()

        _land_all()

    # ------------------------------------------------------------------
    # Helpers for control flow
    # ------------------------------------------------------------------
    def _wait_enter() -> bool:
        """Block until Enter. Returns True if shutdown requested."""
        try:
            input()
            return shutdown
        except (EOFError, KeyboardInterrupt):
            return True

    def _wait_shutdown():
        """Block until Ctrl+C / shutdown flag."""
        try:
            while not shutdown:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

    def _land_all():
        nonlocal shutdown
        shutdown = True
        log.info("CMD_LAND → all drones")
        comms.send_all(CommandPacket(CMD_LAND))
        time.sleep(1.0)
        vfh_log.stop()
        comms.stop()

        log.info("Mission complete. goals=%d  tags_found=%d  bonus=%d  crumbs=%d",
                 sum(goal_counts.values()),
                 len(landed_tags),
                 len(bonus_found),
                 store.total_crumbs())

    # ------------------------------------------------------------------
    # Launch
    # ------------------------------------------------------------------
    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()

    # Main thread: matplotlib event loop (required by Tk backend)
    try:
        vis.run()
    except KeyboardInterrupt:
        pass

    # If visualiser window closed, trigger shutdown
    shutdown = True
    ctrl_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
