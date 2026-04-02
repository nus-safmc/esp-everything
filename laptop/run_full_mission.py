#!/usr/bin/env python3
"""
Full autonomous SAFMC mission — explore then relay, single-keypress execution.

Flow:
  1. Press Enter  → first-phase (exploration) drones arm & take off.
  2. After TAKEOFF_DELAY_S the exploration begins automatically (staggered).
  3. After EXPLORE_DURATION_S (7 min) the exploration stops.
  4. Relay is planned → remaining explorers are diverted to relay positions
     → second-phase (relay) drones arm, take off, and fill the rest.
  5. Drones deploy to relay positions one by one.
  6. Ctrl+C at any point → land all and exit.

Drone sets are read from the ``mission`` section of setup.yaml:
  explore_ids  — first-phase exploration drones
  relay_ids    — second-phase relay drones

Usage:
    python run_full_mission.py [--config setup.yaml]
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
from relay import plan_best_relay
from visualise import Visualiser
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

# ─────────────────────────────────────────────────────────────────────────────
# Timing constants  (edit these to tune the mission)
# ─────────────────────────────────────────────────────────────────────────────
TAKEOFF_DELAY_S        = 15       # wait after CMD_START before exploration begins
EXPLORE_DURATION_S     = 7 * 60   # 7-minute exploration window
STAGGER_DELAY_S        = 5.0      # gap between enabling each explorer
RELAY_DEPLOY_STAGGER_S = 5.0      # gap between deploying each relay drone
RELAY_TAKEOFF_DELAY_S  = 12.0     # wait for relay drones to take off
RELAY_TIMEOUT_S        = 180      # max wait for relay drones to land

# Exploration tuning  (unchanged from run_fleet_exploration.py)
GOAL_INTERVAL_S        = 2.5
RELOC_AGE_THRESHOLD_S  = 120
STATUS_LOG_INTERVAL_S  = 2.0
RELAY_RESEND_INTERVAL_S = 3.0


def main():
    ap = argparse.ArgumentParser(description="Full autonomous SAFMC mission")
    ap.add_argument("--config", default="setup.yaml")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()

    cfg  = yaml.safe_load(Path(args.config).read_text())
    mcfg = cfg.get("mission", {})

    # ── Drone assignments from config ────────────────────────────────────
    all_drone_keys = sorted(int(d) for d in cfg["drones"])
    explore_ids = sorted(mcfg.get("explore_ids", all_drone_keys))
    relay_ids   = sorted(mcfg.get("relay_ids", []))
    all_ids     = sorted(set(explore_ids + relay_ids))

    relay_start   = tuple(mcfg.get("relay_start", [0.0, 0.0]))
    bonus_tag_ids = set(mcfg.get("bonus_tag_ids", []))
    relay_spacing = mcfg.get("relay_spacing_m", 0.9)

    log.info("Explorers  : %s (%d)", explore_ids, len(explore_ids))
    log.info("Relay pool : %s (%d)", relay_ids, len(relay_ids))
    log.info("Relay start: (%.1f, %.1f)  spacing=%.2fm",
             relay_start[0], relay_start[1], relay_spacing)
    if bonus_tag_ids:
        log.info("Bonus tags : %s", bonus_tag_ids)

    # Start positions (for nav-tag odom conversion)
    drone_starts: dict[int, tuple[float, float]] = {}
    for did_key in cfg["drones"]:
        dcfg = cfg["drones"][did_key]
        drone_starts[int(did_key)] = (dcfg["start_x"], dcfg["start_y"])

    arena = cfg["arena"]
    arena_bounds = (arena["min_x"], arena["max_x"],
                    arena["min_y"], arena["max_y"])

    # ── Components ───────────────────────────────────────────────────────
    store    = CrumbStore(start_positions=list(drone_starts.values()),
                          arena_bounds=arena_bounds)
    director = ExplorationDirector(store, args.config)
    comms    = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port,
                         config_path=args.config)

    # ── Shared state ─────────────────────────────────────────────────────
    last_goal_time: dict[int, float]                 = {d: 0.0  for d in all_ids}
    goal_counts:    dict[int, int]                   = {d: 0    for d in all_ids}
    drone_state:    dict[int, TelemetryPacket | None] = {d: None for d in all_ids}
    reloc_sent:     dict[int, bool]                  = {d: False for d in explore_ids}

    # Tag / victim tracking
    landed_tags:  dict[int, int]                 = {}  # drone_id → tag_id
    bonus_found:  dict[int, tuple[float, float]] = {}  # tag_id  → (x, y)

    # Relay execution
    relay_assignments: dict[int, tuple[float, float]] = {}  # drone_id → pos
    relay_deployed:    set[int] = set()   # drones that have been sent their GOTO
    relay_landed:      set[int] = set()

    # Phase flags  (written by control_loop, read by telemetry callback)
    exploring       = False
    relaying        = False
    shutdown        = False
    exploring_drones: set[int] = set()
    last_log_time   = 0.0

    # ── Telemetry callback (runs in CommsNode recv thread) ───────────────
    def on_telemetry(pkt: TelemetryPacket, src_ip: str):
        nonlocal last_log_time

        did = pkt.drone_id
        if did not in all_ids:
            return

        drone_state[did] = pkt
        map_x, map_y = pkt.ned_x, pkt.ned_y
        store.add_position(did, map_x, map_y)
        now = time.monotonic()

        # ── Passive tag-discovery tracking (does not send commands) ──
        if did in explore_ids and pkt.tag_id >= 0 and did not in landed_tags:
            landed_tags[did] = pkt.tag_id
            log.info(">> Drone %d found tag %d at (%.2f, %.2f)",
                     did, pkt.tag_id, map_x, map_y)
            if pkt.tag_id in bonus_tag_ids:
                bonus_found[pkt.tag_id] = (map_x, map_y)
                log.info(">> BONUS VICTIM tag %d discovered!", pkt.tag_id)

        # ── Periodic status log ──────────────────────────────────────
        if now - last_log_time > STATUS_LOG_INTERVAL_S:
            last_log_time = now
            for d in all_ids:
                st = drone_state[d]
                if st is not None:
                    log.info(
                        "drone=%d  pos=(%.2f, %.2f)  nav=%s  goals=%d  crumbs=%d",
                        d, st.ned_x, st.ned_y, st.nav_state_name,
                        goal_counts[d], store.crumb_count(d),
                    )

        # ── Exploration goal logic  (identical to run_fleet_exploration) ─
        if exploring and did in exploring_drones:
            _exploration_tick(did, pkt, now, map_x, map_y)

        # ── Relay arrival / resend logic ─────────────────────────────
        if relaying and did in relay_deployed and did not in relay_landed:
            _relay_tick(did, pkt, now)

    # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    # Exploration tick — UNCHANGED from run_fleet_exploration.py
    # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    def _exploration_tick(did: int, pkt: TelemetryPacket,
                          now: float, map_x: float, map_y: float):
        # ── Relocalisation: send drone to nearest nav tag if odom stale ──
        needs_reloc = (pkt.tag_id < 0
                       and pkt.reloc_age_s != 0xFFFF
                       and pkt.reloc_age_s > RELOC_AGE_THRESHOLD_S)

        if needs_reloc and not reloc_sent.get(did, False):
            best_tag = None
            best_dist = float("inf")
            for tag in director.nav_tags:
                d = math.hypot(tag.map_x - map_x, tag.map_y - map_y)
                if d < best_dist:
                    best_dist = d
                    best_tag = tag
            if best_tag is not None:
                log.warning(
                    "drone=%d  reloc_age=%ds > %ds — sending to nav tag %d "
                    "at (%.2f, %.2f)  dist=%.1fm",
                    did, pkt.reloc_age_s, RELOC_AGE_THRESHOLD_S,
                    best_tag.id, best_tag.map_x, best_tag.map_y, best_dist)
                comms.send_command(did, CommandPacket(
                    CMD_GOTO, goal_x=best_tag.map_x, goal_y=best_tag.map_y))
                reloc_sent[did] = True
                last_goal_time[did] = now
                return

        if reloc_sent.get(did, False) and not needs_reloc:
            log.info("drone=%d  relocalised (reloc_age=%ds) — resuming exploration",
                     did, pkt.reloc_age_s)
            reloc_sent[did] = False

        # ── Send exploration goal when this drone is ready (rate-limited) ─
        if pkt.nav_state in (NAV_IDLE, NAV_ARRIVED, NAV_STUCK) \
                and (now - last_goal_time[did]) >= GOAL_INTERVAL_S:
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
                comms.send_command(did, CommandPacket(
                    CMD_GOTO, goal_x=goal[0], goal_y=goal[1]))
                goal_counts[did] += 1
                last_goal_time[did] = now

    # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    # Relay tick
    # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    def _relay_tick(did: int, pkt: TelemetryPacket, now: float):
        if pkt.nav_state == NAV_ARRIVED:
            log.info("Relay drone %d ARRIVED — landing", did)
            comms.send_command(did, CommandPacket(CMD_LAND))
            relay_landed.add(did)
        elif pkt.nav_state in (NAV_STUCK, NAV_IDLE) \
                and (now - last_goal_time.get(did, 0)) > RELAY_RESEND_INTERVAL_S:
            pos = relay_assignments[did]
            log.info("Relay drone %d resending goal (%.2f, %.2f)",
                     did, pos[0], pos[1])
            comms.send_command(did, CommandPacket(
                CMD_GOTO, goal_x=pos[0], goal_y=pos[1]))
            last_goal_time[did] = now

    # ── Register callbacks & start comms ─────────────────────────────────
    comms.on_telemetry(on_telemetry)

    vis = Visualiser(store, args.config)
    comms.on_telemetry(vis.feed)

    comms.start()
    log.info("Comms started on port %d", args.telem_port)

    # ── Signal handler ───────────────────────────────────────────────────
    def sigint_handler(sig, frame):
        nonlocal shutdown
        if shutdown:
            sys.exit(1)
        shutdown = True

    signal.signal(signal.SIGINT, sigint_handler)

    # ─────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────
    def _sleep_or_abort(seconds: float) -> bool:
        """Sleep in 0.5 s chunks.  Returns True if shutdown was requested."""
        for _ in range(int(seconds * 2)):
            if shutdown:
                return True
            time.sleep(0.5)
        return False

    def _wait_shutdown():
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
        comms.stop()
        log.info("Mission done.  goals=%d  tags=%d  bonus=%d  crumbs=%d",
                 sum(goal_counts.values()), len(landed_tags),
                 len(bonus_found), store.total_crumbs())

    # ─────────────────────────────────────────────────────────────────────
    # Control loop  (daemon thread — main thread runs matplotlib)
    # ─────────────────────────────────────────────────────────────────────
    def control_loop():
        nonlocal exploring, relaying, shutdown

        # ═════════════════════════════════════════════════════════════════
        # SINGLE TRIGGER — press Enter to start the full mission
        # ═════════════════════════════════════════════════════════════════
        print(f"\n{'='*60}")
        print(f"  FULL MISSION")
        print(f"  Phase 1: {len(explore_ids)} explorers  ({EXPLORE_DURATION_S}s)")
        print(f"  Phase 2: up to {len(relay_ids)} relay drones")
        print(f"")
        print(f"  Press Enter to ARM explorers and begin mission")
        print(f"  Ctrl+C at any time → land all and abort")
        print(f"{'='*60}")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            return _land_all()
        if shutdown:
            return _land_all()

        # ═════════════════════════════════════════════════════════════════
        # PHASE 1a — Arm & take off exploration drones
        # ═════════════════════════════════════════════════════════════════
        log.info("CMD_START → explorers %s", explore_ids)
        for did in explore_ids:
            comms.send_command(did, CommandPacket(CMD_START))

        log.info("Waiting %ds for takeoff...", TAKEOFF_DELAY_S)
        if _sleep_or_abort(TAKEOFF_DELAY_S):
            return _land_all()

        # ═════════════════════════════════════════════════════════════════
        # PHASE 1b — Exploration (7 min, auto-start, staggered)
        # ═════════════════════════════════════════════════════════════════
        exploring = True
        explore_start = time.monotonic()
        log.info("=== EXPLORATION STARTED (%ds timer) ===", EXPLORE_DURATION_S)

        # Stagger-enable exploration for each drone
        for i, did in enumerate(explore_ids):
            if shutdown:
                return _land_all()
            exploring_drones.add(did)
            log.info("Exploration enabled → drone %d (%d/%d)",
                     did, i + 1, len(explore_ids))
            if i < len(explore_ids) - 1:
                if _sleep_or_abort(STAGGER_DELAY_S):
                    return _land_all()

        # Wait for 7-minute timer, logging countdown every 60 s
        last_countdown = 0
        while not shutdown:
            elapsed = time.monotonic() - explore_start
            remaining = EXPLORE_DURATION_S - elapsed
            if remaining <= 0:
                break
            mins_left = int(remaining) // 60
            if mins_left != last_countdown and mins_left >= 1:
                last_countdown = mins_left
                log.info("Exploration: %d min remaining", mins_left)
            time.sleep(0.5)

        if shutdown:
            return _land_all()

        # ═════════════════════════════════════════════════════════════════
        # PHASE 2 — End exploration → plan relay
        # ═════════════════════════════════════════════════════════════════
        exploring = False
        log.info("=== EXPLORATION ENDED (%.0fs elapsed) ===",
                 time.monotonic() - explore_start)

        # Hold still-flying explorers
        active_explorers: list[int] = []
        for did in explore_ids:
            if did not in landed_tags:
                comms.send_command(did, CommandPacket(CMD_HOLD))
                active_explorers.append(did)
                log.info("Drone %d → HOLD", did)

        # ── Collect relay targets ────────────────────────────────────
        targets: dict[str, tuple[float, float]] = {
            str(tid): pos for tid, pos in bonus_found.items()
        }
        if not targets:
            log.warning("No bonus victims — falling back to all discovered tags")
            for did, tid in landed_tags.items():
                st = drone_state[did]
                if st is not None:
                    targets[str(tid)] = (st.ned_x, st.ned_y)

        if not targets:
            log.error("No relay targets found at all!")
            _wait_shutdown()
            return _land_all()

        # ── Plan best relay path ─────────────────────────────────────
        log.info("Planning relay from (%.1f, %.1f) to %d candidate target(s)...",
                 relay_start[0], relay_start[1], len(targets))
        plan = plan_best_relay(store, relay_start, targets,
                               max_spacing=relay_spacing)
        if plan is None:
            log.error("No traversable relay path found!")
            _wait_shutdown()
            return _land_all()

        # Last position is the victim — an explorer already landed there
        relay_positions = plan.relay_positions[:-1]
        n_needed = len(relay_positions)

        log.info("Relay plan: target=tag %s  path=%.1fm  positions=%d",
                 plan.target_id, plan.path_length_m, n_needed)
        for i, pos in enumerate(relay_positions):
            log.info("  [%d] (%.2f, %.2f)", i, pos[0], pos[1])

        # ── Assign drones to relay positions ─────────────────────────
        # Explorers → greedily assigned to nearest available position
        # Relay drones → fill the remaining positions (start-area first)
        available_pos_idxs = list(range(n_needed))

        explorer_assign: list[tuple[int, tuple[float, float]]] = []
        for did in active_explorers:
            if not available_pos_idxs:
                break
            st = drone_state[did]
            if st is None:
                continue
            # Pick the closest unassigned relay position
            best_idx = min(
                available_pos_idxs,
                key=lambda idx: math.hypot(
                    relay_positions[idx][0] - st.ned_x,
                    relay_positions[idx][1] - st.ned_y,
                ),
            )
            explorer_assign.append((did, relay_positions[best_idx]))
            available_pos_idxs.remove(best_idx)

        # Remaining positions → relay_ids (sorted closest-to-start first)
        available_pos_idxs.sort()
        relay_drone_assign: list[tuple[int, tuple[float, float]]] = []
        for i, pos_idx in enumerate(available_pos_idxs):
            if i >= len(relay_ids):
                break
            relay_drone_assign.append((relay_ids[i], relay_positions[pos_idx]))

        # Register all assignments for the relay_tick callback
        for did, pos in explorer_assign + relay_drone_assign:
            relay_assignments[did] = pos

        total_assigned = len(explorer_assign) + len(relay_drone_assign)
        log.info("Relay assignments: %d explorers + %d relay drones = %d / %d needed",
                 len(explorer_assign), len(relay_drone_assign),
                 total_assigned, n_needed)
        if total_assigned < n_needed:
            log.warning("Not enough drones to fill all relay positions!")

        if total_assigned == 0:
            log.error("No drones available for relay!")
            _wait_shutdown()
            return _land_all()

        # ═════════════════════════════════════════════════════════════════
        # PHASE 3 — Deploy relay drones  (explorers first, then relay)
        # ═════════════════════════════════════════════════════════════════
        relaying = True

        # Step A: divert active explorers to their relay positions
        if explorer_assign:
            log.info("Diverting %d explorers to relay positions...",
                     len(explorer_assign))
        for did, pos in explorer_assign:
            if shutdown:
                return _land_all()
            log.info("  Explorer %d → relay (%.2f, %.2f)", did, pos[0], pos[1])
            comms.send_command(did, CommandPacket(
                CMD_GOTO, goal_x=pos[0], goal_y=pos[1]))
            last_goal_time[did] = time.monotonic()
            relay_deployed.add(did)
            if _sleep_or_abort(RELAY_DEPLOY_STAGGER_S):
                return _land_all()

        # Step B: arm relay drones, wait for takeoff, then deploy
        if relay_drone_assign:
            used_relay_ids = [did for did, _ in relay_drone_assign]
            log.info("CMD_START → relay drones %s", used_relay_ids)
            for did in used_relay_ids:
                comms.send_command(did, CommandPacket(CMD_START))

            log.info("Waiting %ds for relay drone takeoff...",
                     int(RELAY_TAKEOFF_DELAY_S))
            if _sleep_or_abort(RELAY_TAKEOFF_DELAY_S):
                return _land_all()

            log.info("Deploying %d relay drones one by one...",
                     len(relay_drone_assign))
            for did, pos in relay_drone_assign:
                if shutdown:
                    return _land_all()
                log.info("  Relay drone %d → (%.2f, %.2f)", did, pos[0], pos[1])
                comms.send_command(did, CommandPacket(
                    CMD_GOTO, goal_x=pos[0], goal_y=pos[1]))
                last_goal_time[did] = time.monotonic()
                relay_deployed.add(did)
                if _sleep_or_abort(RELAY_DEPLOY_STAGGER_S):
                    return _land_all()

        # ── Wait for all relay drones to arrive and land ─────────────
        relay_wait_start = time.monotonic()
        log.info("Waiting for %d relay drones to land (timeout %ds)...",
                 total_assigned, RELAY_TIMEOUT_S)
        while not shutdown:
            if len(relay_landed) >= total_assigned:
                break
            if time.monotonic() - relay_wait_start > RELAY_TIMEOUT_S:
                log.warning("Relay timeout! %d/%d landed",
                            len(relay_landed), total_assigned)
                break
            time.sleep(0.5)

        if len(relay_landed) >= total_assigned:
            log.info("=== RELAY COMPLETE — %d drones landed ===",
                     len(relay_landed))
        else:
            remaining = [d for d in relay_assignments if d not in relay_landed]
            log.warning("Relay incomplete: %d/%d landed  (remaining: %s)",
                        len(relay_landed), total_assigned, remaining)

        # Land any leftover explorers not used in the relay
        for did in explore_ids:
            if did not in landed_tags and did not in relay_assignments:
                log.info("Landing unused explorer %d", did)
                comms.send_command(did, CommandPacket(CMD_LAND))

        log.info("Mission sequence complete. Ctrl+C to shut down.")
        _wait_shutdown()
        _land_all()

    # ── Launch ───────────────────────────────────────────────────────────
    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()

    # Main thread: matplotlib event loop (required by Tk backend)
    try:
        vis.run()
    except KeyboardInterrupt:
        pass

    shutdown = True
    ctrl_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
