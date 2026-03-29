"""
CommsNode — UDP communication hub between the laptop and all drones.

Usage:
    from exploration import load_nav_tags, load_drone_starts

    node = CommsNode(listen_port=5005, cmd_port=5006)
    node.set_nav_tags(load_nav_tags("setup.yaml"),
                      load_drone_starts("setup.yaml"))
    node.on_telemetry(lambda pkt, ip: print(pkt.drone_id, pkt.ned_x, pkt.ned_y))
    node.start()
    # Nav tags (with per-drone odom-frame positions) are sent automatically
    # on first contact and re-broadcast every 20 s.

    node.send_command(0, CommandPacket(CMD_GOTO, goal_x=2.0, goal_y=1.6))
    node.send_all(CommandPacket(CMD_LAND))

    node.stop()
"""

import socket
import threading
import logging
import time
from pathlib import Path
from typing import Callable, Optional

import yaml

from protocol import (
    TelemetryPacket, CommandPacket, NavTag,
    parse_telemetry, build_command, build_nav_tags_command, MAX_FOUND_TAGS,
)

log = logging.getLogger(__name__)

TelemetryCallback = Callable[[TelemetryPacket, str], None]

NAV_TAG_RESEND_INTERVAL = 20.0   # seconds between periodic re-broadcasts


def _load_nav_tags(config_path: str) -> tuple[list[NavTag], dict[int, tuple[float, float]]]:
    """Load nav tags and drone start positions from setup.yaml."""
    cfg = yaml.safe_load(Path(config_path).read_text())
    tags = [
        NavTag(id=int(tag_id), map_x=pos["map_x"], map_y=pos["map_y"])
        for tag_id, pos in cfg.get("nav_tags", {}).items()
    ]
    drone_starts = {
        int(did): (float(props["start_x"]), float(props["start_y"]))
        for did, props in cfg.get("drones", {}).items()
    }
    return tags, drone_starts


class CommsNode:
    def __init__(self, listen_port: int = 5005, cmd_port: int = 5006,
                 config_path: str | None = None):
        self._listen_port = listen_port
        self._cmd_port    = cmd_port

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._nav_tag_thread: Optional[threading.Thread] = None
        self._running = False

        self._callbacks: list[TelemetryCallback] = []
        self._drone_ips: dict[int, str] = {}
        self._tag_registry: dict[int, int] = {}
        self._lock = threading.Lock()

        self._nav_tags: list[NavTag] = []
        # Per-drone serialised packets (tag positions in that drone's odom frame)
        self._nav_tags_per_drone: dict[int, bytes] = {}
        # Fallback for drone IDs not in drone_starts (assumes start at map origin)
        self._nav_tags_default: bytes = b""

        if config_path is not None:
            tags, drone_starts = _load_nav_tags(config_path)
            self.set_nav_tags(tags, drone_starts)

    # -----------------------------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------------------------

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self._listen_port))
        self._sock.settimeout(1.0)

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop,
                                        name="comms-recv", daemon=True)
        self._thread.start()
        self._nav_tag_thread = threading.Thread(target=self._nav_tag_loop,
                                                name="comms-navtag", daemon=True)
        self._nav_tag_thread.start()
        log.info("CommsNode listening on UDP port %d", self._listen_port)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._nav_tag_thread:
            self._nav_tag_thread.join(timeout=3.0)
        if self._sock:
            self._sock.close()
            self._sock = None
        log.info("CommsNode stopped")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def on_telemetry(self, callback: TelemetryCallback) -> None:
        """Register a callback invoked for every received telemetry packet."""
        self._callbacks.append(callback)

    # -----------------------------------------------------------------------
    # Sending
    # -----------------------------------------------------------------------

    def send_command(self, drone_id: int, cmd: CommandPacket) -> bool:
        """Send a command to a specific drone.  Returns False if IP unknown."""
        with self._lock:
            ip = self._drone_ips.get(drone_id)
            cmd.found_tag_ids = list(self._tag_registry.keys())[:MAX_FOUND_TAGS]
        if ip is None:
            log.warning("send_command: drone %d IP unknown — not sent", drone_id)
            return False
        self._send_bytes(ip, build_command(cmd))
        return True

    def send_all(self, cmd: CommandPacket) -> None:
        """Send the same command to every drone whose IP is known."""
        with self._lock:
            ips = list(self._drone_ips.values())
            cmd.found_tag_ids = list(self._tag_registry.keys())[:MAX_FOUND_TAGS]
        data = build_command(cmd)
        for ip in ips:
            self._send_bytes(ip, data)

    def known_drones(self) -> dict[int, str]:
        with self._lock:
            return dict(self._drone_ips)

    def tag_registry(self) -> dict[int, int]:
        with self._lock:
            return dict(self._tag_registry)

    def set_nav_tags(self, tags: list[NavTag],
                     drone_starts: dict[int, tuple[float, float]]) -> None:
        """
        Register the nav-tag map and per-drone start positions.

        tags:         NavTag list with map-frame positions (from setup.yaml).
        drone_starts: {drone_id: (start_x, start_y)} in map frame.

        A per-drone packet is pre-built for each drone_id, with tag positions
        pre-converted to that drone's odom frame (map − start_offset).  A
        fallback packet (start = 0, 0) covers any unlisted drone IDs.

        After calling this:
          - Any newly-seen drone gets its packet immediately on first contact.
          - All drones get their packet re-broadcast every 20 s.
        """
        per_drone = {
            drone_id: build_nav_tags_command(tags, sx, sy)
            for drone_id, (sx, sy) in drone_starts.items()
        }
        default = build_nav_tags_command(tags, 0.0, 0.0)

        with self._lock:
            self._nav_tags           = list(tags)
            self._nav_tags_per_drone = per_drone
            self._nav_tags_default   = default

        log.info("Nav-tag map set (%d tags, %d drones configured)",
                 len(tags), len(drone_starts))

    def send_nav_tags(self, drone_id: int | None = None) -> None:
        """
        Manually trigger a nav-tag send.

        drone_id: send to this drone only; None = broadcast to all known drones.
        Uses the map registered via set_nav_tags().
        """
        with self._lock:
            if not self._nav_tags_default:
                log.warning("send_nav_tags: call set_nav_tags() first")
                return
            if drone_id is not None:
                ip   = self._drone_ips.get(drone_id)
                data = self._nav_tags_per_drone.get(drone_id, self._nav_tags_default)
            else:
                targets = [
                    (ip, self._nav_tags_per_drone.get(did, self._nav_tags_default))
                    for did, ip in self._drone_ips.items()
                ]

        if drone_id is not None:
            if ip is None:
                log.warning("send_nav_tags: drone %d IP unknown", drone_id)
                return
            self._send_bytes(ip, data)
        else:
            for ip, data in targets:
                self._send_bytes(ip, data)

    # -----------------------------------------------------------------------
    # Internal
    # -----------------------------------------------------------------------

    def _nav_data_for(self, drone_id: int) -> bytes:
        """Return the pre-built nav-tags packet for this drone (caller holds lock)."""
        return self._nav_tags_per_drone.get(drone_id, self._nav_tags_default)

    def _recv_loop(self) -> None:
        while self._running:
            try:
                data, (src_ip, _) = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            pkt = parse_telemetry(data)
            if pkt is None:
                continue

            is_new_drone = False
            with self._lock:
                if pkt.drone_id not in self._drone_ips:
                    is_new_drone = True
                self._drone_ips[pkt.drone_id] = src_ip
                if pkt.tag_id >= 0 and pkt.tag_id not in self._tag_registry:
                    self._tag_registry[pkt.tag_id] = pkt.drone_id
                    log.info("Tag %d registered to drone %d",
                             pkt.tag_id, pkt.drone_id)

            # Push the per-drone nav-tags packet immediately on first contact
            if is_new_drone:
                with self._lock:
                    nav_data = self._nav_data_for(pkt.drone_id)
                if nav_data:
                    self._send_bytes(src_ip, nav_data)
                    log.info("Drone %d first contact — sent nav tags", pkt.drone_id)

            for cb in self._callbacks:
                try:
                    cb(pkt, src_ip)
                except Exception:
                    log.exception("Exception in telemetry callback")

    def _nav_tag_loop(self) -> None:
        """Periodically send per-drone nav-tag packets to all known drones."""
        while self._running:
            time.sleep(NAV_TAG_RESEND_INTERVAL)
            with self._lock:
                targets = [
                    (ip, self._nav_data_for(did))
                    for did, ip in self._drone_ips.items()
                ]
            if targets and self._nav_tags_default:
                for ip, data in targets:
                    self._send_bytes(ip, data)
                log.debug("Nav tags re-broadcast to %d drone(s)", len(targets))

    def _send_bytes(self, ip: str, data: bytes) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(data, (ip, self._cmd_port))
        except OSError as e:
            log.error("send to %s:%d failed: %s", ip, self._cmd_port, e)
