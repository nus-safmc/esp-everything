"""
CommsNode — UDP communication hub between the laptop and all drones.

Usage:
    node = CommsNode(listen_port=5005, cmd_port=5006)
    node.on_telemetry(lambda pkt, ip: print(pkt.drone_id, pkt.ned_x, pkt.ned_y))
    node.start()

    node.send_command(0, CommandPacket(CMD_GOTO, goal_x=2.0, goal_y=1.6))
    node.send_all(CommandPacket(CMD_LAND))

    node.stop()
"""

import socket
import threading
import logging
from typing import Callable, Optional

from protocol import (
    TelemetryPacket, CommandPacket,
    parse_telemetry, build_command, MAX_FOUND_TAGS,
)

log = logging.getLogger(__name__)

# Type alias for the telemetry callback
TelemetryCallback = Callable[[TelemetryPacket, str], None]


class CommsNode:
    def __init__(self, listen_port: int = 5005, cmd_port: int = 5006):
        """
        listen_port  UDP port to receive telemetry from drones.
        cmd_port     UDP port on each drone that accepts commands.
        """
        self._listen_port = listen_port
        self._cmd_port    = cmd_port

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

        self._callbacks: list[TelemetryCallback] = []
        # drone_id -> last known IP (learned from incoming packets)
        self._drone_ips: dict[int, str] = {}
        # Tag registry: tag_id -> drone_id that found it
        self._tag_registry: dict[int, int] = {}
        self._lock = threading.Lock()

    # -----------------------------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------------------------

    def start(self) -> None:
        """Open the listening socket and start the receive thread."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self._listen_port))
        self._sock.settimeout(1.0)   # allows clean shutdown

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop,
                                        name="comms-recv", daemon=True)
        self._thread.start()
        log.info("CommsNode listening on UDP port %d", self._listen_port)

    def stop(self) -> None:
        """Stop the receive thread and close the socket."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._sock:
            self._sock.close()
            self._sock = None
        log.info("CommsNode stopped")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def on_telemetry(self, callback: TelemetryCallback) -> None:
        """
        Register a callback invoked for every received telemetry packet.
        Called from the receive thread — keep it fast or dispatch to a queue.
            callback(pkt: TelemetryPacket, src_ip: str)
        """
        self._callbacks.append(callback)

    # -----------------------------------------------------------------------
    # Sending
    # -----------------------------------------------------------------------

    def send_command(self, drone_id: int, cmd: CommandPacket) -> bool:
        """
        Send a command to a specific drone.
        Auto-injects all found tag IDs from the registry.
        Returns True if the drone's IP is known, False otherwise.
        """
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
        """Return a snapshot of {drone_id: ip} for all heard drones."""
        with self._lock:
            return dict(self._drone_ips)

    def tag_registry(self) -> dict[int, int]:
        """Return a snapshot of {tag_id: drone_id} for all found tags."""
        with self._lock:
            return dict(self._tag_registry)

    # -----------------------------------------------------------------------
    # Internal
    # -----------------------------------------------------------------------

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

            # Learn / update drone IP and tag registry
            with self._lock:
                self._drone_ips[pkt.drone_id] = src_ip
                if pkt.tag_id >= 0 and pkt.tag_id not in self._tag_registry:
                    self._tag_registry[pkt.tag_id] = pkt.drone_id
                    log.info("Tag %d registered to drone %d",
                             pkt.tag_id, pkt.drone_id)

            for cb in self._callbacks:
                try:
                    cb(pkt, src_ip)
                except Exception:
                    log.exception("Exception in telemetry callback")

    def _send_bytes(self, ip: str, data: bytes) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(data, (ip, self._cmd_port))
        except OSError as e:
            log.error("send to %s:%d failed: %s", ip, self._cmd_port, e)
