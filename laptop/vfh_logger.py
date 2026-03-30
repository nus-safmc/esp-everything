"""
VFH histogram logger.

Writes one CSV row per telemetry packet to a timestamped file.
All file I/O happens in a background thread via a queue so the
telemetry callback returns immediately and never blocks the main loop.

CSV columns:
    wall_time_s, drone_id, heading_rad, nav_state,
    bin_0, bin_1, ..., bin_31    (1 = blocked, 0 = free)

Usage:
    logger = VfhLogger()          # creates vfh_<timestamp>.csv
    comms.on_telemetry(logger.feed)
    ...
    logger.stop()                 # flush and close (call on shutdown)
"""

import csv
import queue
import threading
import time
from pathlib import Path

from protocol import TelemetryPacket, VFH_BINS

_HEADER = (
    ["wall_time_s", "drone_id", "heading_rad", "nav_state"]
    + [f"bin_{i}" for i in range(VFH_BINS)]
)


class VfhLogger:
    def __init__(self, output_dir: str = "."):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path  = Path(output_dir) / f"vfh_{stamp}.csv"

        self._queue: queue.Queue = queue.Queue()
        self._path = path
        self._thread = threading.Thread(target=self._writer, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Telemetry callback — must return fast
    # ------------------------------------------------------------------

    def feed(self, pkt: TelemetryPacket, _src_ip: str) -> None:
        row = [
            f"{time.time():.4f}",
            pkt.drone_id,
            f"{pkt.heading_rad:.5f}",
            pkt.nav_state,
        ] + [int(b) for b in pkt.vfh_blocked]
        self._queue.put_nowait(row)

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------

    def stop(self, timeout: float = 3.0) -> None:
        """Flush the queue and close the file.  Call once on shutdown."""
        self._queue.put(None)          # sentinel
        self._thread.join(timeout=timeout)

    # ------------------------------------------------------------------
    # Background writer
    # ------------------------------------------------------------------

    def _writer(self) -> None:
        with self._path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(_HEADER)
            f.flush()

            while True:
                try:
                    row = self._queue.get(timeout=1.0)
                except queue.Empty:
                    f.flush()
                    continue

                if row is None:   # sentinel — exit
                    f.flush()
                    break

                writer.writerow(row)

                # Flush every 50 rows so the file is readable mid-flight
                if self._queue.empty():
                    f.flush()
