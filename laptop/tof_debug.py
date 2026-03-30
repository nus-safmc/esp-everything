#!/usr/bin/env python3
"""
tof_debug.py — Print raw 8x8 distance grid from the front ToF sensor (sensor 1).

The ESP32 sends WIFI_PKT_TOF_DEBUG (0x03) packets to UDP port 5007 at 10 Hz.
Run while the drone is powered; flight is not required.

    python3 tof_debug.py [--port 5007] [--min-only]

    --min-only   Print only the minimum valid distance per column (collapsed scan view)
                 instead of the full 8x8 grid.
"""

import argparse
import socket
import struct

LISTEN_PORT   = 5007
PKT_TOF_DEBUG = 0x03
PIXELS        = 64       # 8x8 grid

# Wire layout: pkt_type(B) sensor_idx(B) sensor_ok(B) timestamp_ms(I)
#              distance_mm[64](H...) target_status[64](B...)
_FMT  = f"<BBBI{PIXELS}H{PIXELS}B"
_SIZE = struct.calcsize(_FMT)

VALID_STATUS = {5, 9}   # 5 = valid, 9 = valid weak signal


def print_grid(distances, statuses):
    """Print the full 8x8 grid with validity markers."""
    print("  Col:  0     1     2     3     4     5     6     7")
    for row in range(8):
        cells = []
        for col in range(8):
            idx = row * 8 + col
            d = distances[idx]
            ok = statuses[idx] in VALID_STATUS
            cells.append(f"{d:4d}{'v' if ok else '.'}")
        print(f"  Row {row}: " + "  ".join(cells))
    print("  (v=valid, .=invalid)")


def print_collapsed(distances, statuses):
    """Per-column minimum valid distance, matching the collapsed scan used by nav."""
    print("  Col:     0      1      2      3      4      5      6      7")
    mins = []
    for col in range(8):
        valid = [
            distances[row * 8 + col]
            for row in range(3, 8)   # rows 3-7, same as tof_get_collapsed_scan
            if statuses[row * 8 + col] in VALID_STATUS and distances[row * 8 + col] > 0
        ]
        mins.append(min(valid) if valid else 0)
    print("  mm:   " + "  ".join(f"{m:5d}" for m in mins))
    print("  m:    " + "  ".join(f"{m/1000:.3f}" for m in mins))


def main():
    ap = argparse.ArgumentParser(description="Print raw front ToF sensor data")
    ap.add_argument("--port",     type=int, default=LISTEN_PORT)
    ap.add_argument("--min-only", action="store_true",
                    help="Show collapsed (per-column min) view instead of full grid")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.port))
    print(f"Listening for ToF debug on UDP :{args.port}  (Ctrl-C to stop)")
    print(f"Expecting packet size: {_SIZE} bytes\n")

    prev_ts = None
    while True:
        data, addr = sock.recvfrom(512)
        if len(data) < _SIZE or data[0] != PKT_TOF_DEBUG:
            continue

        fields    = struct.unpack_from(_FMT, data[:_SIZE])
        pkt_type  = fields[0]
        sensor_idx = fields[1]
        sensor_ok  = fields[2]
        ts_ms      = fields[3]
        distances  = fields[4 : 4 + PIXELS]
        statuses   = fields[4 + PIXELS :]

        dt_ms = (ts_ms - prev_ts) if prev_ts is not None else 0
        prev_ts = ts_ms

        valid_count = sum(1 for s in statuses if s in VALID_STATUS)

        print(f"--- Sensor {sensor_idx} | ok={sensor_ok} | t={ts_ms} ms "
              f"(+{dt_ms} ms) | valid pixels: {valid_count}/64 ---")

        if args.min_only:
            print_collapsed(distances, statuses)
        else:
            print_grid(distances, statuses)
        print()


if __name__ == "__main__":
    main()
