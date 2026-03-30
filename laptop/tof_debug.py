#!/usr/bin/env python3
"""
tof_debug.py — Print ToF debug packets from the front sensor.

The ESP32 sends WIFI_PKT_TOF_DEBUG (0x03) packets to UDP port 5007 at 10 Hz.
Run while the drone is powered; flight is not required.

    python3 tof_debug.py [--port 5007] [--front-idx 1] [--full-grid]

By default this script collapses each 8x8 frame into the same 64-bin angular
scan format used by tof_get_collapsed_scan() in firmware.

    --front-idx  Matches CONFIG_TOF_FRONT_SENSOR_IDX for angle mapping.
    --full-grid  Print full 8x8 raw grid instead of collapsed scan.
"""

import argparse
import math
import socket
import struct

LISTEN_PORT   = 5007
PKT_TOF_DEBUG = 0x03
PIXELS        = 64       # 8x8 grid
TOF_SENSOR_COUNT = 8
TOF_SENSOR_RESO = 8
TOF_TOTAL_BINS = TOF_SENSOR_COUNT * TOF_SENSOR_RESO
TOF_MIN_VALID_MM = 50
TOF_MAX_VALID_MM = 4000
TOF_FRONT_SENSOR_IDX = 1

# Wire layout: pkt_type(B) sensor_idx(B) sensor_ok(B) timestamp_ms(I)
#              distance_mm[64](H...) target_status[64](B...)
_FMT  = f"<BBBI{PIXELS}H{PIXELS}B"
_SIZE = struct.calcsize(_FMT)

VALID_STATUS = {5, 9}   # 5 = valid, 9 = valid weak signal


def print_grid(distances, statuses):
    """Print the full 8x8 grid with distance and raw target status."""
    print("  Col:  0     1     2     3     4     5     6     7")
    for row in range(8):
        cells = []
        for col in range(8):
            idx = row * 8 + col
            d = distances[idx]
            s = statuses[idx]
            cells.append(f"{d:4d}[{s:2d}]")
        print(f"  Row {row}: " + "  ".join(cells))
    print("  Format: distance_mm[target_status]")


def collapse_like_tof_task(front_sensor_idx, sensor_idx, distances, statuses):
    """Mirror tof_get_collapsed_scan() for one sensor frame into 64 angular bins."""
    ranges_m = [math.inf] * TOF_TOTAL_BINS
    bin_status = [None] * TOF_TOTAL_BINS

    sensor_angle = ((front_sensor_idx - sensor_idx) * 45) % 360

    for col in range(8):
        # Same mapping as firmware: col 0 is +offset (CW), col 7 is -offset (CCW).
        angle_deg = sensor_angle + (3.5 - float(col)) * (45.0 / 8.0)

        # Same rows and validity gating as tof_get_collapsed_scan().
        for row in range(4, 8):
            px = row * 8 + col
            status = statuses[px]
            dist = distances[px]

            if status not in VALID_STATUS:
                continue
            if dist < TOF_MIN_VALID_MM or dist > TOF_MAX_VALID_MM:
                continue

            norm_angle = angle_deg % 360.0
            idx = int(norm_angle * float(TOF_TOTAL_BINS) / 360.0)
            if idx >= TOF_TOTAL_BINS:
                idx = TOF_TOTAL_BINS - 1

            dist_m = float(dist) / 1000.0
            if dist_m < ranges_m[idx]:
                ranges_m[idx] = dist_m
                bin_status[idx] = status

    return ranges_m, bin_status


def print_collapsed(front_sensor_idx, sensor_idx, distances, statuses):
    """Print 64-bin collapsed scan in the same format shape as nav uses."""
    ranges_m, bin_status = collapse_like_tof_task(front_sensor_idx, sensor_idx, distances, statuses)
    finite_count = sum(1 for r in ranges_m if math.isfinite(r))

    print(f"  Collapsed bins populated: {finite_count}/{TOF_TOTAL_BINS}")
    print("  Forward bins (left->right): idx 60 61 62 63 00 01 02 03")
    selected = [60, 61, 62, 63, 0, 1, 2, 3]
    val_cells = []
    for i in selected:
        if math.isfinite(ranges_m[i]):
            val_cells.append(f"{ranges_m[i]:5.3f}[{bin_status[i]:2d}]")
        else:
            val_cells.append("  inf[--]")
    print("  m[s]: " + "  ".join(val_cells))


def main():
    ap = argparse.ArgumentParser(description="Print raw front ToF sensor data")
    ap.add_argument("--port",     type=int, default=LISTEN_PORT)
    ap.add_argument("--front-idx", type=int, default=TOF_FRONT_SENSOR_IDX,
                    help="Front sensor index (CONFIG_TOF_FRONT_SENSOR_IDX, default: 1)")
    ap.add_argument("--full-grid", action="store_true",
                    help="Show full 8x8 raw grid instead of collapsed 64-bin view")
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

        if args.full_grid:
            print_grid(distances, statuses)
        else:
            print_collapsed(args.front_idx, sensor_idx, distances, statuses)
        print()


if __name__ == "__main__":
    main()
