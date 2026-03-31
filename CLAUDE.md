# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Autonomous drone swarm system for the SAFMC 2026 Cat Swarm Challenge. An ESP32-S3 runs onboard each drone as a companion computer, communicating with PX4 via MAVLink over UART. A laptop-side Python application coordinates the fleet over WiFi/UDP.

## Build & Flash (ESP-IDF)

```bash
# First time: clone MAVLink C library
cd components/mavlink/include && git clone --depth 1 https://github.com/mavlink/c_library_v2.git . && cd -

# Build
idf.py build

# Flash and monitor (adjust port as needed)
idf.py -p /dev/ttyUSB0 flash monitor

# Per-drone config (drone ID, GPIO pins, WiFi, front sensor index)
idf.py menuconfig   # under "Drone Configuration"
```

## Laptop-side scripts

Run from the `laptop/` directory. Requires Python 3.10+ with `pyyaml` and `matplotlib`.

```bash
# Full mission (exploration + relay phases, interactive prompts)
python laptop/run_mission.py --config laptop/setup.yaml

# Single-drone exploration
python laptop/run_exploration.py

# Fleet exploration (multiple drones, no relay)
python laptop/run_fleet_exploration.py
```

Fleet configuration (drone IDs, start positions, arena bounds, nav tags) lives in `laptop/setup.yaml`.

## Architecture

### ESP32 firmware (`main/`)

Dual-core FreeRTOS on ESP32-S3. Tasks are pinned to specific cores:

| Task | Core | Priority | Rate | Role |
|------|------|----------|------|------|
| `mavlink_task` | 0 | 5 | 50 Hz | UART↔PX4: parses telemetry, streams setpoints |
| `tof_task` | 0 | 4 | 15 Hz | Reads 8× VL53L5CX ToF sensors via I2C mux (TCA9548A) |
| `wifi_task` | 0 | 2 | 10 Hz | UDP telemetry to laptop, receives commands |
| `nav_task` | 1 | 4 | 20 Hz | VFH obstacle avoidance, goal navigation, collision avoidance, WiFi killswitch |
| `at_detect_task` | 1 | 1 | ~2 Hz | AprilTag detection via camera (tag16h5 family) |
| `mission_task` | 1 | 2 | — | State machine: arm → takeoff → explore → precision land |

**Setpoint ownership**: `mission_task` owns MAVLink setpoints during takeoff/landing. `nav_task` takes over when `nav_set_goal_ned()` is called. `nav_cancel()` returns ownership to mission.

**Coordinate frames**: Everything uses NED (North-East-Down). PX4 odom origin is at takeoff. The `odom` module manages a translation-only `map_T_odom` transform, refined by AprilTag nav-tag sightings.

### Navigation pipeline

1. `tof_task` produces collapsed 2D scans (64 points from 8 sensors × 8 columns)
2. `vfh.c` builds a 32-bin polar histogram, thresholds, grows blocked sectors by drone radius, finds free valleys, scores candidates (goal direction + heading continuity + previous steering)
3. `nav_task` adds peer drone positions as virtual obstacles in the histogram
4. Emergency collision avoidance (`collision_avoid`) pre-empts all navigation if any reading < 0.37m
5. Navigation uses rotate-then-fly: misaligned → position hold + rotate; aligned → velocity forward + position-Z hold

### Laptop coordinator (`laptop/`)

- `protocol.py` — packed struct definitions for the UDP wire format (telemetry and commands)
- `comms.py` — `CommsNode` class: UDP send/recv, drone IP discovery, nav-tag broadcast, peer position relay
- `exploration.py` — `ExplorationDirector`: picks least-explored VFH gap, scores by crumb density + heading continuity + peer goal repulsion
- `crumb_store.py` — breadcrumb trail storage (map frame), cone density queries
- `stuck_monitor.py` — detects drones making no progress, marks stuck zones, issues rescue goals
- `relay.py` — plans a chain of relay drones from start area to a target position
- `run_mission.py` — interactive mission controller with phased execution (explore → relay)
- `setup.yaml` — fleet config: drone IDs/starts, arena bounds, nav tags, exploration tuning

### Communication protocol

UDP between ESP32 (port 5005 out, 5006 in) and laptop:
- **Telemetry** (drone→laptop, 10 Hz): position, heading, nav state, VFH blocked bins, AprilTag sightings, breadcrumb batch
- **Commands** (laptop→drone): `CMD_GOTO`, `CMD_LAND`, `CMD_HOLD`, `CMD_START`, `CMD_SET_NAV_TAGS`, `CMD_SET_PEERS`
- **ToF debug** (drone→laptop, port 5007): raw 8×8 front sensor frame

## Key tuning constants

- `NAV_CRUISE_SPEED_MS` (0.5 m/s), `NAV_YAW_TOL_RAD` (0.1 rad), `NAV_ARRIVE_RADIUS_M` (0.25m) — in `nav_task.h`
- `VFH_DEFAULT_CONFIG` — density threshold, drone radius, max range — in `vfh.h`
- `COLLISION_DANGER_M` (0.37m), `COLLISION_CLEAR_M` (0.47m) — in `nav_task.c`
- `PEER_INJECT_RANGE_M` (4.0m), `PEER_DENSITY_MAX` (9.0) — peer avoidance in `nav_task.c`
- `CRUISE_ALT_M` (0.5m) — mission altitude in `main.c`
- Exploration params (goal distance, cone radius, heading weight) — in `setup.yaml`
