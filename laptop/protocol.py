"""
Packet definitions for ESP32 ↔ laptop UDP protocol.

Telemetry (ESP32 → laptop, 10 Hz):
    Fixed 60-byte header followed by a variable crumb batch.

Command (laptop → ESP32, event-driven):
    Fixed 18-byte packet.
"""

import struct
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PKT_TELEM   = 0x01   # telemetry from drone
PKT_CMD     = 0x02   # command to drone

CMD_GOTO          = 0x01   # navigate to (goal_x, goal_y)
CMD_LAND          = 0x02   # land immediately
CMD_HOLD          = 0x03   # hold position, cancel goal
CMD_SET_NAV_TAGS  = 0x04   # send navigation tag map positions to drone
CMD_START         = 0x05   # arm and take off

VFH_BINS    = 32
MAX_CRUMBS_PKT = 10

# Nav state values (must match nav_state_t in nav_task.h)
NAV_IDLE       = 0
NAV_ROTATING   = 1
NAV_FLYING     = 2
NAV_ARRIVED    = 3
NAV_STUCK      = 4
NAV_RETREATING = 5

NAV_STATE_NAMES = {
    NAV_IDLE:       "IDLE",
    NAV_ROTATING:   "ROTATING",
    NAV_FLYING:     "FLYING",
    NAV_ARRIVED:    "ARRIVED",
    NAV_STUCK:      "STUCK",
    NAV_RETREATING: "RETREATING",
}

# ---------------------------------------------------------------------------
# Telemetry packet
# ---------------------------------------------------------------------------

# Fixed header: pkt_type, drone_id, ned_x, ned_y, heading_rad,
#               nav_state, tag_id, tag_dist_m, crumb_count, crumb_last_sent,
#               vfh_blocked[32], is_stuck, crumb_batch_start, crumb_batch_count
_TELEM_HDR_FMT  = "<BBfffBbfHH32sBHB"
_TELEM_HDR_SIZE = struct.calcsize(_TELEM_HDR_FMT)   # 60 bytes
_CRUMB_FMT      = "<ff"
_CRUMB_SIZE     = struct.calcsize(_CRUMB_FMT)        # 8 bytes


@dataclass
class TelemetryPacket:
    drone_id:          int
    ned_x:             float
    ned_y:             float
    heading_rad:       float
    nav_state:         int
    tag_id:            int              # −1 if no tag visible
    tag_dist_m:        float
    crumb_count:       int              # total crumbs on drone
    crumb_last_sent:   int              # 0xFFFF = none sent yet
    vfh_blocked:       list            # VFH_BINS bools
    is_stuck:          bool
    crumb_batch_start: int
    crumbs:            list            # list of (x, y) floats, new this packet

    @property
    def nav_state_name(self) -> str:
        return NAV_STATE_NAMES.get(self.nav_state, f"UNKNOWN({self.nav_state})")


def parse_telemetry(data: bytes) -> Optional[TelemetryPacket]:
    """Parse a raw UDP payload into a TelemetryPacket. Returns None on error."""
    if len(data) < _TELEM_HDR_SIZE:
        return None

    fields = struct.unpack_from(_TELEM_HDR_FMT, data, 0)
    (pkt_type, drone_id, ned_x, ned_y, heading_rad,
     nav_state, tag_id, tag_dist_m, crumb_count, crumb_last_sent,
     vfh_raw, is_stuck, crumb_batch_start, crumb_batch_count) = fields

    if pkt_type != PKT_TELEM:
        return None

    vfh_blocked = [bool(b) for b in vfh_raw]

    crumbs = []
    offset = _TELEM_HDR_SIZE
    for _ in range(crumb_batch_count):
        if offset + _CRUMB_SIZE > len(data):
            break
        x, y = struct.unpack_from(_CRUMB_FMT, data, offset)
        crumbs.append((x, y))
        offset += _CRUMB_SIZE

    return TelemetryPacket(
        drone_id          = drone_id,
        ned_x             = ned_x,
        ned_y             = ned_y,
        heading_rad       = heading_rad,
        nav_state         = nav_state,
        tag_id            = tag_id,
        tag_dist_m        = tag_dist_m,
        crumb_count       = crumb_count,
        crumb_last_sent   = crumb_last_sent,
        vfh_blocked       = vfh_blocked,
        is_stuck          = bool(is_stuck),
        crumb_batch_start = crumb_batch_start,
        crumbs            = crumbs,
    )

# ---------------------------------------------------------------------------
# Command packet
# ---------------------------------------------------------------------------

# pkt_type, cmd_type, goal_x, goal_y, found_tag_ids[12] (int8_t, −1 = unused)
_CMD_FMT  = "<BBff12b"
_CMD_SIZE = struct.calcsize(_CMD_FMT)   # 22 bytes

MAX_FOUND_TAGS = 12


@dataclass
class CommandPacket:
    cmd_type:      int
    goal_x:        float     = 0.0
    goal_y:        float     = 0.0
    found_tag_ids: list      = field(default_factory=list)  # up to 8 tag IDs


def build_command(cmd: CommandPacket) -> bytes:
    """Serialise a CommandPacket to bytes ready to send over UDP."""
    tag_ids = (cmd.found_tag_ids + [-1] * MAX_FOUND_TAGS)[:MAX_FOUND_TAGS]
    return struct.pack(_CMD_FMT, PKT_CMD, cmd.cmd_type,
                       cmd.goal_x, cmd.goal_y, *tag_ids)


# ---------------------------------------------------------------------------
# Navigation-tag position packet  (laptop → drone)
# ---------------------------------------------------------------------------

MAX_NAV_TAGS = 16

# Wire format per tag entry: int8_t id + float map_x + float map_y
_NAV_TAG_ENTRY_FMT = "<bff"
_NAV_TAG_ENTRY_SIZE = struct.calcsize(_NAV_TAG_ENTRY_FMT)  # 9 bytes


@dataclass
class NavTag:
    """A navigation AprilTag at a known map-frame position."""
    id:    int      # AprilTag ID
    map_x: float    # NED north in map frame (m)
    map_y: float    # NED east  in map frame (m)


def build_nav_tags_command(tags: list[NavTag],
                           start_x: float = 0.0,
                           start_y: float = 0.0) -> bytes:
    """
    Build a CMD_SET_NAV_TAGS packet for one specific drone.

    tags:    list of NavTag in map frame (up to MAX_NAV_TAGS).
    start_x: drone's start position in map frame (NED north, m).
    start_y: drone's start position in map frame (NED east,  m).

    Tag positions are pre-converted to the drone's odom frame
    (map_pos − start_offset) so the drone never needs to know the
    map frame directly.
    """
    count = min(len(tags), MAX_NAV_TAGS)
    # Header: pkt_type, cmd_type, count, start_map_x, start_map_y
    buf = struct.pack("<BBBff", PKT_CMD, CMD_SET_NAV_TAGS, count, start_x, start_y)
    for t in tags[:count]:
        odom_x = t.map_x - start_x
        odom_y = t.map_y - start_y
        buf += struct.pack(_NAV_TAG_ENTRY_FMT, t.id, odom_x, odom_y)
    return buf
