#pragma once

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Configuration — adjust to your hardware
// ---------------------------------------------------------------------------
#define MAV_UART_PORT       UART_NUM_1
#define MAV_TX_PIN          5
#define MAV_RX_PIN          4
#define MAV_BAUD_RATE       921600

// MAVLink identity for this companion computer
#define OBC_SYSID           1           // Same vehicle system as PX4
#define OBC_COMPID          190         // MAV_COMP_ID_ONBOARD_COMPUTER

// PX4 target
#define PX4_SYSID           1
#define PX4_COMPID          1

// PX4 custom main mode — used to verify mode switches via drone_state_t
#define PX4_MAIN_MODE_OFFBOARD  6

// FreeRTOS placement — Core 0, highest priority to meet 20Hz hard deadline
#define MAV_TASK_CORE       0
#define MAV_TASK_PRIORITY   5
#define MAV_TASK_STACK      4096

// ---------------------------------------------------------------------------
// Telemetry readback — written by mavlink_task, read by mission_task
// ---------------------------------------------------------------------------
typedef struct {
    // Position in NED frame, metres (from LOCAL_POSITION_NED)
    float x, y, z;

    // Velocity in NED frame, m/s (from LOCAL_POSITION_NED)
    float vx, vy, vz;

    // Heading: radians, 0 = North, clockwise positive (from ATTITUDE)
    float heading;

    // Derived ENU-Z for altitude controller convenience (= -z)
    float enu_z;

    // PX4 arm + mode status (from HEARTBEAT)
    bool  armed;
    uint8_t custom_main_mode;   // 6 = offboard
    uint8_t custom_sub_mode;

    // Timestamp of last successful parse, ms since boot
    uint32_t last_pos_ms;
    uint32_t last_att_ms;
    uint32_t last_hb_ms;
} drone_state_t;

// ---------------------------------------------------------------------------
// Init & task entry
// ---------------------------------------------------------------------------

// Call once before starting the task
void mavlink_task_init(void);

// FreeRTOS task — pin to Core 0 at Priority 5
// Usage: xTaskCreatePinnedToCore(mavlink_task, "mav", MAV_TASK_STACK,
//                                NULL, MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE);
void mavlink_task(void *arg);

// ---------------------------------------------------------------------------
// Setpoint API  (call from mission_task — thread-safe via mutex)
// ---------------------------------------------------------------------------

// Velocity mode: vx/vy/vz in NED m/s, yaw_rate in rad/s
// Unused position fields are sent as NaN automatically.
void mavlink_set_velocity_ned(float vx, float vy, float vz, float yaw_rate);

// Mixed mode: XY velocity (m/s NED) + Z position (NED metres, negative = above ground)
// + yaw position (NED radians, 0 = North, CW positive).
// PX4 holds altitude and yaw via its own controllers; XY velocity controller
// moves horizontally. Ideal for navigation at a fixed cruise altitude.
void mavlink_set_velocity_xy_position_z(float vx, float vy, float z, float yaw);

// Position hold: x/y/z in NED metres, yaw in radians (0 = North, CW+)
// Unused velocity fields are sent as NaN automatically.
void mavlink_set_position_ned(float x, float y, float z, float yaw);

// Stop all motion — switches to position hold at current location.
// Safe to call at any time; mavlink_task will use last known position.
void mavlink_set_hold(void);

// ---------------------------------------------------------------------------
// Command API
// ---------------------------------------------------------------------------

// Arm or disarm. Does NOT set offboard mode — call mavlink_set_offboard_mode()
// first when arming, or the drone will just idle in the current mode.
void mavlink_arm(bool arm);

// Switch PX4 to OFFBOARD mode via MAV_CMD_DO_SET_MODE.
// Precondition: setpoints must already be streaming (they are, once the task runs).
void mavlink_set_offboard_mode(void);

// Send MAV_CMD_NAV_LAND. PX4 will descend and disarm automatically.
void mavlink_send_land_command(void);

// ---------------------------------------------------------------------------
// Telemetry readback
// ---------------------------------------------------------------------------

// Thread-safe snapshot — safe to call from any task.
drone_state_t mavlink_get_state(void);

// Convenience: true if we have received position data recently (< 500ms)
bool mavlink_position_valid(void);

