#include "mavlink_task.h"

// MAVLink — header-only, common dialect covers everything PX4 needs.
// Do NOT define MAVLINK_USE_CONVENIENCE_FUNCTIONS — that path requires a
// user-provided mavlink_system global and comm_send_ch() callback.
// We use mavlink_msg_to_send_buffer() directly instead.
#include "common/mavlink.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

static const char *TAG = "mav";

// ---------------------------------------------------------------------------
// UART config
// ---------------------------------------------------------------------------
#define UART_TX_BUF     512     // outbound ring buffer
#define UART_RX_BUF     1024    // inbound ring buffer (headroom for bursts)

// ---------------------------------------------------------------------------
// SET_POSITION_TARGET_LOCAL_NED type_mask constants
//
// Bit SET   = ignore this field
// Bit CLEAR = use this field
//
// Bit positions:
//   0-2  : position  (x, y, z)
//   3-5  : velocity  (vx, vy, vz)
//   6-8  : accel     (ax, ay, az)
//   10   : yaw
//   11   : yaw_rate
// ---------------------------------------------------------------------------

// Position-only: use x,y,z + yaw — ignore velocity, accel, yaw_rate
// Bits set: vel(3-5)=0x038, accel(6-8)=0x1C0, yaw_rate(11)=0x800
// Bits clear: pos(0-2), yaw(10)
#define TYPEMASK_POSITION       (uint16_t)(0x038 | 0x1C0 | 0x800)   // 0x9F8

// ---------------------------------------------------------------------------
// PX4 custom modes (needed for MAV_CMD_DO_SET_MODE)
// ---------------------------------------------------------------------------
#define PX4_CUSTOM_MAIN_MODE_OFFBOARD   6

// ---------------------------------------------------------------------------
// Internal setpoint state
// ---------------------------------------------------------------------------
typedef enum {
    SP_POSITION,
    SP_HOLD           // position hold at last known location
} sp_type_t;

typedef struct {
    sp_type_t type;
    float x, y, z;    // NED metres
    float yaw;         // radians, CW from North
} setpoint_t;

// ---------------------------------------------------------------------------
// Shared state — protected by mutexes
// ---------------------------------------------------------------------------
static setpoint_t    s_sp;
static drone_state_t s_state;

static SemaphoreHandle_t s_sp_mutex;
static SemaphoreHandle_t s_state_mutex;


// ---------------------------------------------------------------------------
// MAVLink send helpers
// ---------------------------------------------------------------------------

static void uart_send_buf(const uint8_t *buf, uint16_t len)
{
    uart_write_bytes(MAV_UART_PORT, (const char *)buf, len);
}

// Send a pre-populated mavlink_message_t over UART
static void send_message(mavlink_message_t *msg)
{
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_send_buf(buf, len);
}

// ---------------------------------------------------------------------------
// 1 Hz: HEARTBEAT
// Tells PX4 a companion computer is alive.  Type = onboard controller.
// ---------------------------------------------------------------------------
static void send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,    // we are a companion computer
        MAV_AUTOPILOT_INVALID,          // not an autopilot
        0,                              // base_mode (not used)
        0,                              // custom_mode (not used)
        MAV_STATE_ACTIVE
    );
    send_message(&msg);
}

// ---------------------------------------------------------------------------
// 20 Hz: SET_POSITION_TARGET_LOCAL_NED
//
// This is the offboard keepalive.  PX4 drops offboard mode if this message
// stops arriving for ~500ms.  We must send it every 50ms without fail.
// ---------------------------------------------------------------------------
static void send_setpoint(void)
{
    // Snapshot under mutex
    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    setpoint_t sp = s_sp;
    xSemaphoreGive(s_sp_mutex);

    // Position / hold mode: velocity and yaw_rate fields MUST be NaN.
    // SP_HOLD sends NaN for yaw so PX4 keeps the current heading.
    float yaw = (sp.type == SP_HOLD) ? NAN : sp.yaw;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        (uint32_t)(esp_timer_get_time() / 1000),
        PX4_SYSID,
        PX4_COMPID,
        MAV_FRAME_LOCAL_NED,
        TYPEMASK_POSITION,
        sp.x, sp.y, sp.z,   // position — used
        NAN, NAN, NAN,       // velocity — ignored (MUST be NaN)
        NAN, NAN, NAN,       // accel    — ignored
        yaw,                 // yaw      — used (or NaN = keep current)
        NAN                  // yaw_rate — ignored (MUST be NaN)
    );

    send_message(&msg);
}

// ---------------------------------------------------------------------------
// Command: arm / disarm
// ---------------------------------------------------------------------------
static void send_arm_disarm(bool arm)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        PX4_SYSID,
        PX4_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,              // confirmation
        arm ? 1.0f : 0.0f,  // param1: 1=arm, 0=disarm
        0, 0, 0, 0, 0, 0    // unused params
    );
    send_message(&msg);
    if (arm) {
        ESP_LOGI(TAG, "ARM sent");
    } else {
        ESP_LOGI(TAG, "DISARM sent");
    }
}

// ---------------------------------------------------------------------------
// Command: switch to OFFBOARD mode
// ---------------------------------------------------------------------------
static void send_offboard_mode(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        PX4_SYSID,
        PX4_COMPID,
        MAV_CMD_DO_SET_MODE,
        0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   // param1: use custom mode
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,       // param2: custom_mode = offboard
        0, 0, 0, 0, 0                        // unused
    );
    send_message(&msg);
    ESP_LOGI(TAG, "OFFBOARD mode command sent");
}

// ---------------------------------------------------------------------------
// Command: land
// ---------------------------------------------------------------------------
static void send_land(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        OBC_SYSID,
        OBC_COMPID,
        &msg,
        PX4_SYSID,
        PX4_COMPID,
        MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,         // unused: abort alt, precision land, empty, yaw
        NAN, NAN,            // lat/lon: NaN = land in place
        NAN                  // altitude
    );
    send_message(&msg);
    ESP_LOGI(TAG, "LAND command sent");
}

// ---------------------------------------------------------------------------
// Incoming message parser
// Drains whatever is in the UART RX buffer and updates s_state.
// ---------------------------------------------------------------------------
static void parse_incoming(void)
{
    uint8_t byte;
    mavlink_message_t msg;
    mavlink_status_t  status;

    // Read all available bytes without blocking
    int available = 0;
    uart_get_buffered_data_len(MAV_UART_PORT, (size_t *)&available);

    for (int i = 0; i < available; i++) {
        if (uart_read_bytes(MAV_UART_PORT, &byte, 1, 0) != 1) break;

        if (!mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) continue;

        // Only process messages from PX4 (sysid=1, compid=1)
        if (msg.sysid != PX4_SYSID) continue;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        switch (msg.msgid) {

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t pos;
            mavlink_msg_local_position_ned_decode(&msg, &pos);

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.x  = pos.x;   s_state.y  = pos.y;   s_state.z  = pos.z;
            s_state.vx = pos.vx;  s_state.vy = pos.vy;  s_state.vz = pos.vz;
            s_state.enu_z     = -pos.z;   // NED z-down → ENU z-up
            s_state.last_pos_ms = now_ms;
            xSemaphoreGive(s_state_mutex);
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);

            // Wrap from (-π, π] to [0, 2π) for internal use.
            float heading = att.yaw;
            if (heading < 0.0f) heading += 2.0f * (float)M_PI;

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.heading     = heading;
            s_state.last_att_ms = now_ms;
            xSemaphoreGive(s_state_mutex);
            break;
        }

        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

            // PX4 custom mode is packed: upper byte = main, lower 3 bytes = sub
            uint8_t main_mode = (uint8_t)((hb.custom_mode >> 16) & 0xFF);
            uint8_t sub_mode  = (uint8_t)((hb.custom_mode >> 24) & 0xFF);

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.armed             = armed;
            s_state.custom_main_mode  = main_mode;
            s_state.custom_sub_mode   = sub_mode;
            s_state.last_hb_ms        = now_ms;
            xSemaphoreGive(s_state_mutex);
            break;
        }

        default:
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Public API implementations
// ---------------------------------------------------------------------------

void mavlink_task_init(void)
{
    // Zero state
    memset(&s_sp,    0, sizeof(s_sp));
    memset(&s_state, 0, sizeof(s_state));

    // Default to position hold at origin — safe until mission_task sets a real target
    s_sp.type = SP_HOLD;

    // Create mutexes before the task starts
    s_sp_mutex    = xSemaphoreCreateMutex();
    s_state_mutex = xSemaphoreCreateMutex();

    configASSERT(s_sp_mutex    != NULL);
    configASSERT(s_state_mutex != NULL);
}

void mavlink_set_position_ned(float x, float y, float z, float yaw)
{
    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    s_sp.type = SP_POSITION;
    s_sp.x    = x;
    s_sp.y    = y;
    s_sp.z    = z;
    s_sp.yaw  = yaw;
    xSemaphoreGive(s_sp_mutex);
}

void mavlink_set_hold(void)
{
    // Capture current NED position and freeze there
    drone_state_t st = mavlink_get_state();

    xSemaphoreTake(s_sp_mutex, portMAX_DELAY);
    s_sp.type = SP_HOLD;
    s_sp.x    = st.x;
    s_sp.y    = st.y;
    s_sp.z    = st.z;
    // Yaw set to NaN in send_setpoint() for SP_HOLD — PX4 keeps current heading
    xSemaphoreGive(s_sp_mutex);
}

void mavlink_arm(bool arm)
{
    send_arm_disarm(arm);
}

void mavlink_set_offboard_mode(void)
{
    send_offboard_mode();
}

void mavlink_send_land_command(void)
{
    send_land();
}

drone_state_t mavlink_get_state(void)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    drone_state_t copy = s_state;
    xSemaphoreGive(s_state_mutex);
    return copy;
}

bool mavlink_position_valid(void)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    uint32_t last = s_state.last_pos_ms;
    xSemaphoreGive(s_state_mutex);

    if (last == 0) return false;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    return (now - last) < 500;
}


// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

void mavlink_task(void *arg)
{
    // ---- UART init ----
    const uart_config_t uart_cfg = {
        .baud_rate  = MAV_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        MAV_UART_PORT,
        UART_RX_BUF,
        UART_TX_BUF,
        0,      // no event queue needed
        NULL,
        ESP_INTR_FLAG_IRAM   // IRAM ISR: immune to cache miss from WiFi
    ));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        MAV_UART_PORT,
        MAV_TX_PIN,
        MAV_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_LOGI(TAG, "UART%d ready: TX=GPIO%d RX=GPIO%d @ %d baud",
             MAV_UART_PORT, MAV_TX_PIN, MAV_RX_PIN, MAV_BAUD_RATE);

    // ---- Timing state ----
    TickType_t last_wake_tick = xTaskGetTickCount();
    uint32_t   last_hb_ms     = 0;

    // ---- 20Hz loop ----
    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // 1 Hz: heartbeat
        if (now_ms - last_hb_ms >= 1000) {
            send_heartbeat();
            last_hb_ms = now_ms;
        }

        // 20 Hz: setpoint (MUST NOT be skipped — PX4 watchdog is 500ms)
        send_setpoint();

        // Drain RX buffer and update s_state
        parse_incoming();

        // Precise 50ms period using vTaskDelayUntil to absorb execution time
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(50));
    }
}