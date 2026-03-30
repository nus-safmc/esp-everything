#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

// ---------------------------------------------------------------------------
// Hardware config
// ---------------------------------------------------------------------------
#define TOF_SENSOR_COUNT        8
#define TOF_SENSOR_RESO         8
#define TOF_I2C_PORT            I2C_NUM_0
#define TOF_SDA_PIN             CONFIG_TCA_SDA_GPIO_PORT   // from menuconfig
#define TOF_SCL_PIN             CONFIG_TCA_SCL_GPIO_PORT
#define TCA_I2C_ADDR            0x70
#define TOF_RANGING_FREQ_HZ     15
#define TOF_MAX_VALID_MM        3000    // discard readings above this (sensor max is 4m)
#define TOF_MIN_VALID_MM        50      // discard readings below this (sensor blind zone)

// FreeRTOS placement — Core 0, below mavlink but above mission
#define TOF_TASK_CORE           0
#define TOF_TASK_PRIORITY       4
#define TOF_TASK_STACK          6144

// ---------------------------------------------------------------------------
// Sensor angular layout
//
// Sensors are mounted COUNTER-CLOCKWISE around the ring.  The front-facing
// sensor index is set via menuconfig (TOF_FRONT_SENSOR_IDX).
//
// Angle for sensor i = ((front_idx - i) * 45) mod 360  (degrees, CW+)
//
// The angle table is computed at runtime in tof_task_init().
// ---------------------------------------------------------------------------
#define TOF_FRONT_SENSOR_IDX        CONFIG_TOF_FRONT_SENSOR_IDX
#define TOF_SENSOR_HALF_WIDTH_DEG   22.5f

// Number of pixels per sensor frame (8x8 grid × 1 target per zone).
// Matches VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE without
// pulling vl53l5cx_api.h into this header.
#define TOF_PIXELS_PER_SENSOR       TOF_SENSOR_RESO * TOF_SENSOR_RESO

// ---------------------------------------------------------------------------
// Per-sensor frame — raw data exactly as returned by vl53l5cx_get_ranging_data
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t distance_mm[TOF_PIXELS_PER_SENSOR];  // raw distances, 0 = invalid
    uint8_t  target_status[TOF_PIXELS_PER_SENSOR]; // 5 = valid, 9 = valid weak signal
    uint32_t timestamp_ms;                         // time this frame was received
    uint8_t  valid;                                // 1 once first frame has arrived
} tof_frame_t;

// ---------------------------------------------------------------------------
// Scan snapshot — full 8×8 frame for every sensor in the ring
// ---------------------------------------------------------------------------
typedef struct {
    tof_frame_t frame[TOF_SENSOR_COUNT];
    uint8_t     sensor_ok[TOF_SENSOR_COUNT]; // 1 = sensor initialised and ranging
} tof_scan_t;

// ---------------------------------------------------------------------------
// Init & task entry
// ---------------------------------------------------------------------------

// Call once before spawning the task.
// Creates the I2C master bus internally.
void tof_task_init(void);

// FreeRTOS task entry — pin to Core 0, Priority 4
// xTaskCreatePinnedToCore(tof_task, "tof", TOF_TASK_STACK,
//                         NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE);
void tof_task(void *arg);

// ---------------------------------------------------------------------------
// Data access API (thread-safe, call from mission_task)
// ---------------------------------------------------------------------------

// Returns the minimum range (metres) across all sensors whose coverage arc
// overlaps [angle_min_deg, angle_max_deg].
//
// Angles are body-frame degrees, 0=forward, CW positive.
// The arc may cross 0° (e.g. angle_min=-30, angle_max=+30 for forward sector).
// Returns INFINITY if no valid sensor covers the requested arc.
//
// Primary use: Bug2 obstacle detection
//   Forward sector:    tof_get_min_range_in_sector(-30.0f, 30.0f)
//   Left boundary:     tof_get_min_range_in_sector(45.0f, 135.0f)
float tof_get_min_range_in_sector(float angle_min_deg, float angle_max_deg);

// Full scan snapshot — returns a copy of the latest scan data.
// Use for telemetry or when you need all readings at once.
tof_scan_t tof_get_scan(void);

// Collapse the 8-sensor ring into a flat 2D scan 
// For each sensor, get the minimum of each column for rows 3-7
// Maximum count = 64 (8 sensors × 8 columns).
// Thread-safe: takes the scan mutex internally via tof_get_scan().

typedef struct {
    float ranges[TOF_SENSOR_COUNT * TOF_SENSOR_RESO];
} tof_scan_collapsed_t;

tof_scan_collapsed_t tof_get_collapsed_scan(void);

// True if at least one sensor has been successfully initialised and
// has produced a reading in the last 500ms.
bool tof_is_healthy(void);