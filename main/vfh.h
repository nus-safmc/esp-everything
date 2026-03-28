#pragma once
#include <stdbool.h>
#include "tof_task.h"

/* --------------------------------------------------------------------------
 * VFH+ polar-histogram obstacle avoidance
 *
 * Pure C — no FreeRTOS, no ESP-IDF headers.  Testable with plain gcc.
 *
 * Pipeline:
 *   Stage 1  Build polar histogram from range readings
 *   Stage 2  Threshold to binary blocked / free map
 *   Stage 3  Grow blocked sectors by drone_radius for safety
 *   Stage 4  Find free valleys (contiguous free sectors)
 *   Stage 5  Score each candidate direction
 *   Stage 6  Return centre angle of lowest-cost valley
 * -------------------------------------------------------------------------- */

#define VFH_BINS 32   //2 sensor readings per bin

/* --------------------------------------------------------------------------
 * vfh_config_t
 * -------------------------------------------------------------------------- */
typedef struct {
    float w_goal;             /* weight for goal-direction cost term          */
    float w_current;          /* weight for current-heading (0°) cost term    */
    float w_prev;             /* weight for previous-steering cost term       */
    float density_threshold;  /* histogram density above which a bin is blocked */
    float drone_radius_m;     /* drone inscribed radius (m) — drives growth   */
    float max_range_m;        /* sensor max useful range (m)                  */
} vfh_config_t;

/* --------------------------------------------------------------------------
 * Sensible defaults — copy and tweak as needed.
 *
 *   vfh_config_t cfg = VFH_DEFAULT_CONFIG;
 * -------------------------------------------------------------------------- */
#define VFH_DEFAULT_CONFIG  {   \
    .w_goal            = 2.0f,  \
    .w_current         = 1.0f,  \
    .w_prev            = 0.5f,  \
    .density_threshold = 1.0f,  \
    .drone_radius_m    = 0.20f, \
    .max_range_m       = 4.0f,  \
}

/* --------------------------------------------------------------------------
 * vfh_compute
 *
 * Returns the best steering angle in body frame (radians).
 *   0        = straight ahead
 *   positive = clockwise / right
 *   negative = counter-clockwise / left
 *
 * Returns prev_steering_rad unchanged if the entire histogram is blocked
 * (caller should treat this as a stuck condition).
 *
 * goal_body_angle_rad  desired direction in body frame (rad, CW positive)
 * prev_steering_rad    steering angle returned on the previous call (rad, CW+)
 * -------------------------------------------------------------------------- */
float vfh_compute(const tof_scan_collapsed_t *scan,
                  const vfh_config_t     *cfg,
                  float                   goal_body_angle_rad,
                  float                   prev_steering_rad);

/* --------------------------------------------------------------------------
 * vfh_get_histogram
 *
 * Exposes the raw histogram and the grown binary blocked/free map used
 * internally by vfh_compute.  Useful for telemetry and debug visualisation.
 *
 * hist_out[b]   accumulated obstacle density in bin b (pre-threshold)
 * binary_out[b] true = blocked (after growth), false = free
 * -------------------------------------------------------------------------- */
void vfh_get_histogram(const tof_scan_collapsed_t *scan,
                       const vfh_config_t     *cfg,
                       float                   hist_out[VFH_BINS],
                       bool                    binary_out[VFH_BINS]);
