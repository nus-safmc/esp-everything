#include "nav_task.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "vfh.h"
#include "breadcrumb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>

static const char *TAG = "nav";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------------------------------------------------------------------
 * Internal state
 * --------------------------------------------------------------------------- */
typedef struct {
    nav_state_t state;
    float goal_x, goal_y, goal_z;
    bool  has_goal;
    float prev_steering_rad;    /* VFH prev input for continuity                */
    uint32_t stuck_count;       /* lifetime stuck events                        */
} nav_internal_t;

static nav_internal_t    s_nav;
static nav_status_t      s_status;
static SemaphoreHandle_t s_mutex;

/* ---------------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------------- */

/* Wrap angle to (-π, π] */
static float wrap_pi(float a)
{
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    return a - (float)M_PI;
}

/* ---------------------------------------------------------------------------
 * Emergency collision avoidance
 *
 * Scans all 64 collapsed-scan range points.  If any point is below
 * COLLISION_DANGER_M, computes a repulsion velocity in NED frame and
 * commands it directly, bypassing all normal navigation logic.
 *
 * Returns true if avoidance fired (caller should skip the rest of nav_tick).
 * --------------------------------------------------------------------------- */
#define COLLISION_DANGER_M    0.20f   /* trigger threshold (m)                    */
#define COLLISION_CLEAR_M     0.30f   /* resume normal nav once all points above  */
#define COLLISION_SPEED_MS    0.4f    /* escape velocity magnitude (m/s)          */
#define COLLISION_SCAN_PTS    (TOF_SENSOR_COUNT * TOF_SENSOR_RESO)  /* 64         */

static bool s_collision_active = false;

static bool collision_avoid(float goal_z)
{
    if (!mavlink_position_valid()) return false;

    tof_scan_collapsed_t scan = tof_get_collapsed_scan();
    drone_state_t drone = mavlink_get_state();

    /* Determine the threshold: once active, require CLEAR_M before releasing */
    float threshold = s_collision_active ? COLLISION_CLEAR_M : COLLISION_DANGER_M;

    /* Accumulate repulsion vector in NED frame.
     * Scan index 0 = front (0°), CW.  Each point covers 360/64 = 5.625°. */
    float repulse_n = 0.0f;
    float repulse_e = 0.0f;
    int   threats   = 0;

    for (int i = 0; i < COLLISION_SCAN_PTS; i++) {
        if (scan.ranges[i] >= threshold) continue;

        /* Body-frame angle of this scan point (CW from forward, radians) */
        float body_rad = (float)i * (2.0f * (float)M_PI / (float)COLLISION_SCAN_PTS);
        /* NED angle */
        float ned_rad = drone.heading + body_rad;

        /* Weight: closer = stronger repulsion (1 at 0m, 0 at threshold) */
        float weight = 1.0f - (scan.ranges[i] / threshold);

        /* Push AWAY from this direction */
        repulse_n -= weight * cosf(ned_rad);
        repulse_e -= weight * sinf(ned_rad);
        threats++;
    }

    if (threats == 0) {
        if (s_collision_active) {
            ESP_LOGI(TAG, "Collision clear — resuming nav");
            s_collision_active = false;
        }
        return false;
    }

    /* Normalise and scale to fixed escape speed */
    float mag = sqrtf(repulse_n * repulse_n + repulse_e * repulse_e);
    if (mag < 1e-6f) return false;

    float vn = COLLISION_SPEED_MS * (repulse_n / mag);
    float ve = COLLISION_SPEED_MS * (repulse_e / mag);

    mavlink_set_velocity_xy_position_z(vn, ve, goal_z, drone.heading);

    if (!s_collision_active) {
        ESP_LOGW(TAG, "COLLISION AVOID — %d threats, escaping (%.2f, %.2f) m/s",
                 threats, vn, ve);
        s_collision_active = true;
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * nav_tick — executed every 100 ms (10 Hz) by nav_task
 * --------------------------------------------------------------------------- */
static void nav_tick(const vfh_config_t *vfh_cfg)
{
    /* Snapshot shared state */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    nav_internal_t nav = s_nav;
    xSemaphoreGive(s_mutex);

    /* Nothing to do while idle */
    if (nav.state == NAV_IDLE || !nav.has_goal) return;

    /* --- Emergency collision avoidance (pre-empts everything) --- */
    if (collision_avoid(nav.goal_z)) return;

    /* Require fresh telemetry */
    if (!mavlink_position_valid()) {
        ESP_LOGW(TAG, "Position invalid — skipping tick");
        return;
    }

    drone_state_t drone = mavlink_get_state();

    /* Record position breadcrumb */
    crumb_update(drone.x, drone.y);

    /* ---- Horizontal distance to goal ---- */
    float dx   = nav.goal_x - drone.x;
    float dy   = nav.goal_y - drone.y;
    float dist = sqrtf(dx * dx + dy * dy);

    /* Altitude is held by PX4's own position controller via the Z-position
     * field in the mixed velocity/position setpoint — nothing to do here. */

    /* ---- Check arrival ---- */
    if (dist < NAV_ARRIVE_RADIUS_M) {
        mavlink_set_position_ned(nav.goal_x, nav.goal_y, nav.goal_z, drone.heading);
        ESP_LOGI(TAG, "Goal reached (dist=%.2f m)", dist);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_nav.state    = NAV_ARRIVED;
        s_nav.has_goal = false;
        s_status.state = NAV_ARRIVED;
        s_status.dist_to_goal = dist;
        xSemaphoreGive(s_mutex);
        return;
    }

    /* ---- VFH: stuck check + steering ----
     *
     * Call vfh_get_histogram once to get the free-bin count (stuck detection).
     * If not stuck, call vfh_compute for the best steering direction.
     * vfh_compute re-runs the histogram internally — acceptable at 10 Hz. */
    tof_scan_collapsed_t scan = tof_get_collapsed_scan();

    float hist[VFH_BINS];
    bool  blocked[VFH_BINS];
    vfh_get_histogram(&scan, vfh_cfg, hist, blocked);

    int free_count = 0;
    for (int b = 0; b < VFH_BINS; b++) if (!blocked[b]) free_count++;

    /* ---- NED bearing to goal ----
     * atan2(East_delta, North_delta) gives NED bearing: 0=North, CW positive */
    float goal_ned_angle = atan2f(dy, dx);

    /* Goal direction in body frame: 0=forward, CW positive */
    float goal_body_angle = wrap_pi(goal_ned_angle - drone.heading);

    /* ---- Compute updated state ---- */
    nav_state_t new_state       = nav.state;
    float       new_steering    = nav.prev_steering_rad;
    uint32_t    new_stuck_count = nav.stuck_count;

    if (nav.state == NAV_STUCK) {
        /* Hold position — laptop stuck monitor will send a new goal */
        mavlink_set_hold();

    } else if (free_count == 0) {
        /* All VFH bins blocked — hold and signal stuck to laptop */
        mavlink_set_hold();
        new_state       = NAV_STUCK;
        new_stuck_count = nav.stuck_count + 1;
        ESP_LOGW(TAG, "STUCK — all directions blocked (event #%lu)", (unsigned long)new_stuck_count);

    } else {
        /* Normal navigation */
        float steering = vfh_compute(&scan, vfh_cfg, goal_body_angle,
                                     nav.prev_steering_rad, NULL);
        new_steering = steering;

        /* |steering| is the heading error: how much we must rotate before flying.
         *
         * Strict "face forward" rule:
         *   If misaligned → rotate in place, zero forward velocity.
         *   If aligned    → fly forward at cruise speed, zero yaw.
         *
         * This ensures the front camera always faces the direction of travel. */
        float heading_error = steering;

        /* Desired NED heading = current heading + VFH body-frame steering.
         * PX4's attitude controller rotates to this angle at its own rate —
         * no gain tuning needed on our side. */
        float desired_yaw = drone.heading + steering;

        if (fabsf(heading_error) > NAV_YAW_TOL_RAD) {
            /* ---- ROTATING: hold position, let PX4 rotate to desired_yaw ----
             * Use position mode so PX4's position controller actively fights
             * drift.  Velocity mode with vx=vy=0 only targets zero velocity
             * and lets the drone drift freely. */
            new_state = NAV_ROTATING;
            mavlink_set_position_ned(drone.x, drone.y, nav.goal_z, desired_yaw);

        } else {
            /* ---- FLYING: aligned — fly forward at desired_yaw ----
             * VFH re-evaluates every tick; if an obstacle causes |steering| to
             * exceed NAV_YAW_TOL_RAD, the drone stops and re-aligns. */
            new_state = NAV_FLYING;
            float vx = NAV_CRUISE_SPEED_MS * cosf(desired_yaw);
            float vy = NAV_CRUISE_SPEED_MS * sinf(desired_yaw);
            mavlink_set_velocity_xy_position_z(vx, vy, nav.goal_z, desired_yaw);
        }

        ESP_LOGD(TAG, "state=%d dist=%.2f err=%.1f° steer=%.1f° free=%d",
                 new_state, dist,
                 heading_error * 180.0f / (float)M_PI,
                 steering      * 180.0f / (float)M_PI,
                 free_count);
    }

    /* Write back under mutex */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav.state             = new_state;
    s_nav.prev_steering_rad = new_steering;
    s_nav.stuck_count       = new_stuck_count;

    s_status.state             = new_state;
    s_status.goal_x            = nav.goal_x;
    s_status.goal_y            = nav.goal_y;
    s_status.goal_z            = nav.goal_z;
    s_status.dist_to_goal      = dist;
    s_status.heading_error_rad = wrap_pi(goal_ned_angle - drone.heading);
    s_status.vfh_steering_rad  = new_steering;
    s_status.free_bins         = free_count;
    s_status.stuck_count       = new_stuck_count;
    for (int b = 0; b < VFH_BINS; b++)
        s_status.vfh_blocked[b] = blocked[b];
    xSemaphoreGive(s_mutex);
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

void nav_task_init(void)
{
    memset(&s_nav,    0, sizeof(s_nav));
    memset(&s_status, 0, sizeof(s_status));
    s_nav.state    = NAV_IDLE;
    s_status.state = NAV_IDLE;
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
    crumb_init();
}

void nav_set_goal_ned(float gx, float gy, float gz)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav.goal_x          = gx;
    s_nav.goal_y          = gy;
    s_nav.goal_z          = gz;
    s_nav.has_goal        = true;
    s_nav.state           = NAV_ROTATING;
    s_nav.prev_steering_rad = 0.0f;
    xSemaphoreGive(s_mutex);

    /* Issue hold before nav_task takes over, so PX4 freezes in place
     * during the initial alignment rotation. */
    mavlink_set_hold();

    ESP_LOGI(TAG, "Goal set: NED (%.2f, %.2f, %.2f)", gx, gy, gz);
}

void nav_cancel(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav.state    = NAV_IDLE;
    s_nav.has_goal = false;
    s_status.state = NAV_IDLE;
    xSemaphoreGive(s_mutex);

    mavlink_set_hold();
    ESP_LOGI(TAG, "Navigation cancelled — holding");
}

nav_status_t nav_get_status(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    nav_status_t copy = s_status;
    xSemaphoreGive(s_mutex);
    return copy;
}

/* ---------------------------------------------------------------------------
 * FreeRTOS task — 10 Hz navigation loop
 * --------------------------------------------------------------------------- */

void nav_task(void *arg)
{
    vfh_config_t vfh_cfg    = VFH_DEFAULT_CONFIG;
    TickType_t   last_wake  = xTaskGetTickCount();

    ESP_LOGI(TAG, "Navigator started (%.1f m/s cruise, ±%.0f° yaw tol)",
             NAV_CRUISE_SPEED_MS, NAV_YAW_TOL_RAD * 180.0f / (float)M_PI);

    while (1) {
        nav_tick(&vfh_cfg);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));  /* 10 Hz */
    }
}
