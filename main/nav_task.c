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
    float ned_bearing_rad;      /* NED bearing to avoid                         */
    uint32_t expiry_ms;         /* timestamp when this entry expires            */
} forbidden_entry_t;

typedef struct {
    nav_state_t state;
    float goal_x, goal_y, goal_z;
    bool  has_goal;
    float prev_steering_rad;    /* VFH prev input for continuity                */
    uint32_t stuck_since_ms;    /* timestamp when STUCK was entered             */
    uint32_t stuck_count;       /* lifetime stuck events                        */
    float retreat_bearing_rad;  /* NED bearing to retreat along (opposite goal) */
    uint32_t retreat_start_ms;  /* when retreat began                           */
    forbidden_entry_t forbidden[NAV_MAX_FORBIDDEN];
    int forbidden_count;
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

/* Add a forbidden NED bearing (oldest entry evicted if full) */
static void add_forbidden(nav_internal_t *n, float ned_bearing_rad, uint32_t now_ms)
{
    /* Expire stale entries first */
    for (int i = 0; i < n->forbidden_count; ) {
        if (now_ms >= n->forbidden[i].expiry_ms) {
            n->forbidden[i] = n->forbidden[--n->forbidden_count];
        } else {
            i++;
        }
    }
    /* Evict oldest if full */
    if (n->forbidden_count >= NAV_MAX_FORBIDDEN) {
        n->forbidden[0] = n->forbidden[--n->forbidden_count];
    }
    n->forbidden[n->forbidden_count++] = (forbidden_entry_t){
        .ned_bearing_rad = ned_bearing_rad,
        .expiry_ms       = now_ms + NAV_FORBIDDEN_TTL_MS,
    };
}

/* Build extra_blocked array from forbidden bearings in body frame */
static void build_forbidden_blocked(const nav_internal_t *n, float heading,
                                    uint32_t now_ms, bool out[VFH_BINS])
{
    memset(out, 0, VFH_BINS * sizeof(bool));
    for (int i = 0; i < n->forbidden_count; i++) {
        if (now_ms >= n->forbidden[i].expiry_ms) continue;
        /* Convert NED bearing to body-frame angle, then to bin index */
        float body_angle = wrap_pi(n->forbidden[i].ned_bearing_rad - heading);
        /* body_angle in (-π,π] → degrees in [0,360) */
        float deg = body_angle * 180.0f / (float)M_PI;
        if (deg < 0.0f) deg += 360.0f;
        int center_bin = (int)(deg / (360.0f / VFH_BINS)) % VFH_BINS;
        for (int d = -NAV_FORBIDDEN_HALF_W; d <= NAV_FORBIDDEN_HALF_W; d++) {
            out[(center_bin + d + VFH_BINS) % VFH_BINS] = true;
        }
    }
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
    nav_state_t new_state          = nav.state;
    float       new_steering       = nav.prev_steering_rad;
    uint32_t    new_stuck_count    = nav.stuck_count;
    uint32_t    new_stuck_since_ms = nav.stuck_since_ms;

    /* Build forbidden-zone overlay for VFH */
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    bool extra_blocked[VFH_BINS];
    build_forbidden_blocked(&nav, drone.heading, now_ms, extra_blocked);

    /* Count free bins with forbidden zones applied */
    int effective_free = 0;
    for (int b = 0; b < VFH_BINS; b++)
        if (!blocked[b] && !extra_blocked[b]) effective_free++;

    if (nav.state == NAV_STUCK) {
        /* STUCK: hold position, wait, then start retreating */
        if ((now_ms - nav.stuck_since_ms) >= NAV_STUCK_HOLD_MS) {
            /* Retreat direction = opposite of goal bearing */
            float retreat_brg = wrap_pi(goal_ned_angle + (float)M_PI);
            add_forbidden(&nav, goal_ned_angle, now_ms);
            ESP_LOGI(TAG, "Retreating away from goal bearing (%.0f°)",
                     goal_ned_angle * 180.0f / (float)M_PI);
            new_state = NAV_RETREATING;
            new_steering = 0.0f;

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_nav.retreat_bearing_rad = retreat_brg;
            s_nav.retreat_start_ms    = now_ms;
            /* Copy forbidden array back */
            memcpy(s_nav.forbidden, nav.forbidden, sizeof(nav.forbidden));
            s_nav.forbidden_count = nav.forbidden_count;
            xSemaphoreGive(s_mutex);

            /* Start backing up */
            float tx = drone.x + NAV_RETREAT_STEP_M * cosf(retreat_brg);
            float ty = drone.y + NAV_RETREAT_STEP_M * sinf(retreat_brg);
            mavlink_set_position_ned(tx, ty, nav.goal_z, retreat_brg);
        }

    } else if (nav.state == NAV_RETREATING) {
        /* Back up incrementally until VFH finds viable directions */
        bool timed_out = (now_ms - nav.retreat_start_ms) >= NAV_RETREAT_MAX_MS;

        if (effective_free >= NAV_STUCK_MIN_FREE || timed_out) {
            if (timed_out) {
                ESP_LOGW(TAG, "Retreat timed out — resuming with %d free bins", effective_free);
            } else {
                ESP_LOGI(TAG, "Path found (%d free bins) — resuming goal", effective_free);
            }
            new_state    = NAV_ROTATING;
            new_steering = 0.0f;
        } else {
            /* Keep backing up */
            float tx = drone.x + NAV_RETREAT_STEP_M * cosf(nav.retreat_bearing_rad);
            float ty = drone.y + NAV_RETREAT_STEP_M * sinf(nav.retreat_bearing_rad);
            mavlink_set_position_ned(tx, ty, nav.goal_z, nav.retreat_bearing_rad);
        }

    } else if (free_count == 0) {
        /* All VFH bins blocked — declare stuck */
        mavlink_set_hold();
        new_state          = NAV_STUCK;
        new_stuck_since_ms = now_ms;
        new_stuck_count    = nav.stuck_count + 1;
        ESP_LOGW(TAG, "STUCK — all directions blocked (event #%lu)", (unsigned long)new_stuck_count);

    } else {
        /* Normal navigation: get VFH steering with forbidden zones */
        float steering = vfh_compute(&scan, vfh_cfg, goal_body_angle,
                                     nav.prev_steering_rad, extra_blocked);
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
    s_nav.state            = new_state;
    s_nav.prev_steering_rad = new_steering;
    s_nav.stuck_count      = new_stuck_count;
    s_nav.stuck_since_ms   = new_stuck_since_ms;

    s_status.state            = new_state;
    s_status.goal_x           = nav.goal_x;
    s_status.goal_y           = nav.goal_y;
    s_status.goal_z           = nav.goal_z;
    s_status.dist_to_goal     = dist;
    s_status.heading_error_rad = wrap_pi(goal_ned_angle - drone.heading);
    s_status.vfh_steering_rad  = new_steering;
    s_status.free_bins         = free_count;
    s_status.stuck_count       = new_stuck_count;
    for (int b = 0; b < VFH_BINS; b++)
        s_status.vfh_blocked[b] = blocked[b] || extra_blocked[b];
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
    s_nav.forbidden_count = 0;
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
