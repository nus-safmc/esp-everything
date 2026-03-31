#include "nav_task.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "vfh.h"
#include "wifi_task.h"
#include "odom.h"       /* map_to_odom() — convert goal each tick */

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
    float goal_map_x, goal_map_y;   /* goal in MAP frame (north, east)          */
    float goal_z;                   /* NED down (negative = above ground)       */
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

/* Total collapsed scan points (shared by peer injection + collision avoidance) */
#define COLLISION_SCAN_PTS    (TOF_SENSOR_COUNT * TOF_SENSOR_RESO)  /* 64         */

/* ---------------------------------------------------------------------------
 * Inject peer drone positions as virtual obstacles into the collapsed scan.
 *
 * For each peer within VFH range, computes the body-frame bearing and
 * distance, then writes the distance into the corresponding scan bin(s).
 * This makes VFH treat other drones as physical obstacles — the density
 * weighting and growth step handle safety margins automatically.
 * --------------------------------------------------------------------------- */
/* Peer avoidance tuning:
 *
 *  A peer blocks its VFH bin when density >= density_threshold (1.5).
 *  density_at_d = PEER_DENSITY_MAX * (1 − d / PEER_INJECT_RANGE_M)
 *  Blocking distance = PEER_INJECT_RANGE_M * (1 − 1.5/PEER_DENSITY_MAX)
 *
 *  With the values below:  block at 4.0 * (1 − 1.5/9.0) = 3.33 m
 *  Head-on at 0.6 m/s each → 3.33 / 1.2 = 2.8 s warning.
 */
#define PEER_INJECT_RANGE_M   4.0f   /* ignore peers beyond this distance (m)  */
#define PEER_DENSITY_MAX      10.0f   /* density at distance 0 (threshold=1.5)  */
#define PEER_BIN_SPREAD       3      /* inject into ±N VFH bins around bearing */

static void inject_peers_into_histogram(float hist[VFH_BINS],
                                        float drone_x, float drone_y,
                                        float heading)
{
    wifi_peer_list_t peers = wifi_get_peers();

    for (int p = 0; p < peers.count; p++) {
        float odom_x, odom_y;
        map_to_odom(peers.peers[p].map_x, peers.peers[p].map_y, &odom_x, &odom_y);

        float dx   = odom_x - drone_x;
        float dy   = odom_y - drone_y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist > PEER_INJECT_RANGE_M || dist < 0.01f) continue;

        /* Density boost: strong when close, fades to zero at PEER_INJECT_RANGE_M */
        float density = PEER_DENSITY_MAX * (1.0f - dist / PEER_INJECT_RANGE_M);

        /* Body-frame bearing to peer */
        float ned_bearing  = atan2f(dy, dx);
        float body_bearing = ned_bearing - heading;
        body_bearing = fmodf(body_bearing, 2.0f * (float)M_PI);
        if (body_bearing < 0.0f) body_bearing += 2.0f * (float)M_PI;

        /* Map to VFH bin and spread across ± PEER_BIN_SPREAD */
        float bin_width_rad = 2.0f * (float)M_PI / (float)VFH_BINS;
        int center_bin = (int)(body_bearing / bin_width_rad) % VFH_BINS;

        for (int d = -PEER_BIN_SPREAD; d <= PEER_BIN_SPREAD; d++) {
            int b = (center_bin + d + VFH_BINS) % VFH_BINS;
            /* Taper: full density at center, half at the edges */
            float taper = 1.0f - 0.5f * (float)abs(d) / (float)(PEER_BIN_SPREAD + 1);
            hist[b] += density * taper;
        }
    }
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
#define COLLISION_DANGER_M    0.40f   /* trigger threshold (m)                    */
#define COLLISION_CLEAR_M     0.50f   /* resume normal nav once all points above  */
#define COLLISION_SPEED_MS    0.35f    /* escape velocity magnitude (m/s)          */

static bool s_collision_active = false;

static bool collision_avoid(float goal_z)
{
    if (!mavlink_position_valid()) return false;
    if (!tof_is_healthy()) return false;   /* stale sensors — let nav_tick hold */

    tof_scan_collapsed_t scan = tof_get_collapsed_scan();
    drone_state_t drone = mavlink_get_state();

    /* Determine the threshold: once active, require CLEAR_M before releasing */
    float threshold = s_collision_active ? COLLISION_CLEAR_M : COLLISION_DANGER_M;

    /* Accumulate repulsion vector in body frame.
     * Scan index 0 = front (0°), CW.  Each point covers 360/64 = 5.625°. */
    float repulse_fwd  = 0.0f;   /* body X: positive = forward             */
    float repulse_rght = 0.0f;   /* body Y: positive = right               */
    int   threats      = 0;
    float closest      = 999.0f;

    for (int i = 0; i < COLLISION_SCAN_PTS; i++) {
        if (scan.ranges[i] >= threshold) continue;

        /* Body-frame angle of this scan point (CW from forward, radians) */
        float body_rad = (float)i * (2.0f * (float)M_PI / (float)COLLISION_SCAN_PTS);

        /* Weight: closer = stronger repulsion (1 at 0m, 0 at threshold) */
        float weight = 1.0f - (scan.ranges[i] / threshold);

        /* Push AWAY from this direction (body frame) */
        repulse_fwd  -= weight * cosf(body_rad);
        repulse_rght -= weight * sinf(body_rad);
        threats++;

        if (scan.ranges[i] < closest) closest = scan.ranges[i];
    }

    if (threats == 0) {
        if (s_collision_active) {
            ESP_LOGI(TAG, "Collision clear — resuming nav");
            s_collision_active = false;
        }
        return false;
    }

    /* Check if repulsion vectors nearly cancel (e.g. narrow corridor with
     * walls on both sides).  In that case, let VFH handle navigation
     * instead of oscillating between walls.  Only intervene if the net
     * repulsion has meaningful magnitude relative to the total threat weight. */
    float body_mag = sqrtf(repulse_fwd * repulse_fwd + repulse_rght * repulse_rght);
    if (body_mag < 0.3f && closest > 0.18f) {
        /* Opposing threats nearly cancel and nothing is dangerously close —
         * release collision avoidance so VFH can navigate the corridor. */
        if (s_collision_active) {
            ESP_LOGI(TAG, "Collision: opposing threats cancel (mag=%.2f), releasing to VFH",
                     body_mag);
            s_collision_active = false;
        }
        return false;
    }

    /* Convert body-frame repulsion to NED */
    float cos_h = cosf(drone.heading);
    float sin_h = sinf(drone.heading);
    float repulse_n = repulse_fwd * cos_h - repulse_rght * sin_h;
    float repulse_e = repulse_fwd * sin_h + repulse_rght * cos_h;

    /* Normalise and scale to fixed escape speed */
    float ned_mag = sqrtf(repulse_n * repulse_n + repulse_e * repulse_e);
    if (ned_mag < 1e-6f) return false;

    float vn = COLLISION_SPEED_MS * (repulse_n / ned_mag);
    float ve = COLLISION_SPEED_MS * (repulse_e / ned_mag);

    mavlink_set_velocity_xy_position_z(vn, ve, goal_z, drone.heading);

    if (!s_collision_active) {
        ESP_LOGW(TAG, "COLLISION AVOID — %d threats, closest=%.2fm, escape=(%.2f,%.2f) m/s",
                 threats, closest, vn, ve);
        s_collision_active = true;
    } else {
        ESP_LOGD(TAG, "COLLISION: %d threats, closest=%.2fm, vel=(%.2f,%.2f)",
                 threats, closest, vn, ve);
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

    /* --- Emergency collision avoidance (pre-empts everything, even idle) --- */
    float hold_z = nav.has_goal ? nav.goal_z : -0.5f;  /* fallback cruise alt */
    if (collision_avoid(hold_z)) return;

    /* Nothing to do while idle */
    if (nav.state == NAV_IDLE || !nav.has_goal) return;

    /* Require fresh telemetry */
    if (!mavlink_position_valid()) {
        ESP_LOGW(TAG, "Position invalid — skipping tick");
        return;
    }

    drone_state_t drone = mavlink_get_state();

    /* Convert goal from map frame to odom frame every tick so that
     * relocalization corrections are picked up immediately. */
    float goal_x, goal_y;
    map_to_odom(nav.goal_map_x, nav.goal_map_y, &goal_x, &goal_y);

    /* ---- Horizontal distance to goal ---- */
    float dx   = goal_x - drone.x;
    float dy   = goal_y - drone.y;
    float dist = sqrtf(dx * dx + dy * dy);

    /* Altitude is held by PX4's own position controller via the Z-position
     * field in the mixed velocity/position setpoint — nothing to do here. */

    /* ---- Check arrival ---- */
    if (dist < NAV_ARRIVE_RADIUS_M) {
        mavlink_set_position_ned(goal_x, goal_y, nav.goal_z, drone.heading);
        ESP_LOGI(TAG, "Goal reached (dist=%.2f m)", dist);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_nav.state    = NAV_ARRIVED;
        s_nav.has_goal = false;
        s_status.state = NAV_ARRIVED;
        s_status.dist_to_goal = dist;
        xSemaphoreGive(s_mutex);
        return;
    }

    /* ---- Guard: hold position if ToF sensors are stale ---- */
    if (!tof_is_healthy()) {
        ESP_LOGW(TAG, "ToF sensors stale — holding position");
        mavlink_set_position_ned(drone.x, drone.y, nav.goal_z, drone.heading);
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

    /* Add soft repulsion from peer drones and re-threshold affected bins */
    inject_peers_into_histogram(hist, drone.x, drone.y, drone.heading);
    for (int b = 0; b < VFH_BINS; b++)
        if (hist[b] >= vfh_cfg->density_threshold) blocked[b] = true;

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
                                     nav.prev_steering_rad, blocked);
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
             * exceed NAV_YAW_TOL_RAD, the drone stops and re-aligns.
             *
             * Speed is scaled by the closest obstacle in the forward 90°
             * arc so the drone decelerates smoothly before walls, giving
             * ToF round-robin updates time to catch up. */
            new_state = NAV_FLYING;

            /* Find min range in ±45° forward arc from collapsed scan */
            float min_fwd = 999.0f;
            for (int i = 0; i < COLLISION_SCAN_PTS; i++) {
                float bin_rad = (float)i * (2.0f * (float)M_PI / (float)COLLISION_SCAN_PTS);
                float diff = wrap_pi(bin_rad - steering);
                if (fabsf(diff) <= (float)M_PI / 4.0f && scan.ranges[i] < min_fwd) {
                    min_fwd = scan.ranges[i];
                }
            }

            /* Ramp speed: full cruise above 1.5 m, linearly down to 30%
             * at COLLISION_DANGER_M, clamped so we never command < 30%. */
#define SPEED_RAMP_FULL_M   1.5f
#define SPEED_RAMP_MIN_FRAC 0.30f
            float frac = 1.0f;
            if (min_fwd < SPEED_RAMP_FULL_M) {
                frac = SPEED_RAMP_MIN_FRAC
                     + (1.0f - SPEED_RAMP_MIN_FRAC)
                       * (min_fwd - COLLISION_DANGER_M)
                       / (SPEED_RAMP_FULL_M - COLLISION_DANGER_M);
                if (frac < SPEED_RAMP_MIN_FRAC) frac = SPEED_RAMP_MIN_FRAC;
                if (frac > 1.0f) frac = 1.0f;
            }

            float speed = NAV_CRUISE_SPEED_MS * frac;
            float vx = speed * cosf(desired_yaw);
            float vy = speed * sinf(desired_yaw);
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
    s_status.goal_x            = nav.goal_map_x;
    s_status.goal_y            = nav.goal_map_y;
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
}

void nav_set_goal_map(float map_x, float map_y, float z)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav.goal_map_x      = map_x;
    s_nav.goal_map_y      = map_y;
    s_nav.goal_z          = z;
    s_nav.has_goal        = true;
    s_nav.state           = NAV_ROTATING;
    s_nav.prev_steering_rad = 0.0f;
    xSemaphoreGive(s_mutex);

    /* Issue hold before nav_task takes over, so PX4 freezes in place
     * during the initial alignment rotation. */
    mavlink_set_hold();

    ESP_LOGI(TAG, "Goal set: map (%.2f, %.2f) z=%.2f", map_x, map_y, z);
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
    vfh_config_t vfh_cfg         = VFH_DEFAULT_CONFIG;
    TickType_t   last_wake        = xTaskGetTickCount();
    TickType_t   wifi_discon_tick = 0;   /* 0 = link up; else tick of first drop */
    bool         wifi_kill_sent   = false;

    ESP_LOGI(TAG, "Navigator started (%.1f m/s cruise, ±%.0f° yaw tol)",
             NAV_CRUISE_SPEED_MS, NAV_YAW_TOL_RAD * 180.0f / (float)M_PI);

    while (1) {
        /* ---- WiFi link-loss killswitch ----
         * If the link has been continuously down for >3 s while armed,
         * cancel navigation and disarm immediately (motor cut / drop). */
        if (!wifi_is_connected()) {
            if (wifi_discon_tick == 0) {
                wifi_discon_tick = xTaskGetTickCount();
                ESP_LOGW(TAG, "WiFi link lost — disarming in 3 s if not restored");
            } else if (!wifi_kill_sent &&
                       (xTaskGetTickCount() - wifi_discon_tick) >= pdMS_TO_TICKS(3000) &&
                       mavlink_get_state().armed) {
                ESP_LOGE(TAG, "WiFi lost >3 s — motor kill (disarm)");
                nav_cancel();
                mavlink_arm(false);
                wifi_kill_sent = true;
            }
        } else {
            if (wifi_discon_tick != 0) {
                ESP_LOGI(TAG, "WiFi link restored");
                wifi_discon_tick = 0;
                wifi_kill_sent   = false;
            }
        }

        nav_tick(&vfh_cfg);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}
