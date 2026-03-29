#include "odom.h"
#include "at_detect.h"      /* camera_to_ned() */
#include "mavlink_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = "odom";

/* ---------------------------------------------------------------------------
 * Shared state
 * --------------------------------------------------------------------------- */
static nav_tag_t         s_nav_tags[ODOM_MAX_NAV_TAGS];
static int               s_nav_tag_count = 0;
static SemaphoreHandle_t s_mutex;

/* ---------------------------------------------------------------------------
 * Lifecycle
 * --------------------------------------------------------------------------- */

void odom_init(void)
{
    s_nav_tag_count = 0;
    memset(s_nav_tags, 0, sizeof(s_nav_tags));
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
}

/* ---------------------------------------------------------------------------
 * Nav-tag table management
 * --------------------------------------------------------------------------- */

void odom_set_nav_tags(const nav_tag_t *tags, int count)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav_tag_count = 0;
    for (int i = 0; i < count && s_nav_tag_count < ODOM_MAX_NAV_TAGS; i++) {
        if (tags[i].id >= 0) {
            s_nav_tags[s_nav_tag_count++] = tags[i];
        }
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Nav-tag table updated (%d tags)", s_nav_tag_count);
    for (int i = 0; i < s_nav_tag_count; i++) {
        ESP_LOGI(TAG, "  tag %d → map (%.2f, %.2f)",
                 s_nav_tags[i].id, s_nav_tags[i].map_x, s_nav_tags[i].map_y);
    }
}

bool odom_find_nav_tag(int tag_id, nav_tag_t *out)
{
    bool found = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < s_nav_tag_count; i++) {
        if (s_nav_tags[i].id == (int8_t)tag_id) {
            if (out) *out = s_nav_tags[i];
            found = true;
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return found;
}

/* ---------------------------------------------------------------------------
 * Odometry correction
 *
 * Inverse transform:
 *   1. camera_to_ned() gives the NED offset (dn, de) from drone → tag
 *   2. drone_map = tag_known − (dn, de)
 *   3. Send VISION_POSITION_ESTIMATE to PX4 with the computed position
 *
 * PX4's EKF2 fuses this with IMU to correct drift.  The map frame
 * effectively becomes PX4's local NED frame once the EKF converges.
 * --------------------------------------------------------------------------- */

void odom_on_tag_seen(int tag_id, float tx, float ty, float tz, float heading)
{
    nav_tag_t nav;
    if (!odom_find_nav_tag(tag_id, &nav)) return;

    /* Compute NED offset from drone to tag */
    float dn, de, hdist;
    camera_to_ned(tx, ty, tz, heading, &dn, &de, &hdist);

    /* Inverse transform: drone position = tag position − offset */
    float drone_map_x = nav.map_x - dn;
    float drone_map_y = nav.map_y - de;

    /* Use PX4's current altitude estimate (barometer is reliable for Z) */
    drone_state_t state = mavlink_get_state();

    /* Send to PX4 EKF2 */
    uint64_t usec = (uint64_t)esp_timer_get_time();
    mavlink_send_vision_position(drone_map_x, drone_map_y, state.z, usec);

    ESP_LOGI(TAG, "Odom correction: tag %d @ map(%.2f,%.2f) "
                  "offset(%.2f,%.2f) → drone map(%.2f,%.2f) "
                  "[px4 was (%.2f,%.2f)]",
             tag_id, nav.map_x, nav.map_y,
             dn, de, drone_map_x, drone_map_y,
             state.x, state.y);
}
