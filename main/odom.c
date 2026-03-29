#include "odom.h"
#include "mavlink_task.h"   /* mavlink_get_state() */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "odom";

/* ---------------------------------------------------------------------------
 * Shared state
 * --------------------------------------------------------------------------- */
static nav_tag_t         s_nav_tags[ODOM_MAX_NAV_TAGS];
static int               s_nav_tag_count = 0;

/* map_T_odom: map_pos = odom_pos + s_offset_x/y
 * Initialised to the drone's known start position (= start_offset from
 * setup.yaml, sent by laptop).  Refined on every nav-tag sighting. */
static float             s_offset_x = 0.0f;
static float             s_offset_y = 0.0f;
/* start_offset kept separately so drift can be expressed relative to it */
static float             s_start_x  = 0.0f;
static float             s_start_y  = 0.0f;

static SemaphoreHandle_t s_mutex;

/* ---------------------------------------------------------------------------
 * Lifecycle
 * --------------------------------------------------------------------------- */

void odom_init(void)
{
    s_nav_tag_count = 0;
    s_offset_x = s_offset_y = 0.0f;
    s_start_x  = s_start_y  = 0.0f;
    memset(s_nav_tags, 0, sizeof(s_nav_tags));
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
}

/* ---------------------------------------------------------------------------
 * Initial offset
 * --------------------------------------------------------------------------- */

void odom_set_initial_offset(float start_map_x, float start_map_y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_start_x  = start_map_x;
    s_start_y  = start_map_y;
    s_offset_x = start_map_x;
    s_offset_y = start_map_y;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Initial map_T_odom set to (%.2f, %.2f)", start_map_x, start_map_y);
}

/* ---------------------------------------------------------------------------
 * Nav-tag table
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

    ESP_LOGI(TAG, "Nav-tag table updated (%d tags, odom frame)", s_nav_tag_count);
    for (int i = 0; i < s_nav_tag_count; i++) {
        ESP_LOGI(TAG, "  tag %d → odom (%.2f, %.2f)",
                 s_nav_tags[i].id, s_nav_tags[i].x, s_nav_tags[i].y);
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
 * Transform refinement on tag sighting
 *
 *   cam_tag_x/y = camera position in the tag coordinate frame (= −R^T · t),
 *   computed by at_detect from the full pose (R + t).
 *
 *   Tag frame:  X = east,  Y = north,  Z = up  (tag on floor, top = north)
 *   Odom/NED:   X = north, Y = east,   Z = down
 *
 *   inferred_odom_x = tag_odom_x + cam_tag_y   (tag Y → north)
 *   inferred_odom_y = tag_odom_y + cam_tag_x   (tag X → east)
 *   drift           = inferred_odom − PX4_odom
 *   map_T_odom      = start_offset + drift
 *
 * Heading-independent: the tag's known world orientation replaces the
 * IMU heading, eliminating ring ambiguity from magnetometer drift.
 * --------------------------------------------------------------------------- */

void odom_on_tag_seen(int tag_id, float cam_tag_x, float cam_tag_y)
{
    nav_tag_t nav;
    if (!odom_find_nav_tag(tag_id, &nav)) return;

    /* Tag→NED axis mapping: tag_Y → odom north, tag_X → odom east */
    float inferred_odom_x = nav.x + cam_tag_y;
    float inferred_odom_y = nav.y + cam_tag_x;

    drone_state_t state = mavlink_get_state();
    float drift_x = inferred_odom_x - state.x;
    float drift_y = inferred_odom_y - state.y;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float new_offset_x = s_start_x + drift_x;
    float new_offset_y = s_start_y + drift_y;
    s_offset_x = new_offset_x;
    s_offset_y = new_offset_y;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "map_T_odom refined: tag %d  "
                  "cam_in_tag(%.2f,%.2f)  inferred_odom(%.2f,%.2f)  "
                  "px4_odom(%.2f,%.2f)  drift(%.3f,%.3f)  offset(%.2f,%.2f)",
             tag_id,
             cam_tag_x, cam_tag_y,
             inferred_odom_x, inferred_odom_y,
             state.x, state.y,
             drift_x, drift_y,
             new_offset_x, new_offset_y);
}

/* ---------------------------------------------------------------------------
 * Frame conversion
 * --------------------------------------------------------------------------- */

void odom_to_map(float odom_x, float odom_y, float *map_x, float *map_y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float ox = s_offset_x;
    float oy = s_offset_y;
    xSemaphoreGive(s_mutex);
    *map_x = odom_x + ox;
    *map_y = odom_y + oy;
}

void map_to_odom(float map_x, float map_y, float *odom_x, float *odom_y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float ox = s_offset_x;
    float oy = s_offset_y;
    xSemaphoreGive(s_mutex);
    *odom_x = map_x - ox;
    *odom_y = map_y - oy;
}
