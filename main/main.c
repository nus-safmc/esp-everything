#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "wifi_task.h"
#include "odom.h"

#include <math.h>

static const char *TAG = "mission";

static bool mission_should_land_from_apriltag(const char *phase)
{
    if (!at_detect_land_requested()) return false;
    ESP_LOGW(TAG, "AprilTag land trigger in phase: %s (id=%d)", phase, at_detect_last_id());
    return true;
}

#define PRECISION_ARRIVE_M      0.20f   /* stop approach when this close (m)     */
#define PRECISION_LAND_TIMEOUT_S 20     /* give up and land anyway after this (s) */
#define POSE_LOST_HOLD_MS        500    /* hold position if tag drops out         */

/* ---------------------------------------------------------------------------
 * Test parameters — adjust before flight
 * --------------------------------------------------------------------------- */
#define CRUISE_ALT_M        0.5f    /* target altitude above takeoff (m)        */
#define GOAL_OFFSET_X_M     2.0f    /* goal North offset from takeoff (m)       */
#define GOAL_OFFSET_Y_M     1.6f    /* goal East  offset from takeoff (m)       */
#define TAKEOFF_TIMEOUT_S   10      /* max seconds to wait for altitude (s)     */
#define NAV_TIMEOUT_S       60      /* max seconds to wait for goal arrival (s) */
#define LAND_TIMEOUT_S      20      /* max seconds to wait for disarm (s)       */
#define ALT_TOLERANCE_M     0.15f   /* altitude band considered "at altitude"   */

/* ---------------------------------------------------------------------------
 * Mission task — runs once, handles full flight sequence
 *
 * Sequencing contract with nav_task:
 *   1. nav_task is idle (no goal) — mission_task owns the MAVLink setpoint
 *   2. nav_set_goal_ned() is called — nav_task takes over setpoints
 *   3. nav_cancel() restores ownership to mission_task for landing
 * --------------------------------------------------------------------------- */
static void mission_task(void *arg)
{
    bool  nav_goal_active = false;
    float target_z   = -(CRUISE_ALT_M);   /* moved up — used in precision landing too */
    float takeoff_x  = 0.0f;
    float takeoff_y  = 0.0f;

    /* ------------------------------------------------------------------ */
    /* Phase 1: Wait for valid telemetry                                   */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for telemetry...");
    while (1) {
        drone_state_t st = mavlink_get_state();
        if (mavlink_position_valid() && st.last_hb_ms != 0) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Telemetry valid");

    /* ------------------------------------------------------------------ */
    /* Phase 2: Pre-stream hold setpoint                                   */
    /* PX4 requires setpoints to already be streaming before it will       */
    /* accept an OFFBOARD mode switch.  Stream for 2 s.                    */
    /* ------------------------------------------------------------------ */
    mavlink_set_hold();
    ESP_LOGI(TAG, "Pre-streaming hold setpoint for 2 s...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* ------------------------------------------------------------------ */
    /* Phase 2b: Wait for CMD_START from laptop                           */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for CMD_START from laptop...");
    while (!wifi_start_requested()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    wifi_clear_start_request();
    ESP_LOGI(TAG, "CMD_START received — proceeding to arm");

    /* ------------------------------------------------------------------ */
    /* Phase 3a: Switch to OFFBOARD mode                                   */
    /* Retry every 500 ms until PX4 confirms via heartbeat custom_main_mode */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Requesting OFFBOARD mode...");
    while(1) {
        if (mission_should_land_from_apriltag("offboard_request")) goto precision_landing;
        mavlink_set_offboard_mode();
        vTaskDelay(pdMS_TO_TICKS(500));
        if (mavlink_get_state().custom_main_mode == PX4_MAIN_MODE_OFFBOARD) {
            ESP_LOGI(TAG, "OFFBOARD mode confirmed");
            break;
        }
        ESP_LOGW(TAG, "OFFBOARD not set yet — retrying");
    }

    /* ------------------------------------------------------------------ */
    /* Phase 3b: Arm                                                        */
    /* Retry every 500 ms until PX4 confirms armed via heartbeat           */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Arming...");
    while(1) {
        if (mission_should_land_from_apriltag("arming")) goto precision_landing;
        mavlink_arm(true);
        vTaskDelay(pdMS_TO_TICKS(500));
        if (mavlink_get_state().armed) {
            ESP_LOGI(TAG, "Armed confirmed");
            break;
        }
        ESP_LOGW(TAG, "Not armed yet — retrying");
    }

    /* ------------------------------------------------------------------ */
    /* Phase 4: Take off to cruise altitude                                */
    /* nav_task is still IDLE — mission_task owns the setpoint.            */
    /* ------------------------------------------------------------------ */
    drone_state_t st = mavlink_get_state();
    takeoff_x  = st.x;
    takeoff_y  = st.y;
    // float target_z   = -(CRUISE_ALT_M);    /* NED: negative = above ground */

    mavlink_set_position_ned(takeoff_x, takeoff_y, target_z, st.heading);
    ESP_LOGI(TAG, "Taking off to %.1f m AGL (NED z=%.2f)...", CRUISE_ALT_M, target_z);

    /* Wait until within altitude tolerance or timeout */
    for (int i = 0; i < TAKEOFF_TIMEOUT_S * 10; i++) {
        // if (mission_should_land_from_apriltag("takeoff")) goto landing_phase;
        float current_z = mavlink_get_state().z;
        if (fabsf(current_z - target_z) < ALT_TOLERANCE_M) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    {
        float reached_z = mavlink_get_state().z;
        ESP_LOGI(TAG, "Altitude reached: NED z=%.2f (target=%.2f)", reached_z, target_z);
    }

    /* Hold briefly at altitude before starting navigation */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* ------------------------------------------------------------------ */
    /* Phase 5: Laptop-driven exploration                                 */
    /* Goals arrive via CMD_GOTO from the laptop (handled in wifi_task).  */
    /* This loop just monitors for landing triggers.                       */
    /* ------------------------------------------------------------------ */
    nav_goal_active = true;   /* wifi_task owns nav goals from here */
    ESP_LOGI(TAG, "Exploration mode — waiting for laptop goals...");

    while (1) {
        if (mission_should_land_from_apriltag("exploration")) goto precision_landing;
        if (wifi_land_requested()) {
            wifi_clear_land_request();
            ESP_LOGI(TAG, "CMD_LAND received from laptop");
            nav_goal_active = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
/* ------------------------------------------------------------------ */
/* Phase 6a: Precision approach — fly to directly above the tag        */
/* ------------------------------------------------------------------ */
precision_landing:
    ESP_LOGI(TAG, "Precision landing: flying to tag (id=%d)",
             at_detect_last_id());

    /* Stop any active navigation — mission_task takes back the setpoint */
    if (nav_goal_active || nav_get_status().state != NAV_IDLE) {
        nav_cancel();
        nav_goal_active = false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    {
        uint32_t lost_since_ms = 0;
        bool     tag_ever_seen = false;

        for (int i = 0; i < PRECISION_LAND_TIMEOUT_S * 10; i++) {

            at_detect_pose_t pose  = at_detect_get_pose();
            drone_state_t    drone = mavlink_get_state();

            if (!pose.valid) {
                mavlink_set_hold();
                vTaskDelay(pdMS_TO_TICKS(100));

                if (tag_ever_seen) {
                    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                    if (lost_since_ms == 0) lost_since_ms = now_ms;
                    if ((now_ms - lost_since_ms) > POSE_LOST_HOLD_MS) {
                        ESP_LOGW(TAG, "Tag lost — committing to land anyway");
                        break;
                    }
                }
                continue;
            }

            tag_ever_seen  = true;
            lost_since_ms  = 0;

            float dn, de, hdist;
            camera_to_ned(pose.tx, pose.ty, pose.tz,
                          drone.heading, &dn, &de, &hdist);

            ESP_LOGI(TAG, "Precision: hdist=%.2f  dn=%.2f  de=%.2f  "
                          "[body: fwd=%.2f rgt=%.2f]",
                     hdist, dn, de,
                     cosf(drone.heading)*dn + sinf(drone.heading)*de,   /* body_x */
                    -sinf(drone.heading)*dn + cosf(drone.heading)*de);  /* body_y */

            if (hdist < PRECISION_ARRIVE_M) {
                ESP_LOGI(TAG, "Above tag (hdist=%.2f m) — committing to land", hdist);
                break;
            }

            /* Command: fly to where the tag is, hold cruise altitude */
            mavlink_set_position_ned(drone.x + dn,
                                     drone.y + de,
                                     target_z,
                                     drone.heading);

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    at_detect_clear_land_request();
    /* ------------------------------------------------------------------ */
    /* Phase 6: Land                                                        */
    /* nav_cancel() returns setpoint ownership to mission_task.            */
    /* ------------------------------------------------------------------ */
// landing_phase:
    if (at_detect_land_requested()) {
        ESP_LOGW(TAG, "Landing due to AprilTag detection (id=%d)", at_detect_last_id());
    }

    if (nav_goal_active || nav_get_status().state != NAV_IDLE) {
        nav_cancel();
    }
    vTaskDelay(pdMS_TO_TICKS(200));   /* let hold setpoint be picked up */

    mavlink_send_land_command();
    ESP_LOGI(TAG, "Land command sent");

    /* Wait until disarmed */
    for (int i = 0; i < LAND_TIMEOUT_S * 10; i++) {
        drone_state_t s = mavlink_get_state();
        if (!s.armed && s.last_hb_ms != 0) {
            ESP_LOGI(TAG, "Disarmed — mission complete");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------------------
 * app_main — init and spawn all tasks
 * --------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP starting");

    /* NVS must be init before WiFi */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Init all modules before spawning tasks */
    mavlink_task_init();
    tof_task_init();
    nav_task_init();
    at_detect_init();
    odom_init();
    wifi_task_init();   /* blocks until IP obtained */

    /* Core 0: hardware-facing tasks + WiFi telemetry */
    xTaskCreatePinnedToCore(
        mavlink_task, "mav", MAV_TASK_STACK,
        NULL, MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        tof_task, "tof", TOF_TASK_STACK,
        NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        wifi_task, "wifi", WIFI_TASK_STACK,
        NULL, WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE
    );

    /* Core 1: navigator (Pri 3) + mission (Pri 2)
     * nav_task preempts mission_task on Core 1 when it has work. */
    xTaskCreatePinnedToCore(
        nav_task, "nav", NAV_TASK_STACK,
        NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        at_detect_task, "apriltag", AT_TASK_STACK,
        NULL, AT_TASK_PRIORITY, NULL, AT_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        mission_task, "mission", 4096,
        NULL, 2, NULL, 1
    );

    ESP_LOGI(TAG, "All tasks spawned");
}
