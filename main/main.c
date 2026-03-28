#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mavlink_task.h"
#include "tof_task.h"
#include "nav_task.h"

#include <math.h>

static const char *TAG = "mission";

/* ---------------------------------------------------------------------------
 * Test parameters — adjust before flight
 * --------------------------------------------------------------------------- */
#define CRUISE_ALT_M        0.5f    /* target altitude above takeoff (m)        */
#define GOAL_OFFSET_X_M     2.0f    /* goal North offset from takeoff (m)       */
#define GOAL_OFFSET_Y_M     0.0f    /* goal East  offset from takeoff (m)       */
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
    /* Phase 3a: Switch to OFFBOARD mode                                   */
    /* Retry every 500 ms until PX4 confirms via heartbeat custom_main_mode */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Requesting OFFBOARD mode...");
    while(1) {
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
    float takeoff_x  = st.x;
    float takeoff_y  = st.y;
    float target_z   = -(CRUISE_ALT_M);    /* NED: negative = above ground */

    mavlink_set_position_ned(takeoff_x, takeoff_y, target_z, st.heading);
    ESP_LOGI(TAG, "Taking off to %.1f m AGL (NED z=%.2f)...", CRUISE_ALT_M, target_z);

    /* Wait until within altitude tolerance or timeout */
    for (int i = 0; i < TAKEOFF_TIMEOUT_S * 10; i++) {
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
    /* Phase 5: Navigate to goal using VFH obstacle avoidance             */
    /* nav_task takes ownership of setpoints from here.                    */
    /* ------------------------------------------------------------------ */
    float goal_x = takeoff_x + GOAL_OFFSET_X_M;
    float goal_y = takeoff_y + GOAL_OFFSET_Y_M;
    nav_set_goal_ned(goal_x, goal_y, target_z);
    ESP_LOGI(TAG, "Goal: NED (%.2f, %.2f, %.2f)  —  navigating...", goal_x, goal_y, target_z);

    /* Poll navigation status until arrived, stuck, or timeout */
    for (int i = 0; i < NAV_TIMEOUT_S * 5; i++) {   /* 5 polls/s */
        nav_status_t ns = nav_get_status();

        static const char *state_names[] = {
            "IDLE", "ROTATING", "FLYING", "ARRIVED", "STUCK"
        };
        ESP_LOGI(TAG, "nav=%s  dist=%.2f m  free_bins=%d  steer=%.1f°",
                 state_names[ns.state],
                 ns.dist_to_goal,
                 ns.free_bins,
                 ns.vfh_steering_rad * 180.0f / 3.14159f);

        if (ns.state == NAV_ARRIVED) {
            ESP_LOGI(TAG, "Goal reached!");
            break;
        }
        if (ns.state == NAV_STUCK && ns.stuck_count > 3) {
            ESP_LOGW(TAG, "Navigator stuck repeatedly — aborting mission");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* ------------------------------------------------------------------ */
    /* Phase 6: Land                                                        */
    /* nav_cancel() returns setpoint ownership to mission_task.            */
    /* ------------------------------------------------------------------ */
    nav_cancel();
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

    /* Init all modules before spawning tasks */
    mavlink_task_init();
    tof_task_init();
    nav_task_init();

    /* Core 0: hardware-facing tasks */
    xTaskCreatePinnedToCore(
        mavlink_task, "mav", MAV_TASK_STACK,
        NULL, MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        tof_task, "tof", TOF_TASK_STACK,
        NULL, TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE
    );

    /* Core 1: navigator (Pri 3) + mission (Pri 2)
     * nav_task preempts mission_task on Core 1 when it has work. */
    xTaskCreatePinnedToCore(
        nav_task, "nav", NAV_TASK_STACK,
        NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        mission_task, "mission", 4096,
        NULL, 2, NULL, 1
    );

    ESP_LOGI(TAG, "All tasks spawned");
}
