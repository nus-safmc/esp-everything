#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mavlink_task.h"

static const char *TAG = "main";

static void simple_test_task(void *arg)
{
    ESP_LOGI(TAG, "Simple test: wait telemetry...");

    // 1) Wait until MAVLink state is valid
    while (1) {
        drone_state_t st = mavlink_get_state();
        if (mavlink_position_valid() && st.last_hb_ms != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 2) Freeze at current pose, keep streaming
    mavlink_set_hold();
    ESP_LOGI(TAG, "Hold set, pre-streaming setpoints...");
    vTaskDelay(pdMS_TO_TICKS(20000));

    // 3) OFFBOARD then ARM
    mavlink_set_offboard_mode();
    vTaskDelay(pdMS_TO_TICKS(200));
    mavlink_arm(true);
    ESP_LOGI(TAG, "OFFBOARD + ARM sent");

    // Optional: wait for armed confirmation
    for (int i = 0; i < 40; i++) { // up to 4s
        drone_state_t st = mavlink_get_state();
        if (st.armed) {
            ESP_LOGI(TAG, "Armed confirmed");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 4) Take off to 0.5m (NED => z = -0.5)
    drone_state_t st = mavlink_get_state();
    float target_z = -0.5f;
    mavlink_set_position_ned(st.x, st.y, target_z, st.heading);
    ESP_LOGI(TAG, "Takeoff setpoint sent: x=%.2f y=%.2f z=%.2f", st.x, st.y, target_z);

    // Hold at altitude for a moment
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 5) Land
    mavlink_send_land_command();
    ESP_LOGI(TAG, "Land command sent");

    // Optional: wait until disarmed
    for (int i = 0; i < 200; i++) { // up to 20s
        drone_state_t s = mavlink_get_state();
        if (!s.armed && s.last_hb_ms != 0) {
            ESP_LOGI(TAG, "Disarmed, test complete");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP starting");

    mavlink_task_init();

    xTaskCreatePinnedToCore(
        mavlink_task,
        "mavlink",
        MAV_TASK_STACK,
        NULL,
        MAV_TASK_PRIORITY,
        NULL,
        MAV_TASK_CORE
    );

    ESP_LOGI(TAG, "mavlink_task spawned on Core %d, Priority %d",
             MAV_TASK_CORE, MAV_TASK_PRIORITY);

    // Run test logic on other core, lower priority than MAV task
    xTaskCreatePinnedToCore(
        simple_test_task,
        "simple_test",
        4096,
        NULL,
        3,
        NULL,
        1
    );
}