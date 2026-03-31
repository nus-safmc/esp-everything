#include "tof_task.h"

#include "vl53l5cx_api.h"
#include "platform.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <float.h>

static const char *TAG = "tof";

// ---------------------------------------------------------------------------
// Sensor angle table (body frame, degrees, CW from forward)
// Computed in tof_task_init() from TOF_FRONT_SENSOR_IDX.
// ---------------------------------------------------------------------------
static float SENSOR_ANGLES[TOF_SENSOR_COUNT];

// ---------------------------------------------------------------------------
// Shared scan state — written by tof_task, read by mission_task
// ---------------------------------------------------------------------------
static tof_scan_t    s_scan;
static SemaphoreHandle_t s_scan_mutex;

// ---------------------------------------------------------------------------
// I2C / hardware handles (private to this file)
// ---------------------------------------------------------------------------
static i2c_master_bus_handle_t  s_bus_handle;
static i2c_master_dev_handle_t  s_tca_handle;

static VL53L5CX_Configuration   s_dev;           // single shared device, mux-switched
static uint8_t                  s_sensor_ok[TOF_SENSOR_COUNT]; // 1 = sensor ready

// ---------------------------------------------------------------------------
// TCA9548A mux helper
// ---------------------------------------------------------------------------
static esp_err_t tca_select(uint8_t port)
{
    uint8_t flag = (uint8_t)(1u << port);
    return i2c_master_transmit(s_tca_handle, &flag, 1, -1);
}

// ---------------------------------------------------------------------------
// Compute minimum range (metres) from a stored tof_frame_t.
//
// Status 5/9: valid measurement — use reported distance.
// Status 255: out of range (nothing there) — ignored.
// Any other status: unreliable — treat as 0.3 m (conservative).
// Returns INFINITY only if every pixel is status 255 or out-of-range.
// ---------------------------------------------------------------------------
static float frame_min_range_m(const tof_frame_t *f)
{
    float min_m = INFINITY;
    for (int row = 4; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            int      i      = row * 8 + col;
            uint8_t  status = f->target_status[i];
            uint16_t dist   = f->distance_mm[i];

            float m;
            if (status == 255) {
                continue;
            } else if (status == 5 || status == 9) {
                if (dist < TOF_MIN_VALID_MM || dist > TOF_MAX_VALID_MM) continue;
                m = (float)dist / 1000.0f;
            } else {
                m = 0.30f;
            }
            if (m < min_m) min_m = m;
        }
    }
    return min_m;
}

// ---------------------------------------------------------------------------
// Initialise one sensor via the mux
// Returns true on success.
// ---------------------------------------------------------------------------
static bool init_sensor(uint8_t idx)
{
    esp_err_t err = tca_select(idx);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[%d] TCA select failed: %d", idx, err);
        return false;
    }

    uint8_t alive = 0;
    if (vl53l5cx_is_alive(&s_dev, &alive) || !alive) {
        ESP_LOGW(TAG, "[%d] not detected", idx);
        return false;
    }

    if (vl53l5cx_init(&s_dev)) {
        ESP_LOGE(TAG, "[%d] init failed", idx);
        return false;
    }

    if (vl53l5cx_set_ranging_frequency_hz(&s_dev, TOF_RANGING_FREQ_HZ)) {
        ESP_LOGE(TAG, "[%d] set freq failed", idx);
        return false;
    }

    if (vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_8X8)) {
        ESP_LOGE(TAG, "[%d] set resolution failed", idx);
        return false;
    }

    if (vl53l5cx_start_ranging(&s_dev)) {
        ESP_LOGE(TAG, "[%d] start ranging failed", idx);
        return false;
    }

    ESP_LOGI(TAG, "[%d] OK — %.0f° @ %dHz 8x8",
             idx, SENSOR_ANGLES[idx], TOF_RANGING_FREQ_HZ);
    return true;
}

// ---------------------------------------------------------------------------
// Angle helpers
// ---------------------------------------------------------------------------

// Normalise angle to [0, 360)
static float norm360(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

// Shortest angular distance (unsigned) between two angles, result in [0, 180]
static float angle_dist(float a, float b)
{
    float d = fabsf(norm360(a) - norm360(b));
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

// True if sensor at 'sensor_center_deg' has any coverage overlap with
// the query arc [q_min_deg, q_max_deg].
//
// Logic: the sensor covers ±HALF_WIDTH around its center.
// The query covers from q_min to q_max (may cross 0°).
// Overlap exists if the closest point on the query arc to the sensor center
// is within HALF_WIDTH degrees.
static bool sensor_overlaps_sector(float sensor_center_deg,
                                   float q_min_deg, float q_max_deg)
{
    float sc = norm360(sensor_center_deg);
    float qn = norm360(q_min_deg);
    float qx = norm360(q_max_deg);

    // Check if the sensor center falls inside the query arc
    bool inside;
    if (qn <= qx) {
        inside = (sc >= qn && sc <= qx);
    } else {
        // arc crosses 0°
        inside = (sc >= qn || sc <= qx);
    }
    if (inside) return true;

    // Otherwise, check if either query endpoint is within HALF_WIDTH of sensor center
    return angle_dist(sc, qn) <= TOF_SENSOR_HALF_WIDTH_DEG ||
           angle_dist(sc, qx) <= TOF_SENSOR_HALF_WIDTH_DEG;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void tof_task_init(void)
{
    memset(&s_scan, 0, sizeof(s_scan));

    // Compute CCW sensor angles from front sensor index
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        SENSOR_ANGLES[i] = (float)(((TOF_FRONT_SENSOR_IDX - i) * 45 % 360 + 360) % 360);
    }

    s_scan_mutex = xSemaphoreCreateMutex();
    configASSERT(s_scan_mutex != NULL);
}

float tof_get_min_range_in_sector(float angle_min_deg, float angle_max_deg)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    tof_scan_t snap = s_scan;
    xSemaphoreGive(s_scan_mutex);

    float min_m = INFINITY;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!snap.sensor_ok[i])  continue;
        if (!snap.frame[i].valid) continue;
        if (!sensor_overlaps_sector(SENSOR_ANGLES[i], angle_min_deg, angle_max_deg)) continue;

        float m = frame_min_range_m(&snap.frame[i]);
        if (m < min_m) min_m = m;
    }
    return min_m;
}

tof_scan_t tof_get_scan(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    tof_scan_t copy = s_scan;
    xSemaphoreGive(s_scan_mutex);
    return copy;
}

bool tof_is_healthy(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    // Find the most recently updated frame
    uint32_t latest_ms = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (s_scan.frame[i].valid && s_scan.frame[i].timestamp_ms > latest_ms) {
            latest_ms = s_scan.frame[i].timestamp_ms;
        }
    }
    xSemaphoreGive(s_scan_mutex);

    if (latest_ms == 0) return false;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return (now_ms - latest_ms) < 500;
}

int tof_sensors_ok_count(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++)
        count += s_scan.sensor_ok[i];
    xSemaphoreGive(s_scan_mutex);
    return count;
}

tof_scan_collapsed_t tof_get_collapsed_scan(void)
{
    tof_scan_t snap = tof_get_scan();  /* thread-safe snapshot */

    tof_scan_collapsed_t out;
    for (int i = 0; i < TOF_SENSOR_COUNT * TOF_SENSOR_RESO; i++)
        out.ranges[i] = INFINITY;

    for (int s = 0; s < TOF_SENSOR_COUNT; s++) {
        if (!snap.sensor_ok[s] || !snap.frame[s].valid) continue;
        const tof_frame_t *f = &snap.frame[s];

        for (int col = 0; col < 8; col++) {
            /* Sensors right-side up: raw col 0 = RIGHT of FoV (from lens).
             * col 0 → +offset (CW), col 7 → −offset (CCW). */
            float angle_deg = SENSOR_ANGLES[s] + (3.5f - (float)col) * (45.0f / 8.0f);

            /* Rows 4-7 = physical upper half (sensors mounted upside-down) */
            for (int row = 4; row < 8; row++) {
                int      px     = row * 8 + col;
                uint8_t  status = f->target_status[px];
                uint16_t dist   = f->distance_mm[px];

                /* Status 255: out of range (nothing there) — leave INFINITY.
                 * Status 5/9: valid measurement — use reported distance.
                 * Anything else: unreliable — assume 0.3 m (conservative). */
                float dist_m;
                if (status == 255) {
                    continue;   /* open space, INFINITY is correct */
                } else if (status == 5 || status == 9) {
                    if (dist < TOF_MIN_VALID_MM || dist > TOF_MAX_VALID_MM) continue;
                    dist_m = (float)dist / 1000.0f;
                } else {
                    dist_m = 0.40f;  /* conservative fallback for error statuses */
                }

                float norm_angle = fmodf(angle_deg, 360.0f);
                if (norm_angle < 0.0f) norm_angle += 360.0f;
                int idx = (int)(norm_angle * (float)(TOF_SENSOR_COUNT * TOF_SENSOR_RESO) / 360.0f);
                if (idx >= TOF_SENSOR_COUNT * TOF_SENSOR_RESO) idx = TOF_SENSOR_COUNT * TOF_SENSOR_RESO - 1;

                if (dist_m < out.ranges[idx])
                    out.ranges[idx] = dist_m;
                // printf("(%d: %zu)", idx, dist);
            }
            // printf("\n");
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

void tof_task(void *arg)
{
    // ---- I2C master bus ----
    i2c_master_bus_config_t bus_cfg = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .i2c_port               = TOF_I2C_PORT,
        .scl_io_num             = TOF_SCL_PIN,
        .sda_io_num             = TOF_SDA_PIN,
        .glitch_ignore_cnt      = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus_handle));
    ESP_LOGI(TAG, "I2C master bus created");

    // ---- TCA9548A mux ----
    i2c_device_config_t tca_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TCA_I2C_ADDR,
        .scl_speed_hz    = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus_handle, &tca_cfg, &s_tca_handle));
    ESP_LOGI(TAG, "TCA9548A registered");

    // ---- VL53L5CX shared device handle ----
    i2c_device_config_t vl_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1,
        .scl_speed_hz    = VL53L5CX_MAX_CLK_SPEED,
    };
    s_dev.platform.bus_config = bus_cfg;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus_handle, &vl_cfg, &s_dev.platform.handle));

    // ---- Init all 8 sensors — retry until all are up ----
    // mission_task blocks arming until tof_sensors_ok_count() == TOF_SENSOR_COUNT,
    // so we must keep retrying here rather than giving up after one pass.
    for (;;) {
        int ok_count = 0;
        for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
            if (s_sensor_ok[i]) { ok_count++; continue; }   // already up

            s_sensor_ok[i] = init_sensor(i) ? 1 : 0;
            if (!s_sensor_ok[i]) {
                // Bus may be stuck — reset and give it a moment before next pass
                i2c_master_bus_reset(s_bus_handle);
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
            s_scan.sensor_ok[i] = s_sensor_ok[i];
            xSemaphoreGive(s_scan_mutex);

            if (s_sensor_ok[i]) ok_count++;
        }

        ESP_LOGI(TAG, "%d / %d sensors ready", ok_count, TOF_SENSOR_COUNT);
        if (ok_count == TOF_SENSOR_COUNT) break;

        ESP_LOGW(TAG, "Retrying failed sensors in 500 ms...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // ---- Round-robin polling loop ----
    // Pattern from mapper.c: select mux, check data ready, extract if ready.
    // We use vTaskDelayUntil to keep a steady per-sensor cadence.
    VL53L5CX_ResultsData results;
    TickType_t last_wake = xTaskGetTickCount();

    // I2C bus recovery: after TOF_BUS_FAIL_THRESHOLD consecutive failures
    // across any sensor, reset the bus and re-init all sensors.
    #define TOF_BUS_FAIL_THRESHOLD  10
    int consecutive_fails = 0;

    for (uint8_t sensor = 0; ; sensor = (sensor + 1) % TOF_SENSOR_COUNT) {

        // ---- I2C bus recovery ----
        if (consecutive_fails >= TOF_BUS_FAIL_THRESHOLD) {
            ESP_LOGE(TAG, "I2C bus hung (%d consecutive failures) — resetting",
                     consecutive_fails);

            // Invalidate all frames so nav_task holds position during recovery
            xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
            for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
                s_scan.frame[i].valid = 0;
                s_scan.sensor_ok[i]   = 0;
            }
            xSemaphoreGive(s_scan_mutex);

            // Reset the I2C bus (sends 9 SCL clocks to release a stuck SDA)
            esp_err_t err = i2c_master_bus_reset(s_bus_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "i2c_master_bus_reset failed: %d", err);
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            // Re-init all sensors
            for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
                s_sensor_ok[i] = init_sensor(i) ? 1 : 0;
                xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
                s_scan.sensor_ok[i] = s_sensor_ok[i];
                xSemaphoreGive(s_scan_mutex);
            }

            int ok_count = 0;
            for (int i = 0; i < TOF_SENSOR_COUNT; i++) ok_count += s_sensor_ok[i];
            ESP_LOGW(TAG, "Bus recovery complete — %d / %d sensors restored",
                     ok_count, TOF_SENSOR_COUNT);

            consecutive_fails = 0;
            last_wake = xTaskGetTickCount();
            continue;
        }

        // Periodically re-attempt init on any sensor that dropped out
        if (!s_sensor_ok[sensor]) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            static uint32_t last_reinit_ms[TOF_SENSOR_COUNT] = {0};
            if ((now_ms - last_reinit_ms[sensor]) >= 2000) {
                last_reinit_ms[sensor] = now_ms;
                ESP_LOGW(TAG, "[%d] sensor offline — attempting re-init", sensor);
                s_sensor_ok[sensor] = init_sensor(sensor) ? 1 : 0;
                xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
                s_scan.sensor_ok[sensor] = s_sensor_ok[sensor];
                xSemaphoreGive(s_scan_mutex);
                if (s_sensor_ok[sensor])
                    ESP_LOGI(TAG, "[%d] sensor recovered", sensor);
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
            continue;
        }

        // Select this sensor on the mux
        if (tca_select(sensor) != ESP_OK) {
            ESP_LOGW(TAG, "[%d] mux select failed", sensor);
            consecutive_fails++;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
            continue;
        }

        // Non-blocking data-ready check
        uint8_t is_ready = 0;
        uint8_t status = vl53l5cx_check_data_ready(&s_dev, &is_ready);
        if (status) {
            ESP_LOGW(TAG, "[%d] check_data_ready status=%d", sensor, status);
            consecutive_fails++;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
            continue;
        }

        // Successful I2C transaction — reset failure counter
        consecutive_fails = 0;

        if (is_ready) {
            status = vl53l5cx_get_ranging_data(&s_dev, &results);
            if (status) {
                ESP_LOGW(TAG, "[%d] get_ranging_data status=%d", sensor, status);
            } else {
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

                // Copy the full 64-pixel frame into the shared scan buffer.
                // distance_mm and target_status are the same array sizes as
                // TOF_PIXELS_PER_SENSOR (VL53L5CX_RESOLUTION_8X8 * NB_TARGET_PER_ZONE).
                xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
                memcpy(s_scan.frame[sensor].distance_mm,
                       results.distance_mm,
                       TOF_PIXELS_PER_SENSOR * sizeof(uint16_t));
                memcpy(s_scan.frame[sensor].target_status,
                       results.target_status,
                       TOF_PIXELS_PER_SENSOR * sizeof(uint8_t));
                s_scan.frame[sensor].timestamp_ms = now_ms;
                s_scan.frame[sensor].valid        = 1;
                xSemaphoreGive(s_scan_mutex);
            }
        }

        // Yield for 20ms — matches mapper.c cadence, allows other tasks to run.
        // At 60Hz ranging frequency, new data arrives every ~17ms, so we miss
        // at most one frame per poll, which is fine for obstacle avoidance.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}