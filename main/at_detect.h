#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Run AprilTag detection on Core 1 at low priority to avoid starving control loops. */
#define AT_TASK_CORE        1
#define AT_TASK_PRIORITY    1
#define AT_TASK_STACK       12288

void at_detect_init(void);
void at_detect_task(void* pvParams);
bool at_detect_land_requested(void);
void at_detect_clear_land_request(void);
int at_detect_last_id(void);

/* Pose of the detected tag in camera frame (metres).
 * Camera convention: X = right, Y = down, Z = forward (into scene).
 * Use at_detect_get_pose() to get a thread-safe snapshot. */
typedef struct {
    float tx, ty, tz;   /* translation from camera origin to tag centre */
    bool  valid;        /* true only when a low-error estimate exists    */
    int   tag_id;
} at_detect_pose_t;

/* Thread-safe snapshot of the latest pose estimate.
 * Returns {.valid = false} if no estimate has been computed yet. */
at_detect_pose_t at_detect_get_pose(void);


void camera_to_ned(float tx, float ty, float tz,
                           float heading,
                           float *out_dn,      /* NED north offset to tag (m) */
                           float *out_de,      /* NED east  offset to tag (m) */
                           float *out_hdist);   /* horizontal distance to tag  */
