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

struct at_detect_msg {
  double t[3];
  double R[9];
  int det_id;
  int hamming;
  float decision_margin;
};
