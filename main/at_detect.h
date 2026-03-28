#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void at_detect_task(void* pvParams);

struct at_detect_msg {
  double t[3];
  double R[9];
  int det_id;
  int hamming;
  float decision_margin;
};
