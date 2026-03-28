#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <sys/param.h>
#include <string.h>

// Apriltag dependencies
#include "apriltag.h"
#include "common/image_types.h"
#include "common/zarray.h"
#include "common/image_u8.h"
#include "lwip/sockets.h"
#include "sensor.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"

// Apriltag Pose Estimation dependencies
#include "apriltag_pose.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "at_detect.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"

#define CAMERA_MODEL_XIAO_ESP32S3

#ifdef CAMERA_MODEL_XIAO_ESP32S3
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13
#endif

static const char *TAG = "apriltag_detect";

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    // .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    sensor_t * s = esp_camera_sensor_get();
    // Initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, -2); // lower the saturation
    } else {
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, -2); // lower the saturation
    }
    s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
    return ESP_OK;
}
#endif

#define LOOP_DELAY_MS 100

static volatile bool s_land_requested = false;
static volatile int  s_last_tag_id = -1;

void at_detect_init(void)
{
  s_land_requested = false;
  s_last_tag_id = -1;
}

bool at_detect_land_requested(void)
{
  return s_land_requested;
}

void at_detect_clear_land_request(void)
{
  s_land_requested = false;
}

int at_detect_last_id(void)
{
  return s_last_tag_id;
}

void print_img(image_u8_t* im) {
  for (int i = 0; i < im->height; i++) {
    for (int j = 0; j < im->width; j++)
      printf("%d ", im->buf[i*(im->stride) + j]);
    printf("\n");
  }
}

int avg_img(image_u8_t* im) {
  int sum = 0;
  int len = (im->width) * (im->height);
  for (int i = 0; i < im->height; i++) {
    for (int j = 0; j < im->width; j++)
      sum += (im->buf[i*(im->stride) + j]);
  }

  sum /= len;
  return sum;
}
// ESP32 Cam Parameters!
// Tag Size in meters?
#define TAG_SIZE 0.12
#define F_X 163.5047216
#define F_Y 153.22210511
#define C_X 154.00573087
#define C_Y 107.10222796


void at_detect_task(void* pvParams)
{
#if ESP_CAMERA_SUPPORTED
    if(ESP_OK != init_camera()) {
        return;
    }
    // Socket setup

    // int sock = socket(AF_INET, SOCK_DGRAM, 0);
    // if (sock < 0){
    //   ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    //   return;
    // }

    // int opt = 1;
    // setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    // const struct sockaddr_in target_addr = {
    //   .sin_addr.s_addr = inet_addr(CONFIG_HOST_IPV4_ADDR),
    //   .sin_family      = AF_INET,
    //   .sin_port        = htons(CONFIG_APRILTAG_SEND_PORT),
    // };

    // Create tag family object
    apriltag_family_t *tf = tag16h5_create();

    // Create AprilTag detector object
    apriltag_detector_t *td = apriltag_detector_create();

    // Add tag family to the detector
    apriltag_detector_add_family(td, tf);

    // Tag detector configs
    // quad_sigma is Gaussian blur's sigma
    // quad_decimate: small number = faster but cannot detect small tags
    //                big number = slower but can detect small tags (or tag far away)
    // With quad_sigma = 1.0 and quad_decimate = 4.0, ESP32-CAM can detect 16h5 tag
    // from the distance of about 1 meter (tested with tag on screen. not on paper)
    td->quad_sigma = 1.0;
    td->quad_decimate = 2.0;//6.0;//5.0;
    td->refine_edges = 1;
    td->decode_sharpening = 0.75;
    td->nthreads = 1;
    td->debug = 0;

    while (1)
    {
        camera_fb_t *pic = esp_camera_fb_get();
      if (pic == NULL) {
        ESP_LOGW(TAG, "Camera frame grab failed");
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        continue;
      }

        // ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes (h=%zu, w=%zu, len=%zu)", pic->len, pic->height, pic->width, pic->len);

        image_u8_t at_im = {
          .width  = pic->width,
          .height = pic->height,
          .stride = pic->width,
          .buf    = pic->buf
        };

        // Testing responsiveness of camera
        // print_img(&at_im);
        // ESP_LOGI(TAG, "avg_img=%d", avg_img(&at_im));

        zarray_t *at_detections = apriltag_detector_detect(td, &at_im);

        for (int i = 0; i < zarray_size(at_detections); i++) {
          apriltag_detection_t *det;
          zarray_get(at_detections, i, &det);

          if (det->hamming > 1 || det->decision_margin <= 40.0) continue;

          ESP_LOGI(TAG, "Apriltag found! ID=%d, DM=%f, hamming=%d", det->id, det->decision_margin, det->hamming);

          s_last_tag_id = det->id;
          if (!s_land_requested) {
            s_land_requested = true;
            ESP_LOGW(TAG, "Land request raised from AprilTag detection (id=%d)", det->id);
          }

          // First create an apriltag_detection_info_t struct using your known parameters.
          apriltag_detection_info_t info;
          info.det = det;
          info.tagsize = TAG_SIZE;
          info.fx = F_X;
          info.fy = F_Y;
          info.cx = C_X;
          info.cy = C_Y;

          // Then call estimate_tag_pose.
          // apriltag_pose_t pose;
          // double err = estimate_tag_pose(&info, &pose);
          // if (err > 0.5f) continue; 

          // for (int j = 0; j < 3; j++) 
          //   printf("%f ", MATD_EL(pose.t, j, 0));
          // printf("\n");

          // struct at_detect_msg out_msg = {
          //   .det_id          = det->id,
          //   .decision_margin = det->decision_margin,
          //   .hamming         = det->hamming,
          // };
          // memcpy(out_msg.t, pose.t, sizeof(double)*3);
          // memcpy(out_msg.R, pose.R, sizeof(double)*9);
          // // Sending message via UDP
          // ESP_LOGI(TAG, "Send struct size=%zu", sizeof(out_msg));
          // int udp_err = sendto(sock, (void*)&out_msg, sizeof(struct at_detect_msg), 0, (const struct sockaddr*) &target_addr, sizeof(target_addr));
          // if (udp_err < 0) {
          //   ESP_LOGE(TAG, "Error sending message over UDP, err=%d", udp_err);
          //   // break;
          // }
        }
        ESP_LOGI(TAG, "%d Apriltags Found!", zarray_size(at_detections));

        // cleanup
        apriltag_detections_destroy(at_detections);

        esp_camera_fb_return(pic);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif
}
