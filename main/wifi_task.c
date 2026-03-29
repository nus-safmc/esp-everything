#include "wifi_task.h"
#include "mavlink_task.h"
#include "nav_task.h"
#include "breadcrumb.h"
#include "at_detect.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include <math.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT  BIT0

static EventGroupHandle_t s_wifi_events;

/* ---------------------------------------------------------------------------
 * WiFi event handler — auto-reconnect on disconnect
 * --------------------------------------------------------------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected — reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

/* ---------------------------------------------------------------------------
 * wifi_task_init — connect to AP, block until IP assigned
 * --------------------------------------------------------------------------- */
void wifi_task_init(void)
{
    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid,     CONFIG_WIFI_SSID,     sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();

    ESP_LOGI(TAG, "Connecting to '%s'...", CONFIG_WIFI_SSID);
    xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ---------------------------------------------------------------------------
 * wifi_task — 10 Hz telemetry loop
 * --------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------
 * Process an incoming command packet from the laptop
 * --------------------------------------------------------------------------- */
static void handle_command(const wifi_cmd_pkt_t *cmd)
{
    /* Always update the fleet's known-tag list */
    at_detect_set_known_tags(cmd->found_tag_ids, WIFI_MAX_FOUND_TAGS);

    switch (cmd->cmd_type) {
    case CMD_GOTO:
        ESP_LOGI(TAG, "CMD_GOTO (%.2f, %.2f)", cmd->goal_x, cmd->goal_y);
        /* TODO: hook into nav_set_goal_ned when laptop goal control is wired */
        break;
    case CMD_LAND:
        ESP_LOGI(TAG, "CMD_LAND");
        break;
    case CMD_HOLD:
        ESP_LOGI(TAG, "CMD_HOLD");
        break;
    default:
        ESP_LOGW(TAG, "Unknown cmd type 0x%02x", cmd->cmd_type);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * wifi_task — 10 Hz telemetry loop + non-blocking command receive
 * --------------------------------------------------------------------------- */
void wifi_task(void *arg)
{
    /* Telemetry send socket */
    int tx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(tx_sock >= 0);

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(CONFIG_TELEM_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &dest.sin_addr);
    connect(tx_sock, (struct sockaddr *)&dest, sizeof(dest));

    /* Command receive socket (non-blocking) */
    int rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(rx_sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(WIFI_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(rx_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
    int flags = fcntl(rx_sock, F_GETFL, 0);
    fcntl(rx_sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "Telemetry → %s:%d | Commands ← port %d",
             CONFIG_HOST_IPV4_ADDR, CONFIG_TELEM_PORT, WIFI_CMD_PORT);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* ---- Check for incoming commands (non-blocking) ---- */
        {
            wifi_cmd_pkt_t cmd;
            int len = recvfrom(rx_sock, &cmd, sizeof(cmd), 0, NULL, NULL);
            if (len >= (int)sizeof(cmd) && cmd.pkt_type == WIFI_PKT_CMD) {
                handle_command(&cmd);
            }
        }

        /* ---- Snapshot all shared state ---- */
        drone_state_t    drone = mavlink_get_state();
        nav_status_t     ns    = nav_get_status();
        at_detect_pose_t pose  = at_detect_get_pose();

        /* ---- Build telemetry packet ---- */
        wifi_telem_pkt_t pkt = {};
        pkt.pkt_type    = WIFI_PKT_TELEM;
        pkt.drone_id    = CONFIG_DRONE_ID;
        pkt.ned_x       = drone.x;
        pkt.ned_y       = drone.y;
        pkt.heading_rad = drone.heading;
        pkt.nav_state   = (uint8_t)ns.state;
        pkt.is_stuck    = (ns.state == NAV_STUCK) ? 1 : 0;

        /* AprilTag — always send the latched tag ID (−1 if none found yet).
         * tag_dist_m comes from live pose when available. */
        pkt.tag_id = at_detect_my_tag_id();
        if (pose.valid) {
            pkt.tag_dist_m = sqrtf(pose.tx * pose.tx + pose.tz * pose.tz);
        } else {
            pkt.tag_dist_m = 0.0f;
        }

        /* VFH blocked state */
        for (int b = 0; b < VFH_BINS; b++)
            pkt.vfh_blocked[b] = ns.vfh_blocked[b] ? 1 : 0;

        /* Crumb batch — send any unsent crumbs */
        int total_crumbs = crumb_count();
        int last_sent    = crumb_last_sent();
        int next_to_send = last_sent + 1;

        pkt.crumb_count     = (uint16_t)total_crumbs;
        pkt.crumb_last_sent = (last_sent < 0) ? 0xFFFF : (uint16_t)last_sent;

        int n = crumb_read(next_to_send, pkt.crumb_batch, WIFI_MAX_CRUMBS_PKT);
        pkt.crumb_batch_start = (uint16_t)next_to_send;
        pkt.crumb_batch_count = (uint8_t)n;

        /* ---- Send ---- */
        send(tx_sock, &pkt, sizeof(pkt), 0);

        if (n > 0) {
            crumb_mark_sent(next_to_send + n - 1);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}
