#include "wifi_task.h"
#include "mavlink_task.h"
#include "nav_task.h"
#include "at_detect.h"
#include "odom.h"
#include "tof_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT  BIT0

/* Cruise altitude for CMD_GOTO — must match CRUISE_ALT_M in main.c */
#define WIFI_CRUISE_ALT_M   0.5f

static volatile bool s_land_requested  = false;
static volatile bool s_start_requested = false;
static volatile bool s_wifi_connected  = false;

/* Peer drone positions (map frame), protected by s_peer_mutex */
static wifi_peer_list_t   s_peers = { .count = 0 };
static SemaphoreHandle_t  s_peer_mutex;

static EventGroupHandle_t s_wifi_events;

/* ---------------------------------------------------------------------------
 * WiFi event handler — auto-reconnect on disconnect
 * --------------------------------------------------------------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGW(TAG, "Disconnected — reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

/* ---------------------------------------------------------------------------
 * wifi_task_init — connect to AP, block until IP assigned
 * --------------------------------------------------------------------------- */
void wifi_task_init(void)
{
    s_peer_mutex  = xSemaphoreCreateMutex();
    configASSERT(s_peer_mutex != NULL);
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
    case CMD_START:
        ESP_LOGI(TAG, "CMD_START");
        s_start_requested = true;
        break;
    case CMD_GOTO:
        nav_set_goal_map(cmd->goal_x, cmd->goal_y, -(WIFI_CRUISE_ALT_M));
        ESP_LOGI(TAG, "CMD_GOTO map(%.2f,%.2f)", cmd->goal_x, cmd->goal_y);
        break;
    case CMD_LAND:
        ESP_LOGI(TAG, "CMD_LAND");
        s_land_requested = true;
        break;
    case CMD_HOLD:
        nav_cancel();
        ESP_LOGI(TAG, "CMD_HOLD");
        break;
    default:
        ESP_LOGW(TAG, "Unknown cmd type 0x%02x", cmd->cmd_type);
        break;
    }
}

/* Parse and apply a CMD_SET_NAV_TAGS packet */
static void handle_nav_tags(const uint8_t *buf, int len)
{
    /* Header: pkt_type(1) + cmd_type(1) + tag_count(1) + start_x(4) + start_y(4) = 11 */
    if (len < 11) return;
    uint8_t tag_count = buf[2];
    if (tag_count > WIFI_MAX_NAV_TAGS) tag_count = WIFI_MAX_NAV_TAGS;

    int expected = 11 + tag_count * (int)sizeof(wifi_nav_tag_entry_t);
    if (len < expected) {
        ESP_LOGW(TAG, "CMD_SET_NAV_TAGS truncated (%d < %d)", len, expected);
        return;
    }

    float start_map_x, start_map_y;
    memcpy(&start_map_x, buf + 3, sizeof(float));
    memcpy(&start_map_y, buf + 7, sizeof(float));
    odom_set_initial_offset(start_map_x, start_map_y);

    const wifi_nav_tag_entry_t *entries =
        (const wifi_nav_tag_entry_t *)(buf + 11);

    nav_tag_t tags[WIFI_MAX_NAV_TAGS];
    for (int i = 0; i < tag_count; i++) {
        tags[i].id = entries[i].id;
        tags[i].x  = entries[i].odom_x;
        tags[i].y  = entries[i].odom_y;
    }
    odom_set_nav_tags(tags, tag_count);
    ESP_LOGI(TAG, "CMD_SET_NAV_TAGS: %d tags, start map(%.2f,%.2f)",
             tag_count, start_map_x, start_map_y);
}

/* Parse and apply a CMD_SET_PEERS packet.
 * Wire format: pkt_type(1) + cmd_type(1) + count(1) + count * (float, float) */
static void handle_peers(const uint8_t *buf, int len)
{
    if (len < 3) return;
    uint8_t count = buf[2];
    if (count > WIFI_MAX_PEERS) count = WIFI_MAX_PEERS;

    int expected = 3 + count * (int)(2 * sizeof(float));
    if (len < expected) {
        ESP_LOGW(TAG, "CMD_SET_PEERS truncated (%d < %d)", len, expected);
        return;
    }

    wifi_peer_list_t list;
    list.count = count;
    const uint8_t *p = buf + 3;
    for (int i = 0; i < count; i++) {
        memcpy(&list.peers[i].map_x, p, sizeof(float)); p += sizeof(float);
        memcpy(&list.peers[i].map_y, p, sizeof(float)); p += sizeof(float);
    }

    list.update_ms = (uint32_t)(esp_timer_get_time() / 1000);

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    s_peers = list;
    xSemaphoreGive(s_peer_mutex);

    ESP_LOGD(TAG, "CMD_SET_PEERS: %d peers", count);
}

wifi_peer_list_t wifi_get_peers(void)
{
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    wifi_peer_list_t copy = s_peers;
    xSemaphoreGive(s_peer_mutex);

    /* Discard peer data older than 1 s — positions are too stale to trust */
    if (copy.count > 0) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - copy.update_ms) > 1000) {
            copy.count = 0;
        }
    }
    return copy;
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

    /* ToF debug send socket */
    int tof_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(tof_sock >= 0);
    struct sockaddr_in tof_dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(WIFI_TOF_DEBUG_PORT),
    };
    inet_aton(CONFIG_HOST_IPV4_ADDR, &tof_dest.sin_addr);
    connect(tof_sock, (struct sockaddr *)&tof_dest, sizeof(tof_dest));

    /* Command receive socket (non-blocking) */
    int rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    configASSERT(rx_sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(WIFI_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(rx_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Command socket bind failed (port %d)", WIFI_CMD_PORT);
    }
    int flags = fcntl(rx_sock, F_GETFL, 0);
    fcntl(rx_sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "Telemetry → %s:%d | Commands ← port %d",
             CONFIG_HOST_IPV4_ADDR, CONFIG_TELEM_PORT, WIFI_CMD_PORT);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* ---- Check for incoming commands (non-blocking) ---- */
        {
            uint8_t cmd_buf[256];
            int len = recvfrom(rx_sock, cmd_buf, sizeof(cmd_buf), 0, NULL, NULL);
            if (len >= 2 && cmd_buf[0] == WIFI_PKT_CMD) {
                if (cmd_buf[1] == CMD_SET_NAV_TAGS) {
                    handle_nav_tags(cmd_buf, len);
                } else if (cmd_buf[1] == CMD_SET_PEERS) {
                    handle_peers(cmd_buf, len);
                } else if (len >= (int)sizeof(wifi_cmd_pkt_t)) {
                    handle_command((const wifi_cmd_pkt_t *)cmd_buf);
                }
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
        /* Report position in map frame so laptop works in a consistent frame */
        odom_to_map(drone.x, drone.y, &pkt.ned_x, &pkt.ned_y);
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

        pkt.reloc_age_s = odom_reloc_age_s();

        /* ---- Send telemetry ---- */
        send(tx_sock, &pkt, sizeof(pkt), 0);

        /* ---- Send raw ToF debug frame (front sensor only) ---- */
        {
            tof_scan_t scan = tof_get_scan();
            wifi_tof_debug_pkt_t dbg = {};
            dbg.pkt_type    = WIFI_PKT_TOF_DEBUG;
            dbg.sensor_idx  = TOF_FRONT_SENSOR_IDX;
            dbg.sensor_ok   = scan.sensor_ok[TOF_FRONT_SENSOR_IDX];
            dbg.timestamp_ms = scan.frame[TOF_FRONT_SENSOR_IDX].timestamp_ms;
            memcpy(dbg.distance_mm,   scan.frame[TOF_FRONT_SENSOR_IDX].distance_mm,
                   sizeof(dbg.distance_mm));
            memcpy(dbg.target_status, scan.frame[TOF_FRONT_SENSOR_IDX].target_status,
                   sizeof(dbg.target_status));
            send(tof_sock, &dbg, sizeof(dbg), 0);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));   /* 10 Hz */
    }
}

bool wifi_land_requested(void)
{
    return s_land_requested;
}

void wifi_clear_land_request(void)
{
    s_land_requested = false;
}

bool wifi_start_requested(void)
{
    return s_start_requested;
}

void wifi_clear_start_request(void)
{
    s_start_requested = false;
}

bool wifi_is_connected(void)
{
    return s_wifi_connected;
}
