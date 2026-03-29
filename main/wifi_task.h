#pragma once

#include <stdint.h>
#include "breadcrumb.h"
#include "vfh.h"

/* ---------------------------------------------------------------------------
 * FreeRTOS placement
 * --------------------------------------------------------------------------- */
#define WIFI_TASK_CORE      0
#define WIFI_TASK_PRIORITY  2
#define WIFI_TASK_STACK     4096

/* Max crumbs to batch in a single telemetry packet.
 * At 0.4 m spacing and 0.5 m/s cruise, ~1.25 crumbs/s — 10 per packet is plenty. */
#define WIFI_MAX_CRUMBS_PKT  10

/* Packet type bytes */
#define WIFI_PKT_TELEM       0x01
#define WIFI_PKT_CMD         0x02

/* Command port (drone listens for laptop commands on this UDP port) */
#define WIFI_CMD_PORT        5006
#define WIFI_MAX_FOUND_TAGS  12

/* ---------------------------------------------------------------------------
 * Telemetry packet — sent at 10 Hz over UDP to laptop.
 * Fixed header followed by a variable-length crumb batch.
 * Packed so sizeof() gives the exact wire size.
 * --------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;                      /* WIFI_PKT_TELEM                   */
    uint8_t  drone_id;                      /* CONFIG_DRONE_ID                  */

    float    ned_x;                         /* NED north (m)                    */
    float    ned_y;                         /* NED east  (m)                    */
    float    heading_rad;                   /* NED CW positive (rad)            */

    uint8_t  nav_state;                     /* nav_state_t cast to uint8_t      */
    int8_t   tag_id;                        /* last AprilTag ID, −1 if none     */
    float    tag_dist_m;                    /* horizontal dist to tag (m)       */

    uint16_t crumb_count;                   /* total crumbs recorded            */
    uint16_t crumb_last_sent;               /* last sent index (0xFFFF = none)  */

    uint8_t  vfh_blocked[VFH_BINS];         /* 1 = blocked, 0 = free            */
    uint8_t  is_stuck;                      /* 1 = STUCK or RETREATING          */

    uint16_t crumb_batch_start;             /* index of first crumb in batch    */
    uint8_t  crumb_batch_count;             /* number of crumbs in this packet  */
    crumb_t  crumb_batch[WIFI_MAX_CRUMBS_PKT]; /* crumb data                   */
} wifi_telem_pkt_t;

/* ---------------------------------------------------------------------------
 * Command packet — received from laptop over UDP.
 * --------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;                      /* WIFI_PKT_CMD                     */
    uint8_t  cmd_type;                      /* CMD_GOTO / CMD_LAND / CMD_HOLD   */
    float    goal_x;                        /* NED north (m)                    */
    float    goal_y;                        /* NED east  (m)                    */
    int8_t   found_tag_ids[WIFI_MAX_FOUND_TAGS]; /* −1 = unused slot           */
} wifi_cmd_pkt_t;

#define CMD_GOTO          0x01
#define CMD_LAND          0x02
#define CMD_HOLD          0x03
#define CMD_SET_NAV_TAGS  0x04

/* ---------------------------------------------------------------------------
 * Navigation-tag position packet — received from laptop over UDP.
 * Tells the drone where known AprilTags are in the map frame so it can
 * correct its odometry when it detects them.
 * --------------------------------------------------------------------------- */
#define WIFI_MAX_NAV_TAGS   16

typedef struct __attribute__((packed)) {
    int8_t   id;            /* AprilTag ID                                  */
    float    map_x;         /* NED north in map frame (m)                   */
    float    map_y;         /* NED east  in map frame (m)                   */
} wifi_nav_tag_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;      /* WIFI_PKT_CMD                                */
    uint8_t  cmd_type;      /* CMD_SET_NAV_TAGS                            */
    uint8_t  tag_count;     /* number of valid entries (≤ WIFI_MAX_NAV_TAGS)*/
    wifi_nav_tag_entry_t tags[WIFI_MAX_NAV_TAGS];
} wifi_nav_tags_pkt_t;

/* ---------------------------------------------------------------------------
 * Lifecycle
 * --------------------------------------------------------------------------- */

/* Initialise WiFi station and block until IP is obtained.
 * Call once in app_main before spawning wifi_task. */
void wifi_task_init(void);

/* FreeRTOS task entry — pin to Core 0, Priority 2.
 *   xTaskCreatePinnedToCore(wifi_task, "wifi", WIFI_TASK_STACK,
 *                           NULL, WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE); */
void wifi_task(void *arg);
