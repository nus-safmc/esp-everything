#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "vfh.h"

/* ---------------------------------------------------------------------------
 * FreeRTOS placement — Core 1, Priority 3 (below MAVLink/ToF on Core 0)
 * --------------------------------------------------------------------------- */
#define NAV_TASK_CORE       1
#define NAV_TASK_PRIORITY   4
#define NAV_TASK_STACK      8192

/* ---------------------------------------------------------------------------
 * Tuning constants — adjust to vehicle
 *
 *  NAV_YAW_TOL_RAD     Heading error below which the drone is considered
 *                      "aligned" and switches from rotating to flying forward.
 *                      At 0.10 rad (~6°) the camera is well within frame.
 *
 *  NAV_STUCK_MIN_FREE  Min free VFH bins before exiting STUCK on retry.
 *                      2 avoids retrying on a single-bin noise artefact.
 * --------------------------------------------------------------------------- */
#define NAV_CRUISE_SPEED_MS     0.35f    /* forward cruise speed (m/s)            */
#define NAV_YAW_TOL_RAD         0.1f
#define NAV_ARRIVE_RADIUS_M     0.25f   /* XY goal-reached radius (m)            */
#define NAV_STUCK_HOLD_MS       3500    /* hold duration before declaring stuck   */

/* ---------------------------------------------------------------------------
 * Navigator states
 * --------------------------------------------------------------------------- */
typedef enum {
    NAV_IDLE     = 0,   /* no goal — task is dormant, does not touch setpoints  */
    NAV_ROTATING,       /* spinning in place to face VFH-selected direction      */
    NAV_FLYING,         /* aligned — flying forward at cruise speed              */
    NAV_ARRIVED,        /* within arrival radius — goal reached, hold commanded  */
    NAV_STUCK,          /* VFH fully blocked — holding, awaiting laptop rescue   */
} nav_state_t;

/* ---------------------------------------------------------------------------
 * Status snapshot — returned by nav_get_status()
 * --------------------------------------------------------------------------- */
typedef struct {
    nav_state_t state;
    float       goal_x;              /* current goal map-frame north (m)         */
    float       goal_y;              /* current goal map-frame east  (m)         */
    float       goal_z;              /* current goal NED down  (m, negative AGL) */
    float       dist_to_goal;        /* horizontal distance remaining (m)        */
    float       heading_error_rad;   /* signed error toward goal, body frame     */
    float       vfh_steering_rad;    /* last VFH steering output, body frame     */
    int         free_bins;           /* free VFH bins last cycle                 */
    uint32_t    stuck_count;         /* lifetime stuck events                    */
    bool        vfh_blocked[VFH_BINS]; /* per-bin blocked state (grown + forbidden) */
} nav_status_t;

/* ---------------------------------------------------------------------------
 * Lifecycle
 * --------------------------------------------------------------------------- */

/* Call once before spawning the task. */
void nav_task_init(void);

/* FreeRTOS task entry — pin to Core 1, Priority 3.
 *   xTaskCreatePinnedToCore(nav_task, "nav", NAV_TASK_STACK,
 *                           NULL, NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE); */
void nav_task(void *arg);

/* ---------------------------------------------------------------------------
 * Control API  (thread-safe — safe to call from any task)
 * --------------------------------------------------------------------------- */

/* Set a new goal in MAP frame (NED metres).
 *   map_x, map_y: horizontal position in map frame.
 *   z: NED down (negative = above ground, e.g. -1.5 = 1.5 m AGL).
 * Converted to odom frame each nav tick so reloc corrections apply. */
void nav_set_goal_map(float map_x, float map_y, float z);

/* Cancel navigation — transitions to NAV_IDLE and commands position hold. */
void nav_cancel(void);

/* Thread-safe status snapshot. */
nav_status_t nav_get_status(void);
