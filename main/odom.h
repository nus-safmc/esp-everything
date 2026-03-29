#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * Navigation-tag-based odometry correction
 *
 * Known AprilTags are placed at surveyed positions throughout the maze.
 * When the drone detects one, it computes its own map-frame position via
 * inverse transform and sends VISION_POSITION_ESTIMATE to PX4's EKF2.
 *
 * Required PX4 parameters (set via QGroundControl):
 *   EKF2_EV_CTRL  = 1    (enable external vision position fusion)
 *   EKF2_EVP_NOISE = 0.1  (tune: vision position noise, metres)
 *   EKF2_EV_DELAY  = 50   (tune: vision latency, ms)
 * --------------------------------------------------------------------------- */

#define ODOM_MAX_NAV_TAGS   16

/* A navigation tag at a known map-frame position. */
typedef struct {
    int8_t  id;         /* AprilTag ID                                       */
    float   map_x;      /* known position in map frame: NED north (metres)   */
    float   map_y;      /* known position in map frame: NED east  (metres)   */
} nav_tag_t;

/* Call once before spawning tasks. */
void odom_init(void);

/* Set / replace the navigation tag table (from laptop command).  Thread-safe.
 * Entries with id < 0 are ignored. */
void odom_set_nav_tags(const nav_tag_t *tags, int count);

/* Look up a tag ID in the nav table.
 * Returns true and fills *out if found, false otherwise.  Thread-safe. */
bool odom_find_nav_tag(int tag_id, nav_tag_t *out);

/* Called when a navigation tag is detected with a valid pose estimate.
 *
 *   tag_id : AprilTag ID (must be in the nav table)
 *   tx,ty,tz : tag pose in *camera* frame (OpenCV: X=right, Y=down, Z=fwd)
 *   heading  : current drone heading (NED, rad, 0=North, CW+)
 *
 * Computes the drone's map-frame position and sends
 * VISION_POSITION_ESTIMATE to PX4.  Thread-safe. */
void odom_on_tag_seen(int tag_id, float tx, float ty, float tz, float heading);
