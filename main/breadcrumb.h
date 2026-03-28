#pragma once

#include <stdint.h>

#define CRUMB_MAX       1000
#define CRUMB_DIST_M    0.4f    /* minimum travel distance between crumbs (m) */

typedef struct {
    float x, y;   /* NED north, east (m) */
} crumb_t;

/* Call once before use. */
void crumb_init(void);

/* Call with current position each nav tick.
 * Appends a crumb if the drone has moved >= CRUMB_DIST_M since the last one. */
void crumb_update(float x, float y);

/* Number of crumbs recorded so far. */
int crumb_count(void);

/* Index of the last crumb that has been sent (−1 if none sent yet). */
int crumb_last_sent(void);

/* Mark all crumbs up to and including 'idx' as sent. */
void crumb_mark_sent(int idx);

/* Read up to 'max' crumbs starting at 'start_idx' into 'buf'.
 * Returns the number of crumbs copied. */
int crumb_read(int start_idx, crumb_t *buf, int max);
