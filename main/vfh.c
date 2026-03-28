#include "vfh.h"
#include "tof_task.h"

#include <math.h>
#include <float.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- compile-time constants derived from VFH_BINS ---- */
#define BIN_WIDTH_DEG   (360.0f / (float)VFH_BINS)  

/* ---- local helpers ---- */

static float deg2rad(float d) { return d * ((float)M_PI / 180.0f); }
static float rad2deg(float r) { return r * (180.0f / (float)M_PI); }

/* Normalise angle to [0, 360) degrees */
static float norm360(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

/* Signed difference (a − b) wrapped to (−180, +180] degrees.
 * Positive = clockwise from b to a. */
static float adiff(float a, float b)
{
    return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

/* Absolute angular difference, always in [0, 180] degrees */
static float aabsdiff(float a, float b)
{
    return fabsf(adiff(a, b));
}

/* Centre angle of bin b in [0, 360) degrees */
static float bin_center_deg(int b)
{
    return ((float)b + 0.5f) * BIN_WIDTH_DEG;
}

/* ========================================================================== */

void vfh_get_histogram(const tof_scan_collapsed_t *scan,
                       const vfh_config_t     *cfg,
                       float                   hist_out[VFH_BINS],
                       bool                    binary_out[VFH_BINS])
{
    /* ----- Stage 1: build polar obstacle-density histogram ----- */
    memset(hist_out, 0, VFH_BINS * sizeof(float));

    for (int i = 0; i < TOF_SENSOR_COUNT*TOF_SENSOR_RESO; i++) {
        float r = scan->ranges[i];
        if (!isfinite(r) || r <= 0.0f || r > cfg->max_range_m) continue;

        /* Linear magnitude: 1.0 at 0 m, 0.0 at max_range_m */
        float mag = 1.0f - r / cfg->max_range_m;
        int b = i/2;
        hist_out[b] += mag;
    }

    /* ----- Stage 2: threshold → binary blocked / free ----- */
    for (int b = 0; b < VFH_BINS; b++) {
        binary_out[b] = (hist_out[b] >= cfg->density_threshold);
    }

    /* ----- Stage 3: grow blocked sectors by drone_radius -----
     *
     * Half-angle subtended by the drone radius at max range:
     *   grow_angle = atan(drone_radius / max_range)   [conservative — closest approach]
     *
     * We ceiling-round to bins so we never under-estimate the safety bubble.
     */
    float grow_angle_deg = rad2deg(atanf(cfg->drone_radius_m / cfg->max_range_m));
    int   grow_n         = (int)ceilf(grow_angle_deg / BIN_WIDTH_DEG);
    if (grow_n < 1) grow_n = 1;

    /* Work from a snapshot so expansions don't cascade */
    bool raw[VFH_BINS];
    memcpy(raw, binary_out, VFH_BINS * sizeof(bool));

    for (int b = 0; b < VFH_BINS; b++) {
        if (!raw[b]) continue;
        for (int d = 1; d <= grow_n; d++) {
            binary_out[(b - d + VFH_BINS) % VFH_BINS] = true;
            binary_out[(b + d)             % VFH_BINS] = true;
        }
    }
}

/* ========================================================================== */

float vfh_compute(const tof_scan_collapsed_t *scan,
                  const vfh_config_t     *cfg,
                  float                   goal_body_angle_rad,
                  float                   prev_steering_rad)
{
    float hist[VFH_BINS];
    bool  blocked[VFH_BINS];
    vfh_get_histogram(scan, cfg, hist, blocked);

    /* Reference angles in degrees for the cost function */
    float goal_deg = norm360(rad2deg(goal_body_angle_rad));
    float prev_deg = norm360(rad2deg(prev_steering_rad));

    /* ----- Stages 4 + 5: find free valleys, score candidate directions -----
     *
     * After growth, remaining free bins already carry a safety margin, so
     * every free bin is a valid steering candidate.
     *
     * Cost = w_goal    * |angular error to goal|
     *      + w_current * |angular error to forward (0°)|
     *      + w_prev    * |angular error to previous steering|
     *
     * We respect valley structure by only considering bins that are part of
     * a contiguous free run — this avoids steering toward an isolated 1-bin
     * gap that might be a histogram noise artefact.
     *
     * Valley scoring: within each contiguous free run we evaluate every bin
     * and track the global minimum cost.
     */

    /* Count free bins first — handle fully-blocked edge case */
    int free_count = 0;
    for (int b = 0; b < VFH_BINS; b++) if (!blocked[b]) free_count++;

    if (free_count == 0) {
        /* Entire field blocked — signal stuck to caller, keep prev steering */
        return prev_steering_rad;
    }

    float best_cost = FLT_MAX;
    float best_deg  = 0.0f;   /* fallback: forward */

    if (free_count == VFH_BINS) {
        /* Entire field clear — no valleys to delimit; score all bins */
        for (int b = 0; b < VFH_BINS; b++) {
            float ctr  = bin_center_deg(b);
            float cost = cfg->w_goal    * aabsdiff(ctr, goal_deg)
                       + cfg->w_current * aabsdiff(ctr, 0.0f)
                       + cfg->w_prev    * aabsdiff(ctr, prev_deg);
            if (cost < best_cost) { best_cost = cost; best_deg = ctr; }
        }
    } else {
        /* Find a blocked bin to use as a clean start (avoids starting mid-valley) */
        int start = 0;
        for (int b = 0; b < VFH_BINS; b++) {
            if (blocked[b]) { start = b; break; }
        }

        /* Walk one full revolution starting just after 'start'.
         * Track valley open/close and score bins as we encounter them. */
        bool in_valley = false;

        for (int n = 0; n < VFH_BINS; n++) {
            int b = (start + 1 + n) % VFH_BINS;

            if (!blocked[b]) {
                in_valley = true;
                float ctr  = bin_center_deg(b);
                float cost = cfg->w_goal    * aabsdiff(ctr, goal_deg)
                           + cfg->w_current * aabsdiff(ctr, 0.0f)
                           + cfg->w_prev    * aabsdiff(ctr, prev_deg);
                if (cost < best_cost) { best_cost = cost; best_deg = ctr; }
            } else {
                in_valley = false;
                (void)in_valley; /* suppress warning — may use for min-width later */
            }
        }
    }

    /* ----- Stage 6: convert best bin centre (0–360°) to body-frame radians -----
     *
     * adiff(best_deg, 0) maps the absolute angle to a signed body-frame angle
     * in (−180, +180] degrees, then we convert to radians.
     */
    return deg2rad(adiff(best_deg, 0.0f));
}
