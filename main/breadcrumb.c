#include "breadcrumb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <math.h>
#include <string.h>

static crumb_t           s_crumbs[CRUMB_MAX];
static int               s_count;       /* number of crumbs recorded            */
static int               s_last_sent;   /* index of last sent crumb (−1 = none) */
static float             s_path_accum;  /* path length since last crumb (m)     */
static float             s_prev_x, s_prev_y;
static bool              s_has_prev;
static SemaphoreHandle_t s_mutex;

void crumb_init(void)
{
    memset(s_crumbs, 0, sizeof(s_crumbs));
    s_count      = 0;
    s_last_sent  = -1;
    s_path_accum = 0.0f;
    s_has_prev   = false;
    s_mutex      = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
}

void crumb_update(float x, float y)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_has_prev) {
        float dx = x - s_prev_x;
        float dy = y - s_prev_y;
        s_path_accum += sqrtf(dx * dx + dy * dy);
    }
    s_prev_x   = x;
    s_prev_y   = y;
    s_has_prev = true;

    if (s_path_accum >= CRUMB_DIST_M && s_count < CRUMB_MAX) {
        s_crumbs[s_count++] = (crumb_t){ x, y };
        s_path_accum = 0.0f;
    }

    xSemaphoreGive(s_mutex);
}

int crumb_count(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = s_count;
    xSemaphoreGive(s_mutex);
    return n;
}

int crumb_last_sent(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int idx = s_last_sent;
    xSemaphoreGive(s_mutex);
    return idx;
}

void crumb_mark_sent(int idx)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (idx > s_last_sent && idx < s_count)
        s_last_sent = idx;
    xSemaphoreGive(s_mutex);
}

int crumb_read(int start_idx, crumb_t *buf, int max)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int available = s_count - start_idx;
    if (available <= 0) { xSemaphoreGive(s_mutex); return 0; }
    int n = available < max ? available : max;
    memcpy(buf, &s_crumbs[start_idx], n * sizeof(crumb_t));
    xSemaphoreGive(s_mutex);
    return n;
}
