/**
 * @file    is09_scan.c
 * @brief   IS09 scan_core inside this software (see is09_glue.h).
 *
 * IS09 subsystem 5 (Buddy 5) decides WHEN there is an obstacle and WHAT it
 * looks like: the forward watch (two readings in a row at or inside
 * APP_CAL_D_STOP_CM), the coarse scan (-60..+60 deg every 15 deg), the
 * fine scan (5 deg steps around what the coarse scan found) and the profile
 * (edges, width, closest point, recommended side). The base obstacle task
 * moves the servo, pings, and drives the bypass.
 *
 * Frames: IS09 angle index 0..36 runs from -90 deg (RIGHT) to +90 deg
 * (LEFT) in 5 deg steps, 18 = ahead, lateral y + = LEFT. The base code uses robot
 * degrees + = RIGHT and lateral x + = RIGHT, so deg = (18 - idx) x 5 and
 * x = -y.
 */
#include "is09_glue.h"
#include "is09/shared/app_cal.h"
#include "is09/subsystem5/scan_core.h"

static sc_watch_t   g_watch;
static sc_samples_t g_samples;

bool is09_us_enabled(void)
{
    return (APP_CAL_ECHO_SHIFTER_VERIFIED != 0);
}

uint16_t is09_us_cm(float cm)
{
    if (cm < 0.0f)
    {
        return (uint16_t)IS09_RANGE_NONE;     /* no echo within the 30 ms window */
    }
    if (cm >= (float)APP_RANGE_CM_SENTINEL)
    {
        return (uint16_t)APP_RANGE_CM_SENTINEL;
    }
    return (uint16_t)cm;                      /* app_obstacle_us_to_cm(): whole cm */
}

float is09_scan_deg(uint32_t idx)
{
    return (float)((int32_t)SC_ANGLE_AHEAD - (int32_t)idx) * (float)SC_DEG_PER_IDX;
}

void is09_watch_reset(void)
{
    scan_core_watch_init(&g_watch);
}

bool is09_watch_step(uint16_t range_cm, bool *p_no_echo_fault)
{
    return scan_core_watch_step(&g_watch, range_cm, p_no_echo_fault);
}

uint32_t is09_watch_stop_cm(void)
{
    return APP_CAL_D_STOP_CM;
}

void is09_scan_begin(void)
{
    scan_core_samples_init(&g_samples);
}

uint32_t is09_scan_coarse_plan(uint8_t *p_idx, uint32_t max)
{
    return scan_core_coarse_plan(p_idx, max);
}

uint32_t is09_scan_fine_plan(uint8_t *p_idx, uint32_t max)
{
    return scan_core_fine_plan(&g_samples, p_idx, max);
}

void is09_scan_set(uint32_t idx, uint16_t range_cm)
{
    scan_core_samples_set(&g_samples, idx, range_cm);
}

bool is09_scan_sampled(uint32_t idx)
{
    return (idx < SC_ANGLE_COUNT) && ((g_samples.valid_mask & (1ULL << idx)) != 0ULL);
}

uint16_t is09_scan_get(uint32_t idx)
{
    return is09_scan_sampled(idx) ? g_samples.range_cm[idx] : (uint16_t)APP_RANGE_CM_SENTINEL;
}

void is09_scan_profile(is09_profile_t *p_out)
{
    app_obstacle_profile_t v;

    scan_core_build_profile(&g_samples, is09_scan_get(SC_ANGLE_AHEAD), &v);
    p_out->range_cm      = v.range_cm;
    p_out->closest_cm    = v.closest_cm;
    p_out->closest_deg   = is09_scan_deg(v.scan_angle_idx);
    p_out->width_cm      = v.width_cm;
    p_out->left_edge_cm  = -(float)v.y_max_cm;   /* y + = LEFT -> x + = RIGHT */
    p_out->right_edge_cm = -(float)v.y_min_cm;
    p_out->action        = v.action;
}

const char *is09_action_name(uint8_t action)
{
    static const char * const NAMES[5] = { "STOP", "CONTINUE", "TURN_LEFT", "TURN_RIGHT", "REVERSE" };

    return (action < 5u) ? NAMES[action] : "?";
}
