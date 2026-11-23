/**
 * @file    is09_imu.c
 * @brief   IS09 imu_core inside this software (see is09_glue.h).
 *
 * Same as IS09 task_imu.c s4_imu_cycle(): map the three raw axes with the
 * app_cal.h axis and sign constants, run imu_core_step() once per 10 ms,
 * and hand back the published state and the events of the step. The
 * hardware read, the motion inputs and what the events trigger stay in the
 * base task (tsk_imu.c).
 *
 * Port additions (19 Nov), around the core, not inside it:
 *   - the forward acceleration measured by the wheel encoders can be taken
 *     off the forward axis before the core sees it (a_comp_mg), as the base
 *     imu_logic did (DR A8), so a start or a stop does not read as a slope;
 *   - the two pitch thresholds are variables (IS09_RUNTIME_THETA);
 *   - is09_imu_diag() reports the detector pitch and its extremes.
 */
#include "is09_glue.h"
#include "is09/subsystem4/imu_core.h"

#define HALF_DEG_PER_DEG  (2.0f)

/* the variables behind APP_CAL_THETA_ON/OFF_HALF_DEG (app_cal.h) */
volatile int32_t g_is09_theta_on_half_deg  = 10;
volatile int32_t g_is09_theta_off_half_deg = 4;

static imu_core_t       g_core;
static volatile int16_t g_dp_hi;
static volatile int16_t g_dp_lo;
static volatile bool    g_dp_reset = true;

void is09_imu_init(void)
{
    imu_core_init(&g_core);
    g_dp_reset = true;
}

void is09_imu_set_thresholds(float on_deg, float off_deg)
{
    int32_t on  = (int32_t)((on_deg * HALF_DEG_PER_DEG) + 0.5f);
    int32_t off = (int32_t)((off_deg * HALF_DEG_PER_DEG) + 0.5f);

    on  = (on < 1) ? 1 : ((on > 60) ? 60 : on);
    off = (off < 0) ? 0 : ((off >= on) ? (on - 1) : off);
    g_is09_theta_on_half_deg  = on;
    g_is09_theta_off_half_deg = off;
}

static int16_t clamp16(int32_t v)
{
    return (int16_t)((v > 32767) ? 32767 : ((v < -32768) ? -32768 : v));
}

void is09_imu_step(const is09_imu_in_t *p_in, is09_imu_out_t *p_out)
{
    imu_in_t  in;
    imu_out_t out;
    int16_t   dp;

    imu_core_map_axes(p_in->x_mg, p_in->y_mg, p_in->z_mg, &in);
    in.a_fwd_mg      = clamp16((int32_t)in.a_fwd_mg - (int32_t)p_in->a_comp_mg);
    in.sample_valid  = p_in->b_sample_valid;
    in.motion_fresh  = p_in->b_motion_fresh;
    in.moving        = p_in->b_moving;
    in.turning       = p_in->b_turning;
    in.speed_cm_s    = p_in->speed_cm_s;
    in.dist_delta_mm = p_in->dist_delta_mm;
    in.stop_latched  = p_in->b_stop_latched;

    imu_core_step(&g_core, &in, &out);

    /* pitch relative to the resting tilt imu_core records after
       APP_CAL_T_LEVEL_MS standing still, so the car reads 0 on the floor
       (the hump detector already uses this zero, d_pitch) */
    p_out->pitch_deg = 0.5f * (float)out.state.pitch_half_deg;
    if (in.sample_valid && g_core.rest_valid)
    {
        p_out->pitch_deg -= 0.5f * (float)g_core.rest_pitch_half_deg;
    }
    p_out->b_hump_active   = out.state.hump_active;
    p_out->height_mm       = out.state.height_mm;
    p_out->max_height_mm   = out.state.max_height_mm;
    p_out->motion_class    = out.state.motion_class;
    p_out->b_hump_start    = out.evt_hump_start;
    p_out->b_hump_end      = out.evt_hump_end;
    p_out->end_quality     = out.end_quality;
    p_out->end_height_mm   = out.end_height_mm;
    p_out->b_collision     = out.collision;
    p_out->b_class_changed = out.class_changed;

    /* detector pitch extremes for the diagnostics */
    dp = g_core.d_pitch_half_deg;
    if (g_dp_reset)
    {
        g_dp_reset = false;
        g_dp_hi = dp;
        g_dp_lo = dp;
    }
    else
    {
        g_dp_hi = (dp > g_dp_hi) ? dp : g_dp_hi;
        g_dp_lo = (dp < g_dp_lo) ? dp : g_dp_lo;
    }
}

void is09_imu_map_axes(int16_t x_mg, int16_t y_mg, int16_t z_mg,
                       int16_t *p_fwd_mg, int16_t *p_lat_mg, int16_t *p_up_mg)
{
    imu_in_t in;

    imu_core_map_axes(x_mg, y_mg, z_mg, &in);   /* APP_CAL_IMU_AXIS_* and signs */
    *p_fwd_mg = in.a_fwd_mg;
    *p_lat_mg = in.a_lat_mg;
    *p_up_mg  = in.a_up_mg;
}

void is09_imu_diag(is09_imu_diag_t *p_d)
{
    /* diagnostics only: a step in between may mix two cycles */
    p_d->dp_deg         = 0.5f * (float)g_core.d_pitch_half_deg;
    p_d->dp_hi_deg      = 0.5f * (float)g_dp_hi;
    p_d->dp_lo_deg      = 0.5f * (float)g_dp_lo;
    p_d->on_deg         = 0.5f * (float)g_is09_theta_on_half_deg;
    p_d->off_deg        = 0.5f * (float)g_is09_theta_off_half_deg;
    p_d->hump_state     = g_core.hump_state;
    p_d->last_quality   = g_core.last_quality;
    p_d->last_height_mm = g_core.last_height_mm;
    p_d->max_height_mm  = g_core.max_height_mm;
    p_d->episodes       = (uint8_t)(g_core.stats_episodes & 0xFFu);
    g_dp_reset = true;                       /* next step starts a new window */
}

const char *is09_imu_class_name(uint8_t motion_class)
{
    static const char * const NAMES[7] =
    {
        "STATIONARY", "ACCELERATING", "TURNING", "CLIMBING", "DESCENDING", "IMPACT", "CRUISING"
    };

    return (motion_class < 7u) ? NAMES[motion_class] : "?";
}

const char *is09_imu_quality_name(uint8_t quality)
{
    static const char * const NAMES[4] = { "NORMAL", "REJECTED", "DEGRADED", "INVALID" };

    return (quality < 4u) ? NAMES[quality] : "?";
}
