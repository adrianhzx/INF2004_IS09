/** @file scan_core.c
 *  @brief Pure obstacle-scan algorithm. See scan_core.h.
 */

#include "scan_core.h"

#include "../shared/app_cal.h"

#define SC_Q15 32768
#define SC_PERMILLE 1000
#define SC_DEG_PER_SETTLE_STEP 30U
#define SC_QUARTER_STEPS 18U /* 0..90 deg in 5 deg steps */
#define SC_COARSE_FIRST 6U   /* -60 deg */
#define SC_COARSE_LAST 30U   /* +60 deg */
#define SC_COARSE_STEP 3U    /* 15 deg */
#define SC_Y_MIN (-64)
#define SC_Y_MAX 63
#define SC_HALF 2
#define SC_COUNT_SAT 0xFFU
#define SC_CONFIRM 2U
#define SC_ALL_ANGLES_MASK ((1ULL << SC_ANGLE_COUNT) - 1ULL)

/* sin(5 deg x k), k = 0..18, Q15 */
static int16_t const g_sin_q15[SC_QUARTER_STEPS + 1U] = {
    0,     2856,  5690,  8481,  11207, 13848, 16384, 18795, 21063, 23170,
    25102, 26842, 28378, 29697, 30792, 31651, 32270, 32643, 32767};

static int32_t
clamp_i32 (int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ---- servo -------------------------------------------------------------- */

uint32_t
scan_core_servo_us (uint32_t angle_idx)
{
    uint32_t const idx  = (angle_idx > APP_SCAN_ANGLE_IDX_MAX)
                              ? APP_SCAN_ANGLE_IDX_MAX
                              : angle_idx;
    uint32_t const span = APP_CAL_SERVO_MAX_US - APP_CAL_SERVO_MIN_US;
    uint32_t const off  = (span * idx) / APP_SCAN_ANGLE_IDX_MAX;
#if APP_CAL_SERVO_LEFT_IS_MAX_PULSE
    return APP_CAL_SERVO_MIN_US + off;
#else
    return APP_CAL_SERVO_MAX_US - off;
#endif
}

uint32_t
scan_core_settle_ms (uint32_t from_idx, uint32_t to_idx)
{
    uint32_t const steps =
        (from_idx > to_idx) ? (from_idx - to_idx) : (to_idx - from_idx);
    uint32_t const deg = steps * SC_DEG_PER_IDX;
    uint32_t const ms =
        (APP_CAL_T_SERVO_SETTLE_MS * deg) / SC_DEG_PER_SETTLE_STEP;
    return (ms < APP_CAL_T_SERVO_MIN_SETTLE_MS) ? APP_CAL_T_SERVO_MIN_SETTLE_MS
                                                : ms;
}

/* ---- geometry ----------------------------------------------------------- */

int32_t
scan_core_sin_q15 (uint32_t angle_idx)
{
    int32_t const rel =
        (int32_t) ((angle_idx > APP_SCAN_ANGLE_IDX_MAX) ? APP_SCAN_ANGLE_IDX_MAX
                                                        : angle_idx) -
        (int32_t) SC_ANGLE_AHEAD;
    uint32_t const k = (uint32_t) ((rel < 0) ? -rel : rel);
    int32_t const  s = g_sin_q15[k];
    return (rel < 0) ? -s : s;
}

int32_t
scan_core_cos_q15 (uint32_t angle_idx)
{
    int32_t const rel =
        (int32_t) ((angle_idx > APP_SCAN_ANGLE_IDX_MAX) ? APP_SCAN_ANGLE_IDX_MAX
                                                        : angle_idx) -
        (int32_t) SC_ANGLE_AHEAD;
    uint32_t const k = (uint32_t) ((rel < 0) ? -rel : rel);
    return g_sin_q15[SC_QUARTER_STEPS -
                     k]; /* cos x = sin(90 - x), never negative here */
}

int8_t
scan_core_lateral_cm (uint16_t range_cm, uint32_t angle_idx)
{
    int32_t const prod  = (int32_t) range_cm * scan_core_sin_q15(angle_idx);
    int32_t const round = (prod < 0)
                              ? -(SC_Q15 / SC_HALF)
                              : (SC_Q15 / SC_HALF); /* symmetric rounding */
    return (int8_t) clamp_i32((prod + round) / SC_Q15, SC_Y_MIN, SC_Y_MAX);
}

/* ---- forward watch --------------------------------------------------------
 */

void
scan_core_watch_init (sc_watch_t * p_w)
{
    p_w->inside_count  = 0U;
    p_w->reported      = false;
    p_w->no_echo_count = 0U;
}

bool
scan_core_watch_step (sc_watch_t * p_w, uint16_t range_cm,
                      bool * p_no_echo_fault)
{
    bool raise = false;

    if (APP_RANGE_CM_SENTINEL == range_cm)
    {
        if (p_w->no_echo_count < SC_COUNT_SAT)
        {
            p_w->no_echo_count++;
        }
        p_w->inside_count = 0U; /* an unknown reading never confirms */
    }
    else
    {
        p_w->no_echo_count = 0U;
        if (range_cm <= APP_CAL_D_STOP_CM)
        {
            if (p_w->inside_count < SC_COUNT_SAT)
            {
                p_w->inside_count++;
            }
            if ((p_w->inside_count >= SC_CONFIRM) && !p_w->reported)
            {
                p_w->reported = true;
                raise         = true;
            }
        }
        else
        {
            p_w->inside_count = 0U;
            if (range_cm > (APP_CAL_D_STOP_CM + APP_CAL_D_STOP_HYST_CM))
            {
                p_w->reported = false; /* re-arm once the way is clear again */
            }
        }
    }
    *p_no_echo_fault = (p_w->no_echo_count >= APP_CAL_NO_ECHO_COUNT);
    return raise;
}

/* ---- scans -------------------------------------------------------------- */

void
scan_core_samples_init (sc_samples_t * p_s)
{
    uint32_t i = 0U;
    for (i = 0U; i < SC_ANGLE_COUNT; i++)
    {
        p_s->range_cm[i] = (uint16_t) APP_RANGE_CM_SENTINEL;
    }
    p_s->valid_mask = 0ULL;
}

void
scan_core_samples_set (sc_samples_t * p_s, uint32_t angle_idx,
                       uint16_t range_cm)
{
    if (angle_idx <= APP_SCAN_ANGLE_IDX_MAX)
    {
        p_s->range_cm[angle_idx] = range_cm;
        p_s->valid_mask |= (1ULL << angle_idx);
    }
}

uint32_t
scan_core_coarse_plan (uint8_t * p_idx, uint32_t max)
{
    uint32_t n = 0U;
    uint32_t i = 0U;
    for (i = SC_COARSE_FIRST; (i <= SC_COARSE_LAST) && (n < max);
         i += SC_COARSE_STEP)
    {
        p_idx[n] = (uint8_t) i;
        n++;
    }
    return n;
}

static bool
sample_is_obstacle (sc_samples_t const * p_s, uint32_t i)
{
    return (0ULL != (p_s->valid_mask & (1ULL << i))) &&
           (p_s->range_cm[i] <= APP_CAL_R_PROFILE_CM);
}

static bool
obstacle_span (sc_samples_t const * p_s, uint32_t * p_lo, uint32_t * p_hi)
{
    uint32_t i     = 0U;
    bool     found = false;
    for (i = 0U; i < SC_ANGLE_COUNT; i++)
    {
        if (sample_is_obstacle(p_s, i))
        {
            if (!found)
            {
                *p_lo = i;
            }
            *p_hi = i;
            found = true;
        }
    }
    return found;
}

uint32_t
scan_core_fine_plan (sc_samples_t const * p_coarse, uint8_t * p_idx,
                     uint32_t max)
{
    uint32_t lo = 0U;
    uint32_t hi = 0U;
    uint32_t n  = 0U;
    uint32_t i  = 0U;

    if (!obstacle_span(p_coarse, &lo, &hi))
    {
        return 0U;
    }
    lo = (lo >= SC_COARSE_STEP) ? (lo - SC_COARSE_STEP) : 0U;
    hi = ((hi + SC_COARSE_STEP) > APP_SCAN_ANGLE_IDX_MAX)
             ? APP_SCAN_ANGLE_IDX_MAX
             : (hi + SC_COARSE_STEP);
    for (i = lo; (i <= hi) && (n < max); i++)
    {
        p_idx[n] = (uint8_t) i;
        n++;
    }
    return n;
}

static bool
side_free (sc_samples_t const * p_s, uint32_t from_idx, uint32_t to_idx)
{
    uint32_t i    = 0U;
    bool     seen = false;
    for (i = from_idx; i <= to_idx; i++)
    {
        if (0ULL != (p_s->valid_mask & (1ULL << i)))
        {
            seen = true;
            if (p_s->range_cm[i] < APP_CAL_R_SIDE_FREE_CM)
            {
                return false;
            }
        }
    }
    return seen; /* an unsampled side is not proven free */
}

static uint8_t
choose_action (sc_samples_t const * p_s, uint32_t lo, uint32_t hi,
               int32_t y_min, int32_t y_max, uint16_t forward_cm)
{
    int32_t const half_car   = (int32_t) (APP_CAL_W_CAR_CM / 2U);
    int32_t const left_off   = y_max + half_car + (int32_t) APP_CAL_M_CLEAR_CM;
    int32_t const right_off  = -y_min + half_car + (int32_t) APP_CAL_M_CLEAR_CM;
    bool const    left_valid = (left_off <= (int32_t) APP_CAL_Y_PATH_MAX_CM) &&
                            (hi < APP_SCAN_ANGLE_IDX_MAX) &&
                            side_free(p_s, hi + 1U, APP_SCAN_ANGLE_IDX_MAX);
    bool const right_valid = (right_off <= (int32_t) APP_CAL_Y_PATH_MAX_CM) &&
                             (lo > 0U) && side_free(p_s, 0U, lo - 1U);

    if ((APP_RANGE_CM_SENTINEL != forward_cm) &&
        (forward_cm < (APP_CAL_R_SWEPT_CM + APP_CAL_M_CLEAR_CM)))
    {
        return (uint8_t)
            APP_AVOID_REVERSE; /* pivot would sweep into the obstacle */
    }
    if (left_valid && right_valid)
    {
        return (left_off <= right_off) ? (uint8_t) APP_AVOID_TURN_LEFT
                                       : (uint8_t) APP_AVOID_TURN_RIGHT;
    }
    if (left_valid)
    {
        return (uint8_t) APP_AVOID_TURN_LEFT;
    }
    if (right_valid)
    {
        return (uint8_t) APP_AVOID_TURN_RIGHT;
    }
    return (uint8_t) APP_AVOID_STOP;
}

void
scan_core_build_profile (sc_samples_t const * p_s, uint16_t forward_cm,
                         app_obstacle_profile_t * p_out)
{
    uint32_t lo        = 0U;
    uint32_t hi        = 0U;
    uint32_t i         = 0U;
    int32_t  y_min     = SC_Y_MAX;
    int32_t  y_max     = SC_Y_MIN;
    uint16_t closest   = (uint16_t) APP_RANGE_CM_SENTINEL;
    uint32_t closest_i = SC_ANGLE_AHEAD;
    uint16_t r_lo      = 0U;
    uint16_t r_hi      = 0U;

    p_out->range_cm   = forward_cm;
    p_out->scan_state = (uint8_t) SC_STATE_IDLE;
    p_out->seq        = 0U; /* the task owns the sequence */

    for (i = 0U; i < SC_ANGLE_COUNT; i++)
    {
        if ((0ULL != (p_s->valid_mask & (1ULL << i))) &&
            (p_s->range_cm[i] < closest))
        {
            closest   = p_s->range_cm[i];
            closest_i = i;
        }
    }
    p_out->closest_cm     = closest;
    p_out->scan_angle_idx = (uint8_t) closest_i;

    if (!obstacle_span(p_s, &lo, &hi))
    {
        p_out->width_cm = 0U;
        p_out->y_min_cm = 0;
        p_out->y_max_cm = 0;
        p_out->action   = (uint8_t) APP_AVOID_CONTINUE;
        return;
    }
    for (i = lo; i <= hi; i++)
    {
        if (sample_is_obstacle(p_s, i))
        {
            int32_t const y = scan_core_lateral_cm(p_s->range_cm[i], i);
            if (y < y_min)
            {
                y_min = y;
                r_lo  = p_s->range_cm[i];
            }
            if (y > y_max)
            {
                y_max = y;
                r_hi  = p_s->range_cm[i];
            }
        }
    }
    /* beam correction (A10): each edge sample sees the obstacle up to half a
       beam width early; pull both edges in, never past each other */
    {
        int32_t const shrink_lo =
            ((int32_t) r_lo * (int32_t) APP_CAL_US_BEAM_HALF_SIN_PERMILLE) /
            SC_PERMILLE;
        int32_t const shrink_hi =
            ((int32_t) r_hi * (int32_t) APP_CAL_US_BEAM_HALF_SIN_PERMILLE) /
            SC_PERMILLE;
        if ((y_min + shrink_lo) <= (y_max - shrink_hi))
        {
            y_min += shrink_lo;
            y_max -= shrink_hi;
        }
        else
        {
            int32_t const mid = (y_min + y_max) / SC_HALF;
            y_min             = mid;
            y_max             = mid;
        }
    }
    p_out->y_min_cm = (int8_t) clamp_i32(y_min, SC_Y_MIN, SC_Y_MAX);
    p_out->y_max_cm = (int8_t) clamp_i32(y_max, SC_Y_MIN, SC_Y_MAX);
    p_out->width_cm =
        (uint8_t) clamp_i32(y_max - y_min, 0, (int32_t) APP_OBST_WIDTH_CM_MAX);
    p_out->action = choose_action(p_s, lo, hi, y_min, y_max, forward_cm);
}

/* ---- side ranging ---------------------------------------------------------
 */

bool
scan_core_side_step (uint8_t * p_count, uint16_t range_cm)
{
    if ((APP_RANGE_CM_SENTINEL != range_cm) &&
        (range_cm < APP_CAL_D_SIDE_UNSAFE_CM))
    {
        if (*p_count < SC_COUNT_SAT)
        {
            (*p_count)++;
        }
    }
    else
    {
        *p_count = 0U;
    }
    return *p_count >= SC_CONFIRM;
}
