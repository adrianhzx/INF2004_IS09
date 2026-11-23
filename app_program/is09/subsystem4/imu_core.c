/** @file imu_core.c
 *  @brief Pure IMU algorithm implementation. See imu_core.h.
 */

#include "imu_core.h"

#include "../shared/app_cal.h"
#include "../shared/app_data.h"

#define IC_CYCLE_MS 10U
#define IC_MS_TO_CYCLES(ms) (((ms) + IC_CYCLE_MS - 1U) / IC_CYCLE_MS)
#define IC_Q8 256
#define IC_Q4 16U
#define IC_Q15 32768
#define IC_Q15_SHIFT 15U
#define IC_SEQ_MASK 0x0FU
#define IC_CYC_SAT 0xFFFFU
#define IC_U8_SAT 0xFFU
#define IC_PERMILLE 1000
#define IC_HALF_DEG_90 180 /* 90 deg in 0.5 deg units */
#define IC_HALF_DEG_180 360
#define IC_PITCH_MIN (-127)
#define IC_PITCH_MAX 127
#define IC_ATAN_PI_4_Q15 25736   /* pi/4 */
#define IC_ATAN_K_Q15 8946       /* 0.273 in atan(r) ~ r(pi/4 + 0.273(1-|r|)) */
#define IC_RAD_TO_HALF_DEG 11459 /* x 1/100: 1 rad = 114.59 half-degrees */
#define IC_RAD_SCALE 100
#define IC_SPEED_LP_SHIFT 3U /* speed EMA alpha 1/8 */
#define IC_AXIS_X 0U
#define IC_AXIS_Y 1U
#define IC_AXIS_Z 2U
#define IC_ROUND_Q8 128
#define IC_ATAN_IN_MAX                                                         \
    32767 /* inputs are prescaled into this range: int32 maths only */
#define IC_DIST_STEP_MAX_MM                                                    \
    1000 /* |distance delta| per 10 ms clamp (1 m/s x 100) */
#define IC_LP_SCALE ((int32_t) (1UL << APP_CAL_IMU_LP_SHIFT))
#define IC_SPEED_LP_SCALE ((int32_t) (1UL << IC_SPEED_LP_SHIFT))
#define IC_HALF 2
#define IC_SQRT_TOP_BIT (1UL << 30U) /* top even power of two in 32 bits */
#define IC_SQRT_STEP 2U
#define IC_TLP_SCALE ((int32_t) (1UL << APP_CAL_IMU_TILT_LP_SHIFT))
#define IC_HALF_DEG_TO_RAD_Q15 286 /* pi / 360 in Q15 (0.02 % high) */
#define IC_SIN_K3 6                /* x^3 / 3! */
#define IC_SIN_K5 120              /* x^5 / 5! */
#define IC_Q15_TO_Q8_DIV 128       /* Q15 -> Q8 */

/* ---- maths ------------------------------------------------------------ */

uint32_t
imu_core_isqrt (uint32_t v)
{
    uint32_t r   = 0U;
    uint32_t bit = IC_SQRT_TOP_BIT;

    while (bit > v)
    {
        bit >>= IC_SQRT_STEP;
    }
    while (0U != bit)
    {
        if (v >= (r + bit))
        {
            v -= r + bit;
            r = (r >> 1U) + bit;
        }
        else
        {
            r >>= 1U;
        }
        bit >>= IC_SQRT_STEP;
    }
    return r;
}

/** atan(r) for 0 <= r <= 1 (Q15 in, Q15 radians out). */
static int32_t
atan_unit_q15 (int32_t r_q15)
{
    /* 0 <= r <= 1: every product stays below 2^31 */
    int32_t const k =
        IC_ATAN_PI_4_Q15 + ((IC_ATAN_K_Q15 * (IC_Q15 - r_q15)) / IC_Q15);
    return (r_q15 * k) / IC_Q15;
}

static int32_t
q15_rad_to_half_deg (int32_t a_q15)
{
    /* a_q15 <= pi/4 Q15 = 25736, x 11459 < 2^31 */
    return ((a_q15 * IC_RAD_TO_HALF_DEG) + (IC_Q15 * IC_RAD_SCALE / IC_HALF)) /
           (IC_Q15 * IC_RAD_SCALE);
}

int16_t
imu_core_atan2_half_deg (int32_t y, int32_t x)
{
    int32_t ax    = (x < 0) ? -x : x;
    int32_t ay    = (y < 0) ? -y : y;
    int32_t angle = 0; /* half degrees */
    int32_t r_q15 = 0;

    if ((0 == ax) && (0 == ay))
    {
        return 0;
    }
    while ((ax > IC_ATAN_IN_MAX) || (ay > IC_ATAN_IN_MAX))
    {
        ax /= IC_HALF; /* ratio unchanged; keeps ay * 2^15 inside int32 */
        ay /= IC_HALF;
    }
    if (ay <= ax)
    {
        r_q15 = (ay * IC_Q15) / ax;
        angle = q15_rad_to_half_deg(atan_unit_q15(r_q15));
    }
    else
    {
        r_q15 = (ax * IC_Q15) / ay;
        angle = IC_HALF_DEG_90 - q15_rad_to_half_deg(atan_unit_q15(r_q15));
    }
    if (x < 0)
    {
        angle = IC_HALF_DEG_180 - angle;
    }
    if (y < 0)
    {
        angle = -angle;
    }
    return (int16_t) angle;
}

static int32_t
clamp_i32 (int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

int32_t
imu_core_sin_half_deg_q15 (int32_t half_deg)
{
    int32_t const h  = clamp_i32(half_deg, IC_PITCH_MIN, IC_PITCH_MAX);
    int32_t const ah = (h < 0) ? -h : h;
    /* x <= 127 x 286 = 36322 (1.108 rad Q15); every product < 2^31 */
    int32_t const x  = ah * IC_HALF_DEG_TO_RAD_Q15;
    int32_t const x2 = (x * x) / IC_Q15;
    int32_t const x3 = (x2 * x) / IC_Q15;
    int32_t const x5 = (x3 * x2) / IC_Q15;
    int32_t const s  = x - (x3 / IC_SIN_K3) + (x5 / IC_SIN_K5);
    return (h < 0) ? -s : s;
}

static int16_t
pick_axis (uint32_t axis, int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    int16_t v = 0;
    switch (axis)
    {
        case IC_AXIS_X:
            v = x_mg;
            break;
        case IC_AXIS_Y:
            v = y_mg;
            break;
        case IC_AXIS_Z:
        default:
            v = z_mg;
            break;
    }
    return v;
}

void
imu_core_map_axes (int16_t x_mg, int16_t y_mg, int16_t z_mg, imu_in_t * p_in)
{
    uint32_t const lat_axis = (IC_AXIS_X + IC_AXIS_Y + IC_AXIS_Z) -
                              APP_CAL_IMU_AXIS_FWD - APP_CAL_IMU_AXIS_UP;
    int32_t const fwd =
        (int32_t) pick_axis(APP_CAL_IMU_AXIS_FWD, x_mg, y_mg, z_mg) *
        APP_CAL_IMU_FWD_SIGN;
    int32_t const up =
        (int32_t) pick_axis(APP_CAL_IMU_AXIS_UP, x_mg, y_mg, z_mg) *
        APP_CAL_IMU_UP_SIGN;

    p_in->a_fwd_mg = (int16_t) clamp_i32(fwd, INT16_MIN, INT16_MAX);
    p_in->a_up_mg  = (int16_t) clamp_i32(up, INT16_MIN, INT16_MAX);
    p_in->a_lat_mg = pick_axis(lat_axis, x_mg, y_mg, z_mg);
}

/* ---- core --------------------------------------------------------------- */

void
imu_core_init (imu_core_t * p_core)
{
    p_core->lp_fwd           = 0;
    p_core->lp_lat           = 0;
    p_core->lp_up            = 0;
    p_core->lp_seeded        = false;
    p_core->pitch_half_deg   = 0;
    p_core->hump_state       = (uint8_t) HUMP_FLAT;
    p_core->theta_on_cycles  = 0U;
    p_core->theta_neg_cycles = 0U;
    p_core->flat_cycles      = 0U;
    p_core->episode_cycles   = 0U;
    p_core->episode_dist_mm  = 0U;
    p_core->height_q8        = 0;
    p_core->peak_q8          = 0;
    p_core->seq              = 0U;
    p_core->last_quality =
        (uint8_t) APP_HUMP_Q_INVALID; /* failsafe until the first episode */
    p_core->last_height_mm      = 0U;
    p_core->max_height_mm       = 0U;
    p_core->motion_class        = (uint8_t) APP_MOTION_CLASS_STATIONARY;
    p_core->class_candidate     = (uint8_t) APP_MOTION_CLASS_STATIONARY;
    p_core->class_cycles        = 0U;
    p_core->impact_cycles       = 0U;
    p_core->coll_cycles         = 0U;
    p_core->speed_lp_q4         = 0U;
    p_core->stats_episodes      = 0U;
    p_core->stats_collisions    = 0U;
    p_core->slp_fwd             = 0;
    p_core->slp_lat             = 0;
    p_core->slp_up              = 0;
    p_core->tilt_pitch_half_deg = 0;
    p_core->tilt_roll_half_deg  = 0;
    p_core->rest_pitch_half_deg = 0;
    p_core->rest_roll_half_deg  = 0;
    p_core->rest_valid          = false;
    p_core->still_cycles        = 0U;
    p_core->d_pitch_half_deg    = 0;
    p_core->d_roll_half_deg     = 0;
    p_core->roll_on_cycles      = 0U;
    p_core->roll_flat_cycles    = 0U;
    p_core->ep_roll             = false;
}

static void
clear_out (imu_out_t * p_out)
{
    p_out->evt_hump_start = false;
    p_out->evt_hump_end   = false;
    p_out->end_seq        = 0U;
    p_out->end_quality    = (uint8_t) APP_HUMP_Q_INVALID;
    p_out->end_height_mm  = 0U;
    p_out->collision      = false;
    p_out->class_changed  = false;
    p_out->height_sat     = false;
}

static void count_sat16 (uint16_t * p_c);
static void count_sat8 (uint8_t * p_c);

static void
lp_step (int32_t * p_lp, int32_t sample_mg)
{
    int32_t const target = sample_mg * IC_LP_SCALE;
    *p_lp += (target - *p_lp) / IC_LP_SCALE; /* EMA, alpha = 1 / 2^shift */
}

static void
tlp_step (int32_t * p_lp, int32_t sample_mg)
{
    int32_t const target = sample_mg * IC_TLP_SCALE;
    *p_lp += (target - *p_lp) / IC_TLP_SCALE; /* slower EMA for humps */
}

static int8_t
half_deg_i8 (int32_t y, int32_t x)
{
    return (int8_t) clamp_i32(imu_core_atan2_half_deg(y, x), IC_PITCH_MIN,
                              IC_PITCH_MAX);
}

static void
filter_step (imu_core_t * p_core, imu_in_t const * p_in)
{
    if (!p_core->lp_seeded)
    {
        p_core->lp_fwd    = (int32_t) p_in->a_fwd_mg * IC_LP_SCALE;
        p_core->lp_lat    = (int32_t) p_in->a_lat_mg * IC_LP_SCALE;
        p_core->lp_up     = (int32_t) p_in->a_up_mg * IC_LP_SCALE;
        p_core->slp_fwd   = (int32_t) p_in->a_fwd_mg * IC_TLP_SCALE;
        p_core->slp_lat   = (int32_t) p_in->a_lat_mg * IC_TLP_SCALE;
        p_core->slp_up    = (int32_t) p_in->a_up_mg * IC_TLP_SCALE;
        p_core->lp_seeded = true;
    }
    else
    {
        lp_step(&p_core->lp_fwd, p_in->a_fwd_mg);
        lp_step(&p_core->lp_lat, p_in->a_lat_mg);
        lp_step(&p_core->lp_up, p_in->a_up_mg);
        tlp_step(&p_core->slp_fwd, p_in->a_fwd_mg);
        tlp_step(&p_core->slp_lat, p_in->a_lat_mg);
        tlp_step(&p_core->slp_up, p_in->a_up_mg);
    }
    p_core->pitch_half_deg      = half_deg_i8(p_core->lp_fwd, p_core->lp_up);
    p_core->tilt_pitch_half_deg = half_deg_i8(p_core->slp_fwd, p_core->slp_up);
    p_core->tilt_roll_half_deg  = half_deg_i8(p_core->slp_lat, p_core->slp_up);
}

/** Resting tilt: while the car has been still for T_LEVEL (and no episode is
 *  open), the current slow tilt becomes the zero. Then the changes the hump
 *  detector uses are formed. */
static void
rest_step (imu_core_t * p_core, imu_in_t const * p_in)
{
    int32_t rest_p = 0;
    int32_t rest_r = 0;

    if (!p_in->moving && p_in->sample_valid && p_core->lp_seeded &&
        ((uint8_t) HUMP_FLAT == p_core->hump_state))
    {
        count_sat16(&p_core->still_cycles);
        if (p_core->still_cycles >= IC_MS_TO_CYCLES(APP_CAL_T_LEVEL_MS))
        {
            p_core->rest_pitch_half_deg = p_core->tilt_pitch_half_deg;
            p_core->rest_roll_half_deg  = p_core->tilt_roll_half_deg;
            p_core->rest_valid          = true;
        }
    }
    else
    {
        p_core->still_cycles = 0U;
    }
    if (p_core->rest_valid)
    {
        rest_p = p_core->rest_pitch_half_deg;
        rest_r = p_core->rest_roll_half_deg;
    }
    p_core->d_pitch_half_deg =
        (int16_t) ((int32_t) p_core->tilt_pitch_half_deg - rest_p);
    p_core->d_roll_half_deg =
        (int16_t) ((int32_t) p_core->tilt_roll_half_deg - rest_r);
}

/** sin(pitch) in Q15 from the filtered vector: a_fwd / |(a_fwd, a_up)|. */
static int32_t
sin_pitch_q15 (imu_core_t const * p_core)
{
    /* |lp| <= 2^15 (2 g x 16 mg) so f*f + u*u < 2^31 */
    int32_t const f =
        clamp_i32(p_core->lp_fwd, -IC_ATAN_IN_MAX, IC_ATAN_IN_MAX);
    int32_t const u = clamp_i32(p_core->lp_up, -IC_ATAN_IN_MAX, IC_ATAN_IN_MAX);
    uint32_t const norm = imu_core_isqrt((uint32_t) ((f * f) + (u * u)));
    if (0U == norm)
    {
        return 0;
    }
    return (f * IC_Q15) / (int32_t) norm;
}

static uint8_t
q8_to_mm_u8 (int32_t q8)
{
    int32_t const mm = (q8 + IC_ROUND_Q8) / IC_Q8;
    return (uint8_t) clamp_i32(mm, 0, (int32_t) IC_U8_SAT);
}

static void
count_sat16 (uint16_t * p_c)
{
    if (*p_c < IC_CYC_SAT)
    {
        (*p_c)++;
    }
}

static void
count_sat8 (uint8_t * p_c)
{
    if (*p_c < IC_U8_SAT)
    {
        (*p_c)++;
    }
}

static void
end_episode (imu_core_t * p_core, imu_out_t * p_out, app_hump_quality_t quality)
{
    bool const keep_peak =
        (APP_HUMP_Q_NORMAL == quality) || (APP_HUMP_Q_DEGRADED == quality);
    uint8_t height = 0U;

    if (keep_peak)
    {
        height = q8_to_mm_u8(p_core->peak_q8);
        if (height > APP_HUMP_HEIGHT_MAX)
        {
            height            = (uint8_t) APP_HUMP_HEIGHT_MAX;
            p_out->height_sat = true;
            quality = APP_HUMP_Q_DEGRADED; /* clipped number is never reported
                                              as fact */
        }
        if (((APP_HUMP_Q_NORMAL == quality) ||
             (0 != APP_CAL_HUMP_MAX_ACCEPT_DEGRADED)) &&
            (height > p_core->max_height_mm))
        {
            p_core->max_height_mm = height; /* FR11 */
        }
    }
    p_core->last_quality   = (uint8_t) quality;
    p_core->last_height_mm = height;
    p_core->hump_state     = (uint8_t) HUMP_FLAT;
    p_out->evt_hump_end    = true;
    p_out->end_seq         = p_core->seq;
    p_out->end_quality     = (uint8_t) quality;
    p_out->end_height_mm   = height;
}

static void
start_episode (imu_core_t * p_core, imu_out_t * p_out, bool by_roll)
{
    p_core->ep_roll          = by_roll;
    p_core->seq              = (uint8_t) ((p_core->seq + 1U) & IC_SEQ_MASK);
    p_core->hump_state       = (uint8_t) HUMP_CLIMBING;
    p_core->episode_cycles   = 0U;
    p_core->episode_dist_mm  = 0U;
    p_core->height_q8        = 0;
    p_core->peak_q8          = 0;
    p_core->theta_neg_cycles = 0U;
    p_core->flat_cycles      = 0U;
    p_core->stats_episodes++;
    p_out->evt_hump_start = true;
}

/** The pitch the hump path uses: the slow zeroed tilt, or the original fast
 *  pitch when APP_CAL_HUMP_PITCH_SLOW is 0. */
static int32_t
hump_pitch (imu_core_t const * p_core)
{
    return (0 != APP_CAL_HUMP_PITCH_SLOW) ? (int32_t) p_core->d_pitch_half_deg
                                          : (int32_t) p_core->pitch_half_deg;
}

static void
roll_counters (imu_core_t * p_core)
{
    int32_t const roll  = p_core->d_roll_half_deg;
    int32_t const aroll = (roll < 0) ? -roll : roll;

    if (aroll > APP_CAL_ROLL_ON_HALF_DEG)
    {
        count_sat8(&p_core->roll_on_cycles);
    }
    else
    {
        p_core->roll_on_cycles = 0U;
    }
    if (aroll < APP_CAL_ROLL_OFF_HALF_DEG)
    {
        count_sat16(&p_core->roll_flat_cycles);
    }
    else
    {
        p_core->roll_flat_cycles = 0U;
    }
}

static void
pitch_counters (imu_core_t * p_core)
{
    int32_t const pitch = hump_pitch(p_core);
    int32_t const apit  = (pitch < 0) ? -pitch : pitch;

    if (pitch > APP_CAL_THETA_ON_HALF_DEG)
    {
        count_sat8(&p_core->theta_on_cycles);
    }
    else
    {
        p_core->theta_on_cycles = 0U;
    }
    if (pitch < -APP_CAL_THETA_ON_HALF_DEG)
    {
        count_sat8(&p_core->theta_neg_cycles);
    }
    else
    {
        p_core->theta_neg_cycles = 0U;
    }
    if (apit < APP_CAL_THETA_OFF_HALF_DEG)
    {
        count_sat16(&p_core->flat_cycles);
    }
    else
    {
        p_core->flat_cycles = 0U;
    }
}

static void
integrate_height (imu_core_t * p_core, imu_in_t const * p_in)
{
    int32_t const d_mm = clamp_i32(p_in->dist_delta_mm, -IC_DIST_STEP_MAX_MM,
                                   IC_DIST_STEP_MAX_MM);
    int32_t const sin_q15 =
        (0 != APP_CAL_HUMP_PITCH_SLOW)
            ? imu_core_sin_half_deg_q15(p_core->d_pitch_half_deg)
            : sin_pitch_q15(p_core);
    /* d x sin x 256: (d x sin_q15) <= 2^25, / 128 gives Q8, then K_CAL */
    int32_t const rise_q8 = (((d_mm * sin_q15) / (IC_Q15 / IC_Q8)) *
                             (int32_t) APP_CAL_K_CAL_PERMILLE) /
                            IC_PERMILLE;
    uint32_t const ad_mm = (uint32_t) ((d_mm < 0) ? -d_mm : d_mm);

    p_core->height_q8 += rise_q8;
    if (p_core->height_q8 > p_core->peak_q8)
    {
        p_core->peak_q8 = p_core->height_q8;
    }
    count_sat16(&p_core->episode_cycles);
    p_core->episode_dist_mm =
        (uint16_t) (((uint32_t) p_core->episode_dist_mm + ad_mm > IC_CYC_SAT)
                        ? IC_CYC_SAT
                        : (p_core->episode_dist_mm + ad_mm));
}

/** Roll joins an open episode once it passes ROLL_ON for the confirm time;
 *  while roll is part of the episode the peak also follows the wheel rise
 *  TRACK_WIDTH x sin(|roll change|). */
static void
roll_track (imu_core_t * p_core, uint8_t confirm)
{
    int32_t const roll    = p_core->d_roll_half_deg;
    int32_t const aroll   = (roll < 0) ? -roll : roll;
    int32_t       rise_q8 = 0;

    if ((0 != APP_CAL_HUMP_ROLL_ENABLE) && (p_core->roll_on_cycles >= confirm))
    {
        p_core->ep_roll = true;
    }
    if (p_core->ep_roll)
    {
        /* 140 mm x 32768 < 2^31 */
        rise_q8 = ((int32_t) APP_CAL_TRACK_WIDTH_MM *
                   imu_core_sin_half_deg_q15(aroll)) /
                  IC_Q15_TO_Q8_DIV;
        if (rise_q8 > p_core->peak_q8)
        {
            p_core->peak_q8 = rise_q8;
        }
    }
}

/** Roll is past ROLL_ON but not yet confirmed: a pitch episode is not
 *  aborted then, so the roll can join it instead of opening a second one. */
static bool
roll_rising (imu_core_t const * p_core)
{
    return (0 != APP_CAL_HUMP_ROLL_ENABLE) && (p_core->roll_on_cycles > 0U);
}

/** An episode with roll in it closes NORMAL once pitch and roll are both
 *  level for the confirm time (no climb / descend sequence needed). */
static bool
roll_episode_level (imu_core_t const * p_core, uint8_t confirm)
{
    return p_core->ep_roll && (p_core->flat_cycles >= confirm) &&
           (p_core->roll_flat_cycles >= confirm);
}

static bool
episode_bounds_hit (imu_core_t const * p_core)
{
    return (p_core->episode_dist_mm > APP_CAL_D_HUMP_MAX_MM) ||
           (p_core->episode_cycles > IC_MS_TO_CYCLES(APP_CAL_T_HUMP_MAX_MS));
}

/** Episode interrupted (stop latch, stale motion, sensor fault)? */
static bool
episode_interrupted (imu_in_t const * p_in)
{
    return p_in->stop_latched || !p_in->motion_fresh || !p_in->sample_valid;
}

static void
climbing_step (imu_core_t * p_core, imu_in_t const * p_in, imu_out_t * p_out,
               uint8_t confirm)
{
    if (episode_interrupted(p_in))
    {
        end_episode(p_core, p_out,
                    APP_HUMP_Q_INVALID); /* interrupt guard: highest priority */
        return;
    }
    integrate_height(p_core, p_in);
    roll_track(p_core, confirm);
    if (episode_bounds_hit(p_core))
    {
        end_episode(p_core, p_out,
                    APP_HUMP_Q_REJECTED); /* no descent within the bounds */
    }
    else if (roll_episode_level(p_core, confirm))
    {
        end_episode(p_core, p_out, APP_HUMP_Q_NORMAL);
    }
    else if (p_core->ep_roll)
    {
        /* one-sided hump: wait for pitch and roll to level */
    }
    else if (p_core->theta_neg_cycles >= confirm)
    {
        p_core->hump_state  = (uint8_t) HUMP_DESCENDING;
        p_core->flat_cycles = 0U;
    }
    else if ((p_core->flat_cycles >= IC_MS_TO_CYCLES(APP_CAL_T_ABORT_MS)) &&
             !roll_rising(p_core))
    {
        end_episode(p_core, p_out,
                    APP_HUMP_Q_REJECTED); /* false trigger: back to flat
                                             before descent */
    }
    else
    {
        /* climbing */
    }
}

static void
descending_step (imu_core_t * p_core, imu_in_t const * p_in, imu_out_t * p_out,
                 uint8_t confirm)
{
    if (episode_interrupted(p_in))
    {
        end_episode(p_core, p_out, APP_HUMP_Q_INVALID);
        return;
    }
    integrate_height(p_core, p_in);
    roll_track(p_core, confirm);
    if (episode_bounds_hit(p_core))
    {
        end_episode(p_core, p_out,
                    APP_HUMP_Q_DEGRADED); /* never settled: forced closure
                                             keeps the peak */
    }
    else if ((p_core->flat_cycles >= confirm) &&
             (!p_core->ep_roll || (p_core->roll_flat_cycles >= confirm)))
    {
        end_episode(p_core, p_out, APP_HUMP_Q_NORMAL);
    }
    else
    {
        /* descending */
    }
}

static void
hump_step (imu_core_t * p_core, imu_in_t const * p_in, imu_out_t * p_out)
{
    uint8_t const confirm =
        (uint8_t) IC_MS_TO_CYCLES(APP_CAL_T_THETA_CONFIRM_MS);
    bool const can_start = p_in->motion_fresh && p_in->moving &&
                           !p_in->stop_latched && p_in->sample_valid;
    /* the slow paths are not started inside a commanded turn */
    bool const slow_ok     = can_start && !p_in->turning;
    bool       pitch_start = false;
    bool       roll_start  = false;

    pitch_counters(p_core);
    roll_counters(p_core);
    pitch_start = (p_core->theta_on_cycles >= confirm) &&
                  ((0 != APP_CAL_HUMP_PITCH_SLOW) ? slow_ok : can_start);
    roll_start = (0 != APP_CAL_HUMP_ROLL_ENABLE) &&
                 (p_core->roll_on_cycles >= confirm) && slow_ok;

    switch ((hump_state_t) p_core->hump_state)
    {
        case HUMP_FLAT:
            if (pitch_start || roll_start)
            {
                start_episode(p_core, p_out, roll_start);
                roll_track(p_core, confirm);
            }
            break;

        case HUMP_CLIMBING:
            climbing_step(p_core, p_in, p_out, confirm);
            break;

        case HUMP_DESCENDING:
            descending_step(p_core, p_in, p_out, confirm);
            break;

        default:
            p_core->hump_state = (uint8_t) HUMP_FLAT;
            break;
    }
}

/** Forward deviation from the low-pass baseline, before this sample enters
 *  the filter: a spike is never averaged into the pitch estimate. */
static bool
spike_sample (imu_core_t const * p_core, imu_in_t const * p_in)
{
    int32_t const  lp_mg = p_core->lp_fwd / IC_LP_SCALE;
    int32_t const  dev   = (int32_t) p_in->a_fwd_mg - lp_mg;
    uint32_t const adev  = (uint32_t) ((dev < 0) ? -dev : dev);
    return p_core->lp_seeded && p_in->sample_valid &&
           (adev >= APP_CAL_COLLISION_MG);
}

static void
collision_step (imu_core_t * p_core, imu_in_t const * p_in, bool spike,
                imu_out_t * p_out)
{
    if (spike)
    {
        count_sat8(&p_core->coll_cycles);
    }
    else
    {
        p_core->coll_cycles = 0U;
    }
    if ((0 != APP_CAL_COLLISION_ENABLE) && p_in->moving &&
        (p_core->coll_cycles >= APP_CAL_COLLISION_SAMPLES) &&
        !p_in->stop_latched)
    {
        p_out->collision    = true;
        p_core->coll_cycles = 0U;
        p_core->stats_collisions++;
        p_core->impact_cycles =
            (uint16_t) IC_MS_TO_CYCLES(APP_CAL_IMPACT_HOLD_MS);
    }
}

static uint8_t
classify (imu_core_t * p_core, imu_in_t const * p_in)
{
    uint32_t const v_q4  = (uint32_t) p_in->speed_cm_s * IC_Q4;
    int32_t const  dv_q4 = (int32_t) v_q4 - (int32_t) p_core->speed_lp_q4;
    uint32_t const adv   = (uint32_t) ((dv_q4 < 0) ? -dv_q4 : dv_q4);
    uint8_t        cls   = (uint8_t) APP_MOTION_CLASS_CRUISING;

    p_core->speed_lp_q4 = (uint16_t) ((int32_t) p_core->speed_lp_q4 +
                                      (dv_q4 / IC_SPEED_LP_SCALE));

    if (p_core->impact_cycles > 0U)
    {
        p_core->impact_cycles--;
        cls = (uint8_t) APP_MOTION_CLASS_IMPACT;
    }
    else if (!p_in->moving)
    {
        cls = (uint8_t) APP_MOTION_CLASS_STATIONARY;
    }
    else if ((uint8_t) HUMP_CLIMBING == p_core->hump_state)
    {
        cls = (uint8_t) APP_MOTION_CLASS_CLIMBING;
    }
    else if ((uint8_t) HUMP_DESCENDING == p_core->hump_state)
    {
        cls = (uint8_t) APP_MOTION_CLASS_DESCENDING;
    }
    else if (p_in->turning)
    {
        cls = (uint8_t) APP_MOTION_CLASS_TURNING;
    }
    else if (adv >= (APP_CAL_ACCEL_CM_S * IC_Q4))
    {
        cls = (uint8_t) APP_MOTION_CLASS_ACCELERATING;
    }
    else
    {
        cls = (uint8_t) APP_MOTION_CLASS_CRUISING;
    }
    return cls;
}

static void
class_step (imu_core_t * p_core, imu_in_t const * p_in, imu_out_t * p_out)
{
    uint8_t const cand = classify(p_core, p_in);

    if (cand == p_core->motion_class)
    {
        p_core->class_candidate = cand;
        p_core->class_cycles    = 0U;
        return;
    }
    if (cand != p_core->class_candidate)
    {
        p_core->class_candidate = cand;
        p_core->class_cycles    = 0U;
    }
    count_sat8(&p_core->class_cycles);
    if (((uint8_t) APP_MOTION_CLASS_IMPACT == cand) ||
        (p_core->class_cycles >= IC_MS_TO_CYCLES(APP_CAL_CLASS_HOLD_MS)))
    {
        p_core->motion_class =
            cand; /* IMPACT switches at once, others after the hold */
        p_core->class_cycles = 0U;
        p_out->class_changed = true;
    }
}

void
imu_core_step (imu_core_t * p_core, imu_in_t const * p_in, imu_out_t * p_out)
{
    bool const spike = spike_sample(p_core, p_in);

    clear_out(p_out);

    if (p_in->sample_valid && !spike)
    {
        filter_step(p_core,
                    p_in); /* spikes are rejected from the pitch filter */
    }
    rest_step(p_core, p_in);
    hump_step(p_core, p_in, p_out);
    collision_step(p_core, p_in, spike, p_out);
    class_step(p_core, p_in, p_out);

    p_out->state.pitch_half_deg =
        p_in->sample_valid ? p_core->pitch_half_deg : (int8_t) 0;
    p_out->state.quality     = p_in->sample_valid ? p_core->last_quality
                                                  : (uint8_t) APP_HUMP_Q_INVALID;
    p_out->state.episode_seq = p_core->seq;
    p_out->state.height_mm =
        ((uint8_t) HUMP_FLAT == p_core->hump_state)
            ? p_core->last_height_mm
            : q8_to_mm_u8(p_core->peak_q8); /* live peak while active */
    if (p_out->state.height_mm > APP_HUMP_HEIGHT_MAX)
    {
        p_out->state.height_mm = (uint8_t) APP_HUMP_HEIGHT_MAX;
        p_out->height_sat      = true;
    }
    p_out->state.max_height_mm = p_core->max_height_mm;
    p_out->state.motion_class  = p_core->motion_class;
    p_out->state.hump_active   = ((uint8_t) HUMP_FLAT != p_core->hump_state);
}
