/** @file line_core.c
 *  @brief Pure line-sensing implementation. See line_core.h.
 */

#include "line_core.h"

#include "../shared/app_cal.h"
#include "../shared/app_data.h"

#define LC_CYCLE_MS 10U
#define LC_MS_TO_CYCLES(ms) (((ms) + LC_CYCLE_MS - 1U) / LC_CYCLE_MS)
#define LC_PCT 100
#define LC_Q8 256
#define LC_CYC_SAT 0xFFFFU
#define LC_SEQ_MASK 0x0FU
#define LC_AGE_SAT 0xFFU
#define LC_SIDE_NONE 0U
#define LC_SIDE_LEFT 1U
#define LC_SIDE_CENTRE 2U


static int32_t
clamp_i32 (int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

uint8_t
line_core_normalise (uint16_t raw, uint16_t ref_min, uint16_t ref_max)
{
    uint32_t span = 0U;
    uint32_t n    = 0U;

    if (ref_max <= ref_min)
    {
        return 0U;
    }
    span = (uint32_t) ref_max - (uint32_t) ref_min;
    if (raw <= ref_min)
    {
        n = 0U;
    }
    else if (raw >= ref_max)
    {
        n = (uint32_t) LC_PCT;
    }
    else
    {
        n = (((uint32_t) raw - (uint32_t) ref_min) * (uint32_t) LC_PCT) / span;
    }
#if APP_CAL_IR_DARK_IS_HIGH
    return (uint8_t) n; /* higher ADC = darker */
#else
    return (uint8_t) ((uint32_t) LC_PCT - n);
#endif
}

void
line_core_init (line_core_t * p_core)
{
    p_core->min_l          = (uint16_t) APP_CAL_LINE_INIT_WHITE;
    p_core->max_l          = (uint16_t) APP_CAL_LINE_INIT_BLACK;
    p_core->min_c          = (uint16_t) APP_CAL_LINE_INIT_WHITE;
    p_core->max_c          = (uint16_t) APP_CAL_LINE_INIT_BLACK;
    p_core->decay_cycles   = 0U;
    p_core->calibrated     = false;
    p_core->on_line        = false;
    p_core->lost_reported  = false;
    p_core->junction_armed = true;
    p_core->dark_cycles    = 0U;
    p_core->light_cycles   = 0U;
    p_core->prev_error     = 0;
    p_core->seq            = 0U;
    p_core->light_age_l    = LC_AGE_SAT; /* never seen LIGHT: not abrupt */
    p_core->light_age_c    = LC_AGE_SAT;
    p_core->hold_cycles    = 0U;
    p_core->hold_side      = LC_SIDE_NONE;
}

void
line_core_set_references (line_core_t * p_core, uint16_t white_l,
                          uint16_t black_l, uint16_t white_c, uint16_t black_c)
{
#if APP_CAL_IR_DARK_IS_HIGH
    p_core->min_l = (white_l < white_c) ? white_l : white_c;
    p_core->max_l = (black_l > black_c) ? black_l : black_c;
#else
    p_core->min_l = (black_l < black_c) ? black_l : black_c;
    p_core->max_l = (white_l > white_c) ? white_l : white_c;
#endif
    p_core->min_c      = p_core->min_l; /* D-34: one shared pair */
    p_core->max_c      = p_core->max_l;
    p_core->calibrated = true;
}

/** Running references: expand immediately, relax slowly (bounded) on a
 *  decay step only. */
static void
track (uint16_t raw, uint16_t * p_min, uint16_t * p_max, bool relax)
{
    if (raw < *p_min)
    {
        *p_min = raw;
    }
    else if (relax &&
             (*p_min < (uint16_t) (APP_ADC_MAX - APP_CAL_LINE_CAL_DECAY)))
    {
        *p_min = (uint16_t) (*p_min + APP_CAL_LINE_CAL_DECAY);
    }
    else
    {
        /* at ceiling */
    }
    if (raw > *p_max)
    {
        *p_max = raw;
    }
    else if (relax && (*p_max > (uint16_t) APP_CAL_LINE_CAL_DECAY))
    {
        *p_max = (uint16_t) (*p_max - APP_CAL_LINE_CAL_DECAY);
    }
    else
    {
        /* at floor */
    }
}

static bool
span_ok (uint16_t lo, uint16_t hi)
{
    return (hi > lo) &&
           (((uint32_t) hi - (uint32_t) lo) >= APP_CAL_IR_MIN_CONTRAST);
}

/** Track the references and normalise both channels. D-32: with the span
 *  collapsed, noise would normalise to full-scale swings and raise phantom
 *  JUNCTION / FOUND events (seen on the bench 27 Oct); while uncalibrated
 *  both channels read as light (error 0) so the controller runs the bounded
 *  search instead of acting on noise. */
static void
normalise_pair (line_core_t * p_core, uint16_t adc_left, uint16_t adc_centre,
                uint8_t * p_n_l, uint8_t * p_n_c)
{
    bool relax = false;

    p_core->decay_cycles++;
    if (p_core->decay_cycles >= APP_CAL_LINE_CAL_DECAY_CYCLES)
    {
        p_core->decay_cycles = 0U;
        relax                = true;
    }
    /* D-34: one shared reference pair, fed by both channels */
    track(adc_left, &p_core->min_l, &p_core->max_l, relax);
    track(adc_centre, &p_core->min_l, &p_core->max_l, false);
    p_core->min_c      = p_core->min_l;
    p_core->max_c      = p_core->max_l;
    p_core->calibrated = span_ok(p_core->min_l, p_core->max_l);
    if (p_core->calibrated)
    {
        *p_n_l = line_core_normalise(adc_left, p_core->min_l, p_core->max_l);
        *p_n_c = line_core_normalise(adc_centre, p_core->min_c, p_core->max_c);
    }
    else
    {
        *p_n_l = 0U;
        *p_n_c = 0U;
    }
}

/** Junction / lost / found events and on_line from the two classifications. */
static void
events_step (line_core_t * p_core, bool both_dark, bool both_light,
             line_out_t * p_out)
{
    p_out->evt_junction = false;
    p_out->evt_lost     = false;
    p_out->evt_found    = false;

    /* both dark for T_JUNC -> JUNCTION once; re-armed after leaving both-dark
     */
    if (both_dark)
    {
        if (p_core->dark_cycles < LC_CYC_SAT)
        {
            p_core->dark_cycles++;
        }
        if (p_core->junction_armed &&
            (p_core->dark_cycles >= LC_MS_TO_CYCLES(APP_CAL_T_JUNC_MS)))
        {
            p_core->junction_armed = false;
            p_out->evt_junction    = true;
        }
    }
    else
    {
        p_core->dark_cycles    = 0U;
        p_core->junction_armed = true;
    }

    /* D-36: on_line is instantaneous, "a sensor sees the line now" (dark or
       on an edge). until_line turns need exactly that. With the straddle
       bracket both sensors are light while the car is centred on a straight,
       so both-light is NOT a loss: the LOST event comes only after T_LOST of
       both light, and FOUND when a sensor sees the line again. */
    if (both_light)
    {
        if (p_core->light_cycles < LC_CYC_SAT)
        {
            p_core->light_cycles++;
        }
        if ((p_core->light_cycles >= LC_MS_TO_CYCLES(APP_CAL_T_LOST_MS)) &&
            !p_core->lost_reported)
        {
            p_core->lost_reported = true;
            p_out->evt_lost       = true;
        }
        p_core->on_line = false;
    }
    else
    {
        p_core->light_cycles = 0U;
        if (p_core->lost_reported)
        {
            p_core->lost_reported = false;
            p_out->evt_found      = true;
        }
        p_core->on_line = true;
    }
}

static uint8_t
age_step (uint8_t age, bool light)
{
    return light ? 0U : ((age < LC_AGE_SAT) ? (uint8_t) (age + 1U) : age);
}

/** D-37 branch hold (see line_core.h). Uses the light ages from BEFORE this
 *  sample. Returns true while the steering must be held straight; sets
 *  *p_junction when a crossbar is recognised during a hold. */
static bool
branch_step (line_core_t * p_core, bool dark_l, bool dark_c, bool * p_junction)
{
    uint32_t const hold      = LC_MS_TO_CYCLES(APP_CAL_LINE_BRANCH_HOLD_MS);
    bool           held_dark = false;

    *p_junction = false;
    if (0U == hold)
    {
        return false;
    }
    if (0U == p_core->hold_cycles)
    {
        /* start: exactly one sensor DARK and it was LIGHT a moment ago */
        if (dark_l && !dark_c &&
            (p_core->light_age_l < (uint8_t) APP_CAL_LINE_BRANCH_RISE_CYCLES))
        {
            p_core->hold_side   = LC_SIDE_LEFT;
            p_core->hold_cycles = (uint16_t) hold;
        }
        else if (dark_c && !dark_l &&
                 (p_core->light_age_c <
                  (uint8_t) APP_CAL_LINE_BRANCH_RISE_CYCLES))
        {
            p_core->hold_side   = LC_SIDE_CENTRE;
            p_core->hold_cycles = (uint16_t) hold;
        }
        else
        {
            return false;
        }
    }
    if (dark_l && dark_c)
    {
        /* the other end of a crossbar: junction, steering back to normal */
        *p_junction         = true;
        p_core->hold_cycles = 0U;
        p_core->hold_side   = LC_SIDE_NONE;
        return false;
    }
    held_dark = (LC_SIDE_LEFT == p_core->hold_side) ? dark_l : dark_c;
    if (!held_dark)
    {
        /* side branch passed: ignored */
        p_core->hold_cycles = 0U;
        p_core->hold_side   = LC_SIDE_NONE;
        return false;
    }
    p_core->hold_cycles--;
    if (0U == p_core->hold_cycles)
    {
        /* still DARK after the hold: a real drift, steer from now on */
        p_core->hold_side = LC_SIDE_NONE;
        return false;
    }
    return true;
}

void
line_core_step (line_core_t * p_core, uint16_t adc_left, uint16_t adc_centre,
                line_out_t * p_out)
{
    uint8_t n_l           = 0U;
    uint8_t n_c           = 0U;
    bool    dark_l        = false;
    bool    dark_c        = false;
    bool    light_l       = false;
    bool    light_c       = false;
    int32_t error         = 0;
    int32_t steer         = 0;
    bool    hold          = false;
    bool    hold_junction = false;

    adc_left = (uint16_t) ((adc_left > APP_ADC_MAX) ? APP_ADC_MAX : adc_left);
    adc_centre =
        (uint16_t) ((adc_centre > APP_ADC_MAX) ? APP_ADC_MAX : adc_centre);

    normalise_pair(p_core, adc_left, adc_centre, &n_l, &n_c);
    dark_l  = (n_l >= (uint8_t) APP_CAL_LINE_DARK_PCT);
    dark_c  = (n_c >= (uint8_t) APP_CAL_LINE_DARK_PCT);
    light_l = (n_l <= (uint8_t) APP_CAL_LINE_LIGHT_PCT);
    light_c = (n_c <= (uint8_t) APP_CAL_LINE_LIGHT_PCT);

    error = clamp_i32((int32_t) n_c - (int32_t) n_l -
                          (int32_t) APP_CAL_LINE_ERROR_SETPOINT,
                      APP_LINE_ERROR_MIN, APP_LINE_ERROR_MAX);
    steer = (((int32_t) APP_CAL_LINE_KP_Q8 * error) +
             ((int32_t) APP_CAL_LINE_KD_Q8 *
              (error - (int32_t) p_core->prev_error))) /
            LC_Q8;
    steer = clamp_i32(steer, APP_LINE_ERROR_MIN, APP_LINE_ERROR_MAX);
    p_core->prev_error = (int16_t) error;

    hold                = branch_step(p_core, dark_l, dark_c, &hold_junction);
    p_core->light_age_l = age_step(p_core->light_age_l, light_l);
    p_core->light_age_c = age_step(p_core->light_age_c, light_c);
    if (hold)
    {
        error = 0; /* D-37: straight through a possible junction */
        steer = 0;
        /* no derivative kick when the hold ends on a passed branch */
        p_core->prev_error = 0;
    }

    events_step(p_core, dark_l && dark_c, light_l && light_c, p_out);
    if (hold_junction && p_core->junction_armed)
    {
        /* D-37: crossbar met at an angle; the both-dark T_JUNC path is
           disarmed until both sensors leave the bar */
        p_core->junction_armed = false;
        p_out->evt_junction    = true;
    }

    p_core->seq = (uint8_t) ((p_core->seq + 1U) & LC_SEQ_MASK);

    p_out->state.line_error = (int8_t) error; /* clamped to +-100 above */
    p_out->state.steer      = (int8_t) steer;
    p_out->state.on_line    = p_core->on_line;
    p_out->state.event =
        (uint8_t) (p_out->evt_junction
                       ? APP_LINE_EVT_JUNCTION
                       : (p_out->evt_lost
                              ? APP_LINE_EVT_LOST
                              : (p_out->evt_found ? APP_LINE_EVT_FOUND
                                                  : APP_LINE_EVT_NONE)));
    p_out->state.seq   = p_core->seq;
    p_out->contrast_ok = p_core->calibrated;
    p_out->n_left      = n_l;
    p_out->n_centre    = n_c;
    p_out->branch_hold = hold;
}
