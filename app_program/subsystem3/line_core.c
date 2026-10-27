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
    p_core->calibrated     = false;
    p_core->on_line        = false;
    p_core->lost_reported  = false;
    p_core->junction_armed = true;
    p_core->dark_cycles    = 0U;
    p_core->light_cycles   = 0U;
    p_core->prev_error     = 0;
    p_core->seq            = 0U;
}

void
line_core_set_references (line_core_t * p_core, uint16_t white_l,
                          uint16_t black_l, uint16_t white_c, uint16_t black_c)
{
#if APP_CAL_IR_DARK_IS_HIGH
    p_core->min_l = white_l;
    p_core->max_l = black_l;
    p_core->min_c = white_c;
    p_core->max_c = black_c;
#else
    p_core->min_l = black_l;
    p_core->max_l = white_l;
    p_core->min_c = black_c;
    p_core->max_c = white_c;
#endif
    p_core->calibrated = true;
}

/** Running references: expand immediately, relax slowly (bounded). */
static void
track (uint16_t raw, uint16_t * p_min, uint16_t * p_max)
{
    if (raw < *p_min)
    {
        *p_min = raw;
    }
    else if (*p_min < (uint16_t) (APP_ADC_MAX - APP_CAL_LINE_CAL_DECAY))
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
    else if (*p_max > (uint16_t) APP_CAL_LINE_CAL_DECAY)
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
    track(adc_left, &p_core->min_l, &p_core->max_l);
    track(adc_centre, &p_core->min_c, &p_core->max_c);
    p_core->calibrated = span_ok(p_core->min_l, p_core->max_l) &&
                         span_ok(p_core->min_c, p_core->max_c);
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

void
line_core_step (line_core_t * p_core, uint16_t adc_left, uint16_t adc_centre,
                line_out_t * p_out)
{
    uint8_t n_l    = 0U;
    uint8_t n_c    = 0U;
    bool    dark_l = false;
    bool    dark_c = false;
    int32_t error  = 0;
    int32_t steer  = 0;

    adc_left = (uint16_t) ((adc_left > APP_ADC_MAX) ? APP_ADC_MAX : adc_left);
    adc_centre =
        (uint16_t) ((adc_centre > APP_ADC_MAX) ? APP_ADC_MAX : adc_centre);

    normalise_pair(p_core, adc_left, adc_centre, &n_l, &n_c);
    dark_l = (n_l >= (uint8_t) APP_CAL_LINE_DARK_PCT);
    dark_c = (n_c >= (uint8_t) APP_CAL_LINE_DARK_PCT);

    error = clamp_i32((int32_t) n_c - (int32_t) n_l, APP_LINE_ERROR_MIN,
                      APP_LINE_ERROR_MAX);
    steer = (((int32_t) APP_CAL_LINE_KP_Q8 * error) +
             ((int32_t) APP_CAL_LINE_KD_Q8 *
              (error - (int32_t) p_core->prev_error))) /
            LC_Q8;
    steer = clamp_i32(steer, APP_LINE_ERROR_MIN, APP_LINE_ERROR_MAX);
    p_core->prev_error = (int16_t) error;

    p_out->evt_junction = false;
    p_out->evt_lost     = false;
    p_out->evt_found    = false;

    /* both dark for T_JUNC -> JUNCTION once; re-armed after leaving both-dark
     */
    if (dark_l && dark_c)
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

    /* both light for T_LOST -> LOST (on_line false); any dark after -> FOUND */
    if (!dark_l && !dark_c)
    {
        if (p_core->light_cycles < LC_CYC_SAT)
        {
            p_core->light_cycles++;
        }
        if (p_core->light_cycles >= LC_MS_TO_CYCLES(APP_CAL_T_LOST_MS))
        {
            if (!p_core->lost_reported)
            {
                p_core->lost_reported = true;
                p_out->evt_lost       = true;
            }
            p_core->on_line = false;
        }
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
}
