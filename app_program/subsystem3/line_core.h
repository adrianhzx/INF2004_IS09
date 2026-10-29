/** @file line_core.h
 *  @brief Pure line-sensing algorithm (Rev B 6.1 "line tracking" region):
 *         running black/white calibration with a contrast check, signed line
 *         error and steering demand, hysteretic JUNCTION / LOST / FOUND
 *         events, continuous on_line. Owner: Buddy 3 (Xu Zhiqiang).
 *         No kernel or hardware calls; one call per 10 ms sample pair.
 *
 *  Sensors (A7): LEFT and CENTRE over the line, the line running between
 *  them so each sees one edge. Normalised darkness n = 0 (white reference)
 *  .. 100 (black reference). D-34: the black/white references are ONE pair
 *  learned from both channels together (the two modules are alike and a few
 *  cm apart), so a car placed on the line calibrates at once - one sensor
 *  sees white, the other black - and a lengthy straight does not let a
 *  channel that only ever sees one shade forget the other. They relax
 *  slowly (APP_CAL_LINE_CAL_DECAY per APP_CAL_LINE_CAL_DECAY_CYCLES).
 *  D-35: DARK is n >= APP_CAL_LINE_DARK_PCT, LIGHT is n <=
 * APP_CAL_LINE_LIGHT_PCT; between them a sensor is on an edge, which is on the
 * line and neither a junction (both DARK) nor a loss (both LIGHT). line_error =
 * n_centre - n_left, clamped to +-100: when the line drifts to the RIGHT the
 * left sensor goes lighter and the centre darker, so the error is positive =
 * "line right of centre" (app_data.h), and FOLLOW drives the left wheel harder
 * (motion_core). HV-10 (hand sweep, A7) confirms both the ADC polarity and this
 * sign on the real bracket.
 *
 * D-37 branch hold: with the straddle bracket a sensor that turns DARK means
 * "line drifted that way" - but a side branch (|-) or one end of a crossbar
 * (+) met at a slight angle looks the same, and the steering used to turn
 * the car into it. Drift moves a sensor sideways onto the line slowly, through
 * the edge band; a junction bar arrives head-on and the sensor goes from LIGHT
 * to DARK within APP_CAL_LINE_BRANCH_RISE_CYCLES. On such an abrupt single
 * DARK the steering is held straight (error 0, steer 0) for up to
 * APP_CAL_LINE_BRANCH_HOLD_MS:
 *   - the other sensor turns DARK too  -> JUNCTION (crossbar, even if the two
 *     sensors were never dark together for T_JUNC), hold ends;
 *   - the DARK sensor turns not-DARK  -> side branch passed, ignored;
 *   - still DARK when the hold expires -> real drift, normal steering.
 * APP_CAL_LINE_BRANCH_HOLD_MS 0 disables the hold (pre-D-37 behaviour).
 */

#ifndef LINE_CORE_H
#define LINE_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "../shared/app_cal.h"
#include "../shared/app_line.h"

/* D-37 branch hold. Set in app_cal.h; these defaults keep a tree whose
   app_cal.h predates D-37 building with the same behaviour. */
#ifndef APP_CAL_LINE_BRANCH_HOLD_MS
#define APP_CAL_LINE_BRANCH_HOLD_MS                                            \
    200U /* abrupt single DARK: hold straight for this time (0 = off) [TBC] */
#endif
#ifndef APP_CAL_LINE_BRANCH_RISE_CYCLES
#define APP_CAL_LINE_BRANCH_RISE_CYCLES                                        \
    3U /* LIGHT seen within this many cycles before DARK = abrupt [TBC] */
#endif

typedef struct
{
    uint16_t min_l; /* running references (raw ADC); D-34: one shared pair
                       fed by both channels, mirrored into _c */
    uint16_t max_l;
    uint16_t min_c;
    uint16_t max_c;
    uint16_t decay_cycles; /* cycles since the references last relaxed */
    bool     calibrated;   /* span >= APP_CAL_IR_MIN_CONTRAST on both */
    bool     on_line;
    bool     lost_reported;
    bool     junction_armed; /* re-arms after leaving both-dark */
    uint16_t dark_cycles;    /* consecutive both-dark cycles */
    uint16_t light_cycles;   /* consecutive both-light cycles */
    int16_t  prev_error;
    uint8_t  seq;
    uint8_t  light_age_l; /* D-37: cycles since the sensor was LIGHT */
    uint8_t  light_age_c;
    uint16_t hold_cycles; /* D-37: branch hold cycles left (0 = none) */
    uint8_t  hold_side;   /* D-37: LC_SIDE_* of the sensor that went DARK */
} line_core_t;

typedef struct
{
    app_line_state_t state;       /* to pack and publish */
    bool             contrast_ok; /* false -> IR_CONTRAST fault */
    bool             evt_junction;
    bool             evt_lost;
    bool             evt_found;
    uint8_t          n_left; /* diagnostics, 0..100 */
    uint8_t          n_centre;
    bool             branch_hold; /* D-37: steering held straight */
} line_out_t;

void line_core_init (line_core_t * p_core);

/** Explicit calibration (bench or controller): raw references per channel. */
void line_core_set_references (line_core_t * p_core, uint16_t white_l,
                               uint16_t black_l, uint16_t white_c,
                               uint16_t black_c);

/** One 10 ms step with two raw 12-bit samples. */
void line_core_step (line_core_t * p_core, uint16_t adc_left,
                     uint16_t adc_centre, line_out_t * p_out);

/** Normalise one raw sample against references (exposed for tests). */
uint8_t line_core_normalise (uint16_t raw, uint16_t ref_min, uint16_t ref_max);

#endif /* LINE_CORE_H */
