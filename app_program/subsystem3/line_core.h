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
 *  line_error = n_centre - n_left, clamped to +-100: when the line drifts to
 *  the RIGHT the left sensor goes lighter and the centre darker, so the error
 *  is positive = "line right of centre" (app_data.h), and FOLLOW drives the
 *  left wheel harder (motion_core). HV-10 (hand sweep, A7) confirms both the
 *  ADC polarity and this sign on the real bracket.
 */

#ifndef LINE_CORE_H
#define LINE_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "../shared/app_line.h"

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
