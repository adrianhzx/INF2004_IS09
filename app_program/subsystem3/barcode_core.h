/** @file barcode_core.h
 *  @brief Pure barcode decoder (Rev B 6.1 "barcode decoding" region):
 *         DISARMED / WAIT_QUIET / CAPTURING / DECODING, glitch merging,
 *         quiet-zone end of capture, Code 39 narrow/wide ratio classification
 *         with reverse-pattern support, A/B/C/D mapping, INVALID exits.
 *         Owner: Buddy 3. No kernel or hardware calls.
 *
 *  Symbology (measured from the supplied samples, ledger section 1): one
 *  Code 39 character between '*' start/stop, 29 elements (15 bars + 14
 *  spaces incl. two inter-character gaps), wide:narrow = 3:1.
 *  A = LEFT, B = RIGHT, C = STRAIGHT, D = U-TURN (project write-up p.3).
 *
 *  Input model: the ISR delivers one edge at a time with the time since the
 *  previous edge. The element that just ENDED has that width; elements
 *  alternate bar/space starting with a bar at the first edge after a quiet
 *  zone, so decoding does not depend on the sensor's output polarity.
 */

#ifndef BARCODE_CORE_H
#define BARCODE_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "../shared/app_data.h"

#define BC_MAX_ELEMENTS 64U

typedef enum
{
    BC_DISARMED   = 0,
    BC_WAIT_QUIET = 1,
    BC_CAPTURING  = 2,
    BC_DECODING   = 3 /* transient: decode happens inside the quiet call */
} bc_state_t;

typedef struct
{
    uint8_t  state;         /* bc_state_t (2 bits used) */
    uint8_t  count;         /* elements captured */
    bool     merge_pending; /* a glitch was folded; next width continues it */
    uint16_t width[BC_MAX_ELEMENTS]; /* element widths in 8 us units (as
                                        EdgeRing), task-private */
    uint8_t  seq;                    /* result sequence 0..15 */
    uint32_t stats_decoded;
    uint32_t stats_invalid;
    uint32_t stats_stray;
} barcode_core_t;

typedef struct
{
    bool result_ready; /* publish result + seq, raise EVT_BARCODE */
    app_barcode_result_t result;
    uint8_t              seq;
    bool                 overflow_fault; /* set FAULT_BARCODE_OVERFLOW */
} barcode_out_t;

void       barcode_core_init (barcode_core_t * p_core);
void       barcode_core_arm (barcode_core_t * p_core);
void       barcode_core_disarm (barcode_core_t * p_core);
bc_state_t barcode_core_state (barcode_core_t const * p_core);

/** One edge: @p delta_us since the previous edge (the width of the element
 *  that just ended). Never produces a result by itself. */
void barcode_core_edge (barcode_core_t * p_core, uint32_t delta_us,
                        barcode_out_t * p_out);

/** Quiet-zone timeout (no edge for T_QUIET) or ring overflow notice: ends
 *  the capture and decodes. */
void barcode_core_quiet (barcode_core_t * p_core, bool overflow,
                         barcode_out_t * p_out);

/** Pure decoder over an element width list in any one consistent unit
 *  (exposed for tests and the laptop tool): returns LEFT/RIGHT/STRAIGHT/
 *  UTURN or INV_NO_MATCH; *p_reversed reports a reversed read. */
app_barcode_result_t barcode_core_decode (uint16_t const * p_width,
                                          uint32_t count, bool * p_reversed);

/** Map a Code 39 character (A-D) to a navigation result (tests). */
app_barcode_result_t barcode_core_map_char (char code39_char);

#endif /* BARCODE_CORE_H */
