/** @file drv_encoder.h
 *  @brief Quadrature wheel encoders on GP0/GP1 (left) and GP2/GP3 (right),
 *         decoded in EncoderIsr through the shared GPIO dispatcher.
 *
 *  ISR -> task data (all single-writer, single-copy atomic, no RMW across the
 *  boundary): g_enc_ticks_l / g_enc_ticks_r (uint16, app_motion.h, count up
 *  for forward and down for reverse, wrap), one uint16 last edge-to-edge
 *  interval per wheel in 32 us units (0..65535 = 0..2.1 s, saturating; 0 =
 *  no edge yet) and one saturating uint8 glitch counter (invalid 2-step
 *  transitions). The task only reads these halfwords/bytes.
 *
 *  Decoding is the standard 4x table: previous AB (2 bits) and new AB (2
 *  bits) index a 16-entry table of -1 / 0 / +1 / glitch. The ISR work is
 *  bounded: one read of GPIO_IN, one table lookup, three stores.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/** Configure the four inputs (pull-ups, Schmitt) and attach both-edge
 *  callbacks. Returns false if a pin is already owned by another callback. */
bool drv_encoder_init (void);

/** ISR-written interval of the last two edges, 32 us units (0 = none). */
uint16_t drv_encoder_interval_l_32us (void);
uint16_t drv_encoder_interval_r_32us (void);

/** Saturating count of invalid quadrature transitions (both wheels). */
uint8_t drv_encoder_glitches (void);

/** Total edges seen (diagnostics, S2-U tests). */
uint32_t drv_encoder_edge_count (void);

/** Pure decoder step, exposed for host tests: returns -1/0/+1, sets
 *  *p_glitch when the transition is invalid (both bits changed). */
int8_t drv_encoder_decode (uint8_t prev_ab, uint8_t new_ab, bool * p_glitch);

#endif /* DRV_ENCODER_H */
