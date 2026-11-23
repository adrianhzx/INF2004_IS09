/**
 * @file    pickup_logic.h
 * @brief   Pick-up detection (20 Nov): the car has been lifted off the track.
 *
 * Off the floor no IR light comes back, so the two line sensors and the
 * barcode sensor all read near full scale. The rule uses the raw ADC values,
 * not the line darkness %, because line_core rescales the darkness to its
 * running black reference. See pickup_logic.c for the numbers behind it.
 * Plain C, no kernel calls, so it can be tested on the PC.
 */
#ifndef PICKUP_LOGIC_H
#define PICKUP_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t adc;      /* all three sensors at or above this raw value (0 = off) */
    float    mm;       /* ... for this much wheel travel */
    uint32_t ms;       /* ... or for this long (the wheels may not turn) */
} pickup_par_t;

typedef struct
{
    uint32_t ms;       /* time all three have read high, so far */
    float    mm;       /* wheel travel in that time */
    bool     b_fired;  /* reported once; cleared by pickup_reset() */
} pickup_ctx_t;

void pickup_reset(pickup_ctx_t *p_ctx);

/* One step. raw_l / raw_c: the LEFT and CENTRE line sensors, raw_ir: the
   barcode sensor AO, all 12-bit. ds_mm: wheel travel since the last step,
   dt_ms: time since the last step. Returns true once, on the step where the
   car counts as picked up. */
bool pickup_step(pickup_ctx_t *p_ctx, const pickup_par_t *p_par,
                 uint32_t raw_l, uint32_t raw_c, uint32_t raw_ir,
                 float ds_mm, uint32_t dt_ms);

#endif /* PICKUP_LOGIC_H */
