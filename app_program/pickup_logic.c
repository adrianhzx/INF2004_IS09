/**
 * @file    pickup_logic.c
 * @brief   Pick-up detection (20 Nov), see pickup_logic.h.
 *
 * Numbers from the dashboard logs of 19-20 Nov:
 *  - Car in the air (19 Nov 21:04, wheels spinning; 20 Nov 15:01, lifted
 *    after a stop): LEFT, CENTRE and barcode AO all read about 3300-4095.
 *  - On the track, in every LINE_FOLLOWING / MANEUVER / OBSTACLE_AVOIDANCE
 *    frame, the three never read 2000 or more together. The weak LEFT line
 *    sensor reached at most 2525 raw on black tape.
 * The high reading must also last PICKUP_MM of wheel travel or PICKUP_MS,
 * so crossing an 18 mm black bar does not count even if LEFT reads higher
 * on another day.
 */
#include "pickup_logic.h"

void pickup_reset(pickup_ctx_t *p_ctx)
{
    p_ctx->ms      = 0u;
    p_ctx->mm      = 0.0f;
    p_ctx->b_fired = false;
}

bool pickup_step(pickup_ctx_t *p_ctx, const pickup_par_t *p_par,
                 uint32_t raw_l, uint32_t raw_c, uint32_t raw_ir,
                 float ds_mm, uint32_t dt_ms)
{
    bool b_high;

    if (p_par->adc == 0u)
    {
        pickup_reset(p_ctx);
        return false;                          /* switched off */
    }
    b_high = (raw_l >= p_par->adc) && (raw_c >= p_par->adc) && (raw_ir >= p_par->adc);
    if (!b_high)
    {
        p_ctx->ms = 0u;
        p_ctx->mm = 0.0f;
        return false;                          /* b_fired stays: one report per run */
    }
    p_ctx->ms += dt_ms;
    p_ctx->mm += (ds_mm < 0.0f) ? -ds_mm : ds_mm;
    if (p_ctx->b_fired)
    {
        return false;
    }
    if ((p_ctx->mm >= p_par->mm) || (p_ctx->ms >= p_par->ms))
    {
        p_ctx->b_fired = true;
        return true;
    }
    return false;
}
