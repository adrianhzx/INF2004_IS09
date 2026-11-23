/**
 * @file    hump2.c
 * @brief   Hump profiler beside IS09 imu_core (see hump2.h).
 *
 * imu_pitch_deg(), ema(), hump_reset() and hump_step() are the base
 * imu_logic.c functions (copy in base_replaced/), renamed so they do not
 * clash with IS09 imu_core, with the axes taken from the IS09 mapping and
 * the tilt-height tracking added.
 */
#include <math.h>
#include "hump2.h"

#define RAD_TO_DEG       (57.29578f)
#define DEG_TO_RAD       (0.01745329f)
#define OFFSET_SAMPLES   (50u)          /* base tsk_imu.c: 50 x 20 ms */
#define DECIMATE         (2u)           /* 2 x 10 ms = the base 20 ms cycle */

hump2_t     g_hump2;
hump2_par_t g_hump2_par;

/* base imu_pitch_deg(), with the axes already mapped (nose up = +) */
static float pitch_deg(float fwd, float lat, float up, float a_fwd_g)
{
    float other = sqrtf((lat * lat) + (up * up));

    return atan2f(fwd - a_fwd_g, other) * RAD_TO_DEG;
}

static float ema(float y, float x, float alpha)
{
    return y + (alpha * (x - y));
}

static void hump_reset(hump2_t *p_h)
{
    p_h->state   = HUMP2_FLAT;
    p_h->n_up    = 0u;
    p_h->n_down  = 0u;
    p_h->n_level = 0u;
    p_h->h_mm    = 0.0f;
    p_h->peak_mm = 0.0f;
    p_h->len_mm  = 0.0f;
    p_h->p_max   = 0.0f;
    p_h->p_min   = 0.0f;
    p_h->n_below = 0u;
    p_h->b_confirmed = false;
    p_h->desc_from_mm = 0.0f;
}

static float tilt_mm(const hump2_t *p_h, const hump2_par_t *p_par)
{
    float up   = (p_h->p_max > 0.0f) ? sinf(p_h->p_max * DEG_TO_RAD) : 0.0f;
    float down = (p_h->p_min < 0.0f) ? sinf(-p_h->p_min * DEG_TO_RAD) : 0.0f;

    if (down > 0.0f)
    {
        return p_par->wheelbase_mm * 0.5f * (up + down);   /* climb and drop averaged */
    }
    return p_par->wheelbase_mm * up;
}

/* Only a whole hump (climb and drop, each at least min_climb_mm long) gives
 * an end event; anything else (a vibration spike, a spike pair, no drop
 * within max_len_mm) ends silently and is only counted in n_rejected. */
static void end_hump(hump2_t *p_h, const hump2_par_t *p_par, hump2_ev_t *p_ev, bool b_valid)
{
    if (!b_valid)
    {
        p_h->n_rejected++;                   /* no event */
        hump_reset(p_h);
        return;
    }
    p_ev->b_end      = true;
    p_ev->b_valid    = b_valid;
    p_ev->b_too_long = !b_valid && (p_h->len_mm > p_par->max_len_mm);
    p_ev->height_mm  = p_h->peak_mm;
    p_ev->tilt_mm    = tilt_mm(p_h, p_par);
    p_ev->len_mm     = p_h->len_mm;
    p_h->last_h_mm    = p_ev->height_mm;     /* shown even when the drop was not seen */
    p_h->last_tilt_mm = p_ev->tilt_mm;
    p_h->last_len_mm  = p_ev->len_mm;
    if (b_valid)
    {
        if (p_ev->height_mm > p_h->max_h_mm)
        {
            p_h->max_h_mm = p_ev->height_mm;
        }
        if (p_ev->tilt_mm > p_h->max_tilt_mm)
        {
            p_h->max_tilt_mm = p_ev->tilt_mm;
        }
        p_h->n_valid++;
    }
    else
    {
        p_h->n_rejected++;
    }
    hump_reset(p_h);
}

/* base hump_step(), plus the highest / lowest pitch of the episode */
static void hump_step(hump2_t *p_h, const hump2_par_t *p_par, float pitch, float ds_mm,
                      hump2_ev_t *p_ev)
{
    bool b_level = (fabsf(pitch) <= p_par->level_deg);

    if (ds_mm < 0.0f)
    {
        ds_mm = -ds_mm;
    }
    p_h->n_up    = (pitch >= p_par->start_deg) ? (p_h->n_up + 1u) : 0u;
    p_h->n_down  = (pitch <= -p_par->start_deg) ? (p_h->n_down + 1u) : 0u;
    p_h->n_level = b_level ? (p_h->n_level + 1u) : 0u;
    p_h->n_below = (pitch < p_par->level_deg) ? (p_h->n_below + 1u) : 0u;

    if (p_h->state == HUMP2_FLAT)
    {
        if (p_h->drive_ms < p_par->start_blank_ms)
        {
            p_h->n_up = 0u;                  /* the start jolt (19 Nov, 20:42 run) */
        }
        if (p_h->n_up >= p_par->start_samples)
        {
            p_h->state   = HUMP2_ASCENT;
            p_h->h_mm    = 0.0f;
            p_h->peak_mm = 0.0f;
            p_h->len_mm  = 0.0f;
            p_h->p_max   = pitch;
            p_h->p_min   = 0.0f;
            p_h->n_below = 0u;
            /* no event yet: the climb is confirmed at the crest (19 Nov) */
        }
        return;
    }

    /* integrate the height in every hump state */
    p_h->h_mm   += sinf(pitch * DEG_TO_RAD) * ds_mm;
    p_h->len_mm += ds_mm;
    if (p_h->h_mm > p_h->peak_mm)
    {
        p_h->peak_mm = p_h->h_mm;
    }
    if (pitch > p_h->p_max)
    {
        p_h->p_max = pitch;
    }
    if (pitch < p_h->p_min)
    {
        p_h->p_min = pitch;
    }

    if (p_h->len_mm > p_par->max_len_mm)
    {
        end_hump(p_h, p_par, p_ev, false);
        return;
    }

    switch (p_h->state)
    {
        case HUMP2_ASCENT:
            /* the base code decided on the first sample below level; one vibration
             * spike then ended a real climb. Now it must stay below level for
             * start_samples (19 Nov). */
            if (p_h->n_below >= p_par->start_samples)
            {
                if ((p_h->h_mm > p_par->min_height_mm) && (p_h->len_mm >= p_par->min_climb_mm))
                {
                    p_h->state      = HUMP2_CREST;
                    p_ev->b_start   = !p_h->b_confirmed;   /* first crest: a real climb */
                    p_ev->b_peak    = true;
                    p_ev->height_mm = p_h->peak_mm;
                    p_ev->tilt_mm   = tilt_mm(p_h, p_par);
                    p_h->b_confirmed = true;
                }
                else
                {
                    end_hump(p_h, p_par, p_ev, false);   /* false slope */
                }
            }
            break;

        case HUMP2_CREST:
            if (p_h->n_down >= p_par->start_samples)
            {
                p_h->state = HUMP2_DESCENT;
                p_h->desc_from_mm = p_h->len_mm;
            }
            else if (p_h->n_up >= p_par->start_samples)
            {
                p_h->state = HUMP2_ASCENT;       /* double hump: keep the running peak */
            }
            else
            {
                /* flat top */
            }
            break;

        case HUMP2_DESCENT:
            if (p_h->n_level >= p_par->end_samples)
            {
                /* the drop must last as long as a real climb (19 Nov) */
                end_hump(p_h, p_par, p_ev, (p_h->len_mm - p_h->desc_from_mm) >= p_par->min_climb_mm);
            }
            break;

        default:
            hump_reset(p_h);
            break;
    }
}

void hump2_new_run(hump2_t *p_h)
{
    hump_reset(p_h);
    p_h->last_h_mm    = 0.0f;
    p_h->last_tilt_mm = 0.0f;
    p_h->last_len_mm  = 0.0f;
    p_h->max_h_mm     = 0.0f;
    p_h->max_tilt_mm  = 0.0f;
    p_h->n_valid      = 0u;
    p_h->n_rejected   = 0u;
}

void hump2_init(hump2_t *p_h)
{
    hump2_new_run(p_h);
    p_h->pitch_deg     = 0.0f;
    p_h->theta0_deg    = 0.0f;
    p_h->off_sum       = 0.0f;
    p_h->off_n         = 0u;
    p_h->b_offset      = false;
    p_h->sub           = 0u;
    p_h->fwd_sum       = 0;
    p_h->lat_sum       = 0;
    p_h->up_sum        = 0;
    p_h->n_sum         = 0u;
    p_h->ds_sum        = 0.0f;
    p_h->trace_n       = 0u;
    p_h->drive_ms      = 0u;
    p_h->p_hi          = 0.0f;
    p_h->p_lo          = 0.0f;
    p_h->b_range_reset = true;
}

bool hump2_step(hump2_t *p_h, const hump2_par_t *p_par,
                    const hump2_in_t *p_in, hump2_ev_t *p_ev)
{
    float raw;
    float n;

    p_ev->b_start    = false;
    p_ev->b_peak     = false;
    p_ev->b_end      = false;
    p_ev->b_valid    = false;
    p_ev->b_too_long = false;
    p_ev->height_mm  = 0.0f;
    p_ev->tilt_mm    = 0.0f;
    p_ev->len_mm     = 0.0f;

    if (p_in->b_sample_valid)
    {
        p_h->fwd_sum += p_in->fwd_mg;
        p_h->lat_sum += p_in->lat_mg;
        p_h->up_sum  += p_in->up_mg;
        p_h->n_sum++;
    }
    p_h->ds_sum += p_in->ds_mm;
    p_h->sub++;
    if (p_h->sub < DECIMATE)
    {
        return false;
    }
    p_h->sub = 0u;
    if (p_h->n_sum == 0u)
    {
        return false;                     /* no sample: the distance is kept for the next cycle */
    }

    /* 20 ms cycle: the mean of the two samples, in g */
    n   = 1000.0f * (float)p_h->n_sum;
    raw = pitch_deg((float)p_h->fwd_sum / n, (float)p_h->lat_sum / n, (float)p_h->up_sum / n,
                    p_par->b_accel_comp ? p_in->a_fwd_g : 0.0f);
    p_h->fwd_sum = 0;
    p_h->lat_sum = 0;
    p_h->up_sum  = 0;
    p_h->n_sum   = 0u;

    /* mounting offset: re-learnt while idle and still (base) */
    if (p_in->b_learn_offset)
    {
        p_h->off_sum += raw;
        p_h->off_n++;
        if (p_h->off_n >= OFFSET_SAMPLES)
        {
            p_h->theta0_deg = p_h->off_sum / (float)p_h->off_n;
            p_h->b_offset   = true;
            p_h->off_sum    = 0.0f;
            p_h->off_n      = 0u;
        }
    }
    else
    {
        p_h->off_sum = 0.0f;
        p_h->off_n   = 0u;
    }
    p_h->pitch_deg = ema(p_h->pitch_deg, raw - p_h->theta0_deg, p_par->alpha);
    {
        float t = p_h->pitch_deg * 10.0f;

        t = (t > 32000.0f) ? 32000.0f : ((t < -32000.0f) ? -32000.0f : t);
        p_h->trace[p_h->trace_n % HUMP2_TRACE_LEN] = (int16_t)lroundf(t);
        p_h->trace_n++;                      /* written after the sample */
    }

    if (p_h->b_range_reset)
    {
        p_h->b_range_reset = false;
        p_h->p_hi = p_h->pitch_deg;
        p_h->p_lo = p_h->pitch_deg;
    }
    else
    {
        p_h->p_hi = (p_h->pitch_deg > p_h->p_hi) ? p_h->pitch_deg : p_h->p_hi;
        p_h->p_lo = (p_h->pitch_deg < p_h->p_lo) ? p_h->pitch_deg : p_h->p_lo;
    }

    /* humps only while driving (base) */
    if (p_in->b_driving)
    {
        if (p_h->drive_ms < 100000u)
        {
            p_h->drive_ms += 20u;            /* this cycle counts as driven */
        }
        hump_step(p_h, p_par, p_h->pitch_deg, p_h->ds_sum, p_ev);
    }
    else if (p_h->state != HUMP2_FLAT)
    {
        p_h->drive_ms = 0u;
        /* stopped in the middle: only counted. On the 20:25 run the stop jolt
         * itself read +8 deg for ~15 mm. */
        p_h->n_rejected++;
        hump_reset(p_h);
    }
    else
    {
        p_h->drive_ms = 0u;                  /* flat and not driving */
    }
    p_h->ds_sum = 0.0f;
    return true;
}

float hump2_live_tilt_mm(const hump2_t *p_h, const hump2_par_t *p_par)
{
    return (p_h->state == HUMP2_FLAT) ? 0.0f : tilt_mm(p_h, p_par);
}

const char *hump2_state_name(hump2_state_t st)
{
    static const char * const NAME[4] = { "FLAT", "ASCENT", "CREST", "DESCENT" };

    return ((uint32_t)st < 4u) ? NAME[st] : "?";
}

uint32_t hump2_trace(int16_t *p_out, uint32_t max, uint32_t *p_rd)
{
    uint32_t wr = g_hump2.trace_n;       /* one read: the writer only adds */
    uint32_t n  = wr - *p_rd;
    uint32_t i;

    if (n > max)
    {
        n = max;                             /* the frame was late: keep the newest */
    }
    if (n > (HUMP2_TRACE_LEN - 2u))
    {
        n = HUMP2_TRACE_LEN - 2u;        /* margin against the writer */
    }
    for (i = 0u; i < n; i++)
    {
        p_out[i] = g_hump2.trace[(wr - n + i) % HUMP2_TRACE_LEN];
    }
    *p_rd = wr;
    return n;
}

void hump2_diag(hump2_diag_t *p_d)
{
    const hump2_t *p_h = &g_hump2;

    /* diagnostics only: a step in between may mix two cycles */
    p_d->pitch_deg    = p_h->pitch_deg;
    p_d->p_hi         = p_h->b_range_reset ? p_h->pitch_deg : p_h->p_hi;
    p_d->p_lo         = p_h->b_range_reset ? p_h->pitch_deg : p_h->p_lo;
    p_d->theta0_deg   = p_h->theta0_deg;
    p_d->state        = (uint8_t)p_h->state;
    p_d->h_mm         = (p_h->state == HUMP2_FLAT) ? 0.0f : p_h->h_mm;
    p_d->peak_mm      = (p_h->state == HUMP2_FLAT) ? 0.0f : p_h->peak_mm;
    p_d->tilt_mm      = hump2_live_tilt_mm(p_h, &g_hump2_par);
    p_d->last_h_mm    = p_h->last_h_mm;
    p_d->last_tilt_mm = p_h->last_tilt_mm;
    p_d->max_h_mm     = p_h->max_h_mm;
    p_d->max_tilt_mm  = p_h->max_tilt_mm;
    p_d->n_valid      = p_h->n_valid;
    p_d->n_rejected   = p_h->n_rejected;
    g_hump2.b_range_reset = true;
}
