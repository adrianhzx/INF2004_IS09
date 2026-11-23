/**
 * @file    line_meas.c
 * @brief   Black seen by each line sensor, in mm (see line_meas.h).
 */
#include "line_meas.h"

line_meas_t g_line_meas[2];

void line_meas_reset(line_meas_t *p_m)
{
    p_m->b_in         = false;
    p_m->b_black_prev = false;
    p_m->b_rose       = false;
    p_m->b_new_black  = false;
    p_m->cur.dark_mm  = 0.0f;
    p_m->cur.black_mm = 0.0f;
    p_m->cur.rise_mm  = 0.0f;
    p_m->cur.peak     = 0u;
    p_m->last         = p_m->cur;
    p_m->n            = 0u;
}

void line_meas_step(line_meas_t *p_m, uint8_t n, bool b_black, float ds_mm, uint8_t light_pct)
{
    if (ds_mm < 0.0f)
    {
        ds_mm = -ds_mm;
    }
    p_m->b_new_black = b_black && !p_m->b_black_prev;
    p_m->b_black_prev = b_black;

    if ((n > light_pct) || b_black)
    {
        if (!p_m->b_in)
        {
            p_m->b_in         = true;        /* a new mark */
            p_m->b_rose       = false;
            p_m->cur.dark_mm  = 0.0f;
            p_m->cur.black_mm = 0.0f;
            p_m->cur.rise_mm  = 0.0f;
            p_m->cur.peak     = 0u;
        }
        p_m->cur.dark_mm += ds_mm;
        if (b_black)
        {
            p_m->cur.black_mm += ds_mm;
            p_m->b_rose = true;
        }
        if (n > p_m->cur.peak)
        {
            p_m->cur.peak = n;
            if (!p_m->b_rose)
            {
                p_m->cur.rise_mm = p_m->cur.dark_mm;   /* to the peak, while not black */
            }
        }
        if (p_m->b_new_black)
        {
            p_m->cur.rise_mm = p_m->cur.dark_mm;       /* to the moment it turned black */
        }
    }
    else if (p_m->b_in)
    {
        p_m->b_in = false;                   /* the mark has ended */
        p_m->last = p_m->cur;
        p_m->n++;
    }
    else
    {
        /* white */
    }
}
