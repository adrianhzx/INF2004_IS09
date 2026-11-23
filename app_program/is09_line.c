/**
 * @file    is09_line.c
 * @brief   IS09 analog line sensors -> the two black/white levels the base
 *          line follower expects (see is09_glue.h).
 *
 * IS09 line_core (Buddy 3) keeps one running white/black reference pair fed
 * by both sensors (D-34) and normalises each sample to darkness 0..100. The
 * base follower (line_logic.c) wants "this sensor sees the line": a sensor
 * turns black at dark_pct and back to white below dark_pct - hys_pct, so a
 * sensor sitting on the line's edge does not chatter. While the references
 * are too close together (line_core not calibrated) both read white, as in
 * IS09 (D-32).
 */
#include "is09_glue.h"
#include "is09/subsystem3/line_core.h"

static line_core_t      g_core;
static is09_line_diag_t g_diag;
static volatile bool    g_black_l;
static volatile bool    g_black_c;
static uint32_t         g_dark_l;        /* 20 Nov: LEFT's own black level, 0 = dark_pct */

void is09_line_set_left_dark(uint32_t dark_pct)
{
    g_dark_l = dark_pct;
}

void is09_line_init(void)
{
    line_core_init(&g_core);
    g_black_l = false;
    g_black_c = false;
    g_diag.raw_l = 0u;
    g_diag.raw_c = 0u;
    g_diag.n_l = 0u;
    g_diag.n_c = 0u;
    g_diag.ref_white = g_core.min_l;
    g_diag.ref_black = g_core.max_l;
    g_diag.calibrated = false;
}

static bool level(bool b_black, uint8_t n, uint32_t dark_pct, uint32_t hys_pct)
{
    uint32_t off = (dark_pct > hys_pct) ? (dark_pct - hys_pct) : 0u;

    if (b_black)
    {
        return (uint32_t)n >= off;
    }
    return (uint32_t)n >= dark_pct;
}

void is09_line_update(uint32_t adc_left, uint32_t adc_centre, uint32_t dark_pct, uint32_t hys_pct)
{
    line_out_t out;
    uint16_t   l = (uint16_t)((adc_left > 4095u) ? 4095u : adc_left);
    uint16_t   c = (uint16_t)((adc_centre > 4095u) ? 4095u : adc_centre);

    line_core_step(&g_core, l, c, &out);
    g_diag.raw_l      = l;
    g_diag.raw_c      = c;
    g_diag.n_l        = out.n_left;
    g_diag.n_c        = out.n_centre;
    g_diag.ref_white  = g_core.min_l;
    g_diag.ref_black  = g_core.max_l;
    g_diag.calibrated = g_core.calibrated;
    if (!g_core.calibrated)
    {
        g_black_l = false;                   /* D-32: no contrast, no line */
        g_black_c = false;
        return;
    }
    g_black_l = level(g_black_l, out.n_left, (g_dark_l > 0u) ? g_dark_l : dark_pct, hys_pct);
    g_black_c = level(g_black_c, out.n_centre, dark_pct, hys_pct);
}

bool is09_line_black_left(void)
{
    return g_black_l;
}

bool is09_line_black_right(void)
{
    return g_black_c;
}

void is09_line_diag(is09_line_diag_t *p_d)
{
    *p_d = g_diag;                           /* diagnostics: may mix two steps */
}
