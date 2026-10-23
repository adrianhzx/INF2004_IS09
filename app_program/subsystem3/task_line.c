/** @file task_line.c
 *  @brief LineSensingTask (priority 4, 10 ms): reads ADC0/ADC1, runs
 *         line_core, publishes the LineState word and the line events.
 *         Only writer of g_line_state. Owner: Buddy 3.
 */

#include "subsystem3.h"

#include "../shared/app_hw.h"
#include "../shared/app_ipc.h"
#include "../shared/app_line.h"
#include "../shared/app_port.h"
#include "../shared/app_safety.h"
#include "drv_line_adc.h"
#include "line_core.h"

static line_core_t       g_s3_line;
static volatile uint32_t g_s3_line_cycles    = 0U;
static volatile uint32_t g_s3_line_max_us    = 0U;
static volatile uint16_t g_s3_raw_left       = 0U; /* diagnostics only */
static volatile uint16_t g_s3_raw_centre     = 0U;
static volatile uint8_t  g_s3_n_left         = 0U;
static volatile uint8_t  g_s3_n_centre       = 0U;
static bool              g_s3_contrast_fault = false;

void
s3_line_cycle (void)
{
    uint32_t const t0   = app_hw_time_us();
    uint16_t       left = g_s3_raw_left; /* hold last sample on a read error */
    uint16_t       cent = g_s3_raw_centre;
    line_out_t     out;
    uint32_t       word = 0U;
    UINT           evt  = 0U;

    (void) drv_line_adc_read(DRV_LINE_ADC_LEFT, &left);
    (void) drv_line_adc_read(DRV_LINE_ADC_CENTRE, &cent);
    g_s3_raw_left   = left;
    g_s3_raw_centre = cent;

    line_core_step(&g_s3_line, left, cent, &out);
    g_s3_n_left   = out.n_left;
    g_s3_n_centre = out.n_centre;

    (void) app_line_pack(&out.state, &word);
    g_line_state = word; /* one aligned store (rule 3) */

    if (out.evt_junction)
    {
        evt |= (UINT) APP_EVT_JUNCTION;
    }
    if (out.evt_lost)
    {
        evt |= (UINT) APP_EVT_LINE_LOST;
    }
    if (out.evt_found)
    {
        evt |= (UINT) APP_EVT_LINE_FOUND;
    }
    if (0U != evt)
    {
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), evt);
        app_ipc_telem_post(APP_TEV_LINE_EVENT, APP_SRC_LINE,
                           (uint16_t) out.state.event);
    }
    if (!out.contrast_ok && !g_s3_contrast_fault)
    {
        g_s3_contrast_fault = true;
        app_fault_set(APP_FAULT_IR_CONTRAST); /* diagnostic: never a stop */
    }
    else if (out.contrast_ok && g_s3_contrast_fault)
    {
        g_s3_contrast_fault = false;
        app_fault_clr(APP_FAULT_IR_CONTRAST);
    }
    else
    {
        /* unchanged */
    }

    g_s3_line_cycles++;
    {
        uint32_t const dt = (uint32_t) (app_hw_time_us() - t0);
        if (dt > g_s3_line_max_us)
        {
            g_s3_line_max_us = dt;
        }
    }
}

void
s3_line_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    line_core_init(&g_s3_line);
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR); /* woken by CYC_10MS slot LINE */
        (void) tk_can_wup(TSK_SELF);
        s3_line_cycle();
    }
}

void
s3_line_set_references (uint16_t white_l, uint16_t black_l, uint16_t white_c,
                        uint16_t black_c)
{
    line_core_set_references(&g_s3_line, white_l, black_l, white_c, black_c);
}

uint32_t
s3_line_cycle_count (void)
{
    return g_s3_line_cycles;
}

uint32_t
s3_line_max_cycle_us (void)
{
    return g_s3_line_max_us;
}

void
s3_line_raw (uint16_t * p_left, uint16_t * p_centre, uint8_t * p_n_left,
             uint8_t * p_n_centre)
{
    *p_left     = g_s3_raw_left;
    *p_centre   = g_s3_raw_centre;
    *p_n_left   = g_s3_n_left;
    *p_n_centre = g_s3_n_centre;
}
