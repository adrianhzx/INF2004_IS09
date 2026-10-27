/** @file task_barcode.c
 *  @brief BarcodeEdgeIsr (GP7, both edges, through the shared dispatcher) and
 *         BarcodeDecodeTask (priority 7, event-driven). Owner: Buddy 3.
 *
 *  ISR: one timestamp difference, one ring push, one flag set. Nothing else.
 *  Task: waits on FLG_BARCODE with the quiet-zone timeout while capturing,
 *  drains EdgeRing into barcode_core, publishes the result byte, raises
 *  EVT_BARCODE and posts a telemetry event. ARM / DISARM come from the
 *  controller (Phase 7); the task starts DISARMED with the pin IRQ off.
 */

#include "subsystem3.h"

#include "../shared/app_barcode.h"
#include "../shared/app_cal.h"
#include "../shared/app_gpio_irq.h"
#include "../shared/app_hw.h"
#include "../shared/app_ipc.h"
#include "../shared/app_port.h"
#include "../shared/app_safety.h"
#include "barcode_core.h"

#define S3_BC_DRAIN_MAX APP_EDGE_RING_ENTRIES

static barcode_core_t    g_s3_bc;
static volatile uint32_t g_s3_bc_last_edge_us = 0U; /* ISR-private */
static volatile uint32_t g_s3_bc_edges        = 0U;
static volatile uint32_t g_s3_bc_results      = 0U;
static bool              g_s3_bc_attached     = false;

/* Task-independent context (dispatcher callback). */
static void
barcode_edge_isr (uint32_t pin, uint32_t events, uint32_t stamp_us)
{
    uint32_t const delta = (uint32_t) (stamp_us - g_s3_bc_last_edge_us);
    bool const     level = app_hw_gpio_read(APP_PIN_BARCODE_DOUT);
    bool           sat   = false;

    (void) pin;
    (void) events;
    g_s3_bc_last_edge_us = stamp_us;
    g_s3_bc_edges++;
    (void) app_edge_ring_push(&g_edge_ring, app_edge_pack(level, delta, &sat));
    (void) tk_set_flg(flg_barcode, (UINT) APP_BC_EDGE_AVAIL); /* ISR-legal */
}

static void
publish_result (barcode_out_t const * p_out)
{
    if (p_out->overflow_fault)
    {
        app_fault_set(APP_FAULT_BARCODE_OVERFLOW); /* diagnostic only */
        app_edge_ring_reset(&g_edge_ring);
    }
    if (p_out->result_ready)
    {
        g_barcode_byte =
            app_barcode_pack(p_out->result, p_out->seq); /* one byte store */
        g_s3_bc_results++;
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_BARCODE);
        app_ipc_telem_post(
            APP_TEV_BARCODE, APP_SRC_BARCODE,
            (uint16_t) (((uint16_t) p_out->result << 8U) | p_out->seq));
    }
}

static void
drain_ring (void)
{
    uint16_t      entry = 0U;
    uint32_t      n     = 0U;
    barcode_out_t out;

    while ((n < S3_BC_DRAIN_MAX) && app_edge_ring_pop(&g_edge_ring, &entry))
    {
        n++;
        barcode_core_edge(&g_s3_bc, app_edge_delta_us(entry), &out);
        publish_result(&out);
    }
    if (0U != g_edge_ring.overflow)
    {
        g_edge_ring.overflow = 0U; /* consumer clears after noticing */
        barcode_core_quiet(&g_s3_bc, true, &out);
        publish_result(&out);
    }
}

void
s3_barcode_arm (bool arm)
{
    if (!g_s3_bc_attached)
    {
        return;
    }
    if (arm)
    {
        app_edge_ring_reset(&g_edge_ring);
        g_s3_bc_last_edge_us = app_hw_time_us();
        barcode_core_arm(&g_s3_bc);
        app_gpio_irq_enable(APP_PIN_BARCODE_DOUT);
    }
    else
    {
        app_gpio_irq_disable(APP_PIN_BARCODE_DOUT);
        barcode_core_disarm(&g_s3_bc);
        app_edge_ring_reset(&g_edge_ring);
    }
}

bool
s3_barcode_init_isr (void)
{
    app_hw_gpio_input(APP_PIN_BARCODE_DOUT, APP_GPIO_PULL_UP);
    g_s3_bc_attached = app_gpio_irq_attach(
        APP_PIN_BARCODE_DOUT, APP_GPIO_IRQ_EDGE_BOTH, barcode_edge_isr);
    if (g_s3_bc_attached)
    {
        app_gpio_irq_disable(
            APP_PIN_BARCODE_DOUT); /* DISARMED until the controller arms */
    }
    return g_s3_bc_attached;
}

void
s3_barcode_task (INT stacd, void * p_exinf)
{
    UINT          ptn = 0U;
    ER            er  = E_OK;
    barcode_out_t out;

    (void) stacd;
    (void) p_exinf;

    barcode_core_init(&g_s3_bc);
    for (;;)
    {
        TMO const tmo = ((uint8_t) BC_CAPTURING == g_s3_bc.state)
                            ? (TMO) APP_CAL_T_QUIET_MS
                            : TMO_FEVR;

        er = tk_wai_flg(flg_barcode,
                        (UINT) (APP_BC_EDGE_AVAIL | APP_BC_ARM | APP_BC_DISARM),
                        TWF_ORW | TWF_BITCLR, &ptn, tmo);
        if (E_TMOUT == er)
        {
            barcode_core_quiet(&g_s3_bc, false, &out); /* quiet zone: decode */
            publish_result(&out);
            continue;
        }
        if (er < E_OK)
        {
            continue;
        }
        if (0U != (ptn & APP_BC_DISARM))
        {
            s3_barcode_arm(false);
        }
        if (0U != (ptn & APP_BC_ARM))
        {
            s3_barcode_arm(true);
        }
        if (0U != (ptn & APP_BC_EDGE_AVAIL))
        {
            drain_ring();
        }
    }
}

uint32_t
s3_barcode_edge_count (void)
{
    return g_s3_bc_edges;
}

uint32_t
s3_barcode_result_count (void)
{
    return g_s3_bc_results;
}

uint8_t
s3_barcode_state (void)
{
    return g_s3_bc.state;
}

#if defined(APP_TEST) && APP_TEST
bool
s3_barcode_dbg_last (uint16_t * p_width, uint8_t * p_count, uint32_t * p_seq)
{
    uint32_t const s1 = g_s3_bc.dbg_seq;
    uint32_t       k  = 0U;
    uint8_t const  n  = g_s3_bc.dbg_count;

    if (0U != (s1 & 1U))
    {
        return false; /* being written */
    }
    for (k = 0U; (k < n) && (k < BC_MAX_ELEMENTS); k++)
    {
        p_width[k] = g_s3_bc.dbg_width[k];
    }
    *p_count = n;
    *p_seq   = s1;
    return s1 == g_s3_bc.dbg_seq;
}

void
s3_barcode_dbg_stats (uint32_t * p_decoded, uint32_t * p_invalid,
                      uint32_t * p_stray)
{
    *p_decoded = g_s3_bc.stats_decoded;
    *p_invalid = g_s3_bc.stats_invalid;
    *p_stray   = g_s3_bc.stats_stray;
}
#endif
