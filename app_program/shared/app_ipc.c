/** @file app_ipc.c
 *  @brief FLG_CTRL, the 10 ms cyclic handler and the telemetry event post.
 */

#include "app_ipc.h"

#include "app_hw.h"
#include "app_pack.h"
#include "app_port.h"

#define APP_CYC_PERIOD_MS 10U
#define APP_CTRL_TICK_DIV 2U  /* 20 ms controller tick */
#define APP_SCAN_TICK_DIV 10U /* 100 ms forward-watch tick */

static int32_t           g_flg_ctrl   = 0;
static int32_t           g_cyc_10ms   = 0;
static int32_t           g_flg_scan   = 0;
static int32_t           g_mbf_telem  = 0;
static int32_t           g_mbf_motion = 0;
static volatile int32_t  g_cyc_tid[APP_CYC_SLOT_COUNT];
static volatile uint32_t g_cyc_count     = 0U;
static volatile uint32_t g_ms            = 0U;
static volatile uint8_t  g_telem_dropped = 0U;

/* Cyclic handler: task-independent context. Only tk_wup_tsk / tk_set_flg
   (both ISR-legal in this port, ledger section 2). Bounded work. */
static void
app_cyc_handler (void * p_exinf)
{
    uint32_t       slot  = 0U;
    uint32_t const count = g_cyc_count + 1U;

    (void) p_exinf;
    g_cyc_count = count;
    g_ms        = g_ms + APP_CYC_PERIOD_MS;

    for (slot = 0U; slot < (uint32_t) APP_CYC_SLOT_COUNT; slot++)
    {
        int32_t const tid = g_cyc_tid[slot];
        if (0 != tid)
        {
            (void) tk_wup_tsk((ID) tid);
        }
    }
    if ((0 != g_flg_ctrl) && (0U == (count % APP_CTRL_TICK_DIV)))
    {
        (void) tk_set_flg((ID) g_flg_ctrl, (UINT) APP_EVT_TICK);
    }
    if ((0 != g_flg_scan) && (0U == (count % APP_SCAN_TICK_DIV)))
    {
        (void) tk_set_flg((ID) g_flg_scan, (UINT) APP_SCAN_TICK);
    }
}

int32_t
app_ipc_init (void)
{
    T_CFLG   cflg;
    T_CCYC   ccyc;
    uint32_t slot = 0U;

    for (slot = 0U; slot < (uint32_t) APP_CYC_SLOT_COUNT; slot++)
    {
        g_cyc_tid[slot] = 0;
    }

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL; /* several bits, one waiting task */
    cflg.iflgptn = 0U;
    g_flg_ctrl   = (int32_t) tk_cre_flg(&cflg);
    if (g_flg_ctrl < E_OK)
    {
        return g_flg_ctrl;
    }

    ccyc.exinf  = NULL;
    ccyc.cycatr = TA_HLNG; /* created stopped; app_ipc_start() starts it */
    ccyc.cychdr = (FP) app_cyc_handler;
    ccyc.cyctim = APP_CYC_PERIOD_MS;
    ccyc.cycphs = APP_CYC_PERIOD_MS;
    g_cyc_10ms  = (int32_t) tk_cre_cyc(&ccyc);
    if (g_cyc_10ms < E_OK)
    {
        return g_cyc_10ms;
    }
    return E_OK;
}

int32_t
app_ipc_start (void)
{
    if (g_cyc_10ms <= 0)
    {
        return E_OBJ;
    }
    return (int32_t) tk_sta_cyc((ID) g_cyc_10ms);
}

void
app_ipc_register_cyclic_task (app_cyc_slot_t slot, int32_t tid)
{
    if ((uint32_t) slot < (uint32_t) APP_CYC_SLOT_COUNT)
    {
        g_cyc_tid[slot] = tid;
    }
}

void
app_ipc_register_scan_flag (int32_t flg_id)
{
    g_flg_scan = flg_id;
}

void
app_ipc_register_telem_mbf (int32_t mbf_id)
{
    g_mbf_telem = mbf_id;
}

void
app_ipc_register_motion_mbf (int32_t mbf_id)
{
    g_mbf_motion = mbf_id;
}

int32_t
app_ipc_mbf_motion (void)
{
    return g_mbf_motion;
}

int32_t
app_ipc_flg_ctrl (void)
{
    return g_flg_ctrl;
}

int32_t
app_ipc_flg_scan (void)
{
    return g_flg_scan;
}

uint32_t
app_ipc_ms (void)
{
    return g_ms;
}

uint32_t
app_ipc_cyclic_count (void)
{
    return g_cyc_count;
}

uint32_t
app_ipc_telem_pack (app_telem_event_t type, app_telem_source_t source,
                    uint16_t payload, uint8_t stamp_ms)
{
    uint32_t word = 0U;
    word          = app_pack_set(word, APP_TE_TYPE_SHIFT, APP_TE_TYPE_BITS,
                                 (uint32_t) type);
    word          = app_pack_set(word, APP_TE_SOURCE_SHIFT, APP_TE_SOURCE_BITS,
                                 (uint32_t) source);
    word =
        app_pack_set(word, APP_TE_PAYLOAD_SHIFT, APP_TE_PAYLOAD_BITS, payload);
    word = app_pack_set(word, APP_TE_STAMP_SHIFT, APP_TE_STAMP_BITS, stamp_ms);
    return word;
}

void
app_ipc_telem_unpack (uint32_t record, uint8_t * p_type, uint8_t * p_source,
                      uint16_t * p_payload, uint8_t * p_stamp)
{
    *p_type =
        (uint8_t) app_pack_get(record, APP_TE_TYPE_SHIFT, APP_TE_TYPE_BITS);
    *p_source =
        (uint8_t) app_pack_get(record, APP_TE_SOURCE_SHIFT, APP_TE_SOURCE_BITS);
    *p_payload = (uint16_t) app_pack_get(record, APP_TE_PAYLOAD_SHIFT,
                                         APP_TE_PAYLOAD_BITS);
    *p_stamp =
        (uint8_t) app_pack_get(record, APP_TE_STAMP_SHIFT, APP_TE_STAMP_BITS);
}

void
app_ipc_telem_post (app_telem_event_t type, app_telem_source_t source,
                    uint16_t payload)
{
    /* 8-bit ms stamp: ms since boot modulo 256 (D-05 documents the wrap). */
    uint8_t const  stamp  = (uint8_t) ((app_hw_time_us() / 1000U) & 0xFFU);
    uint32_t const record = app_ipc_telem_pack(type, source, payload, stamp);
    ER             er     = E_OBJ;

    if (0 != g_mbf_telem)
    {
        er = tk_snd_mbf((ID) g_mbf_telem, &record, (INT) APP_TE_RECORD_BYTES,
                        TMO_POL);
    }
    if (er < E_OK)
    {
        uint32_t const saved = app_hw_irq_save();
        g_telem_dropped      = app_pack_inc_sat8(g_telem_dropped);
        app_hw_irq_restore(saved);
    }
}

uint8_t
app_ipc_telem_dropped (void)
{
    return g_telem_dropped;
}
