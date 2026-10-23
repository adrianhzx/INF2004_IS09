/** @file app_safety.c
 *  @brief STOP_LATCH / stop_cause / fault_mask implementation.
 *
 *  Concurrency: g_stop_latch is written with a single aligned byte store
 *  (ARMv6-M single-copy atomic). The safety word and the fault mask are
 *  multi-writer and change only inside app_hw_irq_save()/restore(), which is
 *  PRIMASK on the single-core demo image and an SIO spinlock + PRIMASK on an
 *  SMP build (ledger section 2, A3).
 */

#include "app_safety.h"

#include "app_hw.h"
#include "app_ipc.h"
#include "app_port.h"

volatile uint8_t  g_stop_latch          = APP_STOP_LATCH_CLEAR;
volatile uint16_t g_fault_mask          = 0U;
volatile uint32_t g_safety_helper_calls = 0U;

/** safety_word: bits 0-3 stop_cause (app_data.h). */
static volatile uint8_t g_safety_word = 0U;

static int32_t  g_ctrl_tid      = 0;
static int32_t  g_flg_ctrl_id   = 0;
static uint32_t g_evt_safe_stop = 0U;

void
app_safety_init (void)
{
    g_stop_latch    = APP_STOP_LATCH_CLEAR;
    g_safety_word   = 0U;
    g_fault_mask    = 0U;
    g_ctrl_tid      = 0;
    g_flg_ctrl_id   = 0;
    g_evt_safe_stop = 0U;
}

void
app_safety_bind (int32_t ctrl_tid, int32_t flg_ctrl_id,
                 uint32_t evt_safe_stop_bit)
{
    g_ctrl_tid      = ctrl_tid;
    g_flg_ctrl_id   = flg_ctrl_id;
    g_evt_safe_stop = evt_safe_stop_bit;
}

/* ---- helpers ------------------------------------------------------------- */

/* Single-core target: mask with an inline CPSID so the whole masked window
   (save, RMW, restore) is a handful of SRAM-resident instructions with no call
   into flash. SMP and host builds keep the app_hw_irq_save() path. */
#if !defined(APP_HOST_TEST) && !TK_SUPPORT_SMP
static inline uint32_t
helper_irq_save (void)
{
    uint32_t primask;
    __asm volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask) : : "memory");
    return primask;
}

static inline void
helper_irq_restore (uint32_t primask)
{
    __asm volatile("msr primask, %0" : : "r"(primask) : "memory");
}
#else
static inline uint32_t
helper_irq_save (void)
{
    return app_hw_irq_save();
}

static inline void
helper_irq_restore (uint32_t saved)
{
    app_hw_irq_restore(saved);
}
#endif

APP_RAMFUNC void
app_shared_bits_set (volatile uint16_t * p_word, uint16_t bits)
{
    uint32_t const saved = helper_irq_save();
    *p_word              = (uint16_t) (*p_word | bits);
    g_safety_helper_calls++;
    helper_irq_restore(saved);
}

APP_RAMFUNC void
app_shared_bits_clr (volatile uint16_t * p_word, uint16_t bits)
{
    uint32_t const saved = helper_irq_save();
    *p_word              = (uint16_t) (*p_word & (uint16_t) ~bits);
    g_safety_helper_calls++;
    helper_irq_restore(saved);
}

APP_RAMFUNC void
app_shared_bits_set8 (volatile uint8_t * p_byte, uint8_t bits)
{
    uint32_t const saved = helper_irq_save();
    *p_byte              = (uint8_t) (*p_byte | bits);
    g_safety_helper_calls++;
    helper_irq_restore(saved);
}

APP_RAMFUNC void
app_shared_bits_clr8 (volatile uint8_t * p_byte, uint8_t bits)
{
    uint32_t const saved = helper_irq_save();
    *p_byte              = (uint8_t) (*p_byte & (uint8_t) ~bits);
    g_safety_helper_calls++;
    helper_irq_restore(saved);
}

/* ---- stop path ----------------------------------------------------------- */

void
app_safe_stop_request (app_stop_cause_t cause)
{
    uint32_t saved = 0U;
    bool     first = false;

    if ((cause <= APP_STOP_CAUSE_NONE) || (cause >= APP_STOP_CAUSE_COUNT))
    {
        cause = APP_STOP_CAUSE_CRITICAL_RUNTIME; /* never a silent no-op */
    }
#if !APP_ADOPT_REMOTE_STOP_CAUSE
    if (APP_STOP_CAUSE_REMOTE_STOP == cause)
    {
        cause = APP_STOP_CAUSE_OPERATOR_STOP; /* D-01 default mapping */
    }
#endif

    /* Step 1: latch first, plain byte store, before any other work. */
    g_stop_latch = APP_STOP_LATCH_SET;

    /* Step 2: first cause wins (write-if-NONE as one RMW under the helper). */
    saved = app_hw_irq_save();
    if (0U == (g_safety_word & APP_SAFETY_CAUSE_MASK))
    {
        g_safety_word =
            (uint8_t) (((uint32_t) g_safety_word & ~APP_SAFETY_CAUSE_MASK) |
                       ((uint32_t) cause & APP_SAFETY_CAUSE_MASK));
        first = true;
    }
    app_hw_irq_restore(saved);

    /* Step 3: signal the controller. ISR-legal (ledger section 2). Only the
       first request raises the telemetry event, so a burst of causes does
       not flood the event buffer. */
    if (0 != g_flg_ctrl_id)
    {
        (void) tk_set_flg((ID) g_flg_ctrl_id, (UINT) g_evt_safe_stop);
    }
    if (first)
    {
        app_ipc_telem_post(APP_TEV_SAFE_STOP, APP_SRC_SHARED, (uint16_t) cause);
    }
}

app_stop_cause_t
app_stop_cause_get (void)
{
    return (app_stop_cause_t) (g_safety_word & APP_SAFETY_CAUSE_MASK);
}

bool
app_safety_controller_clear (void)
{
    uint32_t saved   = 0U;
    bool     cleared = false;

    if ((0 == g_ctrl_tid) || ((int32_t) tk_get_tid() != g_ctrl_tid))
    {
        return false; /* fail closed: only the bound controller task */
    }
    saved = app_hw_irq_save();
    if (APP_STOP_LATCH_SET == g_stop_latch)
    {
        g_safety_word =
            (uint8_t) (g_safety_word & (uint8_t) ~APP_SAFETY_CAUSE_MASK);
        g_stop_latch = APP_STOP_LATCH_CLEAR; /* cause cleared before latch */
        cleared      = true;
    }
    app_hw_irq_restore(saved);
    if (cleared)
    {
        app_ipc_telem_post(APP_TEV_RECOVERY, APP_SRC_CONTROLLER, 0U);
    }
    return cleared;
}

/* ---- fault mask ---------------------------------------------------------- */

void
app_fault_set (uint16_t bits)
{
    uint16_t const before = g_fault_mask;
    app_shared_bits_set(&g_fault_mask, bits);
    if (0U != (bits & (uint16_t) ~before))
    {
        app_ipc_telem_post(APP_TEV_FAULT_SET, APP_SRC_SHARED, bits);
    }
}

void
app_fault_clr (uint16_t bits)
{
    uint16_t const before = g_fault_mask;
    app_shared_bits_clr(&g_fault_mask, bits);
    if (0U != (bits & before))
    {
        app_ipc_telem_post(APP_TEV_FAULT_CLR, APP_SRC_SHARED, bits);
    }
}

uint16_t
app_fault_get (void)
{
    return g_fault_mask; /* one 16-bit load */
}
