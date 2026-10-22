/** @file app_shared_test.c
 *  @brief On-target self-tests for the shared foundation (make TEST=1).
 *
 *  These are the A3 spikes from Rev B assumption A3 and the Phase 0 spike
 *  list, runnable without any external hardware: GPIO interrupts are forced
 *  through IO_BANK0 PROC0_INTF on a spare pin.
 *
 *  Test IDs: SH-U01 .. SH-U08 (testing/shared/TEST_LOG.md).
 *  NOTE: SH-U08 leaves the system latched in S7 on purpose; the TEST image
 *  is never the demo image.
 */

#include "app_port.h"

#if APP_TEST

#include "../test_report.h"
#include "app_data.h"
#include "app_gpio_irq.h"
#include "app_hw.h"
#include "app_imu.h"
#include "app_ipc.h"
#include "app_line.h"
#include "app_motion.h"
#include "app_safety.h"
#include "app_telemetry.h"
#include "vehicle_controller.h"

#define SH_TEST_PIN APP_PIN_SPARE_19
#define SH_RACE_ITERS 10000U
#define SH_BIT_TASK 0x0001U
#define SH_BIT_ISR 0x0002U

static volatile uint32_t g_sh_isr_hits     = 0U;
static volatile uint16_t g_sh_race_word    = 0U;
static volatile uint32_t g_sh_lost_bits    = 0U;
static volatile uint32_t g_sh_max_di_us    = 0U;
static volatile uint32_t g_sh_max_setup_us = 0U;
static ID                g_sh_flg          = 0;
static volatile uint8_t  g_sh_isr_mode = 0U; /* 0 count, 1 race, 2 safestop */

static void
sh_test_isr (uint32_t pin, uint32_t events, uint32_t stamp_us)
{
    (void) pin;
    (void) events;
    (void) stamp_us;
    app_hw_gpio_irq_force(SH_TEST_PIN, APP_GPIO_IRQ_EDGE_RISE,
                          false); /* release force */
    g_sh_isr_hits++;
    if (1U == g_sh_isr_mode)
    {
        app_shared_bits_set(&g_sh_race_word, SH_BIT_ISR);
    }
    else if (2U == g_sh_isr_mode)
    {
        app_safe_stop_request(APP_STOP_CAUSE_COLLISION);
    }
    else
    {
        (void) tk_set_flg(g_sh_flg,
                          0x1U); /* A3: tk_set_flg from a GPIO handler */
    }
}

void
app_shared_test (void)
{
    T_CFLG   cflg;
    UINT     ptn = 0U;
    uint32_t count0;
    uint32_t count1;
    uint32_t i;
    ER       er;

    TEST_INIT();

    /* SH-U01: the 10 ms cyclic handler runs ~100 times per second. */
    count0 = app_ipc_cyclic_count();
    (void) tk_dly_tsk(1000);
    count1 = app_ipc_cyclic_count();
    TEST_CHECK("SH-U01",
               ((count1 - count0) >= 95U) && ((count1 - count0) <= 105U),
               "cyclic handler 100 +-5 per second");

    /* SH-U02: microsecond timer advances (app_hw_init self-check repeated). */
    {
        uint32_t const t0 = app_hw_time_us();
        (void) tk_dly_tsk(10);
        TEST_CHECK("SH-U02", ((uint32_t) (app_hw_time_us() - t0)) >= 9000U,
                   "1 MHz timer advances >= 9 ms over a 10 ms delay");
    }

    /* SH-U03: tk_set_flg from a forced GPIO interrupt reaches a waiting task.
     */
    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_sh_flg     = tk_cre_flg(&cflg);
    app_hw_gpio_input(SH_TEST_PIN, APP_GPIO_PULL_DOWN);
    TEST_CHECK("SH-U03a", g_sh_flg > 0, "test flag created");
    TEST_CHECK(
        "SH-U03b",
        app_gpio_irq_attach(SH_TEST_PIN, APP_GPIO_IRQ_EDGE_RISE, sh_test_isr),
        "dispatcher attach on spare pin");
    TEST_CHECK(
        "SH-U03c",
        !app_gpio_irq_attach(SH_TEST_PIN, APP_GPIO_IRQ_EDGE_RISE, sh_test_isr),
        "second attach to the same pin is refused");
    g_sh_isr_mode = 0U;
    app_hw_gpio_irq_force(SH_TEST_PIN, APP_GPIO_IRQ_EDGE_RISE, true);
    er = tk_wai_flg(g_sh_flg, 0x1U, TWF_ORW | TWF_BITCLR, &ptn, 100);
    TEST_CHECK("SH-U03d", (E_OK == er) && (1U == g_sh_isr_hits),
               "flag set from GPIO ISR received within 100 ms");

    /* SH-U04: fault-mask race - ISR writer vs task helper, 10 000 iterations,
       no lost bit. Each iteration: pend the IRQ while masked, do the task RMW
       through the helper, unmask (ISR runs), check both bits, clear both. */
#if TK_SUPPORT_SMP
    /* The nested app_hw_irq_save() below would recurse on the non-recursive
       application spinlock. The race spike is specified for the single-core
       demo profile (ledger section 4). */
    tm_printf((UB *) "[TEST] SH-U04 NOT RUN (SMP=1 build; run on SMP=0)\n");
#else
    g_sh_isr_mode     = 1U;
    g_sh_lost_bits    = 0U;
    g_sh_max_di_us    = 0U;
    g_sh_max_setup_us = 0U;
    g_sh_isr_hits     = 0U;
    for (i = 0U; i < SH_RACE_ITERS; i++)
    {
        uint32_t const saved = app_hw_irq_save();
        uint32_t const t0    = app_hw_time_us();
        uint32_t       t1;
        app_hw_gpio_irq_force(SH_TEST_PIN, APP_GPIO_IRQ_EDGE_RISE, true);
        t1 = app_hw_time_us();
        app_shared_bits_set(&g_sh_race_word, SH_BIT_TASK);
        {
            uint32_t const t2 = app_hw_time_us();
            uint32_t const dh = (uint32_t) (t2 - t1); /* helper only */
            uint32_t const ds = (uint32_t) (t2 - t0); /* incl. test setup */
            if (dh > g_sh_max_di_us)
            {
                g_sh_max_di_us = dh;
            }
            if (ds > g_sh_max_setup_us)
            {
                g_sh_max_setup_us = ds;
            }
        }
        app_hw_irq_restore(saved);
        /* The ISR has now run (level 1, enabled). */
        if ((SH_BIT_TASK | SH_BIT_ISR) !=
            (g_sh_race_word & (SH_BIT_TASK | SH_BIT_ISR)))
        {
            g_sh_lost_bits++;
        }
        app_shared_bits_clr(&g_sh_race_word, SH_BIT_TASK | SH_BIT_ISR);
    }
    TEST_CHECK("SH-U04a", 0U == g_sh_lost_bits,
               "no lost fault bit over 10000 ISR/task RMW");
    TEST_CHECK("SH-U04b", SH_RACE_ITERS == g_sh_isr_hits,
               "ISR ran once per iteration");
    tm_printf((UB *) "[TEST] SH-U04c INFO  longest helper window = %u us; "
                     "incl. test force/timer setup = %u us\n",
              (unsigned) g_sh_max_di_us, (unsigned) g_sh_max_setup_us);
    TEST_CHECK("SH-U04c", g_sh_max_di_us <= 5U,
               "helper masked window <= 5 us (design: ~10 cycles, SRAM)");
#endif /* TK_SUPPORT_SMP */

    /* SH-U05: packed round trips on the target toolchain (mirror of host). */
    {
        app_line_state_t ls_in;
        app_line_state_t ls_out;
        uint32_t         word;
        bool             sat;
        ls_in.line_error = -100;
        ls_in.steer      = 100;
        ls_in.on_line    = true;
        ls_in.event      = APP_LINE_EVT_FOUND;
        ls_in.seq        = 15U;
        sat              = app_line_pack(&ls_in, &word);
        app_line_unpack(word, &ls_out);
        TEST_CHECK(
            "SH-U05a",
            !sat && (-100 == ls_out.line_error) && (100 == ls_out.steer) &&
                ls_out.on_line && (APP_LINE_EVT_FOUND == ls_out.event) &&
                (15U == ls_out.seq) && (0U == (word >> APP_LS_USED_BITS)),
            "LineState -100/+100 round trip, upper bits clear");
    }
    {
        app_motion_status_t ms_in;
        app_motion_status_t ms_out;
        uint32_t            wa;
        uint32_t            wb;
        ms_in.speed_l_cm_s  = 127U;
        ms_in.speed_r_cm_s  = 0U;
        ms_in.distance_mm   = -32768;
        ms_in.cycle_tag     = 3U;
        ms_in.motion_state  = APP_MOTION_STOP_LATCHED;
        ms_in.done_seq      = 9U;
        ms_in.active_seq    = 15U;
        ms_in.dir_l_reverse = true;
        ms_in.dir_r_reverse = false;
        ms_in.done_result   = APP_MOTION_RESULT_TIMEOUT;
        (void) app_motion_pack(&ms_in, &wa, &wb);
        app_motion_unpack(wa, wb, &ms_out);
        TEST_CHECK("SH-U05b",
                   (-32768 == ms_out.distance_mm) &&
                       (127U == ms_out.speed_l_cm_s) && ms_out.dir_l_reverse &&
                       !ms_out.dir_r_reverse &&
                       (APP_MOTION_STOP_LATCHED == ms_out.motion_state),
                   "MotionStatus signed distance min + flags round trip");
    }

    /* SH-U06: telemetry event post is non-blocking even when the buffer is
       absent/full: 100 posts complete in < 10 ms. */
    {
        uint32_t const t0 = app_hw_time_us();
        for (i = 0U; i < 100U; i++)
        {
            app_ipc_telem_post(APP_TEV_NONE, APP_SRC_SHARED, (uint16_t) i);
        }
        TEST_CHECK("SH-U06", ((uint32_t) (app_hw_time_us() - t0)) < 10000U,
                   "100 event posts without waiting (< 10 ms)");
    }

    /* SH-U07: controller refuses to clear the latch from a foreign task. */
    TEST_CHECK("SH-U07", !app_safety_controller_clear(),
               "latch clear refused from a non-controller task");

    /* SH-U08: SafeStop from an ISR: latch set, first cause kept, controller
       reaches S7 within two ticks. Leaves the system latched. */
    g_sh_isr_mode = 2U;
    app_hw_gpio_irq_force(SH_TEST_PIN, APP_GPIO_IRQ_EDGE_RISE, true);
    (void) tk_dly_tsk(5);
    TEST_CHECK("SH-U08a", app_stop_latched(), "STOP_LATCH set by ISR request");
    app_safe_stop_request(APP_STOP_CAUSE_STALL); /* second cause must not win */
    TEST_CHECK("SH-U08b", APP_STOP_CAUSE_COLLISION == app_stop_cause_get(),
               "first stop cause wins");
    (void) tk_dly_tsk(50);
    TEST_CHECK("SH-U08c", APP_SYS_S7_SAFE_STOP == vehicle_controller_state(),
               "controller in S7 within 50 ms of EVT_SAFE_STOP");

    TEST_SUMMARY("shared");
}

#endif /* APP_TEST */
