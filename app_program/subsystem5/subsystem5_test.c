/** @file subsystem5_test.c
 *  @brief On-target tests for subsystem 5 (make TEST=1). IDs S5-U01..S5-U08.
 *         Logged in testing/subsystem5/TEST_LOG.md. Safe with both hardware
 *         gates unsigned: the servo pin is not routed, TRIG never rises, and
 *         the echo path is exercised by forcing GP17 interrupts.
 */

#include "subsystem5.h"

#if APP_TEST

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_safety.h"
#include "../test_report.h"
#include "drv_servo.h"
#include "scan_core.h"

#define S5_T_ECHO_US 580U /* 10 cm */
#define S5_T_ECHO_TOL_US 60U
#define S5_T_SCAN_WAIT_MS 300U
#define S5_T_WATCH_MS 1000U
#define S5_T_TRIG_US 10U        /* HC-SR04 trigger pulse */
#define S5_T_LIVE_GAP_MS 200U   /* S5-U07: 5 readings per second */
#define S5_T_SWEEP_STEP 3U      /* S5-U08: 3 x 5 deg = 15 deg per step */
#define S5_T_SWEEP_HALF_IDX 12U /* S5-U08: +-60 deg around ahead */
#define S5_T_SWEEP_EXTRA_MS 80U /* S5-U08: margin over the settle time */
#define S5_T_SWEEP_START_MS 1000U
/* S5-U07 / S5-U08 live loops (never return): 1 = run them when the HV-5 /
   HV-7 gates are set; 0 = skip them so the run continues (needed for the
   integration run INT-13..INT-20 with the gates set). */
#define S5_T_LIVE_LOOPS 0

static void
force_echo (uint32_t width_us)
{
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_RISE, true);
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_RISE, false);
    app_hw_delay_us(width_us);
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_FALL, true);
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_FALL, false);
}

#if S5_T_LIVE_LOOPS && APP_CAL_ECHO_SHIFTER_VERIFIED
/** One real HC-SR04 ranging (TRIG pulse, wait for EchoIsr). Returns true
 *  and the echo width when a valid echo was measured. */
static bool
s5t_range (uint16_t * p_width_us)
{
    UINT ptn = 0U;
    ER   er  = E_OK;
    bool ok  = false;

    (void) tk_wai_flg(flg_scan, (UINT) APP_SCAN_ECHO_DONE, TWF_ORW | TWF_BITCLR,
                      &ptn, TMO_POL);
    g_echo_status = APP_ECHO_ARMED;
    app_hw_gpio_write(APP_PIN_US_TRIG, true);
    app_hw_delay_us(S5_T_TRIG_US);
    app_hw_gpio_write(APP_PIN_US_TRIG, false);
    er = tk_wai_flg(flg_scan, (UINT) APP_SCAN_ECHO_DONE, TWF_ORW | TWF_BITCLR,
                    &ptn, (TMO) (APP_CAL_T_ECHO_TIMEOUT_MS + 5U));
    if ((E_OK == er) && (APP_ECHO_DONE == g_echo_status))
    {
        *p_width_us = g_echo_width_us;
        ok          = true;
    }
    g_echo_status = APP_ECHO_IDLE;
    return ok;
}
#endif

#if S5_T_LIVE_LOOPS && APP_CAL_ECHO_SHIFTER_VERIFIED && !APP_CAL_SERVO_ARMED
/** S5-U07: sensor only, servo not needed. Prints distance 5 times/s. */
static void
s5t_live_range_loop (void)
{
    uint32_t i = 0U;
    uint16_t w = 0U;

    tm_printf((UB *) "[TEST] S5-U07 INFO  live ranging LOOP (never ends; "
                     "unplug to stop), move an object 5-100 cm in front of "
                     "the sensor\n");
    s5_echo_irq_enable(true);
    for (i = 0U;; i++)
    {
        if (s5t_range(&w))
        {
            tm_printf((UB *) "[TEST] S5-U07 #%04u  distance %3u cm  "
                             "(echo %5u us)\n",
                      (unsigned) i, (unsigned) app_obstacle_us_to_cm(w),
                      (unsigned) w);
        }
        else
        {
            tm_printf((UB *) "[TEST] S5-U07 #%04u  no echo (too close, "
                             "nothing in range, or wiring)\n",
                      (unsigned) i);
        }
        (void) tk_dly_tsk(S5_T_LIVE_GAP_MS);
    }
}
#endif

#if S5_T_LIVE_LOOPS && APP_CAL_SERVO_ARMED
/** S5-U08: servo sweep -60..+60 deg in 15 deg steps and back, forever.
 *  With the sensor gate set it also ranges at every angle (a "radar").
 *  Angles: 0 = ahead, positive = LEFT. */
static void
s5t_sweep_loop (void)
{
    uint32_t       idx  = SC_ANGLE_AHEAD;
    uint32_t       prev = SC_ANGLE_AHEAD;
    uint32_t       pass = 0U;
    int32_t        step = (int32_t) S5_T_SWEEP_STEP;
    uint16_t       w    = 0U;
    uint32_t const lo   = SC_ANGLE_AHEAD - S5_T_SWEEP_HALF_IDX;
    uint32_t const hi   = SC_ANGLE_AHEAD + S5_T_SWEEP_HALF_IDX;

    tm_printf((UB *) "[TEST] S5-U08 INFO  servo sweep LOOP -60..+60 deg "
                     "(never ends; unplug to stop). Pulse limits %u-%u us\n",
              (unsigned) APP_CAL_SERVO_MIN_US, (unsigned) APP_CAL_SERVO_MAX_US);
    drv_servo_set_us(scan_core_servo_us(SC_ANGLE_AHEAD));
    (void) tk_dly_tsk(S5_T_SWEEP_START_MS);
#if APP_CAL_ECHO_SHIFTER_VERIFIED
    s5_echo_irq_enable(true);
#endif
    idx = lo;
    for (;;)
    {
        int32_t const deg = ((int32_t) idx - (int32_t) SC_ANGLE_AHEAD) *
                            (int32_t) SC_DEG_PER_IDX;

        drv_servo_set_us(scan_core_servo_us(idx));
        (void) tk_dly_tsk(scan_core_settle_ms(prev, idx) + S5_T_SWEEP_EXTRA_MS);
        prev = idx;
#if APP_CAL_ECHO_SHIFTER_VERIFIED
        if (s5t_range(&w))
        {
            tm_printf((UB *) "[TEST] S5-U08 pass %u  angle %+4d deg  pulse "
                             "%4u us  distance %3u cm\n",
                      (unsigned) pass, (int) deg, (unsigned) drv_servo_get_us(),
                      (unsigned) app_obstacle_us_to_cm(w));
        }
        else
        {
            tm_printf((UB *) "[TEST] S5-U08 pass %u  angle %+4d deg  pulse "
                             "%4u us  no echo\n",
                      (unsigned) pass, (int) deg,
                      (unsigned) drv_servo_get_us());
        }
#else
        (void) w;
        tm_printf((UB *) "[TEST] S5-U08 pass %u  angle %+4d deg  pulse %4u "
                         "us (sensor gate off: no ranging)\n",
                  (unsigned) pass, (int) deg, (unsigned) drv_servo_get_us());
#endif
        if (((step > 0) && (idx >= hi)) || ((step < 0) && (idx <= lo)))
        {
            step = -step;
            pass++;
        }
        idx = (uint32_t) ((int32_t) idx + step);
    }
}
#endif

EXPORT void
subsystem5_test (void)
{
    UINT                   ptn   = 0U;
    uint32_t               c0    = 0U;
    uint32_t               fails = 0U;
    uint32_t               w0    = 0U;
    uint32_t               w1    = 0U;
    app_obstacle_profile_t p;

    TEST_INIT();

    /* S5-U01: servo slice configured, pin routed only when armed. */
    TEST_CHECK("S5-U01a",
               scan_core_servo_us(SC_ANGLE_AHEAD) == drv_servo_get_us(),
               "slice 7B compare = ahead pulse");
    TEST_CHECK("S5-U01b", (0 != APP_CAL_SERVO_ARMED) == drv_servo_pin_routed(),
               "GP15 routed to PWM only when APP_CAL_SERVO_ARMED");
    tm_printf(
        (UB *) "[TEST] S5-U01 INFO  servo pulse=%u us routed=%u enabled=%u\n",
        (unsigned) drv_servo_get_us(), (unsigned) drv_servo_pin_routed(),
        (unsigned) s5_scan_enabled());

    /* S5-U02: EchoIsr path with forced GP17 edges (no sensor needed). */
    g_echo_status = APP_ECHO_ARMED;
    s5_echo_irq_enable(true);
    force_echo(S5_T_ECHO_US);
    (void) tk_dly_tsk(2);
    TEST_CHECK("S5-U02a", APP_ECHO_DONE == g_echo_status,
               "forced rise/fall -> ECHO_DONE");
    TEST_CHECK("S5-U02b",
               (g_echo_width_us + S5_T_ECHO_TOL_US >= S5_T_ECHO_US) &&
                   (g_echo_width_us <= S5_T_ECHO_US + S5_T_ECHO_TOL_US),
               "width 580 +-60 us (10 cm)");
    TEST_CHECK("S5-U02c", 10U == app_obstacle_us_to_cm(g_echo_width_us),
               "us -> 10 cm");
    TEST_CHECK("S5-U02d",
               (E_OK == tk_wai_flg(flg_scan, (UINT) APP_SCAN_ECHO_DONE,
                                   TWF_ORW | TWF_BITCLR, &ptn, TMO_POL)),
               "ECHO_DONE set on FLG_SCAN");
    tm_printf((UB *) "[TEST] S5-U02 INFO  width=%u us\n",
              (unsigned) g_echo_width_us);
    /* a falling edge without a rising one is ignored */
    g_echo_status = APP_ECHO_IDLE;
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_FALL, true);
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_FALL, false);
    (void) tk_dly_tsk(2);
    TEST_CHECK("S5-U02e", APP_ECHO_IDLE == g_echo_status,
               "stray falling edge ignored");
    /* overrun */
    g_echo_status = APP_ECHO_ARMED;
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_RISE, true);
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_RISE, false);
    (void) tk_dly_tsk(35);
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_FALL, true);
    app_hw_gpio_irq_force(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_FALL, false);
    (void) tk_dly_tsk(2);
    TEST_CHECK("S5-U02f", APP_ECHO_OVERRUN == g_echo_status,
               "> 30 ms echo -> OVERRUN");
    (void) tk_wai_flg(flg_scan, (UINT) APP_SCAN_ECHO_DONE, TWF_ORW | TWF_BITCLR,
                      &ptn, TMO_POL);
    g_echo_status = APP_ECHO_IDLE;
    s5_echo_irq_enable(s5_scan_enabled());

    /* S5-U03: profile request answered within the budget. */
    fails = s5_scan_fail_count();
    c0    = s5_scan_done_count();
    (void) tk_set_flg(flg_scan, (UINT) APP_SCAN_REQ_PROFILE);
    (void) tk_dly_tsk(s5_scan_enabled() ? (APP_CAL_T_SCAN_TIMEOUT_MS + 200U)
                                        : S5_T_SCAN_WAIT_MS);
    TEST_CHECK("S5-U03a",
               (s5_scan_fail_count() + s5_scan_done_count()) ==
                   (fails + c0 + 1U),
               "one profile request -> exactly one DONE or FAIL");
    TEST_CHECK("S5-U03b", s5_profile_copy(&p, &w0, &w1),
               "profile copy under MTX_OBSTACLE");
    TEST_CHECK("S5-U03c",
               s5_scan_enabled()
                   ? ((uint8_t) SC_STATE_FINE == p.scan_state) ||
                         ((uint8_t) SC_STATE_FAIL == p.scan_state)
                   : ((uint8_t) SC_STATE_DISABLED == p.scan_state),
               "scan_state reflects the hardware gates");
    tm_printf((UB *) "[TEST] S5-U03 INFO  scan %u ms state=%u action=%u "
                     "width=%u closest=%u ymin=%d ymax=%d\n",
              (unsigned) s5_last_scan_ms(), p.scan_state, p.action, p.width_cm,
              p.closest_cm, p.y_min_cm, p.y_max_cm);
    TEST_CHECK("S5-U03d",
               s5_scan_enabled()
                   ? (s5_last_scan_ms() <= APP_CAL_T_SCAN_TIMEOUT_MS)
                   : true,
               "scan time within T_SCAN_TIMEOUT (NFR14 T_profile)");

    /* S5-U04: forward watch runs on the 100 ms tick and stops on request. */
    c0 = s5_watch_count();
    (void) tk_set_flg(flg_scan, (UINT) APP_SCAN_REQ_WATCH_ON);
    (void) tk_dly_tsk(S5_T_WATCH_MS);
    TEST_CHECK("S5-U04a",
               ((s5_watch_count() - c0) >= 8U) &&
                   ((s5_watch_count() - c0) <= 12U),
               "10 +-2 watch rangings per second");
    (void) tk_set_flg(flg_scan, (UINT) APP_SCAN_REQ_WATCH_OFF);
    (void) tk_dly_tsk(150);
    c0 = s5_watch_count();
    (void) tk_dly_tsk(300);
    TEST_CHECK("S5-U04b", c0 == s5_watch_count(), "WATCH_OFF stops ranging");

    /* S5-U05: TRIG never driven while the shifter is unverified. */
    TEST_CHECK("S5-U05",
               (0 != APP_CAL_ECHO_SHIFTER_VERIFIED) ||
                   !app_hw_gpio_read(APP_PIN_US_TRIG),
               "TRIG low while APP_CAL_ECHO_SHIFTER_VERIFIED == 0");

    /* S5-U06: no stop was latched by any of this. */
    TEST_CHECK("S5-U06", !app_stop_latched(),
               "scan subsystem never latches a stop by itself");

    /* S5-U07 / S5-U08: live hardware loops (never return). They run only
       once the matching gates are set, after HV-5 / HV-7 bench checks. */
#if S5_T_LIVE_LOOPS && APP_CAL_SERVO_ARMED
    s5t_sweep_loop();
#elif S5_T_LIVE_LOOPS && APP_CAL_ECHO_SHIFTER_VERIFIED
    s5t_live_range_loop();
#else
    tm_printf((UB *) "[TEST] S5-U07 INFO  skipped: live ranging needs "
                     "S5_T_LIVE_LOOPS = 1 and APP_CAL_ECHO_SHIFTER_VERIFIED "
                     "= 1 (HV-5)\n");
    tm_printf((UB *) "[TEST] S5-U08 INFO  skipped: servo sweep needs "
                     "S5_T_LIVE_LOOPS = 1 and APP_CAL_SERVO_ARMED = 1 "
                     "(HV-7)\n");
#endif

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
