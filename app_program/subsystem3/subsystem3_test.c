/** @file subsystem3_test.c
 *  @brief On-target tests for subsystem 3 (make TEST=1). IDs S3-U01..S3-U08.
 *         Logged in testing/subsystem3/TEST_LOG.md. No sensor is needed: the
 *         barcode path is exercised by forcing GP7 interrupts with the
 *         measured Code 39 timing, which proves ISR -> EdgeRing -> task ->
 *         decoder -> EVT_BARCODE on the real kernel.
 */

#include "subsystem3.h"

#if APP_TEST

#include "../shared/app_barcode.h"
#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_line.h"
#include "../test_report.h"
#include "barcode_core.h"
#include "drv_line_adc.h"

#define S3_T_NARROW_US 3000U /* 3 mm at 1 m/s equivalent: fast but legal */
#define S3_T_WIDE_US 9000U
#define S3_T_QUIET_WAIT (APP_CAL_T_QUIET_MS + 60U)
#define S3_T_PREQUIET_MS 200U
#define S3_MAX_CYCLE_US 300U
/* S3-U07 live line + barcode loop: 1 = print live readings forever after
   S3-U06 (the rest of the test run never runs); set 0 before committing. */
#define S3_T_LIVE_LOOP 0
#define S3_T_LIVE_GAP_MS 250U /* 4 lines per second */
/* S3-U08 live BARCODE loop: 1 = after S3-U06, show every barcode pass in
   detail (element count, widths, narrow/wide pattern, result) forever.
   Takes priority over S3_T_LIVE_LOOP. Set 0 before committing. */
#define S3_T_BC_LOOP 0
#define S3_T_BC_TICK_MS 100U
#define S3_T_BC_STATUS_TICKS 10U /* status line every 1 s */
#define S3_T_BC_PER_ROW 15U
#define S3_T_US_PER_UNIT 8U

static void
force_edge (void)
{
    app_hw_gpio_irq_force(APP_PIN_BARCODE_DOUT, APP_GPIO_IRQ_EDGE_RISE, true);
    /* the dispatcher runs here (level 1 IRQ) before the next statement */
    app_hw_gpio_irq_force(APP_PIN_BARCODE_DOUT, APP_GPIO_IRQ_EDGE_RISE, false);
}

/** Play one Code 39 symbol (as '*' X '*' with narrow gaps) by forcing edges. */
static void
play_symbol (char const * p_middle)
{
    char const * start = "nwnnwnwnn";
    char         seq[APP_CAL_BC_ELEMENTS];
    uint32_t     i = 0U;

    for (i = 0U; i < 9U; i++)
    {
        seq[i]       = start[i];
        seq[10U + i] = p_middle[i];
        seq[20U + i] = start[i];
    }
    seq[9U]  = 'n';
    seq[19U] = 'n';

    (void) tk_dly_tsk(S3_T_PREQUIET_MS); /* quiet zone before the first bar */
    force_edge();                        /* first bar starts */
    for (i = 0U; i < APP_CAL_BC_ELEMENTS; i++)
    {
        app_hw_delay_us(('w' == seq[i]) ? S3_T_WIDE_US : S3_T_NARROW_US);
        force_edge(); /* element i ends */
    }
}

#if S3_T_LIVE_LOOP && !S3_T_BC_LOOP
static char const * const s3t_evt_name[4] = {"-", "JUNCTION", "LOST", "FOUND"};
static char const * const s3t_bc_name[7]  = {
    "NONE",  "LEFT",         "RIGHT",       "STRAIGHT",
    "UTURN", "INV_NO_MATCH", "INV_OVERFLOW"};
static char const * const s3t_bc_state[4] = {"DISARMED", "WAIT_QUIET",
                                             "CAPTURING", "DECODING"};

/** S3-U07: print live line-sensor readings and every barcode decode,
 *  forever. Move the car (or a black line) under the IR sensors; pass a
 *  Code 39 barcode under the barcode sensor. The decoder stays armed. */
static void
s3t_live_loop (void)
{
    uint32_t         i      = 0U;
    uint32_t         res0   = 0U;
    uint16_t         raw_l  = 0U;
    uint16_t         raw_c  = 0U;
    uint8_t          n_l    = 0U;
    uint8_t          n_c    = 0U;
    uint8_t          r      = 0U;
    uint8_t          bstate = 0U;
    app_line_state_t ls;

    tm_printf((UB *) "[TEST] S3-U07 INFO  live line + barcode LOOP (never "
                     "ends; unplug to stop). Move a black line under the IR "
                     "sensors, pass a barcode under GP7\n");
    (void) tk_set_flg(flg_barcode, (UINT) APP_BC_ARM);
    res0 = s3_barcode_result_count();
    for (i = 0U;; i++)
    {
        (void) tk_dly_tsk(S3_T_LIVE_GAP_MS);
        s3_line_raw(&raw_l, &raw_c, &n_l, &n_c);
        app_line_read(&ls);
        bstate = s3_barcode_state();
        tm_printf((UB *) "[TEST] S3-U07 #%04u  raw L=%4u C=%4u  norm L=%3u "
                         "C=%3u  err=%4d steer=%4d  %s %-8s",
                  (unsigned) i, (unsigned) raw_l, (unsigned) raw_c,
                  (unsigned) n_l, (unsigned) n_c, (int) ls.line_error,
                  (int) ls.steer, ls.on_line ? "ON-LINE " : "off-line",
                  s3t_evt_name[ls.event & 3U]);
        tm_printf((UB *) "  | GP7=%u bc=%s edges=%u\n",
                  (unsigned) app_hw_gpio_read(APP_PIN_BARCODE_DOUT),
                  s3t_bc_state[bstate & 3U],
                  (unsigned) s3_barcode_edge_count());
        if (s3_barcode_result_count() != res0)
        {
            res0 = s3_barcode_result_count();
            r    = (uint8_t) app_barcode_result(g_barcode_byte);
            tm_printf((UB *) "[TEST] S3-U07 BARCODE  >>> %s <<<  (result #%u, "
                             "seq %u)\n",
                      (r < 7U) ? s3t_bc_name[r] : "?", (unsigned) res0,
                      (unsigned) app_barcode_seq(g_barcode_byte));
        }
    }
}
#endif

#if S3_T_BC_LOOP
static char const * const s3t_bc_res_name[7] = {
    "NONE",  "LEFT",         "RIGHT",       "STRAIGHT",
    "UTURN", "INV_NO_MATCH", "INV_OVERFLOW"};
static char const * const s3t_bc_st_name[4] = {"DISARMED", "WAIT_QUIET",
                                               "CAPTURING", "DECODING"};

/** Print one width (8 us units) as milliseconds with one decimal. */
static void
s3t_print_ms (uint16_t units)
{
    uint32_t const us   = (uint32_t) units * S3_T_US_PER_UNIT;
    uint32_t const ms10 = (us + 50U) / 100U;
    tm_printf((UB *) " %u.%u", (unsigned) (ms10 / 10U),
              (unsigned) (ms10 % 10U));
}

/** Explain one captured pass: count, widths, n/w pattern, decode. */
static void
s3t_report_pass (uint32_t pass, uint16_t const * p_w, uint8_t n)
{
    uint32_t             k     = 0U;
    uint32_t             w_min = 0xFFFFU;
    uint32_t             w_max = 0U;
    uint32_t             thr   = 0U;
    uint32_t             r10   = 0U;
    bool                 rev   = false;
    app_barcode_result_t res   = APP_BC_INV_NO_MATCH;

    tm_printf((UB *) "[TEST] S3-U08 PASS #%u: %u elements (a barcode is 29)\n",
              (unsigned) pass, (unsigned) n);
    if (0U == n)
    {
        return;
    }
    for (k = 0U; k < n; k++)
    {
        w_min = (p_w[k] < w_min) ? p_w[k] : w_min;
        w_max = (p_w[k] > w_max) ? p_w[k] : w_max;
    }
    tm_printf((UB *) "[TEST] S3-U08   widths ms (bar,space,bar,...):");
    for (k = 0U; k < n; k++)
    {
        if ((0U != k) && (0U == (k % S3_T_BC_PER_ROW)))
        {
            tm_printf((UB *) "\n[TEST] S3-U08                              ");
        }
        s3t_print_ms(p_w[k]);
    }
    tm_printf((UB *) "\n");
    thr = (w_min + w_max) / 2U;
    r10 = (0U != w_min) ? ((w_max * 10U) / w_min) : 0U;
    tm_printf((UB *) "[TEST] S3-U08   narrowest");
    s3t_print_ms((uint16_t) w_min);
    tm_printf((UB *) " ms, widest");
    s3t_print_ms((uint16_t) w_max);
    tm_printf((UB *) " ms, ratio %u.%u (needs 1.5-6.0, printed 3.0)\n",
              (unsigned) (r10 / 10U), (unsigned) (r10 % 10U));
    tm_printf((UB *) "[TEST] S3-U08   pattern: ");
    for (k = 0U; k < n; k++)
    {
        tm_printf((UB *) "%c", (p_w[k] > thr) ? 'w' : 'n');
        if ((8U == k) || (9U == k) || (18U == k) || (19U == k))
        {
            tm_printf((UB *) " ");
        }
    }
    tm_printf((UB *) "\n[TEST] S3-U08   expect *A*: nwnnwnwnn n wnnnnwnnw n "
                     "nwnnwnwnn\n");
    if (n < (uint8_t) APP_CAL_BC_N_MIN)
    {
        tm_printf((UB *) "[TEST] S3-U08   -> IGNORED: fewer than %u elements = "
                         "stray mark (partial pass, or sensor missed bars)\n",
                  (unsigned) APP_CAL_BC_N_MIN);
        return;
    }
    res = barcode_core_decode(p_w, n, &rev);
    tm_printf((UB *) "[TEST] S3-U08   -> %s%s\n",
              ((uint32_t) res < 7U) ? s3t_bc_res_name[res] : "?",
              rev ? " (read backwards, still fine)" : "");
    if (n > (uint8_t) APP_CAL_BC_ELEMENTS)
    {
        tm_printf((UB *) "[TEST] S3-U08   hint: TOO MANY elements -> sensor "
                         "flickers on bar edges: turn the pot, mount 5-10 mm "
                         "above the paper\n");
    }
    else if (n < (uint8_t) APP_CAL_BC_ELEMENTS)
    {
        tm_printf((UB *) "[TEST] S3-U08   hint: MISSING elements -> narrow "
                         "bars not seen or pass stopped/reversed: one smooth "
                         "steady pass, start and end on white\n");
    }
    else if ((r10 < 15U) || (r10 > 60U))
    {
        tm_printf((UB *) "[TEST] S3-U08   hint: no clear narrow/wide -> "
                         "uneven speed; keep the speed constant\n");
    }
    else
    {
        /* 29 elements and a usable ratio */
    }
}

/** S3-U08: live barcode diagnostics forever. Decoder stays armed. */
static void
s3t_barcode_loop (void)
{
    uint16_t w[BC_MAX_ELEMENTS];
    uint8_t  n     = 0U;
    uint32_t seq   = 0U;
    uint32_t last  = 0U;
    uint32_t pass  = 0U;
    uint32_t tick  = 0U;
    uint32_t e0    = 0U;
    uint32_t res0  = 0U;
    uint32_t dec   = 0U;
    uint32_t inv   = 0U;
    uint32_t stray = 0U;
    uint8_t  r     = 0U;

    tm_printf((UB *) "[TEST] S3-U08 INFO  barcode LOOP (never ends; unplug "
                     "to stop)\n");
    tm_printf((UB *) "[TEST] S3-U08 INFO  1) hold the sensor STILL over white, "
                     "then over a black bar: 'edges +0' both times, GP7 "
                     "different\n");
    tm_printf((UB *) "[TEST] S3-U08 INFO  2) one smooth steady pass over the "
                     "whole printed barcode, white before and after\n");
    (void) tk_set_flg(flg_barcode, (UINT) APP_BC_ARM);
    (void) tk_dly_tsk(20);
    (void) s3_barcode_dbg_last(w, &n, &last);
    e0   = s3_barcode_edge_count();
    res0 = s3_barcode_result_count();
    for (;;)
    {
        (void) tk_dly_tsk(S3_T_BC_TICK_MS);
        tick++;
        if (s3_barcode_dbg_last(w, &n, &seq) && (seq != last))
        {
            last = seq;
            pass++;
            s3t_report_pass(pass, w, n);
        }
        if (s3_barcode_result_count() != res0)
        {
            res0 = s3_barcode_result_count();
            r    = (uint8_t) app_barcode_result(g_barcode_byte);
            tm_printf((UB *) "[TEST] S3-U08 BARCODE  >>> %s <<<  (published "
                             "to the car, result #%u)\n",
                      (r < 7U) ? s3t_bc_res_name[r] : "?", (unsigned) res0);
        }
        if (0U == (tick % S3_T_BC_STATUS_TICKS))
        {
            s3_barcode_dbg_stats(&dec, &inv, &stray);
            tm_printf((UB *) "[TEST] S3-U08 status  GP7=%u  edges +%u in last "
                             "1 s (still sensor must be +0)  %s  decoded=%u "
                             "invalid=%u ignored=%u\n",
                      (unsigned) app_hw_gpio_read(APP_PIN_BARCODE_DOUT),
                      (unsigned) (s3_barcode_edge_count() - e0),
                      s3t_bc_st_name[s3_barcode_state() & 3U], (unsigned) dec,
                      (unsigned) inv, (unsigned) stray);
            e0 = s3_barcode_edge_count();
        }
    }
}
#endif

EXPORT void
subsystem3_test (void)
{
    uint16_t         raw_l = 0U;
    uint16_t         raw_c = 0U;
    uint32_t         c0    = 0U;
    uint32_t         c1    = 0U;
    app_line_state_t ls0;
    app_line_state_t ls1;

    TEST_INIT();

    /* S3-U01: ADC conversions succeed and are in range. */
    TEST_CHECK("S3-U01a",
               drv_line_adc_read(DRV_LINE_ADC_LEFT, &raw_l) &&
                   (raw_l <= APP_ADC_MAX),
               "ADC0 (GP26) converts, 12-bit range");
    TEST_CHECK("S3-U01b",
               drv_line_adc_read(DRV_LINE_ADC_CENTRE, &raw_c) &&
                   (raw_c <= APP_ADC_MAX),
               "ADC1 (GP27) converts, 12-bit range");
    tm_printf((UB *) "[TEST] S3-U01 INFO  raw left=%u centre=%u errors=%u\n",
              raw_l, raw_c, drv_line_adc_errors());

    /* S3-U02: LineSensingTask cadence and cost. */
    c0 = s3_line_cycle_count();
    (void) tk_dly_tsk(1000);
    c1 = s3_line_cycle_count();
    TEST_CHECK("S3-U02a", ((c1 - c0) >= 95U) && ((c1 - c0) <= 105U),
               "LineSensingTask 100 +-5 cycles/s");
    tm_printf((UB *) "[TEST] S3-U02b INFO  longest line cycle = %u us\n",
              (unsigned) s3_line_max_cycle_us());
    TEST_CHECK("S3-U02b", s3_line_max_cycle_us() < S3_MAX_CYCLE_US,
               "line cycle < 300 us");

    /* S3-U03: LineState freshness: seq advances every 10 ms (NFR3). */
    app_line_read(&ls0);
    (void) tk_dly_tsk(15);
    app_line_read(&ls1);
    TEST_CHECK("S3-U03", ls0.seq != ls1.seq,
               "LineState seq changes within 15 ms");

    /* S3-U04: disarmed: a forced GP7 edge produces no result. */
    c0 = s3_barcode_result_count();
    force_edge();
    (void) tk_dly_tsk(S3_T_QUIET_WAIT);
    TEST_CHECK("S3-U04",
               (c0 == s3_barcode_result_count()) &&
                   (BC_DISARMED == s3_barcode_state()),
               "disarmed decoder ignores edges");

    /* S3-U05: armed: forced Code 39 'A' decodes as LEFT through the real
       ISR -> EdgeRing -> task path; 'D' as UTURN; 'Z' as NO_MATCH. */
    (void) tk_set_flg(flg_barcode, (UINT) APP_BC_ARM);
    (void) tk_dly_tsk(20);
    TEST_CHECK("S3-U05a", BC_WAIT_QUIET == s3_barcode_state(),
               "ARM -> WAIT_QUIET");
    c0 = s3_barcode_result_count();
    play_symbol("wnnnnwnnw"); /* A */
    (void) tk_dly_tsk(S3_T_QUIET_WAIT);
    TEST_CHECK("S3-U05b",
               (s3_barcode_result_count() == c0 + 1U) &&
                   (APP_BC_LEFT == app_barcode_result(g_barcode_byte)),
               "forced 'A' -> LEFT published");
    play_symbol("nnnnwwnnw"); /* D */
    (void) tk_dly_tsk(S3_T_QUIET_WAIT);
    TEST_CHECK("S3-U05c", APP_BC_UTURN == app_barcode_result(g_barcode_byte),
               "forced 'D' -> UTURN");
    play_symbol("nwwnwnnnn"); /* Z (sample page 3) */
    (void) tk_dly_tsk(S3_T_QUIET_WAIT);
    TEST_CHECK("S3-U05d",
               APP_BC_INV_NO_MATCH == app_barcode_result(g_barcode_byte),
               "forced 'Z' -> INV_NO_MATCH (known symbol, not a command)");
    TEST_CHECK("S3-U05e", BC_WAIT_QUIET == s3_barcode_state(),
               "decoder back in WAIT_QUIET");
    tm_printf(
        (UB *) "[TEST] S3-U05 INFO  edges=%u results=%u ring overflow=%u\n",
        (unsigned) s3_barcode_edge_count(),
        (unsigned) s3_barcode_result_count(), (unsigned) g_edge_ring.overflow);

    /* S3-U06: DISARM stops capture. */
    (void) tk_set_flg(flg_barcode, (UINT) APP_BC_DISARM);
    (void) tk_dly_tsk(20);
    TEST_CHECK("S3-U06", BC_DISARMED == s3_barcode_state(),
               "DISARM -> DISARMED");

#if S3_T_BC_LOOP
    s3t_barcode_loop(); /* S3-U08: never returns */
#elif S3_T_LIVE_LOOP
    s3t_live_loop(); /* S3-U07: never returns */
#endif

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
