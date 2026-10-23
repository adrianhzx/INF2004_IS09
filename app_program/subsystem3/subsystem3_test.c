/** @file subsystem3_test.c
 *  @brief On-target tests for subsystem 3 (make TEST=1). IDs S3-U01..S3-U06.
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

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
