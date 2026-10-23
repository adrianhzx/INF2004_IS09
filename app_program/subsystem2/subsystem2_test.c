/** @file subsystem2_test.c
 *  @brief On-target tests for subsystem 2 (make TEST=1). Test IDs S2-U01..
 *         Logged in testing/subsystem2/TEST_LOG.md.
 *
 *  SAFETY: no test drives a motor unless APP_CAL_MOTION_ARMED is 1 (HV-3/
 *  HV-4 signed). With it 0 the energising checks print NOT RUN.
 */

#include "subsystem2.h"

#if APP_TEST

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_motion.h"
#include "../shared/app_safety.h"
#include "../test_report.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "motion_core.h"

#define S2_TEST_SETTLE_MS 50U
#define S2_TEST_MOVE_TIMEOUT_MS 5000U
#define S2_TEST_MOVE_MM 200
#define S2_TEST_SPEED 20U
#define S2_TEST_MAX_CYCLE_US 1000U

static void
sim_in (motion_in_t * p_in)
{
    p_in->stop_latched    = false;
    p_in->motion_armed    = true; /* simulation only: no pin is touched */
    p_in->ticks_l         = 0U;
    p_in->ticks_r         = 0U;
    p_in->interval_l_32us = 0U;
    p_in->interval_r_32us = 0U;
    p_in->glitches        = 0U;
    p_in->has_cmd         = false;
    p_in->cmd_record      = 0U;
    p_in->line_word       = 0U;
    p_in->ctrl_heartbeat  = 0U;
}

EXPORT void
subsystem2_test (void)
{
    uint16_t cc_a   = 0U;
    uint16_t cc_b   = 0U;
    uint32_t count0 = 0U;
    uint32_t count1 = 0U;
    uint32_t edges0 = 0U;

    TEST_INIT();

    /* S2-U01: driver brought up in BRAKE: both compares 0 on both slices. */
    drv_motor_get_compare(DRV_MOTOR_LEFT, &cc_a, &cc_b);
    TEST_CHECK("S2-U01a", (0U == cc_a) && (0U == cc_b),
               "left motor compares 0 (brake) after init");
    drv_motor_get_compare(DRV_MOTOR_RIGHT, &cc_a, &cc_b);
    TEST_CHECK("S2-U01b", (0U == cc_a) && (0U == cc_b),
               "right motor compares 0 (brake) after init");
    TEST_CHECK("S2-U01c", 6249U == drv_motor_top(),
               "PWM wrap 6249 = 20 kHz at 125 MHz");

    /* S2-U02: task runs every 10 ms and never exceeds 1 ms of work. */
    count0 = s2_motion_cycle_count();
    (void) tk_dly_tsk(1000);
    count1 = s2_motion_cycle_count();
    TEST_CHECK("S2-U02a",
               ((count1 - count0) >= 95U) && ((count1 - count0) <= 105U),
               "MotionControlTask 100 +-5 cycles per second");
    tm_printf((UB *) "[TEST] S2-U02b INFO  longest motion cycle = %u us\n",
              (unsigned) s2_motion_max_cycle_us());
    TEST_CHECK("S2-U02b", s2_motion_max_cycle_us() < S2_TEST_MAX_CYCLE_US,
               "cycle work < 1 ms");

    /* S2-U03: encoders attached, quiet at rest. */
    edges0 = drv_encoder_edge_count();
    (void) tk_dly_tsk(200);
    TEST_CHECK("S2-U03a", drv_encoder_edge_count() == edges0,
               "no encoder edges while stationary");
    TEST_CHECK("S2-U03b", 0U == drv_encoder_glitches(),
               "no quadrature glitches at rest");

    /* S2-U04: a drive command while disarmed is rejected and nothing moves. */
    {
        app_motion_cmd_t    cmd;
        uint32_t            rec = 0U;
        app_motion_status_t st;
        cmd.type       = (uint8_t) APP_MC_MOVE;
        cmd.speed_cm_s = (uint8_t) S2_TEST_SPEED;
        cmd.param      = S2_TEST_MOVE_MM;
        cmd.until_line = false;
        cmd.seq        = 9U;
        (void) app_motion_cmd_pack(&cmd, &rec);
        (void) tk_snd_mbf(mbf_motion_cmd, &rec, (INT) APP_MC_RECORD_BYTES,
                          TMO_POL);
        (void) tk_dly_tsk(S2_TEST_SETTLE_MS);
        (void) app_motion_read(&st);
        drv_motor_get_compare(DRV_MOTOR_LEFT, &cc_a, &cc_b);
#if APP_CAL_MOTION_ARMED
        tm_printf((UB *) "[TEST] S2-U04 NOT RUN (motion armed; rejection path "
                         "needs disarmed build)\n");
        (void) st;
#else
        TEST_CHECK("S2-U04",
                   (APP_MOTION_BRAKED == st.motion_state) && (0U == cc_a) &&
                       (0U == cc_b),
                   "MOVE rejected while disarmed: BRAKED, compares 0");
#endif
    }

    /* S2-U05: core on the ARM toolchain - stop latency and odometry signs. */
    {
        motion_core_t    core;
        motion_in_t      in;
        motion_out_t     out;
        app_motion_cmd_t cmd;
        uint32_t         i = 0U;

        motion_core_init(&core);
        sim_in(&in);
        cmd.type       = (uint8_t) APP_MC_MOVE;
        cmd.speed_cm_s = (uint8_t) S2_TEST_SPEED;
        cmd.param      = -S2_TEST_MOVE_MM;
        cmd.until_line = false;
        cmd.seq        = 3U;
        (void) app_motion_cmd_pack(&cmd, &in.cmd_record);
        in.has_cmd = true;
        motion_core_step(&core, &in, &out);
        TEST_CHECK("S2-U05a",
                   (APP_MOTION_MOVING == out.status.motion_state) &&
                       (out.pwm_l_pct < 0) && (out.pwm_r_pct < 0),
                   "reverse MOVE accepted: both PWM negative");
        in.has_cmd = false;
        for (i = 0U; i < 5U; i++)
        {
            in.ctrl_heartbeat++;
            in.ticks_l--; /* wheels turning backwards, 1 tick per cycle */
            in.ticks_r--;
            in.interval_l_32us = 300U;
            in.interval_r_32us = 300U;
            motion_core_step(&core, &in, &out);
        }
        TEST_CHECK("S2-U05b",
                   (out.status.distance_mm < 0) && out.status.dir_l_reverse,
                   "signed reverse distance and direction bits");
        in.stop_latched = true;
        motion_core_step(&core, &in, &out);
        TEST_CHECK(
            "S2-U05c",
            (0 == out.pwm_l_pct) && (0 == out.pwm_r_pct) && out.flush_queue &&
                (APP_MOTION_STOP_LATCHED == out.status.motion_state) &&
                out.done_event &&
                (APP_MOTION_RESULT_ABORTED == out.status.done_result),
            "latch -> brake, flush, STOP_LATCHED, ABORTED in the same cycle");
        in.has_cmd = true; /* stale command after the stop */
        motion_core_step(&core, &in, &out);
        TEST_CHECK("S2-U05d", out.reject_event && (0 == out.pwm_l_pct),
                   "command after latch rejected");
    }

    /* S2-U06: energising check through the real task - ONLY when armed. */
#if APP_CAL_MOTION_ARMED
    {
        app_motion_cmd_t    cmd;
        app_motion_status_t st;
        uint32_t            rec    = 0U;
        uint32_t            waited = 0U;
        uint32_t const      e0     = drv_encoder_edge_count();
        cmd.type                   = (uint8_t) APP_MC_MOVE;
        cmd.speed_cm_s             = (uint8_t) S2_TEST_SPEED;
        cmd.param                  = S2_TEST_MOVE_MM;
        cmd.until_line             = false;
        cmd.seq                    = 11U;
        (void) app_motion_cmd_pack(&cmd, &rec);
        (void) tk_snd_mbf(mbf_motion_cmd, &rec, (INT) APP_MC_RECORD_BYTES,
                          TMO_POL);
        do
        {
            (void) tk_dly_tsk(S2_TEST_SETTLE_MS);
            waited += S2_TEST_SETTLE_MS;
            (void) app_motion_read(&st);
        } while ((st.done_seq != 11U) && (waited < S2_TEST_MOVE_TIMEOUT_MS));
        tm_printf((UB *) "[TEST] S2-U06 INFO  MOVE %d mm @ %u cm/s: result=%u "
                         "dist=%d mm edges=%u in %u ms\n",
                  S2_TEST_MOVE_MM, (unsigned) S2_TEST_SPEED, st.done_result,
                  st.distance_mm, (unsigned) (drv_encoder_edge_count() - e0),
                  (unsigned) waited);
        TEST_CHECK("S2-U06",
                   (11U == st.done_seq) &&
                       (APP_MOTION_RESULT_OK == st.done_result),
                   "MOVE completed through the task (on blocks, HV-4)");
    }
#else
    tm_printf((UB *) "[TEST] S2-U06 NOT RUN (APP_CAL_MOTION_ARMED = 0; see "
                     "HV-3/HV-4)\n");
#endif

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
