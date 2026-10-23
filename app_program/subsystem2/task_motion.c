/** @file task_motion.c
 *  @brief MotionControlTask (priority 2, 10 ms): the only writer of GP8-GP11
 *         and of the MotionStatus words. Owner: Buddy 2.
 *
 *  Cycle (woken by CYC_10MS slot MOTION):
 *   1. read g_stop_latch (one byte load) and the ISR words,
 *   2. dequeue at most one MotionCmd record (TMO_POL),
 *   3. motion_core_step(),
 *   4. apply PWM (brake first when latched), flush the queue if asked,
 *   5. call SafeStop / set faults for a core-detected hazard,
 *   6. publish MotionStatus words (one aligned store each) and events.
 *  Nothing in the cycle waits: no mutex, no printf, no network.
 */

#include "subsystem2.h"

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_ipc.h"
#include "../shared/app_line.h"
#include "../shared/app_motion.h"
#include "../shared/app_port.h"
#include "../shared/app_safety.h"
#include "../shared/vehicle_controller.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "motion_core.h"

#define S2_FLUSH_MAX 8U /* bounded drain of a 4-deep queue */

static motion_core_t     g_s2_core;
static volatile uint32_t g_s2_cycles       = 0U;
static volatile uint32_t g_s2_max_cycle_us = 0U;
static volatile uint32_t g_s2_latch_cycles =
    0U; /* cycles entered with latch set */

static void
flush_queue (void)
{
    uint32_t rec = 0U;
    uint32_t n   = 0U;
    while ((n < S2_FLUSH_MAX) &&
           (tk_rcv_mbf(mbf_motion_cmd, &rec, TMO_POL) > 0))
    {
        n++;
    }
}

static void
apply_outputs (motion_out_t const * p_out)
{
    if ((0 == p_out->pwm_l_pct) && (0 == p_out->pwm_r_pct))
    {
        drv_motor_brake_all();
    }
    else
    {
        drv_motor_set(DRV_MOTOR_LEFT, p_out->pwm_l_pct);
        drv_motor_set(DRV_MOTOR_RIGHT, p_out->pwm_r_pct);
    }
}

static void
publish (motion_out_t const * p_out)
{
    uint32_t word_a = 0U;
    uint32_t word_b = 0U;

    (void) app_motion_pack(&p_out->status, &word_a, &word_b);
    g_motion_status_a = word_a; /* one aligned store each (rule 3) */
    g_motion_status_b = word_b;

    if (p_out->done_event)
    {
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_MOTION_DONE);
        app_ipc_telem_post(
            APP_TEV_MOTION_DONE, APP_SRC_MOTION,
            (uint16_t) (((uint16_t) p_out->status.done_seq << 8U) |
                        p_out->status.done_result));
    }
    if (p_out->reject_event)
    {
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(),
                          (UINT) APP_EVT_MOTION_REJECT);
        app_ipc_telem_post(APP_TEV_MOTION_REJECT, APP_SRC_MOTION,
                           (uint16_t) ((uint16_t) p_out->reject_seq << 8U));
    }
}

void
s2_motion_cycle (void)
{
    motion_in_t    in;
    motion_out_t   out;
    uint32_t       rec   = 0U;
    uint32_t const t0    = app_hw_time_us();
    bool const     latch = app_stop_latched(); /* FIRST: one byte load */

    if (latch)
    {
        drv_motor_brake_all(); /* brake before anything else is computed */
        g_s2_latch_cycles++;
    }

    in.stop_latched    = latch;
    in.motion_armed    = (0 != APP_CAL_MOTION_ARMED);
    in.ticks_l         = g_enc_ticks_l;
    in.ticks_r         = g_enc_ticks_r;
    in.interval_l_32us = drv_encoder_interval_l_32us();
    in.interval_r_32us = drv_encoder_interval_r_32us();
    in.glitches        = drv_encoder_glitches();
    in.line_word       = g_line_state;
    in.ctrl_heartbeat  = g_ctrl_heartbeat;
    in.has_cmd         = false;
    in.cmd_record      = 0U;
    if (!latch && (tk_rcv_mbf(mbf_motion_cmd, &rec, TMO_POL) > 0))
    {
        in.has_cmd    = true;
        in.cmd_record = rec;
    }

    motion_core_step(&g_s2_core, &in, &out);

    apply_outputs(&out);
    if (out.flush_queue)
    {
        flush_queue();
    }
    if (APP_STOP_CAUSE_NONE != out.stop_request)
    {
        app_safe_stop_request(
            out.stop_request); /* latch + cause + EVT_SAFE_STOP */
    }
    if (0U != out.fault_set)
    {
        app_fault_set(out.fault_set);
    }
    publish(&out);

    g_s2_cycles++;
    {
        uint32_t const dt = (uint32_t) (app_hw_time_us() - t0);
        if (dt > g_s2_max_cycle_us)
        {
            g_s2_max_cycle_us = dt;
        }
    }
}

void
s2_motion_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    motion_core_init(&g_s2_core);
    drv_motor_brake_all();

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR); /* woken by CYC_10MS */
        (void) tk_can_wup(
            TSK_SELF); /* never "catch up" with stacked wake-ups */
        s2_motion_cycle();
    }
}

uint32_t
s2_motion_cycle_count (void)
{
    return g_s2_cycles;
}

uint32_t
s2_motion_max_cycle_us (void)
{
    return g_s2_max_cycle_us;
}

uint32_t
s2_motion_latch_cycles (void)
{
    return g_s2_latch_cycles;
}

uint8_t
s2_motion_state (void)
{
    return g_s2_core.state;
}
