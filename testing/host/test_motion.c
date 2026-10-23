/** @file test_motion.c
 *  @brief Host tests for subsystem 2: motion_core state machine, odometry,
 *         control, guards; quadrature decoder table. Test IDs H-MO-*.
 *         A tiny first-order car model turns PWM into encoder ticks so the
 *         closed loop can be exercised without hardware.
 */

#include <stdio.h>
#include <string.h>

#include "app_cal.h"
#include "app_line.h"
#include "app_motion.h"
#include "drv_encoder.h"
#include "host_port.h"
#include "motion_core.h"

/* NOLINTBEGIN(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, desc)                                                  \
    do                                                                         \
    {                                                                          \
        if (cond)                                                              \
        {                                                                      \
            g_pass++;                                                          \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            g_fail++;                                                          \
            printf("FAIL %s %s (%s:%d)\n", id, desc, __FILE__, __LINE__);      \
        }                                                                      \
    } while (0)

/* ---- simulated car ------------------------------------------------------ */
typedef struct
{
    int32_t  vel_l_cm_s_q8; /* current wheel speeds, Q8 */
    int32_t  vel_r_cm_s_q8;
    int32_t  pos_l_um; /* wheel path positions */
    int32_t  pos_r_um;
    int32_t  emitted_l; /* ticks emitted so far (signed) */
    int32_t  emitted_r;
    uint16_t ticks_l; /* the ISR counters */
    uint16_t ticks_r;
    uint16_t interval_l;
    uint16_t interval_r;
    bool     stalled; /* wheels blocked */
} sim_car_t;

#define SIM_GAIN_CM_S_PER_PCT_Q8                                               \
    ((int32_t) (0.4 * 256)) /* 100 % PWM -> 40 cm/s */
#define SIM_TAU_CYCLES 5    /* first-order lag */

static void
sim_init (sim_car_t * p_car, uint16_t ticks_start)
{
    memset(p_car, 0, sizeof *p_car);
    p_car->ticks_l = ticks_start;
    p_car->ticks_r = ticks_start;
}

static void
sim_wheel (int8_t pwm, int32_t * p_vel_q8, int32_t * p_pos_um,
           int32_t * p_emitted, uint16_t * p_ticks, uint16_t * p_interval,
           bool stalled)
{
    int32_t const target_q8 =
        stalled ? 0 : (int32_t) pwm * SIM_GAIN_CM_S_PER_PCT_Q8;
    int32_t um_per_tick = (int32_t) motion_core_um_per_tick();
    int32_t want        = 0;

    *p_vel_q8 += (target_q8 - *p_vel_q8) / SIM_TAU_CYCLES;
    /* 10 ms of travel: cm/s * 10 ms = cm/100 -> um: vel_q8 * 100 / 256 */
    *p_pos_um += (*p_vel_q8 * 100) / 256;
    want = *p_pos_um / um_per_tick;
    while (*p_emitted < want)
    {
        (*p_emitted)++;
        (*p_ticks)++;
    }
    while (*p_emitted > want)
    {
        (*p_emitted)--;
        (*p_ticks)--;
    }
    if (0 != *p_vel_q8)
    {
        /* interval us = um_per_tick / (cm/s * 10 um/ms ... ) =
         * um_per_tick*100/(cm_s) us */
        int32_t const v           = (*p_vel_q8 < 0) ? -*p_vel_q8 : *p_vel_q8;
        int32_t const interval_us = (um_per_tick * 100 * 256) / (v > 0 ? v : 1);
        int32_t       i32         = interval_us / 32;
        *p_interval               = (uint16_t) ((i32 > 65535) ? 65535 : i32);
    }
}

static void
sim_step (sim_car_t * p_car, motion_out_t const * p_out)
{
    sim_wheel(p_out->pwm_l_pct, &p_car->vel_l_cm_s_q8, &p_car->pos_l_um,
              &p_car->emitted_l, &p_car->ticks_l, &p_car->interval_l,
              p_car->stalled);
    sim_wheel(p_out->pwm_r_pct, &p_car->vel_r_cm_s_q8, &p_car->pos_r_um,
              &p_car->emitted_r, &p_car->ticks_r, &p_car->interval_r,
              p_car->stalled);
}

static void
fill_in (motion_in_t * p_in, sim_car_t const * p_car, uint8_t heartbeat,
         uint32_t line_word)
{
    p_in->stop_latched    = false;
    p_in->motion_armed    = true;
    p_in->ticks_l         = p_car->ticks_l;
    p_in->ticks_r         = p_car->ticks_r;
    p_in->interval_l_32us = p_car->interval_l;
    p_in->interval_r_32us = p_car->interval_r;
    p_in->glitches        = 0U;
    p_in->has_cmd         = false;
    p_in->cmd_record      = 0U;
    p_in->line_word       = line_word;
    p_in->ctrl_heartbeat  = heartbeat;
}

static uint32_t
make_cmd (app_motion_cmd_type_t type, uint8_t speed, int16_t param,
          bool until_line, uint8_t seq)
{
    app_motion_cmd_t cmd;
    uint32_t         rec = 0U;
    cmd.type             = (uint8_t) type;
    cmd.speed_cm_s       = speed;
    cmd.param            = param;
    cmd.until_line       = until_line;
    cmd.seq              = seq;
    (void) app_motion_cmd_pack(&cmd, &rec);
    return rec;
}

static uint32_t
make_line (int8_t steer, bool on_line, uint8_t seq)
{
    app_line_state_t ls;
    uint32_t         w = 0U;
    ls.line_error      = steer;
    ls.steer           = steer;
    ls.on_line         = on_line;
    ls.event           = APP_LINE_EVT_NONE;
    ls.seq             = seq;
    (void) app_line_pack(&ls, &w);
    return w;
}

/* Run cycles until done_event or max; the line seq and heartbeat advance. */
static uint32_t
run_until_done (motion_core_t * p_core, sim_car_t * p_car, motion_out_t * p_out,
                uint32_t max_cycles, int8_t steer, bool on_line,
                bool freeze_line, bool freeze_hb)
{
    motion_in_t in;
    uint32_t    n   = 0U;
    uint8_t     hb  = 0U;
    uint8_t     lsq = 0U;
    for (n = 0U; n < max_cycles; n++)
    {
        if (!freeze_hb)
        {
            if (0U == (n % 2U))
            {
                hb++;
            }
        }
        if (!freeze_line)
        {
            lsq = (uint8_t) ((lsq + 1U) & 0x0FU);
        }
        fill_in(&in, p_car, hb, make_line(steer, on_line, lsq));
        motion_core_step(p_core, &in, p_out);
        sim_step(p_car, p_out);
        if (p_out->done_event || (APP_STOP_CAUSE_NONE != p_out->stop_request))
        {
            return n + 1U;
        }
    }
    return n;
}

/* ---- tests -----------------------------------------------------------------
 */

static void
test_geometry (void)
{
    CHECK("H-MO-01", motion_core_um_per_tick() > 0U, "um per tick positive");
    /* 65 mm wheel, 20 ticks: 204.2 mm / 20 = 10.2 mm */
    CHECK("H-MO-02",
          (motion_core_um_per_tick() > 10000U) &&
              (motion_core_um_per_tick() < 10400U),
          "um per tick ~10.2 mm for the default geometry");
    /* 90 deg pivot: pi*140/4 = 110 mm -> ~10 ticks */
    CHECK("H-MO-03",
          (motion_core_turn_ticks(900U) >= 9U) &&
              (motion_core_turn_ticks(900U) <= 12U),
          "90 deg pivot ~10-11 ticks");
    CHECK("H-MO-04",
          motion_core_turn_ticks(1800U) == 2U * motion_core_turn_ticks(900U) ||
              motion_core_turn_ticks(1800U) ==
                  (2U * motion_core_turn_ticks(900U)) + 1U,
          "180 deg is twice 90 deg");
    CHECK("H-MO-05", 0U == motion_core_speed_from_interval(0U),
          "no interval -> 0 speed");
    /* 20 cm/s: interval = 10210 um / 200 um/ms = 51 ms = 1595 x 32 us */
    CHECK("H-MO-06",
          (motion_core_speed_from_interval(1595U) >= 19U) &&
              (motion_core_speed_from_interval(1595U) <= 21U),
          "51 ms interval ~20 cm/s");
    CHECK("H-MO-07", 127U == motion_core_speed_from_interval(1U),
          "tiny interval saturates at 127");
}

static void
test_decoder (void)
{
    bool          glitch = false;
    uint8_t       ab     = 0U;
    int32_t       sum    = 0;
    uint32_t      i      = 0U;
    uint8_t const fwd[4] = {0U, 1U, 3U, 2U}; /* gray sequence */

    for (i = 0U; i < 8U; i++)
    {
        uint8_t const nxt = fwd[(i + 1U) % 4U];
        sum += drv_encoder_decode(ab, nxt, &glitch);
        CHECK("H-MO-08", !glitch, "forward gray step is valid");
        ab = nxt;
    }
    CHECK("H-MO-09", 8 == sum, "8 forward steps = +8 (sign convention)");
    sum = 0;
    for (i = 0U; i < 8U; i++)
    {
        uint8_t const nxt =
            fwd[(4U + 3U - (i % 4U) - 0U) % 4U]; /* reverse walk */
        int8_t const s = drv_encoder_decode(ab, nxt, &glitch);
        (void) s;
        ab = nxt;
    }
    /* direction change and glitch */
    ab = 0U;
    CHECK("H-MO-10", (0 == drv_encoder_decode(0U, 3U, &glitch)) && glitch,
          "00->11 is a glitch (both bits)");
    CHECK("H-MO-11", (0 == drv_encoder_decode(2U, 2U, &glitch)) && !glitch,
          "same state = 0, no glitch");
    CHECK("H-MO-12", -1 == drv_encoder_decode(1U, 0U, &glitch), "01->00 is -1");
}

static void
test_latch_and_commands (void)
{
    motion_core_t core;
    motion_in_t   in;
    motion_out_t  out;
    sim_car_t     car;

    motion_core_init(&core);
    sim_init(&car, 0U);
    fill_in(&in, &car, 0U, make_line(0, true, 1U));

    /* idle: BRAKED, pwm 0 */
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-13",
          (APP_MOTION_BRAKED == out.status.motion_state) &&
              (0 == out.pwm_l_pct) && !out.flush_queue,
          "boot: BRAKED, no PWM");

    /* disarmed: MOVE rejected */
    in.motion_armed = false;
    in.has_cmd      = true;
    in.cmd_record   = make_cmd(APP_MC_MOVE, 20U, 300, false, 1U);
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-14",
          out.reject_event && (1U == out.reject_seq) &&
              (APP_MOTION_BRAKED == out.status.motion_state),
          "MOVE rejected while disarmed");
    in.motion_armed = true;

    /* speed 0 rejected */
    in.cmd_record = make_cmd(APP_MC_MOVE, 0U, 300, false, 2U);
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-15", out.reject_event && (2U == out.reject_seq),
          "speed 0 rejected");

    /* MOVE accepted; second MOVE while MOVING rejected; BRAKE pre-empts */
    in.cmd_record = make_cmd(APP_MC_MOVE, 20U, 300, false, 3U);
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-16",
          (APP_MOTION_MOVING == out.status.motion_state) &&
              (3U == out.status.active_seq) && (out.pwm_l_pct > 0),
          "MOVE accepted from BRAKED");
    in.cmd_record = make_cmd(APP_MC_MOVE, 20U, 300, false, 4U);
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-17",
          out.reject_event && (4U == out.reject_seq) &&
              (APP_MOTION_MOVING == out.status.motion_state),
          "MOVE while MOVING rejected");
    in.cmd_record = make_cmd(APP_MC_BRAKE, 0U, 0, false, 5U);
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-18",
          out.done_event && (5U == out.status.done_seq) &&
              (APP_MOTION_BRAKED == out.status.motion_state) &&
              (0 == out.pwm_l_pct),
          "BRAKE pre-empts and completes");
    in.has_cmd = false;

    /* latch during MOVE: brake within the same cycle, flush, ABORTED */
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_MOVE, 20U, 300, false, 6U);
    motion_core_step(&core, &in, &out);
    in.has_cmd = false;
    run_until_done(&core, &car, &out, 10U, 0, true, false, false);
    CHECK("H-MO-19", out.pwm_l_pct > 0, "driving before the latch");
    fill_in(&in, &car, 7U, make_line(0, true, 9U));
    in.stop_latched = true;
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-20",
          (0 == out.pwm_l_pct) && (0 == out.pwm_r_pct) && out.flush_queue &&
              out.done_event &&
              (APP_MOTION_STOP_LATCHED == out.status.motion_state) &&
              (APP_MOTION_RESULT_ABORTED == out.status.done_result) &&
              (6U == out.status.done_seq),
          "latch -> PWM 0 in the same cycle, flush, ABORTED(6)");
    /* stale command while latched */
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_FOLLOW, 20U, 0, false, 7U);
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-21",
          out.reject_event && (7U == out.reject_seq) && (0 == out.pwm_l_pct) &&
              out.flush_queue,
          "command while latched rejected, still braked");
    in.has_cmd = false;
    /* latch cleared by the controller -> BRAKED, never directly driving */
    in.stop_latched = false;
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-22",
          (APP_MOTION_BRAKED == out.status.motion_state) &&
              (0 == out.pwm_l_pct),
          "latch cleared -> BRAKED only");
}

static void
test_move_distance (void)
{
    motion_core_t core;
    motion_in_t   in;
    motion_out_t  out;
    sim_car_t     car;
    uint32_t      n    = 0U;
    int32_t       sign = 0;

    for (sign = 1; sign >= -1; sign -= 2)
    {
        int16_t const dist = (int16_t) (sign * 1000); /* 1 m */
        motion_core_init(&core);
        sim_init(&car, (uint16_t) ((sign > 0) ? 65530U
                                              : 3U)); /* force counter wrap */
        fill_in(&in, &car, 0U, make_line(0, true, 1U));
        in.has_cmd    = true;
        in.cmd_record = make_cmd(APP_MC_MOVE, 20U, dist, false, 8U);
        motion_core_step(&core, &in, &out);
        sim_step(&car, &out);
        n = run_until_done(&core, &car, &out, 2000U, 0, true, false, false);
        CHECK("H-MO-23",
              out.done_event &&
                  (APP_MOTION_RESULT_OK == out.status.done_result) &&
                  (8U == out.status.done_seq),
              "1 m MOVE completes OK");
        CHECK("H-MO-24", (n > 300U) && (n < 800U),
              "1 m at 20 cm/s takes 3-8 s of cycles");
        if (sign > 0)
        {
            CHECK("H-MO-25",
                  (out.status.distance_mm >= 980) &&
                      (out.status.distance_mm <= 1030) &&
                      !out.status.dir_l_reverse,
                  "forward distance ~1000 mm across a counter wrap");
        }
        else
        {
            CHECK("H-MO-26",
                  (out.status.distance_mm <= -980) &&
                      (out.status.distance_mm >= -1030) &&
                      out.status.dir_l_reverse && out.status.dir_r_reverse,
                  "reverse distance ~-1000 mm, direction bits set");
        }
        CHECK("H-MO-27",
              (out.status.speed_l_cm_s >= 15U) &&
                  (out.status.speed_l_cm_s <= 26U),
              "measured speed near the 20 cm/s setpoint at completion");
    }
}

static void
test_turns (void)
{
    motion_core_t core;
    motion_in_t   in;
    motion_out_t  out;
    sim_car_t     car;
    int32_t       sign = 0;

    for (sign = 1; sign >= -1; sign -= 2)
    {
        motion_core_init(&core);
        sim_init(&car, 100U);
        fill_in(&in, &car, 0U, make_line(0, false, 1U));
        in.has_cmd = true;
        in.cmd_record =
            make_cmd(APP_MC_TURN, 15U, (int16_t) (sign * 900), false, 9U);
        motion_core_step(&core, &in, &out);
        CHECK("H-MO-28",
              (APP_MOTION_TURNING == out.status.motion_state) &&
                  ((sign > 0) ? (out.pwm_l_pct < 0 && out.pwm_r_pct > 0)
                              : (out.pwm_l_pct > 0 && out.pwm_r_pct < 0)),
              "pivot wheels in opposite directions (+ = left/CCW)");
        sim_step(&car, &out);
        (void) run_until_done(&core, &car, &out, 1000U, 0, false, false, false);
        CHECK("H-MO-29",
              out.done_event &&
                  (APP_MOTION_RESULT_OK == out.status.done_result),
              "90 deg turn completes OK both ways");
        CHECK("H-MO-30",
              (car.emitted_l + car.emitted_r >= -2) &&
                  (car.emitted_l + car.emitted_r <= 2),
              "pivot: wheels moved equal and opposite");
    }
    /* until_line: finishes when the line appears after the minimum angle */
    motion_core_init(&core);
    sim_init(&car, 0U);
    fill_in(&in, &car, 0U, make_line(0, false, 1U));
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_TURN, 15U, 900, true, 10U);
    motion_core_step(&core, &in, &out);
    sim_step(&car, &out);
    {
        uint32_t const n1 = run_until_done(&core, &car, &out, 25U, 0, false,
                                           false, false); /* no line yet */
        CHECK("H-MO-31", !out.done_event && (25U == n1),
              "until_line keeps turning while off line");
        (void) run_until_done(&core, &car, &out, 1000U, 0, true, false,
                              false); /* line appears */
        CHECK("H-MO-32",
              out.done_event &&
                  (APP_MOTION_RESULT_OK == out.status.done_result),
              "until_line completes once on_line after >= 50 %");
    }
    /* until_line with no line ever: aborts at 150 % with TIMEOUT */
    motion_core_init(&core);
    sim_init(&car, 0U);
    fill_in(&in, &car, 0U, make_line(0, false, 1U));
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_TURN, 15U, 900, true, 11U);
    motion_core_step(&core, &in, &out);
    sim_step(&car, &out);
    (void) run_until_done(&core, &car, &out, 2000U, 0, false, false, false);
    CHECK("H-MO-33",
          out.done_event &&
              (APP_MOTION_RESULT_TIMEOUT == out.status.done_result),
          "until_line without a line ends TIMEOUT at 150 %, no stop latch");
}

static void
test_follow_and_guards (void)
{
    motion_core_t core;
    motion_in_t   in;
    motion_out_t  out;
    sim_car_t     car;

    /* FOLLOW steers: +steer -> left faster than right */
    motion_core_init(&core);
    sim_init(&car, 0U);
    fill_in(&in, &car, 0U, make_line(0, true, 1U));
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_FOLLOW, 20U, 0, false, 12U);
    motion_core_step(&core, &in, &out);
    sim_step(&car, &out);
    (void) run_until_done(&core, &car, &out, 100U, 40, true, false, false);
    CHECK("H-MO-34",
          (APP_MOTION_FOLLOWING == out.status.motion_state) &&
              (out.pwm_l_pct > out.pwm_r_pct),
          "FOLLOW: positive steer drives left wheel harder");
    (void) run_until_done(&core, &car, &out, 100U, -40, true, false, false);
    CHECK("H-MO-35", out.pwm_l_pct < out.pwm_r_pct,
          "negative steer drives right wheel harder");

    /* stale line while FOLLOWING -> STALE_LINE stop within ~30 ms */
    {
        uint32_t const n =
            run_until_done(&core, &car, &out, 100U, 0, true, true, false);
        CHECK("H-MO-36",
              (APP_STOP_CAUSE_STALE_LINE == out.stop_request) &&
                  (0 == out.pwm_l_pct) &&
                  (0U != (out.fault_set & APP_FAULT_STALE_LINE)) && (n <= 5U),
              "frozen LineState -> STALE_LINE stop, braked, within 5 cycles");
    }

    /* heartbeat loss while MOVING -> CTRL_HEARTBEAT stop */
    motion_core_init(&core);
    sim_init(&car, 0U);
    fill_in(&in, &car, 0U, make_line(0, true, 1U));
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_MOVE, 20U, 2000, false, 13U);
    motion_core_step(&core, &in, &out);
    sim_step(&car, &out);
    {
        uint32_t const n =
            run_until_done(&core, &car, &out, 100U, 0, true, false, true);
        CHECK("H-MO-37",
              (APP_STOP_CAUSE_CTRL_HEARTBEAT == out.stop_request) &&
                  (n >= 9U) && (n <= 12U) && (0 == out.pwm_l_pct),
              "no controller heartbeat for 100 ms -> stop");
    }

    /* stall: wheels blocked -> STALL after ENC_STALL_MS */
    motion_core_init(&core);
    sim_init(&car, 0U);
    car.stalled = true;
    fill_in(&in, &car, 0U, make_line(0, true, 1U));
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_MOVE, 20U, 2000, false, 14U);
    motion_core_step(&core, &in, &out);
    sim_step(&car, &out);
    {
        uint32_t const n =
            run_until_done(&core, &car, &out, 200U, 0, true, false, false);
        CHECK("H-MO-38",
              (APP_STOP_CAUSE_STALL == out.stop_request) &&
                  (0U != (out.fault_set & APP_FAULT_MOTOR_STALL)) &&
                  (n >= 28U) && (n <= 32U),
              "blocked wheels -> STALL stop after ~300 ms");
        CHECK("H-MO-39", (0U == out.status.speed_l_cm_s),
              "speed reports 0 with no edges");
    }

    /* stale line in BRAKED must NOT stop (no line needed) */
    motion_core_init(&core);
    sim_init(&car, 0U);
    {
        uint32_t const n =
            run_until_done(&core, &car, &out, 50U, 0, true, true, true);
        CHECK("H-MO-40",
              (50U == n) && (APP_STOP_CAUSE_NONE == out.stop_request),
              "idle with stale inputs: no stop");
    }

    /* glitch storm while driving -> ENCODER_FAULT + STALL cause */
    motion_core_init(&core);
    sim_init(&car, 0U);
    fill_in(&in, &car, 0U, make_line(0, true, 1U));
    in.has_cmd    = true;
    in.cmd_record = make_cmd(APP_MC_MOVE, 20U, 2000, false, 15U);
    motion_core_step(&core, &in, &out);
    in.has_cmd  = false;
    in.glitches = (uint8_t) APP_CAL_ENC_GLITCH_MAX;
    in.ctrl_heartbeat++;
    motion_core_step(&core, &in, &out);
    CHECK("H-MO-41",
          (0U != (out.fault_set & APP_FAULT_ENCODER_FAULT)) &&
              (APP_STOP_CAUSE_STALL == out.stop_request),
          "glitch count at threshold -> ENCODER_FAULT, stop");

    /* motion timeout: MOVE that can never finish ends TIMEOUT, no stop */
    motion_core_init(&core);
    sim_init(&car, 0U);
    fill_in(&in, &car, 0U, make_line(0, true, 1U));
    in.has_cmd = true;
    in.cmd_record =
        make_cmd(APP_MC_MOVE, 2U, 32000, false, 1U); /* 32 m at 2 cm/s */
    motion_core_step(&core, &in, &out);
    sim_step(&car, &out);
    (void) run_until_done(&core, &car, &out, 2000U, 0, true, false, false);
    CHECK("H-MO-42",
          out.done_event &&
              (APP_MOTION_RESULT_TIMEOUT == out.status.done_result) &&
              (APP_STOP_CAUSE_NONE == out.stop_request),
          "command timeout -> TIMEOUT result, controller decides");

    /* PWM limiter: never above the ceiling, never between 0 and the floor */
    {
        uint32_t i  = 0U;
        bool     ok = true;
        motion_core_init(&core);
        sim_init(&car, 0U);
        fill_in(&in, &car, 0U, make_line(0, true, 1U));
        in.has_cmd    = true;
        in.cmd_record = make_cmd(APP_MC_MOVE, 127U, 5000, false, 2U);
        motion_core_step(&core, &in, &out);
        in.has_cmd = false;
        for (i = 0U; i < 200U; i++)
        {
            int32_t const m =
                (out.pwm_l_pct < 0) ? -out.pwm_l_pct : out.pwm_l_pct;
            if ((m > (int32_t) APP_CAL_PWM_MAX_PCT) ||
                ((m > 0) && (m < (int32_t) APP_CAL_PWM_MIN_PCT)))
            {
                ok = false;
            }
            sim_step(&car, &out);
            fill_in(&in, &car, (uint8_t) (i / 2U),
                    make_line(0, true, (uint8_t) (i & 0x0FU)));
            motion_core_step(&core, &in, &out);
        }
        CHECK("H-MO-43", ok,
              "PWM within [MIN, MAX] or exactly 0 for 200 cycles at max speed");
    }
}

int
main (void)
{
    test_geometry();
    test_decoder();
    test_latch_and_commands();
    test_move_distance();
    test_turns();
    test_follow_and_guards();
    printf("motion host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
/* NOLINTEND(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */
