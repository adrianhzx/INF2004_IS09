/** @file integration_test.c
 *  @brief On-target integration run INT-13..INT-20 (make TEST=1).
 *
 *  Drives the real car through the same MBF_MOTION_CMD path as the mission
 *  controller (vehicle_controller_send_motion), while the controller idles
 *  in S1 (it sends no motion there). Every drive command is sent from
 *  BRAKED and waited for (Rev B 6.2 acceptance rule). The STOP button stays
 *  the emergency stop: a latched stop aborts the rest of the run.
 *
 *    INT-13  motor: rotate 360 deg LEFT on the spot
 *    INT-14  motor: rotate 360 deg RIGHT on the spot
 *    INT-15  motor: move forward
 *    INT-16  motor: move backward
 *    INT-17  IR:    wait for the black line, then line-follow (FOLLOW)
 *    INT-18  servo: keeps sweeping during the whole run (background task)
 *    INT-19  ultrasonic: drive forward, brake when the obstacle is <= 60 cm
 *    INT-20  barcode: decode a barcode and do its action (A left, B right,
 *            C straight, D U-turn)
 *
 *  Hardware gates (app_cal.h) decide what really moves; a stage whose gate
 *  is 0 runs in "detect only" mode or prints NOT RUN:
 *    APP_CAL_MOTION_ARMED          wheels (HV-3/HV-4)
 *    APP_CAL_ECHO_SHIFTER_VERIFIED ultrasonic TRIG/ECHO (HV-5)
 *    APP_CAL_SERVO_ARMED           scan servo (HV-7)
 */

#include "integration_test.h"

#if APP_TEST

#include "../shared/app_barcode.h"
#include "../shared/app_cal.h"
#include "../shared/app_data.h"
#include "../shared/app_hw.h"
#include "../shared/app_ipc.h"
#include "../shared/app_line.h"
#include "../shared/app_motion.h"
#include "../shared/app_obstacle.h"
#include "../shared/app_safety.h"
#include "../shared/vehicle_controller.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem5/drv_servo.h"
#include "../subsystem5/scan_core.h"
#include "../subsystem5/subsystem5.h"

/* ---- Switches and test parameters ------------------------------------- */

/** 1 = run INT-13..INT-20 in the TEST image; 0 = skip (prints one line). */
#define IT_ENABLE 1

#define IT_COUNTDOWN_S 5U /* warning before each stage that moves */
#define IT_POLL_MS 20U    /* status polling period */
#define IT_BRAKE_WAIT_MS 1000U
#define IT_TURN_WAIT_MS 9000U /* > APP_CAL_T_MOTION_TIMEOUT_MS (8 s) */
#define IT_FULL_TURN 3600     /* 0.1 deg: one full rotation */
#define IT_QUARTER_TURN 900
#define IT_HALF_TURN 1800
#define IT_DRIVE_MM 300        /* INT-15 / INT-16 distance */
#define IT_STRAIGHT_MM 150     /* barcode C: drive on across the junction */
#define IT_LINE_WAIT_MS 20000U /* INT-17: time to put the car on the line */
#define IT_FOLLOW_MS 8000U     /* INT-17: line-follow time */
#define IT_FOLLOW_MIN_MS 2000U /* INT-17 pass: followed at least this long */
#define IT_LINE_GONE_MS 1600U  /* INT-17: no line this long -> stop */
#define IT_STATUS_MS 500U      /* progress line period while driving */
#define IT_STOP_CM 60U         /* INT-19: brake at or inside this range */
#define IT_STOP_CONFIRM 2U     /* INT-19: consecutive readings inside */
#define IT_STOP_MIN_CM 30U     /* INT-19 pass band after braking */
#define IT_STOP_MAX_CM 70U
#define IT_OBST_MOVE_MM 1000 /* INT-19: drive at most this far */
#define IT_OBST_WAIT_MS 9000U
#define IT_OBST_DETECT_MS 15000U /* INT-19 detect-only window (disarmed) */
#define IT_BC_WAIT_MS 30000U     /* INT-20: wait for a barcode */
#define IT_SERVO_MIN_SWEEPS 3U   /* INT-18 pass */

/* Background servo sweep (INT-18). */
#define IT_SWEEP_PRI 9
#define IT_SWEEP_STACK 1536U
#define IT_SWEEP_STEP 3U        /* 3 x 5 deg = 15 deg per step */
#define IT_SWEEP_WIDE_HALF 12U  /* +-60 deg */
#define IT_SWEEP_NARROW_HALF 3U /* +-15 deg while watching ahead (INT-19) */
#define IT_SWEEP_EXTRA_MS 40U   /* margin over the servo settle time */
#define IT_SWEEP_IDLE_MS 50U
#define IT_AHEAD_DEG 15 /* readings within +-15 deg count as ahead */
#define IT_TRIG_US 10U

#define IT_SEQ_MAX 15U
#define IT_RESULT_NONE 0xFFU /* no MotionStatus result: abort or timeout */
#define IT_MS_PER_S 1000U

/* ---- Run state ----------------------------------------------------------- */

static INT     g_it_pass  = 0;
static INT     g_it_fail  = 0;
static INT     g_it_skip  = 0;
static uint8_t g_it_seq   = 0U;
static bool    g_it_abort = false;

/* Sweep task <-> test task (single writer each, aligned stores). */
static volatile bool     g_it_sweep_on   = false;
static volatile uint32_t g_it_sweep_half = IT_SWEEP_WIDE_HALF;
static volatile uint32_t g_it_sweeps     = 0U; /* end-to-end sweeps */
static volatile uint32_t g_it_steps      = 0U; /* servo positions visited */
static volatile uint16_t g_it_ahead_cm   = (uint16_t) APP_RANGE_CM_SENTINEL;
static volatile uint32_t g_it_ahead_seq  = 0U;
static volatile int32_t  g_it_last_deg   = 0;
static volatile uint16_t g_it_last_cm    = (uint16_t) APP_RANGE_CM_SENTINEL;
static ID                g_it_sweep_tid  = 0;

static char const * const g_it_bc_name[7] = {
    "NONE",  "LEFT",         "RIGHT",       "STRAIGHT",
    "UTURN", "INV_NO_MATCH", "INV_OVERFLOW"};
static char const * const g_it_result_name[4] = {"OK", "REJECTED", "ABORTED",
                                                 "TIMEOUT"};

/* ---- Reporting -------------------------------------------------------- */

static void
it_check (char const * p_id, bool ok, char const * p_desc)
{
    if (ok)
    {
        g_it_pass++;
        tm_printf((UB *) "[TEST] %s PASS  %s\n", p_id, p_desc);
    }
    else
    {
        g_it_fail++;
        tm_printf((UB *) "[TEST] %s FAIL  %s\n", p_id, p_desc);
    }
}

static void
it_not_run (char const * p_id, char const * p_why)
{
    g_it_skip++;
    tm_printf((UB *) "[TEST] %s NOT RUN  %s\n", p_id, p_why);
}

static char const *
it_result_text (uint8_t r)
{
    return (r < 4U) ? g_it_result_name[r] : "NO RESULT";
}

/** Abort check: a latched stop (STOP button, collision, ...) ends the run. */
static bool
it_aborted (void)
{
    if (!g_it_abort && app_stop_latched())
    {
        g_it_abort = true;
        tm_printf((UB *) "[INT] SAFE STOP latched (cause %u): remaining "
                         "stages are not run\n",
                  (unsigned) app_stop_cause_get());
    }
    return g_it_abort;
}

static void
it_countdown (char const * p_what)
{
    uint32_t s = 0U;

    tm_printf((UB *) "[INT] next: %s in %u s (STOP button = emergency stop)\n",
              p_what, (unsigned) IT_COUNTDOWN_S);
    for (s = IT_COUNTDOWN_S; s > 0U; s--)
    {
        tm_printf((UB *) "[INT]   %u...\n", (unsigned) s);
        (void) tk_dly_tsk(IT_MS_PER_S);
        if (it_aborted())
        {
            return;
        }
    }
}

/* ---- Motion helpers --------------------------------------------------- */

static uint8_t
it_next_seq (void)
{
    g_it_seq = (uint8_t) ((g_it_seq % IT_SEQ_MAX) + 1U); /* 1..15 */
    return g_it_seq;
}

/** BRAKE and wait until MotionStatus reports BRAKED. */
static bool
it_brake (void)
{
    app_motion_status_t st;
    uint32_t            waited = 0U;

    (void) vehicle_controller_send_motion(APP_MC_BRAKE, 0U, 0, false,
                                          it_next_seq());
    do
    {
        (void) tk_dly_tsk(IT_POLL_MS);
        waited += IT_POLL_MS;
        (void) app_motion_read(&st);
    } while (((uint8_t) APP_MOTION_BRAKED != st.motion_state) &&
             ((uint8_t) APP_MOTION_STOP_LATCHED != st.motion_state) &&
             (waited < IT_BRAKE_WAIT_MS));
    return (uint8_t) APP_MOTION_BRAKED == st.motion_state;
}

/** Send one MOVE/TURN from BRAKED and wait for its done_seq. Returns the
 *  app_motion_result_t, or IT_RESULT_NONE on abort/timeout. */
static uint8_t
it_drive (app_motion_cmd_type_t type, uint8_t speed, int16_t param,
          uint32_t timeout_ms, int16_t * p_dist_mm, uint32_t * p_ms)
{
    app_motion_status_t st;
    uint32_t            waited = 0U;
    uint8_t const       seq    = it_next_seq();

    *p_dist_mm = 0;
    *p_ms      = 0U;
    if (it_aborted() || !it_brake())
    {
        return IT_RESULT_NONE;
    }
    if (vehicle_controller_send_motion(type, speed, param, false, seq) < 0)
    {
        return IT_RESULT_NONE;
    }
    do
    {
        (void) tk_dly_tsk(IT_POLL_MS);
        waited += IT_POLL_MS;
        (void) app_motion_read(&st);
        if (it_aborted())
        {
            return IT_RESULT_NONE;
        }
    } while ((st.done_seq != seq) && (waited < timeout_ms));
    *p_dist_mm = st.distance_mm;
    *p_ms      = waited;
    if (st.done_seq != seq)
    {
        (void) it_brake();
        return IT_RESULT_NONE;
    }
    return st.done_result;
}

/** One motor stage (INT-13..INT-16): countdown, drive, report. */
static void
it_motor_stage (char const * p_id, char const * p_what,
                app_motion_cmd_type_t type, uint8_t speed, int16_t param,
                char const * p_pass_desc)
{
    int16_t  dist = 0;
    uint32_t ms   = 0U;
    uint8_t  r    = IT_RESULT_NONE;

    if (it_aborted())
    {
        it_not_run(p_id, "run aborted by a safe stop");
        return;
    }
    it_countdown(p_what);
    r = it_drive(type, speed, param, IT_TURN_WAIT_MS, &dist, &ms);
    tm_printf((UB *) "[INT] %s INFO  %s: result=%s, distance=%d mm, "
                     "%u ms\n",
              p_id, p_what, it_result_text(r), (int) dist, (unsigned) ms);
    it_check(p_id, (uint8_t) APP_MOTION_RESULT_OK == r, p_pass_desc);
}

/* ---- Background servo sweep (INT-18) and ranging ----------------------- */

/** One HC-SR04 ranging through EchoIsr. Returns the range in cm, or the
 *  sentinel when there is no valid echo (or the HV-5 gate is off). */
static uint16_t
it_range_cm (void)
{
#if APP_CAL_ECHO_SHIFTER_VERIFIED
    UINT     ptn = 0U;
    ER       er  = E_OK;
    uint16_t cm  = (uint16_t) APP_RANGE_CM_SENTINEL;

    (void) tk_wai_flg(flg_scan, (UINT) APP_SCAN_ECHO_DONE, TWF_ORW | TWF_BITCLR,
                      &ptn, TMO_POL);
    g_echo_status = APP_ECHO_ARMED;
    app_hw_gpio_write(APP_PIN_US_TRIG, true);
    app_hw_delay_us(IT_TRIG_US);
    app_hw_gpio_write(APP_PIN_US_TRIG, false);
    er = tk_wai_flg(flg_scan, (UINT) APP_SCAN_ECHO_DONE, TWF_ORW | TWF_BITCLR,
                    &ptn, (TMO) (APP_CAL_T_ECHO_TIMEOUT_MS + 5U));
    if ((E_OK == er) && (APP_ECHO_DONE == g_echo_status))
    {
        cm = app_obstacle_us_to_cm(g_echo_width_us);
    }
    g_echo_status = APP_ECHO_IDLE;
    return cm;
#else
    return (uint16_t) APP_RANGE_CM_SENTINEL;
#endif
}

/** Servo always moving: bounces between ahead +- g_it_sweep_half (5 deg
 *  units) in 15 deg steps and ranges at every position. Readings within
 *  +-15 deg of ahead are published as "ahead" for INT-19. */
static void
it_sweep_task (INT stacd, void * p_exinf)
{
    uint32_t idx  = SC_ANGLE_AHEAD;
    uint32_t prev = SC_ANGLE_AHEAD;
    bool     up   = true;

    (void) stacd;
    (void) p_exinf;
    for (;;)
    {
        uint32_t half = 0U;
        uint32_t lo   = 0U;
        uint32_t hi   = 0U;
        int32_t  deg  = 0;
        uint16_t cm   = 0U;

        if (!g_it_sweep_on)
        {
            (void) tk_dly_tsk(IT_SWEEP_IDLE_MS);
            continue;
        }
#if APP_CAL_SERVO_ARMED
        half = g_it_sweep_half;
#else
        half = 0U; /* servo not energised (HV-7): sensor stays ahead */
#endif
        lo = SC_ANGLE_AHEAD - half;
        hi = SC_ANGLE_AHEAD + half;
        if ((idx < lo) || (idx > hi))
        {
            idx = SC_ANGLE_AHEAD; /* window just changed: restart ahead */
        }
        drv_servo_set_us(scan_core_servo_us(idx));
        (void) tk_dly_tsk(scan_core_settle_ms(prev, idx) + IT_SWEEP_EXTRA_MS);
        prev = idx;
        cm   = it_range_cm();
        deg  = ((int32_t) idx - (int32_t) SC_ANGLE_AHEAD) *
              (int32_t) SC_DEG_PER_IDX;
        g_it_last_deg = deg;
        g_it_last_cm  = cm;
        if ((deg >= -IT_AHEAD_DEG) && (deg <= IT_AHEAD_DEG))
        {
            g_it_ahead_cm = cm;
            g_it_ahead_seq++;
        }
        g_it_steps++;
        if (0U == half)
        {
            continue;
        }
        if (up && ((idx + IT_SWEEP_STEP) > hi))
        {
            up = false;
            g_it_sweeps++;
        }
        else if (!up && (idx < (lo + IT_SWEEP_STEP)))
        {
            up = true;
            g_it_sweeps++;
        }
        else
        {
            /* keep direction */
        }
        idx = up ? (idx + IT_SWEEP_STEP) : (idx - IT_SWEEP_STEP);
    }
}

static void
it_sweep_start (void)
{
    if (0 == g_it_sweep_tid)
    {
        g_it_sweep_tid =
            app_start_task_stack((FP) it_sweep_task, (PRI) IT_SWEEP_PRI,
                                 (SZ) IT_SWEEP_STACK, "ItSweep");
    }
#if APP_CAL_ECHO_SHIFTER_VERIFIED
    s5_echo_irq_enable(true);
#endif
    g_it_sweep_half = IT_SWEEP_WIDE_HALF;
    g_it_sweep_on   = true;
}

static void
it_sweep_stop (void)
{
    g_it_sweep_on = false;
    (void) tk_dly_tsk(IT_STATUS_MS); /* let the last step finish */
    drv_servo_set_us(scan_core_servo_us(SC_ANGLE_AHEAD));
    s5_echo_irq_enable(s5_scan_enabled());
}

/* ---- INT-17: IR line ---------------------------------------------------- */

static void
it_ir_stage (void)
{
    app_line_state_t ls;
    uint32_t         waited  = 0U;
    uint32_t         driven  = 0U;
    uint32_t         gone_ms = 0U;
    uint32_t         next    = 0U;
    bool             seen    = false;
    bool             armed   = (0 != APP_CAL_MOTION_ARMED);

    if (it_aborted())
    {
        it_not_run("INT-17", "run aborted by a safe stop");
        return;
    }
    tm_printf((UB *) "[INT] INT-17 INFO  IR line: put the car (or a black "
                     "line) under the IR sensors within %u s%s\n",
              (unsigned) (IT_LINE_WAIT_MS / IT_MS_PER_S),
              armed ? "; the car starts line-following when it sees it"
                    : " (motion disarmed: detect only)");
    while ((waited < IT_LINE_WAIT_MS) && !seen)
    {
        (void) tk_dly_tsk(IT_POLL_MS);
        waited += IT_POLL_MS;
        app_line_read(&ls);
        seen = ls.on_line;
        if (it_aborted())
        {
            it_not_run("INT-17", "run aborted by a safe stop");
            return;
        }
    }
    if (!seen)
    {
        it_check("INT-17", false, "no black line seen by the IR sensors");
        return;
    }
    tm_printf((UB *) "[INT] INT-17 INFO  black line detected after %u ms "
                     "(err=%d)\n",
              (unsigned) waited, (int) ls.line_error);
    if (!armed)
    {
        it_check("INT-17", true,
                 "black line detected (motion disarmed: car would start "
                 "line-following now)");
        return;
    }
    if (!it_brake() || (vehicle_controller_send_motion(
                            APP_MC_FOLLOW, (uint8_t) APP_CAL_V_NOM_CM_S, 0,
                            false, it_next_seq()) < 0))
    {
        it_check("INT-17", false, "FOLLOW command not accepted");
        return;
    }
    next = 0U;
    while ((driven < IT_FOLLOW_MS) && (gone_ms < IT_LINE_GONE_MS))
    {
        (void) tk_dly_tsk(IT_POLL_MS);
        driven += IT_POLL_MS;
        app_line_read(&ls);
        gone_ms = ls.on_line ? 0U : (gone_ms + IT_POLL_MS);
        if (driven >= next)
        {
            next += IT_STATUS_MS;
            tm_printf((UB *) "[INT] INT-17   t=%4u ms  %s err=%4d steer=%4d\n",
                      (unsigned) driven, ls.on_line ? "ON-LINE " : "off-line",
                      (int) ls.line_error, (int) ls.steer);
        }
        if (it_aborted())
        {
            break;
        }
    }
    (void) it_brake();
    tm_printf((UB *) "[INT] INT-17 INFO  followed the line for %u ms%s\n",
              (unsigned) driven,
              (gone_ms >= IT_LINE_GONE_MS) ? " (stopped: line gone)" : "");
    it_check("INT-17", !g_it_abort && (driven >= IT_FOLLOW_MIN_MS),
             "car moved along the black line after detecting it");
}

/* ---- INT-19: ultrasonic stop at 60 cm ---------------------------------- */

static void
it_ultrasonic_stage (void)
{
    app_motion_status_t st;
    uint32_t            seq0    = 0U;
    uint32_t            waited  = 0U;
    uint32_t            inside  = 0U;
    uint32_t            next    = 0U;
    uint16_t            cm      = 0U;
    uint16_t            stop_cm = (uint16_t) APP_RANGE_CM_SENTINEL;
    uint16_t            min_cm  = (uint16_t) APP_RANGE_CM_SENTINEL;
    uint8_t             seq     = 0U;
    bool                braked  = false;

    if (it_aborted())
    {
        it_not_run("INT-19", "run aborted by a safe stop");
        return;
    }
    if (0 == APP_CAL_ECHO_SHIFTER_VERIFIED)
    {
        it_not_run("INT-19",
                   "APP_CAL_ECHO_SHIFTER_VERIFIED = 0 (HV-5): no ranging");
        return;
    }
    g_it_sweep_half = IT_SWEEP_NARROW_HALF; /* still moving, watching ahead */
    seq0            = g_it_ahead_seq;

    if (0 == APP_CAL_MOTION_ARMED)
    {
        tm_printf((UB *) "[INT] INT-19 INFO  motion disarmed: detect only. "
                         "Move an object to <= %u cm in front within %u s\n",
                  (unsigned) IT_STOP_CM,
                  (unsigned) (IT_OBST_DETECT_MS / IT_MS_PER_S));
        while ((waited < IT_OBST_DETECT_MS) && (inside < IT_STOP_CONFIRM))
        {
            (void) tk_dly_tsk(IT_POLL_MS);
            waited += IT_POLL_MS;
            if (g_it_ahead_seq != seq0)
            {
                seq0   = g_it_ahead_seq;
                cm     = g_it_ahead_cm;
                inside = (cm <= IT_STOP_CM) ? (inside + 1U) : 0U;
                tm_printf((UB *) "[INT] INT-19   ahead %3u cm\n",
                          (unsigned) cm);
            }
        }
        g_it_sweep_half = IT_SWEEP_WIDE_HALF;
        it_check("INT-19", inside >= IT_STOP_CONFIRM,
                 "obstacle at <= 60 cm detected (motion disarmed: car would "
                 "brake now)");
        return;
    }

    it_countdown("drive forward up to 1 m and STOP at 60 cm from the "
                 "obstacle (put a box 1-1.5 m ahead)");
    if (it_aborted() || !it_brake())
    {
        g_it_sweep_half = IT_SWEEP_WIDE_HALF;
        it_not_run("INT-19", "not BRAKED / aborted");
        return;
    }
    seq = it_next_seq();
    (void) vehicle_controller_send_motion(
        APP_MC_MOVE, (uint8_t) APP_CAL_V_NOM_CM_S, IT_OBST_MOVE_MM, false, seq);
    seq0 = g_it_ahead_seq;
    do
    {
        (void) tk_dly_tsk(IT_POLL_MS);
        waited += IT_POLL_MS;
        (void) app_motion_read(&st);
        if (g_it_ahead_seq != seq0)
        {
            seq0   = g_it_ahead_seq;
            cm     = g_it_ahead_cm;
            min_cm = (cm < min_cm) ? cm : min_cm;
            inside = (cm <= IT_STOP_CM) ? (inside + 1U) : 0U;
            if (waited >= next)
            {
                next = waited + IT_STATUS_MS;
                tm_printf((UB *) "[INT] INT-19   t=%4u ms  ahead %3u cm  "
                                 "driven %d mm\n",
                          (unsigned) waited, (unsigned) cm,
                          (int) st.distance_mm);
            }
            if (inside >= IT_STOP_CONFIRM)
            {
                stop_cm = cm;
                braked  = true;
                (void) it_brake();
            }
        }
        if (it_aborted())
        {
            break;
        }
    } while (!braked && (st.done_seq != seq) && (waited < IT_OBST_WAIT_MS));
    if (!braked)
    {
        (void) it_brake();
    }
    (void) tk_dly_tsk(IT_STATUS_MS);
    cm = g_it_ahead_cm; /* range after the car has stopped */
    tm_printf((UB *) "[INT] INT-19 INFO  %s: brake at %u cm, after stopping "
                     "%u cm, closest %u cm, %u ms\n",
              braked ? "obstacle stop" : "no obstacle stop", (unsigned) stop_cm,
              (unsigned) cm, (unsigned) min_cm, (unsigned) waited);
    g_it_sweep_half = IT_SWEEP_WIDE_HALF;
    it_check("INT-19",
             braked && (cm >= IT_STOP_MIN_CM) && (cm <= IT_STOP_MAX_CM),
             "drove forward and braked about 60 cm from the obstacle");
}

/* ---- INT-20: barcode -> action ------------------------------------------ */

static void
it_barcode_stage (void)
{
    app_line_state_t ls;
    uint32_t         res0   = 0U;
    uint32_t         waited = 0U;
    int16_t          dist   = 0;
    uint32_t         ms     = 0U;
    uint8_t          r      = 0U;
    uint8_t          act    = IT_RESULT_NONE;
    bool             armed  = (0 != APP_CAL_MOTION_ARMED);
    bool             moving = false;
    bool             got    = false;

    if (it_aborted())
    {
        it_not_run("INT-20", "run aborted by a safe stop");
        return;
    }
    (void) tk_set_flg(flg_barcode, (UINT) APP_BC_ARM);
    (void) tk_dly_tsk(IT_POLL_MS);
    res0 = s3_barcode_result_count();
    app_line_read(&ls);
    if (armed && ls.on_line && it_brake() &&
        (vehicle_controller_send_motion(APP_MC_FOLLOW,
                                        (uint8_t) APP_CAL_V_NOM_CM_S, 0, false,
                                        it_next_seq()) >= 0))
    {
        moving = true;
        tm_printf((UB *) "[INT] INT-20 INFO  following the line towards the "
                         "barcode (%u s max)\n",
                  (unsigned) (IT_BC_WAIT_MS / IT_MS_PER_S));
    }
    else
    {
        tm_printf((UB *) "[INT] INT-20 INFO  car stays still: pass a barcode "
                         "(*A* *B* *C* *D*) under the barcode sensor within "
                         "%u s\n",
                  (unsigned) (IT_BC_WAIT_MS / IT_MS_PER_S));
    }
    while ((waited < IT_BC_WAIT_MS) && !got)
    {
        (void) tk_dly_tsk(IT_POLL_MS);
        waited += IT_POLL_MS;
        got = (s3_barcode_result_count() != res0);
        if (it_aborted())
        {
            break;
        }
    }
    if (moving)
    {
        (void) it_brake();
    }
    (void) tk_set_flg(flg_barcode, (UINT) APP_BC_DISARM);
    if (!got)
    {
        it_check("INT-20", false, "no barcode decoded");
        return;
    }
    r = (uint8_t) app_barcode_result(g_barcode_byte);
    tm_printf((UB *) "[INT] INT-20 INFO  barcode >>> %s <<< after %u ms\n",
              (r < 7U) ? g_it_bc_name[r] : "?", (unsigned) waited);
    if ((r < (uint8_t) APP_BC_LEFT) || (r > (uint8_t) APP_BC_UTURN))
    {
        it_check("INT-20", false, "barcode read but it is not a command (A-D)");
        return;
    }
    if (!armed)
    {
        it_check("INT-20", true,
                 "command barcode decoded (motion disarmed: action not "
                 "driven)");
        return;
    }
    switch ((app_barcode_result_t) r)
    {
        case APP_BC_LEFT:
            act = it_drive(APP_MC_TURN, (uint8_t) APP_CAL_V_TURN_CM_S,
                           IT_QUARTER_TURN, IT_TURN_WAIT_MS, &dist, &ms);
            break;
        case APP_BC_RIGHT:
            act = it_drive(APP_MC_TURN, (uint8_t) APP_CAL_V_TURN_CM_S,
                           (int16_t) -IT_QUARTER_TURN, IT_TURN_WAIT_MS, &dist,
                           &ms);
            break;
        case APP_BC_STRAIGHT:
            act = it_drive(APP_MC_MOVE, (uint8_t) APP_CAL_V_NOM_CM_S,
                           IT_STRAIGHT_MM, IT_TURN_WAIT_MS, &dist, &ms);
            break;
        case APP_BC_UTURN:
            act = it_drive(APP_MC_TURN, (uint8_t) APP_CAL_V_TURN_CM_S,
                           IT_HALF_TURN, IT_TURN_WAIT_MS, &dist, &ms);
            break;
        default:
            act = IT_RESULT_NONE;
            break;
    }
    tm_printf((UB *) "[INT] INT-20 INFO  action for %s: result=%s, %u ms\n",
              g_it_bc_name[r], it_result_text(act), (unsigned) ms);
    it_check("INT-20", (uint8_t) APP_MOTION_RESULT_OK == act,
             "barcode decoded and its action done (A left 90, B right 90, "
             "C straight, D U-turn)");
}

/* ---- Run --------------------------------------------------------------- */

EXPORT void
integration_test (void)
{
    bool const armed = (0 != APP_CAL_MOTION_ARMED);

    tm_printf((UB *) "[INT] === integration run INT-13..INT-20 ===\n");
    tm_printf((UB *) "[INT] gates: motion armed=%d, echo verified=%d, servo "
                     "armed=%d\n",
              APP_CAL_MOTION_ARMED, APP_CAL_ECHO_SHIFTER_VERIFIED,
              APP_CAL_SERVO_ARMED);
#if !IT_ENABLE
    tm_printf((UB *) "[INT] skipped (IT_ENABLE = 0 in integration_test.c)\n");
    return;
#endif
    if (it_aborted())
    {
        tm_printf((UB *) "[INT] safe stop already latched: integration "
                         "skipped\n");
        return;
    }

    /* INT-18 starts first: the servo keeps sweeping through every stage. */
    it_sweep_start();

    if (armed)
    {
        tm_printf((UB *) "[INT] the car WILL MOVE. Clear 1 m around it; "
                         "STOP button = emergency stop\n");
        it_motor_stage("INT-13", "rotate 360 deg LEFT", APP_MC_TURN,
                       (uint8_t) APP_CAL_V_TURN_CM_S, IT_FULL_TURN,
                       "rotated 360 deg left on the spot");
        it_motor_stage("INT-14", "rotate 360 deg RIGHT", APP_MC_TURN,
                       (uint8_t) APP_CAL_V_TURN_CM_S, (int16_t) -IT_FULL_TURN,
                       "rotated 360 deg right on the spot");
        it_motor_stage("INT-15", "move FORWARD 30 cm", APP_MC_MOVE,
                       (uint8_t) APP_CAL_V_NOM_CM_S, IT_DRIVE_MM,
                       "moved forward");
        it_motor_stage("INT-16", "move BACKWARD 30 cm", APP_MC_MOVE,
                       (uint8_t) APP_CAL_V_REVERSE_CM_S, (int16_t) -IT_DRIVE_MM,
                       "moved backward");
    }
    else
    {
        it_not_run("INT-13", "APP_CAL_MOTION_ARMED = 0 (HV-3/HV-4)");
        it_not_run("INT-14", "APP_CAL_MOTION_ARMED = 0 (HV-3/HV-4)");
        it_not_run("INT-15", "APP_CAL_MOTION_ARMED = 0 (HV-3/HV-4)");
        it_not_run("INT-16", "APP_CAL_MOTION_ARMED = 0 (HV-3/HV-4)");
    }

    it_ir_stage();         /* INT-17 */
    it_ultrasonic_stage(); /* INT-19 */
    it_barcode_stage();    /* INT-20 */

    /* INT-18: report the sweep that ran during all of the above. */
    tm_printf((UB *) "[INT] INT-18 INFO  servo: %u sweeps, %u positions, "
                     "last %d deg at %u cm\n",
              (unsigned) g_it_sweeps, (unsigned) g_it_steps,
              (int) g_it_last_deg, (unsigned) g_it_last_cm);
    if (0 == APP_CAL_SERVO_ARMED)
    {
        it_not_run("INT-18", "APP_CAL_SERVO_ARMED = 0 (HV-7): servo not "
                             "energised");
    }
    else
    {
        it_check("INT-18", g_it_sweeps >= IT_SERVO_MIN_SWEEPS,
                 "servo kept sweeping -60..+60 deg during the whole run");
    }
    it_sweep_stop();
    (void) it_brake();

    tm_printf((UB *) "[TEST] integration SUMMARY %d passed, %d failed, %d "
                     "not run\n",
              g_it_pass, g_it_fail, g_it_skip);
}

#else /* !APP_TEST */

EXPORT void
integration_test (void)
{
}

#endif /* APP_TEST */
