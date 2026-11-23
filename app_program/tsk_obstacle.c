/**
 * @file    tsk_obstacle.c
 * @brief   TSK_OBSTACLE_SCAN (DR Section 3 / 6.2): forward monitoring,
 *          coarse + fine scan, obstacle profile, bypass and line
 *          reacquisition (FR9-FR12, NFR1, NFR8, NFR21).
 *
 * IS09 port (19 Nov): STAGE0-STAGE3 use the IS09 ultrasonic module
 * (is09/subsystem5/scan_core, Buddy 5) through is09_scan.c; the bypass
 * (STAGE4-5) is from the base code. While the IS09 gate APP_CAL_ECHO_SHIFTER_VERIFIED
 * is 0 (HV-5 not signed: ECHO not yet measured <= 3.3 V), TRIG is never
 * pulsed, so no obstacle is ever seen.
 * STAGE0 (LINE FOLLOWING): servo straight ahead, one ping every ~70 ms;
 *   IS09 forward watch: two readings in a row at or inside
 *   APP_CAL_D_STOP_CM (25 cm) -> STOP at once (NFR1) and
 *   EVF_OBSTACLE_TRIGGER (TSK_NAV_PLANNER switches to OBSTACLE AVOIDANCE),
 *   once per approach. Five no-echo readings in a row -> FAULT_ULTRASONIC
 *   (report only, IS09 NO_ECHO). Two raw readings < 8 cm -> EVF_PROXIMITY
 *   (E-stop).
 * STAGE1 IS09 coarse scan -60..+60 deg every 15 deg, one ping each; nothing
 *   at or inside APP_CAL_R_PROFILE_CM (45 cm) = IS09 CONTINUE -> back to
 *   line following.
 * STAGE2 IS09 fine scan, 5 deg steps across what the coarse scan found,
 *   one coarse step wider each side.
 * STAGE3 IS09 profile: edges, width, closest point and the recommended
 *   side (TURN_LEFT / TURN_RIGHT: room for the car within
 *   APP_CAL_Y_PATH_MAX_CM and nothing within APP_CAL_R_SIDE_FREE_CM beyond
 *   that edge). The same samples also build the base profile
 *   (obstacle_logic.c), which the bypass planner needs -> MBX_OBSTACLE.
 *   IS09's side is used when it recommends one that is not blocked;
 *   when IS09 says STOP or REVERSE (too close to pivot) the base choice
 *   is used: the side with more clearance, a tie -> a 90 deg look each way
 *   (pivot 30 + servo 60); no room -> reverse 100 mm and scan again (once).
 *   Then reverse only if the first 45 deg leg would pass the obstacle's
 *   near corner closer than half the car + OBST_MARGIN_MM
 *   (obstacle_standoff_mm).
 * STAGE4 bypass: pivot 45 away, leg d1, pivot back, leg d2 past it, each
 *   waiting for EVF_MOTION_DONE. Leg d2 (28 Oct) follows the obstacle's
 *   side at 60 deg (the servo's limit) and keeps going until the side has
 *   ended, so long obstacles work; before turning back the car pivots 30
 *   deg towards it and looks square at its side (servo 60), driving on in
 *   100 mm steps while it is still there. Every forward leg (and the STAGE5 return)
 *   is watched: the sensor sweeps 0 / 30 / 0 / 60 deg towards the obstacle
 *   side; two pings at the same angle inside the car's path closer than
 *   OBST_LEG_STOP_CM = this side is blocked (an angled or soft obstacle the
 *   scan missed, a second obstacle): stop, back out along the legs to the scan pose, then
 *   go round the other side if the scan showed room there, else reverse
 *   100 mm and scan again without the blocked side (once). The 8 cm
 *   proximity E-stop stays as the last resort.
 * STAGE5 (28 Oct) the bypass keeps an encoder dead-reckoned pose relative to
 *   the original path; pivot back to 45 deg towards it and drive diagonally
 *   until TSK_LINE_BARCODE reports EVF_LINE_FOUND (re-armed watch), or until
 *   OBST_RETURN_PAST_MM past the path. Found: forward to put the axle on
 *   the line and turn parallel to the path. Not seen: reverse back onto the
 *   path, turn parallel, and line following's search finds the line
 *   (instead of the DR's 300 mm abort) -> EVF_BYPASS_COMPLETE.
 * Anything failing -> AVOIDANCE_ABORT: stop, EVF_BYPASS_ABORT (-> ERROR).
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "motion_api.h"
#include "obstacle_logic.h"
#include "is09_glue.h"
#include "app_params.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"

#define PING_GAP_MS       (70)        /* HC-SR04: > 60 ms between pings */
#define ECHO_WAIT_MS      (30)
LOCAL scan_t             g_fine;
LOCAL obstacle_profile_t g_prof;
LOCAL uint32_t           g_prox_n;
LOCAL float              g_servo_now;         /* last commanded robot angle */

#define SERVO_STEP_DEG    (10.0f)     /* ramp the pan in small steps: a 60-120 deg  */
#define SERVO_STEP_MS     (25)        /* jump whips the HC-SR04 and its cable loose */

/* robot angle (+ right) -> servo physical angle */
LOCAL void servo_to(float robot_deg)
{
    float phys = params_servo_phys(robot_deg);          /* SERVO_TRIM_DEG included */

    hal_servo_deg(phys);
    g_servo_now = robot_deg;
}

LOCAL float ping(void)
{
    float cm;

    if (!is09_us_enabled())
    {
        (void)tk_dly_tsk(ECHO_WAIT_MS);        /* IS09 HV-5 gate: TRIG never pulsed */
        g_status.obst_cm = -1.0f;
        return -1.0f;
    }
    hal_us_trigger();
    (void)tk_dly_tsk(ECHO_WAIT_MS);
    cm = hal_us_read_cm();
    g_status.obst_cm = cm;

    /* critical proximity, checked on every ping (2 in a row) */
    g_prox_n = ((cm >= 0.0f) && (cm < (float)PROXIMITY_STOP_CM)) ? (g_prox_n + 1u) : 0u;
    if (g_prox_n >= 2u)
    {
        flag_set(EVF_PROXIMITY);
    }
    return cm;
}

/* bench mode (19 Nov): one ping in IDLE for the dashboard. No proximity
 * E-stop and no watch: the car is standing still and a hand is expected. */
LOCAL void bench_ping(void)
{
    hal_us_trigger();
    (void)tk_dly_tsk(ECHO_WAIT_MS);
    g_status.obst_cm = hal_us_read_cm();
}

LOCAL float ping_median3(void)
{
    float a = ping();
    float b;
    float c;

    (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
    b = ping();
    (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
    c = ping();
    /* no echo counts as "far" */
    a = (a < 0.0f) ? 1000.0f : a;
    b = (b < 0.0f) ? 1000.0f : b;
    c = (c < 0.0f) ? 1000.0f : c;
    a = median3(a, b, c);
    return (a >= 999.0f) ? -1.0f : a;
}

LOCAL void servo_move(float robot_deg, float from_deg)
{
    float a = g_servo_now;

    (void)from_deg;                           /* the real start is g_servo_now */
    while ((robot_deg - a) > SERVO_STEP_DEG)
    {
        a += SERVO_STEP_DEG;
        servo_to(a);
        (void)tk_dly_tsk(SERVO_STEP_MS);
    }
    while ((a - robot_deg) > SERVO_STEP_DEG)
    {
        a -= SERVO_STEP_DEG;
        servo_to(a);
        (void)tk_dly_tsk(SERVO_STEP_MS);
    }
    servo_to(robot_deg);
    (void)tk_dly_tsk((TMO)SERVO_SETTLE_MS + SERVO_STEP_MS);   /* DR A6 settle */
}

LOCAL bool aborted(void)
{
    return ((flag_peek() & (EVF_ESTOP | EVF_PROXIMITY)) != 0u) ||
           (sys_state() != SYS_OBSTACLE_AVOIDANCE);
}

/* One IS09 scan plan: servo to each angle index, one ping each. */
LOCAL bool scan_plan(const uint8_t *p_idx, uint32_t n)
{
    for (uint32_t i = 0u; i < n; i++)
    {
        servo_move(is09_scan_deg(p_idx[i]), g_servo_now);   /* >= 75 ms, so pings stay > 60 ms apart */
        is09_scan_set(p_idx[i], is09_us_cm(ping()));
        if (aborted())
        {
            return false;
        }
    }
    return true;
}

/* STAGE1-3 with the IS09 module: coarse plan, fine plan, IS09 profile. The
   same samples fill the base coarse[5] (-60 -30 0 30 60) and fine scan_t
   (every sampled angle, left to right) for obstacle_profile(). */
LOCAL bool is09_scan(float *p_coarse, is09_profile_t *p_v)
{
    static const uint8_t COARSE_IDX[COARSE_N] = { 30u, 24u, 18u, 12u, 6u };   /* = COARSE_DEG[] */
    uint8_t  plan[IS09_SCAN_MAX];
    uint32_t n;

    is09_scan_begin();
    n = is09_scan_coarse_plan(plan, IS09_SCAN_MAX);
    if (!scan_plan(plan, n))
    {
        return false;
    }
    n = is09_scan_fine_plan(plan, IS09_SCAN_MAX);
    if (!scan_plan(plan, n))
    {
        return false;
    }
    servo_move(0.0f, g_servo_now);

    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        uint16_t r = is09_scan_get(COARSE_IDX[i]);

        p_coarse[i] = (r == IS09_RANGE_NONE) ? -1.0f : (float)r;
        g_status.scan_cm[i] = p_coarse[i];
    }
    g_fine.n = 0u;
    for (uint32_t k = IS09_SCAN_MAX; (k > 0u) && (g_fine.n < OBS_MAX_POINTS); k--)
    {
        uint32_t idx = k - 1u;                /* 36 (left) .. 0 (right) = -90 .. +90 deg */

        if (is09_scan_sampled(idx))
        {
            uint16_t r = is09_scan_get(idx);

            g_fine.deg[g_fine.n] = is09_scan_deg(idx);
            g_fine.cm[g_fine.n]  = (r == IS09_RANGE_NONE) ? -1.0f : (float)r;
            g_fine.n++;
        }
    }
    is09_scan_profile(p_v);
    return true;
}

LOCAL void is09_profile_event(const is09_profile_t *p_v)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "IS09 ");
    jb_raw(&jb, is09_action_name(p_v->action));
    jb_raw(&jb, " closest ");
    jb_uint(&jb, p_v->closest_cm);
    jb_raw(&jb, "cm at ");
    jb_fixed(&jb, p_v->closest_deg, 0u);
    jb_raw(&jb, " w ");
    jb_uint(&jb, p_v->width_cm);
    jb_raw(&jb, "cm edges ");
    jb_fixed(&jb, p_v->left_edge_cm, 0u);
    jb_char(&jb, '/');
    jb_fixed(&jb, p_v->right_edge_cm, 0u);
    (void)post_event("IS09_PROFILE", text);
    log_line(text);
}

/* ---- bypass legs: a log for backing out, and STAGE0-style watching ---- */
#define LEG_LOG_MAX       (8u)

typedef enum
{
    LEG_OK = 0,
    LEG_BLOCKED,          /* something ahead closer than OBST_LEG_STOP_CM: back out */
    LEG_SEARCH,           /* back on the original path without seeing the line: line following searches */
    LEG_FAILED            /* E-stop, proximity, timeout, state change */
} leg_res_t;

typedef struct
{
    bool  b_turn;
    float amount;         /* deg (+ clockwise) or mm (+ forward) actually done */
} leg_t;

LOCAL leg_t       g_legs[LEG_LOG_MAX];
LOCAL uint32_t    g_n_legs;
LOCAL float       g_block_cm;           /* distance when a leg was blocked */
LOCAL float       g_block_deg;          /* ... and the sensor angle */

/* leg watch: sensor angles (x the side the obstacle is on), one ping each */
#define WATCH_N           (4u)
LOCAL const float WATCH_DEG[WATCH_N] = { 0.0f, 30.0f, 0.0f, 60.0f };
LOCAL float       g_backed_mm;          /* reversed so far in this avoidance */
LOCAL const char *g_why;                /* why run_bypass() failed */
LOCAL float       g_scan_w_cm;          /* 20 Nov: width the scan measured, before OBST_WIDTH_CM */

/* 20 Nov (OBST_MEASURE): the box measured while driving, in the bypass
   frame (x + = right of the original path, y + = along it, from where the
   bypass started). Distances use the measured angle k, not the encoders'. */
typedef struct
{
    bool  b_edge;         /* near edge found on the diagonal */
    float edge_x_mm;      /* near edge, lateral */
    float face_y_mm;      /* box front */
    float k;              /* cos of the real diagonal angle (measured) */
    float x_end_mm;       /* axle at the end of the diagonal */
    float y_end_mm;
    bool  b_side;         /* side gap measured on the side pass */
    float gap_mm;         /* sensor to the box side, square to it */
    float side_x_mm;      /* the box side, lateral */
    bool  b_back;         /* back corner found */
    float back_y_mm;
} meas_t;
LOCAL meas_t      g_meas;
LOCAL edge_meas_t g_edge;
#define OBST_MEAS_MM_S    (45.0f)     /* slower on the measuring leg: a ping every ~5 mm */
#define OBST_MEAS_MAX_MM  (900.0f)
#define OBST_MEAS_NEAR_MM (80.0f)     /* box front this close to the sensor before the edge: blocked */

/* Dead reckoning from the encoders, relative to where the bypass started
   (on the line, heading along it): x + = right of the original path,
   y + = along it, h = heading (deg, + = clockwise). Updated in step(). */
typedef struct
{
    float x_mm;
    float y_mm;
    float h_deg;
    float h0_deg;
    float d_prev;
} pose_t;

LOCAL pose_t g_pose;

LOCAL void event2(const char *p_name, const char *p_a, float x, const char *p_b);

LOCAL void pose_reset(void)
{
    odometry_t o;

    odometry_get(&o);
    g_pose.x_mm   = 0.0f;
    g_pose.y_mm   = 0.0f;
    g_pose.h_deg  = 0.0f;
    g_pose.h0_deg = o.heading_deg;
    g_pose.d_prev = 0.5f * (o.dist_r_mm + o.dist_l_mm);
    g_status.byp_x_mm  = 0.0f;
    g_status.byp_y_mm  = 0.0f;
    g_status.byp_h_deg = 0.0f;
}

LOCAL void pose_update(void)
{
    odometry_t o;
    float      d;
    float      h;

    odometry_get(&o);
    d = 0.5f * (o.dist_r_mm + o.dist_l_mm);
    h = o.heading_deg - g_pose.h0_deg;
    obstacle_pose_advance(&g_pose.x_mm, &g_pose.y_mm, g_pose.h_deg, h, d - g_pose.d_prev);
    g_pose.h_deg  = h;
    g_pose.d_prev = d;
    g_status.byp_x_mm  = g_pose.x_mm;          /* 20 Nov: shown live on the dashboard */
    g_status.byp_y_mm  = g_pose.y_mm;
    g_status.byp_h_deg = g_pose.h_deg;
}

LOCAL float odo_fwd_mm(void)
{
    odometry_t o;

    odometry_get(&o);
    return 0.5f * (o.dist_r_mm + o.dist_l_mm);
}

/* Runs one primitive; pings ahead while it moves (forward legs only).
   watch_sign != 0 (bypass legs): the sensor sweeps 0 / 30 / 0 / 60 deg
   towards the obstacle side (+1 = right) and a ping inside the car's path
   (obstacle_in_path: car width + OBST_PATH_SIDE_MM, closer than
   OBST_LEG_STOP_CM ahead), confirmed by a second ping at the same angle,
   stops it: LEG_BLOCKED. This sees an obstacle edge coming at the car's
   front corner, which a straight-ahead ping cannot. stop_bits (e.g.
   EVF_LINE_FOUND) end it early with LEG_OK and *p_stop_hit set. */
LOCAL leg_res_t step(uint16_t id, bool b_watch_ahead, float watch_sign, uint32_t stop_bits, uint32_t *p_stop_hit)
{
    uint32_t t0 = hal_time_ms();
    bool     b_sweep = b_watch_ahead && (watch_sign != 0.0f) && (g_params.obst_leg_stop_cm > 0.0f);
    uint32_t k = 0u;

    *p_stop_hit = 0u;
    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;
        bool            b_done;

        b_done = motion_wait(id, stop_bits | EVF_ESTOP | EVF_PROXIMITY, b_sweep ? 10u : (uint32_t)PING_GAP_MS, &res, &other);
        pose_update();
        if (b_done)
        {
            return (res == MOTION_OK) ? LEG_OK : LEG_FAILED;
        }
        if ((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u)
        {
            return LEG_FAILED;
        }
        if ((other & stop_bits) != 0u)
        {
            *p_stop_hit = other & stop_bits;
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(200);
            pose_update();
            return LEG_OK;
        }
        if (sys_state() != SYS_OBSTACLE_AVOIDANCE)
        {
            motion_stop(SRC_OBSTACLE);
            return LEG_FAILED;
        }
        if (b_sweep)
        {
            float a = watch_sign * WATCH_DEG[k % WATCH_N];
            float cm;

            k++;
            servo_move(a, g_servo_now);       /* >= 75 ms: pings stay > 60 ms apart */
            cm = ping();
            if (obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
            {
                (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
                cm = ping();                  /* NFR8: one echo is not enough */
                if (obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
                {
                    motion_stop(SRC_OBSTACLE);
                    g_block_cm  = cm;
                    g_block_deg = a;
                    (void)tk_dly_tsk(300);    /* come to rest before the distance is read */
                    pose_update();
                    return LEG_BLOCKED;
                }
            }
        }
        else if (b_watch_ahead)
        {
            (void)ping();
        }
        else
        {
            /* turns, reversing: no pings */
        }
        if ((hal_time_ms() - t0) > 30000u)
        {
            motion_stop(SRC_OBSTACLE);
            return LEG_FAILED;
        }
    }
}

LOCAL void leg_log(bool b_turn, float amount)
{
    if (g_n_legs < LEG_LOG_MAX)
    {
        g_legs[g_n_legs].b_turn = b_turn;
        g_legs[g_n_legs].amount = amount;
        g_n_legs++;
    }
}

/* pivot, logged for backing out */
LOCAL leg_res_t leg_turn(float deg)
{
    uint32_t  hit;
    leg_res_t r = step(motion_start_turn(deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);

    if (r == LEG_OK)
    {
        leg_log(true, deg);
    }
    return r;
}

/* forward leg, watched (sweep towards watch_sign), logged (the distance really driven) */
LOCAL leg_res_t leg_move(float mm, float watch_sign, uint32_t stop_bits, uint32_t *p_hit)
{
    float     s0 = odo_fwd_mm();
    leg_res_t r  = step(motion_start_move(mm, g_params.bypass_mm_s, SRC_OBSTACLE), true, watch_sign, stop_bits, p_hit);

    if (r != LEG_FAILED)
    {
        leg_log(false, odo_fwd_mm() - s0);
    }
    return r;
}

/* straight back (not logged: used before a scan, the scan pose moves) */
LOCAL bool reverse_mm(float mm)
{
    uint32_t hit;
    bool     b_ok = step(motion_start_move(-mm, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) == LEG_OK;

    if (b_ok)
    {
        g_backed_mm += mm;
    }
    return b_ok;
}

/* undo the logged legs in reverse order: back to the pose of the last scan,
   along the path just driven (the only ground known to be free: no rear sensor) */
LOCAL bool back_out(void)
{
    uint32_t hit;

    while (g_n_legs > 0u)
    {
        const leg_t *p_l = &g_legs[--g_n_legs];
        leg_res_t    r   = LEG_OK;

        if (p_l->b_turn)
        {
            r = step(motion_start_turn(-p_l->amount, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
        }
        else if ((p_l->amount > 2.0f) || (p_l->amount < -2.0f))
        {
            r = step(motion_start_move(-p_l->amount, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
        }
        else
        {
            /* blocked at once: nothing to undo */
        }
        if (r != LEG_OK)
        {
            return false;
        }
    }
    return true;
}

/* 20 Nov (OBST_MEASURE): diagonal leg that measures the box's near edge
   (edge_meas_t, obstacle_logic.h). The servo points along the original
   path; the leg ends when the axle is gap_mm past the measured edge
   (gap_mm = half car + margin). Only pings within EDGE_GATE_MM of where
   the scan puts the box front count as the box (20:00: the background was
   taken for the box, front 1300 mm). Box not seen in the first 3 pings
   (narrow box, or a turned face): edge assumed 50 mm beyond the sensor.
   The front used later is the pings' if 3+ saw the box, else the scan's,
   and never more than 300 mm beyond the scan's. Box front closer than OBST_MEAS_NEAR_MM before
   the edge, or no edge within OBST_MEAS_MAX_MM: LEG_BLOCKED (back out and
   retry, as for the other legs). No 0/30/60 sweep on this leg: the sensor
   is busy measuring; the 8 cm proximity stop still works on every ping. */
LOCAL leg_res_t leg_diag_measure(float gap_mm, float side, float fallback_mm)
{
    float     ua = (float)OBST_US_AHEAD_MM;
    float     s0 = odo_fwd_mm();
    float     x0 = g_pose.x_mm;
    float     y0 = g_pose.y_mm;
    float     look = -g_pose.h_deg;
    float     target = -1.0f;
    float     s_end;
    float     face_scan = ua + (g_prof.r_min_cm * 10.0f);      /* box front, bypass frame (scan) */
    uint32_t  t0 = hal_time_ms();
    uint16_t  id;
    leg_res_t r = LEG_OK;
    char      text[TLM_EVENT_DETAIL_LEN];
    jbuf_t    jb;

    look = (look > 60.0f) ? 60.0f : ((look < -60.0f) ? -60.0f : look);
    (void)fallback_mm;
    edge_reset(&g_edge, g_pose.h_deg, 0.0f);
    g_edge.a_scan = face_scan - (y0 + (ua * g_edge.k));        /* sensor to front along the path, s = 0 */
    g_edge.a      = g_edge.a_scan;
    g_meas.b_edge = false;
    g_meas.b_side = false;
    g_meas.b_back = false;
    servo_move(look, g_servo_now);
    id = motion_start_move(OBST_MEAS_MAX_MM, OBST_MEAS_MM_S, SRC_OBSTACLE);
    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;
        float           cm;
        float           s;

        if (motion_wait(id, EVF_ESTOP | EVF_PROXIMITY, 10u, &res, &other))
        {
            pose_update();
            if (res != MOTION_OK)
            {
                r = LEG_FAILED;
                break;
            }
            g_block_cm  = OBST_MEAS_MAX_MM / 10.0f;
            g_block_deg = 999.0f;             /* marker: no edge within the leg */
            r = LEG_BLOCKED;
            break;
        }
        pose_update();
        if (((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u) || (sys_state() != SYS_OBSTACLE_AVOIDANCE) ||
            ((hal_time_ms() - t0) > 40000u))
        {
            motion_stop(SRC_OBSTACLE);
            r = LEG_FAILED;
            break;
        }
        cm = ping();
        s  = odo_fwd_mm() - s0;
        edge_add(&g_edge, s, (cm < 0.0f) ? -1.0f : (cm * 10.0f));
        if (!g_edge.b_edge && ((g_edge.a - (g_edge.k * s)) < OBST_MEAS_NEAR_MM))
        {
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(300);
            pose_update();
            g_block_cm  = (g_edge.a - (g_edge.k * s)) / 10.0f;
            g_block_deg = look;
            r = LEG_BLOCKED;
            break;
        }
        if (g_edge.b_edge && (g_edge.n_cross > 0u))
        {
            target = edge_s_cross(&g_edge) + ua + (gap_mm / edge_sin(&g_edge));
        }
        if ((target >= 0.0f) && (s >= target))
        {
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(200);
            pose_update();
            r = LEG_OK;
            break;
        }
        (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
    }
    s_end = odo_fwd_mm() - s0;
    leg_log(false, s_end);
    servo_move(0.0f, g_servo_now);
    if (r != LEG_OK)
    {
        return r;
    }
    g_meas.k        = g_edge.k;
    g_meas.x_end_mm = x0 + (side * s_end * edge_sin(&g_edge));
    g_meas.y_end_mm = y0 + (s_end * g_edge.k);
    jb_init(&jb, text, sizeof(text));
    if (g_edge.n_cross > 0u)
    {
        float sx = edge_s_cross(&g_edge);
        float face = (g_edge.n >= 3u) ? (y0 + (ua * g_edge.k) + g_edge.a) : face_scan;

        /* 20:00: never trust a front far beyond the scan's (that was the background) */
        face = (face > (face_scan + 300.0f)) ? (face_scan + 300.0f) : face;
        face = (face < (face_scan - 50.0f)) ? (face_scan - 50.0f) : face;
        g_meas.b_edge    = true;
        g_meas.edge_x_mm = x0 + (side * (sx + ua) * edge_sin(&g_edge));
        g_meas.face_y_mm = face;
        jb_raw(&jb, "x=");     jb_fixed(&jb, g_meas.edge_x_mm, 0u);
        jb_raw(&jb, " face="); jb_fixed(&jb, g_meas.face_y_mm, 0u);
        jb_raw(&jb, " fs=");   jb_fixed(&jb, face_scan, 0u);
        jb_raw(&jb, " seen="); jb_uint(&jb, g_edge.n);
        jb_raw(&jb, g_edge.b_unseen ? " un=1" : " un=0");
        jb_raw(&jb, " leg=");  jb_fixed(&jb, s_end, 0u);
        jb_raw(&jb, " w2=");   jb_fixed(&jb, (g_meas.edge_x_mm < 0.0f) ? (-2.0f * g_meas.edge_x_mm) : (2.0f * g_meas.edge_x_mm), 0u);
        (void)post_event("OBST_EDGE", text);
    }
    else
    {
        jb_raw(&jb, "edge not measured (");
        jb_raw(&jb, "no crossing");
        jb_raw(&jb, "): diagonal ");
        jb_fixed(&jb, s_end, 0u);
        jb_raw(&jb, " mm");
        (void)post_event("OBST_EDGE", text);
    }
    return r;
}

/* STAGE4 leg 2, parallel to the path (28 Oct). The servo cannot look 90 deg
   sideways, so the obstacle's side is followed at 60 deg: the leg drives at
   least min_mm (the plan, with OBST_DEPTH_MM), then on while the 60 deg
   ping still hits the side (obstacle_side_seen), and trail_mm past where it
   was last seen (obstacle_side_trail_mm), at most OBST_SIDE_MAX_MM (then:
   LEG_BLOCKED, the side never ended). Pings alternate 0 / 60 deg and
   anything inside the car's path stops it, as on the other legs. */
LOCAL leg_res_t leg_side_follow(float min_mm, float gap_mm, float trail_mm, float ws)
{
    float     side_r[3] = { -1.0f, -1.0f, -1.0f };   /* 20 Nov: last 60 deg pings (mm) for the side gap */
    uint32_t  side_k = 0u;
    uint32_t  back_n = 0u;
    bool      b_meas = (g_params.obst_measure > 0.5f) && g_meas.b_edge;
    float     s0 = odo_fwd_mm();
    float     seen_at = -1.0f;              /* leg distance where the side was last seen */
    uint32_t  clear_n = 0u;
    uint32_t  k = 0u;
    uint32_t  t0 = hal_time_ms();
    uint16_t  id = motion_start_move((float)OBST_SIDE_MAX_MM, g_params.bypass_mm_s, SRC_OBSTACLE);
    leg_res_t r;

    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;
        float           a;
        float           cm;
        float           done;

        if (motion_wait(id, EVF_ESTOP | EVF_PROXIMITY, 10u, &res, &other))
        {
            pose_update();
            if (res != MOTION_OK)
            {
                return LEG_FAILED;
            }
            g_block_cm  = (float)OBST_SIDE_MAX_MM / 10.0f;
            g_block_deg = 999.0f;               /* marker: the side did not end */
            r = LEG_BLOCKED;
            break;
        }
        pose_update();
        if (((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u) || (sys_state() != SYS_OBSTACLE_AVOIDANCE) ||
            ((hal_time_ms() - t0) > 40000u))
        {
            motion_stop(SRC_OBSTACLE);
            return LEG_FAILED;
        }

        a = ((k % 2u) == 0u) ? 0.0f : (ws * 60.0f);
        k++;
        servo_move(a, g_servo_now);
        cm = ping();
        if ((g_params.obst_leg_stop_cm > 0.0f) &&
            obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
        {
            (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
            cm = ping();
            if (obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
            {
                motion_stop(SRC_OBSTACLE);
                g_block_cm  = cm;
                g_block_deg = a;
                (void)tk_dly_tsk(300);
                pose_update();
                r = LEG_BLOCKED;
                break;
            }
        }
        done = odo_fwd_mm() - s0;
        if (b_meas && (a != 0.0f) && (cm >= 0.0f))
        {
            /* 20 Nov: at 60 deg the wide beam includes the square-on point of
               the box side, so once the sensor is beside the box the range is
               the side gap (3 pings within 8 mm). Past the back corner it is
               the corner: r^2 = gap^2 + dy^2, dy behind the sensor. */
            float rr = cm * 10.0f;

            side_r[side_k % 3u] = rr;
            side_k++;
            if (!g_meas.b_side && (side_k >= 3u))
            {
                float lo = side_r[0], hi = side_r[0];

                for (uint32_t i = 1u; i < 3u; i++)
                {
                    lo = (side_r[i] < lo) ? side_r[i] : lo;
                    hi = (side_r[i] > hi) ? side_r[i] : hi;
                }
                if ((hi - lo) <= 8.0f)
                {
                    g_meas.b_side    = true;
                    g_meas.gap_mm    = lo;
                    g_meas.side_x_mm = g_meas.x_end_mm + (ws * lo);
                }
            }
            else if (g_meas.b_side && !g_meas.b_back)
            {
                g_meas.gap_mm = (rr < g_meas.gap_mm) ? rr : g_meas.gap_mm;
                back_n = (rr >= (g_meas.gap_mm + EDGE_TOL_MM)) ? (back_n + 1u) : 0u;
                if (back_n >= 2u)
                {
                    float  dy = obstacle_leg_of(rr, g_meas.gap_mm);
                    char   text[TLM_EVENT_DETAIL_LEN];
                    jbuf_t jb;

                    g_meas.b_back    = true;
                    g_meas.side_x_mm = g_meas.x_end_mm + (ws * g_meas.gap_mm);
                    g_meas.back_y_mm = (g_meas.y_end_mm + (float)OBST_US_AHEAD_MM + done) - dy;
                    jb_init(&jb, text, sizeof(text));
                    jb_raw(&jb, "gap=");    jb_fixed(&jb, g_meas.gap_mm, 0u);
                    jb_raw(&jb, " x=");     jb_fixed(&jb, g_meas.side_x_mm, 0u);
                    jb_raw(&jb, " back=");  jb_fixed(&jb, g_meas.back_y_mm, 0u);
                    jb_raw(&jb, " depth="); jb_fixed(&jb, g_meas.back_y_mm - g_meas.face_y_mm, 0u);
                    jb_raw(&jb, " w2=");    jb_fixed(&jb, (g_meas.side_x_mm < 0.0f) ? (-2.0f * g_meas.side_x_mm) : (2.0f * g_meas.side_x_mm), 0u);
                    (void)post_event("OBST_SIDE", text);
                }
            }
            else
            {
                /* done */
            }
        }
        if (a != 0.0f)
        {
            if (obstacle_side_seen(cm, gap_mm))
            {
                seen_at = done;
                clear_n = 0u;
            }
            else
            {
                clear_n++;
            }
        }
        if ((done >= min_mm) && ((seen_at < 0.0f) || ((clear_n >= 2u) && (done >= (seen_at + trail_mm)))))
        {
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(200);
            pose_update();
            r = LEG_OK;
            break;
        }
    }
    leg_log(false, odo_fwd_mm() - s0);
    return r;
}

/* Before the 45 deg turn back (28 Oct). The servo stops at 60 deg and a
   flat side often gives no echo at 60, so the car pivots OBST_LOOK_PIVOT_DEG
   towards the obstacle (the way it is about to turn anyway) and pings with
   the servo at 60 (square to the side) and 30, then pivots back. Side still
   there: drive on OBST_SIDE_STEP_MM (watched) and look again, at most
   OBST_TURN_LOOK_STEPS times (then LEG_BLOCKED: the side did not end).
   Side gone: drive on obstacle_side_trail90_mm, then the turn is clear. */
LOCAL leg_res_t look_before_turn(float ws, float gap_mm, float margin_mm)
{
    uint32_t hit = 0u;
    float    piv = (float)OBST_LOOK_PIVOT_DEG;

    for (uint32_t n = 0u; ; n++)
    {
        float     d90;
        float     d60;
        leg_res_t r;

        if (step(motion_start_turn(ws * piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
        {
            return LEG_FAILED;
        }
        servo_move(ws * 60.0f, g_servo_now);
        d90 = ping_median3();
        servo_move(ws * 30.0f, g_servo_now);
        d60 = ping_median3();
        servo_move(0.0f, g_servo_now);
        if ((step(motion_start_turn(-ws * piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK) || aborted())
        {
            return LEG_FAILED;
        }
        if (!obstacle_side_beside(d90, gap_mm) &&
            !((d60 >= 0.0f) && ((d60 * 10.0f * 0.8660254f) <= ((gap_mm - 40.0f) + 100.0f))))
        {
            if ((g_params.obst_measure > 0.5f) && g_meas.b_edge)
            {
                return LEG_OK;      /* 20 Nov: the side leg was sized for the turn already (bypass_side_leg_mm) */
            }
            return leg_move(obstacle_side_trail90_mm(gap_mm, (float)OBST_US_AHEAD_MM, g_params.robot_half_width_mm, margin_mm),
                            ws, 0u, &hit);
        }
        if (n >= (uint32_t)OBST_TURN_LOOK_STEPS)
        {
            g_block_cm  = (float)OBST_SIDE_MAX_MM / 10.0f;
            g_block_deg = 999.0f;                   /* the side did not end */
            return LEG_BLOCKED;
        }
        event2("BYPASS_EXTEND", "obstacle still beside the car (", (d90 >= 0.0f) ? d90 : d60, " cm): 100 mm further");
        r = leg_move((float)OBST_SIDE_STEP_MM, ws, 0u, &hit);
        if (r != LEG_OK)
        {
            return r;
        }
    }
}

/* 20 Nov: rejoin by the line sensors, not by the encoder angle. After the
   bypass the encoders' "parallel to the path" can be well off (the pivots
   over-turn), so the car met the line at an angle, saw both sensors black,
   took it for a junction and drove straight across. Now the car pivots
   slowly until the line is between the two sensors: the leading sensor (the
   left one for a left pivot) has been black and is white again, or the
   trailing one goes black. 20 Nov 20:16 run: the first version pivoted the
   first 15 deg on the encoders without looking, the line passed both
   sensors in that time (the pivots over-turn until TURN_CAL), and the slow
   part found nothing. The sensors are now watched from the start: the line
   was just crossed, so it is close. Gives up after the angle + REJOIN_EXTRA_DEG.
   20:30 / 20:32 runs: the CENTRE (leading) sensor was on the line when the
   pivot started, went white (the line was then between the sensors) and the
   car kept going, because a sensor that was black at the start was ignored;
   it swept ~90 deg on the encoders (~130 deg real) and line following then
   searched: a near 180 deg turn. Rule now in rejoin_watch_step()
   (obstacle_logic.c): leading black at the start counts. If the trailing
   sensor is on the line at the start, the axle has not reached the line:
   creep forward (up to REJOIN_CREEP_MM) until it is white, then pivot. If
   the line is not found, turn back REJOIN_EXTRA_DEG to the encoder angle. */
#define REJOIN_EXTRA_DEG   (45.0f)
#define REJOIN_MM_S        (45.0f)
#define REJOIN_CREEP_MM    (40.0f)

LOCAL bool line_black(bool b_left)
{
    bool lv = b_left ? hal_ir_left_level() : hal_ir_right_level();

    return lv == (g_params.line_do_black_high > 0.5f);
}

LOCAL bool rejoin_snap(float deg)
{
    float     dir   = (deg < 0.0f) ? -1.0f : 1.0f;
    float     mag   = (deg < 0.0f) ? -deg : deg;
    uint16_t       id;
    rejoin_watch_t w;
    uint32_t       t0;
    uint32_t       hit;
    char           text[TLM_EVENT_DETAIL_LEN];
    jbuf_t         jb;

    if (line_black(dir > 0.0f))          /* trailing sensor on the line: axle short of it */
    {
        id = motion_start_move(REJOIN_CREEP_MM, REJOIN_MM_S, SRC_OBSTACLE);
        t0 = hal_time_ms();
        for (;;)
        {
            motion_result_t res;
            uint32_t        other = 0u;

            if (!line_black(dir > 0.0f))
            {
                motion_stop(SRC_OBSTACLE);
                break;
            }
            if (motion_wait(id, EVF_ESTOP | EVF_PROXIMITY, 10u, &res, &other))
            {
                break;
            }
            if (((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u) || (sys_state() != SYS_OBSTACLE_AVOIDANCE) ||
                ((hal_time_ms() - t0) > 3000u))
            {
                motion_stop(SRC_OBSTACLE);
                return false;
            }
        }
        (void)post_event("BYPASS_REJOIN", "trailing sensor on the line: crept forward before the pivot");
    }
    rejoin_watch_reset(&w);
    id = motion_start_turn(dir * (mag + REJOIN_EXTRA_DEG), REJOIN_MM_S, SRC_OBSTACLE);
    t0 = hal_time_ms();
    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;
        bool            bl = line_black(true);
        bool            bc = line_black(false);

        /* left pivot: the LEFT sensor leads */
        if (rejoin_watch_step(&w, (dir < 0.0f) ? bl : bc, (dir < 0.0f) ? bc : bl))
        {
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(150);
            pose_update();
            jb_init(&jb, text, sizeof(text));
            jb_raw(&jb, "on the line: heading ");
            jb_fixed(&jb, g_pose.h_deg, 0u);
            jb_raw(&jb, " deg from the original path (encoders)");
            (void)post_event("BYPASS_REJOIN", text);
            return true;
        }
        if (motion_wait(id, EVF_ESTOP | EVF_PROXIMITY, 10u, &res, &other))
        {
            if (res != MOTION_OK)
            {
                return false;
            }
            /* not found: go back to the planned heading rather than staying
               REJOIN_EXTRA_DEG past it */
            if (step(motion_start_turn(-dir * REJOIN_EXTRA_DEG, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
            {
                return false;
            }
            pose_update();
            (void)post_event("BYPASS_REJOIN", "line not seen while pivoting: back to the encoder angle, line following will search");
            return true;
        }
        if (((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u) || (sys_state() != SYS_OBSTACLE_AVOIDANCE) ||
            ((hal_time_ms() - t0) > 6000u))
        {
            motion_stop(SRC_OBSTACLE);
            return false;
        }
    }
}

/* 20 Nov: the dashboard shows the plan and each step as it starts.
   BYPASS_STEP details are "<n> <what>"; turns say left / right. */
LOCAL void step_turn_event(uint32_t n, float deg, const char *p_why)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_uint(&jb, n);
    jb_raw(&jb, " pivot ");
    jb_fixed(&jb, (deg < 0.0f) ? -deg : deg, 0u);
    jb_raw(&jb, (deg < 0.0f) ? " deg left" : " deg right");
    jb_raw(&jb, p_why);
    (void)post_event("BYPASS_STEP", text);
}

LOCAL void step_move_event(uint32_t n, const char *p_a, float mm, const char *p_b)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_uint(&jb, n);
    jb_char(&jb, ' ');
    jb_raw(&jb, p_a);
    jb_fixed(&jb, mm, 0u);
    jb_raw(&jb, p_b);
    (void)post_event("BYPASS_STEP", text);
}

/* OBST_BOX and BYPASS_PLAN, "key=value" so the dashboard can draw them.
   All relative to where the bypass starts (after any reversing): r = the
   ultrasonic's distance to the box front (cm), xl / xr = the box's left /
   right edge (cm, + = right), d = assumed depth (mm), ua = sensor ahead of
   the axle (mm). Plan: t = pivots (deg, + = right), d1 = diagonal leg,
   d2 = at least this far along the side, sh = sideways shift, ret = longest
   return (all mm), m = side margin used (mm). */
LOCAL void plan_events(const bypass_plan_t *p_plan, float margin_mm)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "r=");    jb_fixed(&jb, g_prof.r_min_cm, 0u);
    jb_raw(&jb, " xl=");  jb_fixed(&jb, g_prof.x_l_cm, 0u);
    jb_raw(&jb, " xr=");  jb_fixed(&jb, g_prof.x_r_cm, 0u);
    jb_raw(&jb, " w=");   jb_fixed(&jb, g_prof.width_cm, 0u);
    jb_raw(&jb, " w0=");  jb_fixed(&jb, g_scan_w_cm, 0u);
    jb_raw(&jb, " cL=");  jb_fixed(&jb, g_prof.clear_left_cm, 0u);
    jb_raw(&jb, " cR=");  jb_fixed(&jb, g_prof.clear_right_cm, 0u);
    jb_raw(&jb, " d=");   jb_fixed(&jb, g_params.obst_depth_mm, 0u);
    jb_raw(&jb, " ua=");  jb_uint(&jb, (uint32_t)OBST_US_AHEAD_MM);
    (void)post_event("OBST_BOX", text);

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "side=");  jb_char(&jb, (p_plan->side > 0) ? 'R' : 'L');
    jb_raw(&jb, " t1=");   jb_fixed(&jb, p_plan->turn1_deg, 0u);
    jb_raw(&jb, " d1=");   jb_fixed(&jb, p_plan->d1_mm, 0u);
    jb_raw(&jb, " t2=");   jb_fixed(&jb, p_plan->turn2_deg, 0u);
    jb_raw(&jb, " d2=");   jb_fixed(&jb, p_plan->d2_mm, 0u);
    jb_raw(&jb, " t3=");   jb_fixed(&jb, p_plan->turn3_deg, 0u);
    jb_raw(&jb, " sh=");   jb_fixed(&jb, p_plan->shift_mm, 0u);
    jb_raw(&jb, " ret=");  jb_fixed(&jb, p_plan->return_max_mm, 0u);
    jb_raw(&jb, " m=");    jb_fixed(&jb, margin_mm, 0u);
    jb_raw(&jb, (g_params.obst_measure > 0.5f) ? " me=1" : " me=0");
    (void)post_event("BYPASS_PLAN", text);
}

/* STAGE4 + STAGE5 for one side. The turns back use the measured heading
   (pose), and the return is aimed at the ORIGINAL PATH, not a fixed 300 mm:
   it drives until it is OBST_RETURN_PAST_MM past the path, stopping as soon
   as a line sensor sees the line. If it got past the path without seeing
   the line, it reverses back onto the path, turns parallel to it and hands
   over to line following, whose search finds the line (LEG_SEARCH). */
LOCAL leg_res_t run_bypass(const bypass_plan_t *p_plan, float gap_mm, float margin_mm)
{
    uint32_t  hit = 0u;
    leg_res_t r;
    float     ws = -(float)p_plan->side;      /* the obstacle is on the other side of the car */
    float     toward;
    float     s;
    float     dist;
    float     d2;

    g_n_legs = 0u;
    g_why    = "bypass leg failed (timeout, E-stop or proximity)";
    pose_reset();
    step_turn_event(1u, p_plan->turn1_deg, ", away from the box");
    if ((r = leg_turn(p_plan->turn1_deg)) != LEG_OK)
    {
        return r;
    }
    if (g_params.obst_measure > 0.5f)
    {
        (void)post_event("BYPASS_STEP", "2 diagonal: measuring the box edge (sensor along the path)");
        r = leg_diag_measure(gap_mm, (float)p_plan->side, p_plan->d1_mm);
    }
    else
    {
        step_move_event(2u, "forward ", p_plan->d1_mm, " mm on the diagonal");
        r = leg_move(p_plan->d1_mm, ws, 0u, &hit);
    }
    if (r != LEG_OK)
    {
        return r;
    }
    step_turn_event(3u, -g_pose.h_deg, ", parallel to the path");
    if ((r = leg_turn(-g_pose.h_deg)) != LEG_OK)                       /* parallel to the path */
    {
        return r;
    }
    d2 = p_plan->d2_mm;
    if ((g_params.obst_measure > 0.5f) && g_meas.b_edge)
    {
        /* from the measured box front: past it by the assumed depth + car */
        float gap_now = g_meas.x_end_mm - g_meas.edge_x_mm;           /* axle to the box side */

        gap_now = (gap_now < 0.0f) ? -gap_now : gap_now;
        d2 = bypass_side_leg_mm(g_meas.face_y_mm, g_params.obst_depth_mm, g_params.robot_half_width_mm,
                                gap_mm - g_params.robot_half_width_mm, gap_now, g_meas.y_end_mm);
    }
    step_move_event(4u, "along the box side: at least ", d2, " mm, until the side ends");
    if ((r = leg_side_follow(d2, gap_mm,
                             obstacle_side_trail_mm(gap_mm, (float)OBST_US_AHEAD_MM, g_params.robot_half_width_mm, margin_mm),
                             ws)) != LEG_OK)
    {
        return r;
    }
    (void)post_event("BYPASS_STEP", "5 look before turning back: is the box side still beside the car?");
    if ((r = look_before_turn(ws, gap_mm, margin_mm)) != LEG_OK)
    {
        return r;
    }

    /* STAGE5: 45 deg back towards the original path (x = 0) */
    toward = (g_pose.x_mm < 0.0f) ? 1.0f : -1.0f;      /* left of it: turn right */
    step_turn_event(6u, (toward * 45.0f) - g_pose.h_deg, ", back towards the path");
    if ((r = leg_turn((toward * 45.0f) - g_pose.h_deg)) != LEG_OK)
    {
        return r;
    }
    s    = obstacle_sin_deg((g_pose.h_deg < 0.0f) ? -g_pose.h_deg : g_pose.h_deg);
    s    = (s < 0.3f) ? 0.3f : s;
    dist = (((g_pose.x_mm < 0.0f) ? -g_pose.x_mm : g_pose.x_mm) + (float)OBST_RETURN_PAST_MM) / s;
    step_move_event(7u, "return: up to ", dist, " mm, until a line sensor sees the line");
    g_reacq_seq++;                                      /* TSK_LINE: fresh line watch */
    (void)tk_dly_tsk(30);
    flag_clear(EVF_LINE_FOUND);
    r = leg_move(dist, ws, EVF_LINE_FOUND, &hit);
    if (r != LEG_OK)
    {
        return r;
    }
    if ((hit & EVF_LINE_FOUND) != 0u)
    {
        /* axle onto the line, then parallel to the original path */
        (void)post_event("BYPASS_STEP", "8 line found: axle onto the line, then pivot until the line is between the sensors");
        if ((step(motion_start_move(g_params.ir_to_axle_mm, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK) ||
            !rejoin_snap(-g_pose.h_deg))
        {
            g_why = "realign failed";
            return LEG_FAILED;
        }
        return LEG_OK;
    }

    /* past the original path and no line seen: back onto the path (watching
       for the line again), parallel to it, and let line following search */
    {
        float past = toward * g_pose.x_mm;              /* > 0: beyond the path */

        (void)post_event("BYPASS_RETURN", "line not seen crossing the original path: back to it, then search");
        flag_clear(EVF_LINE_FOUND);
        if ((past > 5.0f) &&
            (step(motion_start_move(-past / s, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, EVF_LINE_FOUND, &hit) != LEG_OK))
        {
            g_why = "reverse onto the original path failed";
            return LEG_FAILED;
        }
        if (step(motion_start_turn(-g_pose.h_deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
        {
            g_why = "realign failed";
            return LEG_FAILED;
        }
    }
    return LEG_SEARCH;
}

LOCAL void profile_event(const obstacle_profile_t *p)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "r ");
    jb_fixed(&jb, p->r_min_cm, 0u);
    jb_raw(&jb, "cm w ");
    jb_fixed(&jb, p->width_cm, 0u);
    jb_raw(&jb, "cm off ");
    jb_fixed(&jb, p->offset_cm, 0u);
    jb_raw(&jb, " clrL ");
    jb_fixed(&jb, p->clear_left_cm, 0u);
    jb_raw(&jb, " clrR ");
    jb_fixed(&jb, p->clear_right_cm, 0u);
    jb_raw(&jb, " side ");
    jb_char(&jb, (p->side != 0) ? p->side : '-');
    (void)post_event("OBSTACLE_PROFILE", text);
}

LOCAL void abort_avoidance(const char *p_why)
{
    motion_stop(SRC_OBSTACLE);
    servo_move(0.0f, 0.0f);
    if (sys_state() != SYS_OBSTACLE_AVOIDANCE)
    {
        return;              /* 20 Nov: the run already ended (STOP, pick-up, E-stop): not a bypass fault */
    }
    fault_set(FAULT_BYPASS_ABORT);
    (void)post_event("AVOIDANCE_ABORT", p_why);
    flag_set(EVF_BYPASS_ABORT);
}

LOCAL void event2(const char *p_name, const char *p_a, float x, const char *p_b)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, p_a);
    jb_fixed(&jb, x, 0u);
    jb_raw(&jb, p_b);
    (void)post_event(p_name, text);
}

/* After the scan (and before each bypass attempt): reverse only if the
   first 45 deg leg would pass the near corner closer than the car's half
   width + OBST_MARGIN_MM (obstacle_standoff_mm). The profile is moved with
   the car, so the plan and a later back-out stay consistent. */
LOCAL bool standoff(char side, float margin_mm)
{
    float need = obstacle_standoff_mm(&g_prof, side, g_params.robot_half_width_mm, margin_mm) -
                 (g_prof.r_min_cm * 10.0f);

    if (need < 5.0f)
    {
        return true;
    }
    if (need > (float)OBST_BACKOFF_MAX_MM)
    {
        need = (float)OBST_BACKOFF_MAX_MM;
    }
    event2("OBSTACLE_BACKOFF", "reversing ", need,
           (side == 'L') ? " mm: a 45 deg turn would clip the left corner" : " mm: a 45 deg turn would clip the right corner");
    if (!reverse_mm(need))
    {
        return false;
    }
    g_prof.r_min_cm += need / 10.0f;
    return true;
}

/* 20 Nov (OBST_MEASURE): before the measuring diagonal the box front should
   be OBST_MEAS_FACE_MM from the axle (the scan's width is not used: it is
   the beam, not the box). r_min is the ultrasonic's distance; the sensor
   is OBST_US_AHEAD_MM ahead of the axle. */
LOCAL bool standoff_measure(float add_mm)
{
    float need = (g_params.obst_meas_face_mm + add_mm) - ((g_prof.r_min_cm * 10.0f) + (float)OBST_US_AHEAD_MM);

    if (need < 5.0f)
    {
        return true;
    }
    need = (need > 400.0f) ? 400.0f : need;
    event2("OBSTACLE_BACKOFF", "reversing ", need, " mm before measuring (box front to OBST_MEAS_FACE_MM)");
    if (!reverse_mm(need))
    {
        return false;
    }
    g_prof.r_min_cm += need / 10.0f;
    return true;
}

/* both sides have room and about the same clearance (e.g. both "no echo") */
LOCAL bool is_tie(float half, float margin, const uint32_t *p_tries)
{
    float d = g_prof.clear_left_cm - g_prof.clear_right_cm;

    return (p_tries[0] == 0u) && (p_tries[1] == 0u) &&
           obstacle_side_room(&g_prof, 'L', half, margin) && obstacle_side_room(&g_prof, 'R', half, margin) &&
           (d <= OBST_SIDE_TIE_CM) && (d >= -OBST_SIDE_TIE_CM);
}

/* Tie (28 Oct): instead of defaulting to the smaller detour, look 90 deg
   each way - the servo only reaches 60, so the car pivots OBST_TIE_PIVOT_DEG
   and the servo adds 60 - and take the side with more room there. Back to
   the starting heading afterwards (these pivots are not bypass legs). */
LOCAL char tie_look(char side)
{
    uint32_t hit;
    float    piv = (float)OBST_TIE_PIVOT_DEG;
    float    d_l;
    float    d_r;
    char     text[TLM_EVENT_DETAIL_LEN];
    jbuf_t   jb;

    if (step(motion_start_turn(-piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
    {
        return side;
    }
    servo_move(-60.0f, g_servo_now);
    d_l = ping_median3();
    if ((step(motion_start_turn(2.0f * piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK))
    {
        return side;
    }
    servo_move(60.0f, g_servo_now);
    d_r = ping_median3();
    (void)step(motion_start_turn(-piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
    servo_move(0.0f, g_servo_now);

    d_l = (d_l < 0.0f) ? 200.0f : d_l;      /* no echo = open */
    d_r = (d_r < 0.0f) ? 200.0f : d_r;
    if (d_l > (d_r + OBST_SIDE_TIE_CM))
    {
        side = 'L';
    }
    else if (d_r > (d_l + OBST_SIDE_TIE_CM))
    {
        side = 'R';
    }
    else
    {
        /* still equal: keep the smaller detour */
    }
    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "tie: 90 deg look L ");
    jb_fixed(&jb, d_l, 0u);
    jb_raw(&jb, " cm, R ");
    jb_fixed(&jb, d_r, 0u);
    jb_raw(&jb, " cm -> side ");
    jb_char(&jb, side);
    (void)post_event("OBSTACLE_TIE", text);
    return side;
}

/* STAGE1..STAGE5. Returns when the avoidance is over (either way).
 *   scan -> side with room and more clearance -> reverse only if the 45 deg
 *   leg needs it (standoff) -> bypass, every forward leg watched.
 *   Blocked on a side -> back out along the legs to where it started, then
 *     1. the other side if it was not tried and the scan shows room there
 *        (reverse and bypass);
 *     2. else a side already tried, the one that got further, 100 mm wider
 *        (OBST_RETRY_WIDER_MM; each side at most twice);
 *     3. else reverse 100 mm and scan again (reverse and rescan, once);
 *     4. else AVOIDANCE_ABORT.
 *   No room on either side at the scan -> 3. */
LOCAL void avoid(void)
{
    float         coarse[COARSE_N];
    bypass_plan_t plan;
    uint32_t      rescans = 0u;
    uint32_t      tries[2] = { 0u, 0u };      /* [0] = left, [1] = right */
    uint32_t      progress[2] = { 0u, 0u };   /* legs done before the block */
    float         extra[2] = { 0.0f, 0.0f };  /* added side margin, mm */
    bool          b_scan = true;
    float         half = g_params.robot_half_width_mm;
    float         margin = g_params.obst_margin_mm;
    char          side = 0;

    g_backed_mm = 0.0f;
    g_n_legs    = 0u;

    for (uint32_t round = 0u; round < 6u; round++)
    {
        leg_res_t res;
        uint32_t  si;

        if (b_scan)
        {
            is09_profile_t v;
            uint32_t       blocked = ((tries[0] >= 2u) ? OBST_BLOCKED_L : 0u) | ((tries[1] >= 2u) ? OBST_BLOCKED_R : 0u);
            char           is09_side;

            b_scan = false;
            /* STAGE1-3: IS09 coarse + fine scan and profile */
            if (!is09_scan(coarse, &v) || aborted())
            {
                return;
            }
            is09_profile_event(&v);
            if (v.action == IS09_AVOID_CONTINUE)
            {
                (void)post_event("OBSTACLE_CLEARED", "IS09 scan: nothing within 45 cm, resume line following");
                flag_set(EVF_BYPASS_COMPLETE);
                return;
            }
            if (!obstacle_profile(&g_fine, coarse, half, margin, &g_prof))
            {
                abort_avoidance("profile failed (no echo in the fine scan)");
                return;
            }
            g_scan_w_cm = g_prof.width_cm;
            obstacle_set_width(&g_prof, g_params.obst_width_cm);    /* 20 Nov: known box width */
            /* IS09's recommended side, unless that side is blocked twice */
            is09_side = (v.action == IS09_AVOID_TURN_LEFT) ? 'L' : ((v.action == IS09_AVOID_TURN_RIGHT) ? 'R' : 0);
            if (((is09_side == 'L') && ((blocked & OBST_BLOCKED_L) != 0u)) ||
                ((is09_side == 'R') && ((blocked & OBST_BLOCKED_R) != 0u)))
            {
                is09_side = 0;
            }
            side = (is09_side != 0) ? is09_side : obstacle_choose_side(&g_prof, half, margin, blocked);
            g_prof.side = side;
            g_status.scan_side = side;
            profile_event(&g_prof);
            (void)post_obstacle(&g_prof);
            if ((is09_side == 0) && (side != 0) && is_tie(half, margin, tries))
            {
                side = tie_look(side);
                if (aborted())
                {
                    return;
                }
                g_prof.side        = side;
                g_status.scan_side = side;
            }
            if (side == 0)
            {
                if (rescans >= 1u)
                {
                    abort_avoidance("no room on either side");
                    return;
                }
                rescans++;
                b_scan = true;
                (void)post_event("OBSTACLE_RESCAN", "no room: reversing 100 mm and scanning again");
                if (!reverse_mm((float)OBST_RESCAN_REVERSE_MM))
                {
                    abort_avoidance("reverse failed (E-stop, proximity or timeout)");
                    return;
                }
                continue;
            }
        }

        si = (side == 'L') ? 0u : 1u;
        g_prof.side = side;
        if ((tries[si] >= 1u) && (extra[si] < 1.0f))
        {
            extra[si] = (float)OBST_RETRY_WIDER_MM;    /* after a rescan: a blocked side only goes wider */
        }
        /* measuring: each retry starts 150 mm further back (a wide box blocks the
           diagonal when the car starts close) */
        if (!((g_params.obst_measure > 0.5f) ? standoff_measure(150.0f * (float)(tries[0] + tries[1]))
                                             : standoff(side, margin + extra[si])))
        {
            abort_avoidance("reverse failed (E-stop, proximity or timeout)");
            return;
        }
        if (!bypass_plan(&g_prof, half, margin + extra[si], g_params.obst_depth_mm, (float)REACQ_MAX_LATERAL_MM, &plan))
        {
            abort_avoidance("no bypass plan");
            return;
        }
        plan_events(&plan, margin + extra[si]);
        res = run_bypass(&plan, half + margin + extra[si], margin);
        if ((res == LEG_OK) || (res == LEG_SEARCH))
        {
            g_status.obstacles++;
            (void)post_event("BYPASS_COMPLETE", (res == LEG_SEARCH) ? ((plan.side > 0) ? "passed on the right; on the path, searching for the line"
                                                                                       : "passed on the left; on the path, searching for the line")
                                                                    : ((plan.side > 0) ? "passed on the right" : "passed on the left"));
            flag_set(EVF_BYPASS_COMPLETE);
            return;
        }
        if (res != LEG_BLOCKED)
        {
            abort_avoidance(g_why);
            return;
        }

        /* blocked: note how far it got, back out to where it started */
        tries[si]++;
        progress[si] = (g_n_legs > progress[si]) ? g_n_legs : progress[si];
        {
            char   text[TLM_EVENT_DETAIL_LEN];
            jbuf_t jb;

            jb_init(&jb, text, sizeof(text));
            if (g_block_deg > 500.0f)
            {
                jb_raw(&jb, (side == 'R') ? "right side: obstacle side did not end in 1 m; backing out"
                                          : "left side: obstacle side did not end in 1 m; backing out");
            }
            else
            {
                jb_raw(&jb, (side == 'R') ? "right side: something at " : "left side: something at ");
                jb_fixed(&jb, g_block_cm, 0u);
                jb_raw(&jb, " cm, ");
                jb_fixed(&jb, g_block_deg, 0u);
                jb_raw(&jb, " deg; backing out");
            }
            (void)post_event("BYPASS_BLOCKED", text);
        }
        if (!back_out())
        {
            abort_avoidance("backing out failed (E-stop, proximity or timeout)");
            return;
        }

        /* 1. the other side, untried, with room in this scan */
        {
            char     other = (side == 'L') ? 'R' : 'L';
            uint32_t oi    = 1u - si;

            if ((tries[oi] == 0u) && obstacle_side_room(&g_prof, other, half, margin))
            {
                side               = other;
                g_status.scan_side = other;
                (void)post_event("BYPASS_RETRY", (other == 'R') ? "trying the right side (same scan)" : "trying the left side (same scan)");
                continue;
            }
            /* 2. a side already tried, the one that got further, wider */
            {
                int32_t best = -1;

                for (uint32_t i = 0u; i < 2u; i++)
                {
                    if ((tries[i] == 1u) && obstacle_side_room(&g_prof, (i == 0u) ? 'L' : 'R', half, margin + extra[i] + (float)OBST_RETRY_WIDER_MM) &&
                        ((best < 0) || (progress[i] > progress[(uint32_t)best])))
                    {
                        best = (int32_t)i;
                    }
                }
                if (best >= 0)
                {
                    side = (best == 0) ? 'L' : 'R';
                    extra[best] += (float)OBST_RETRY_WIDER_MM;
                    g_status.scan_side = side;
                    (void)post_event("BYPASS_RETRY", (side == 'R') ? "right side again, 100 mm wider" : "left side again, 100 mm wider");
                    continue;
                }
            }
            /* 3. reverse and rescan, once */
            if ((rescans < 1u) && ((tries[0] < 2u) || (tries[1] < 2u)))
            {
                rescans++;
                b_scan = true;
                (void)post_event("OBSTACLE_RESCAN", "blocked: reversing 100 mm and scanning again");
                if (!reverse_mm((float)OBST_RESCAN_REVERSE_MM))
                {
                    abort_avoidance("reverse failed (E-stop, proximity or timeout)");
                    return;
                }
                continue;
            }
        }
        abort_avoidance("blocked on both sides");
        return;
    }
    abort_avoidance("too many attempts");
}

void tsk_obstacle(INT stacd, void *exinf)
{
    bool b_no_echo = false;

    (void)stacd;
    (void)exinf;
    is09_watch_reset();
    servo_to(0.0f);

    for (;;)
    {
        sys_state_t st = sys_state();

        if (st == SYS_LINE_FOLLOWING)
        {
            /* STAGE0: IS09 forward watch */
            float      cm = ping();
            uint16_t   r = is09_us_cm(cm);
            bool       b_fault = false;
            odometry_t o;
            float      rate;

            /* In a tight curve the sensor points off the track (cones, walls,
               people at the edge of the board): those echoes must not stop
               the car. ping() still checks the 8 cm proximity E-stop. */
            odometry_get(&o);
            rate = (o.turn_dps < 0.0f) ? -o.turn_dps : o.turn_dps;
            if ((g_params.obst_max_turn_dps > 0.0f) && (rate > g_params.obst_max_turn_dps))
            {
                is09_watch_reset();
                (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
                continue;
            }
            if (is09_watch_step(r, &b_fault))
            {
                uint64_t t0 = hal_time_us();
                char     text[TLM_EVENT_DETAIL_LEN];
                jbuf_t   jb;

                motion_stop(SRC_OBSTACLE);        /* NFR1: first avoidance command */
                flag_set(EVF_OBSTACLE_TRIGGER);
                is09_watch_reset();
                jb_init(&jb, text, sizeof(text));
                jb_raw(&jb, "at ");
                jb_uint(&jb, r);
                jb_raw(&jb, " cm (IS09 watch), STOP issued in ");
                jb_uint(&jb, (uint32_t)(hal_time_us() - t0));
                jb_raw(&jb, " us");
                (void)post_event("OBSTACLE", text);
                log_line(text);
            }
            b_fault = b_fault && is09_us_enabled();
            if (b_fault != b_no_echo)
            {
                b_no_echo = b_fault;              /* IS09 NO_ECHO: report only */
                if (b_fault)
                {
                    fault_set(FAULT_ULTRASONIC);
                }
                else
                {
                    fault_clear(FAULT_ULTRASONIC);
                }
            }
            (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
        }
        else if (st == SYS_OBSTACLE_AVOIDANCE)
        {
            avoid();
            servo_move(0.0f, 0.0f);
            /* wait for the planner to leave OBSTACLE AVOIDANCE */
            while (sys_state() == SYS_OBSTACLE_AVOIDANCE)
            {
                (void)tk_dly_tsk(20);
            }
            is09_watch_reset();
        }
        else if ((st == SYS_IDLE) && (g_params.us_idle_ping > 0.5f) && is09_us_enabled())
        {
            is09_watch_reset();
            g_prox_n = 0u;
            bench_ping();                         /* bench mode: distance on the dashboard */
            (void)tk_dly_tsk(200 - ECHO_WAIT_MS);
        }
        else
        {
            is09_watch_reset();
            g_prox_n = 0u;
            (void)tk_dly_tsk(50);
        }
    }
}
