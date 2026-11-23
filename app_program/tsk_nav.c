/**
 * @file    tsk_nav.c
 * @brief   TSK_NAV_PLANNER (DR Section 3 / 5, event-driven): mission
 *          sequencer and system-level state transitions.
 *
 *   IDLE            --BTN1 / START, calibration complete--> LINE FOLLOWING
 *   LINE FOLLOWING  --barcode A/B/D--> MANEUVER (encoder turn) --> LINE FOLLOWING
 *                   --barcode C------> keep going (FR8)
 *                   --EVF_OBSTACLE_TRIGGER--> OBSTACLE AVOIDANCE
 *                   --EVF_LINE_LOST--> IDLE (stopped, operator re-places the car)
 *                   --BTN1 / STOP----> MISSION COMPLETE
 *   any driving state --picked up (TSK_LINE_BARCODE, 20 Nov)--> MISSION COMPLETE
 *   OBSTACLE AVOIDANCE --EVF_BYPASS_COMPLETE--> LINE FOLLOWING
 *                      --EVF_BYPASS_ABORT----> ERROR (via the safety monitor)
 *   MISSION COMPLETE --BTN1--> IDLE
 *   any driving state: EVF_HUMP_START -> SPEED_LIMIT 60 %, EVF_HUMP_END -> 100 %
 * EMERGENCY STOP and ERROR belong to TSK_SAFETY_MON.
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "motion_api.h"
#include "app_params.h"
#include "app_check.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"
#include "nav_map.h"
#include "is09_glue.h"
#include "obstacle_logic.h"

LOCAL char     g_pending;              /* BARCODE_AT_JUNCTION mode */
LOCAL uint32_t g_t_start;

LOCAL bool driving(sys_state_t st)
{
    return (st == SYS_LINE_FOLLOWING) || (st == SYS_MANEUVER) || (st == SYS_OBSTACLE_AVOIDANCE);
}

LOCAL void start_run(void)
{
    char missing[160];

    if (!config_check(missing, sizeof(missing)))
    {
        log_line("[nav] cannot start, calibration missing:");
        log_line(missing);
        (void)post_event("START_REFUSED", "calibration missing (see console)");
        buzzer_request(BUZZ_ERROR);
        return;
    }
    g_pending             = 0;
    g_t_start             = hal_time_ms();
    {
        char m[80];

        if (nav_map_describe(m, sizeof(m)) < 4u)
        {
            /* not a refusal: the map may legitimately arrive after START */
            (void)post_event("MAP_WARNING", (m[0] != '\0') ? m : "no barcode map yet (send MAP:...)");
        }
    }
    g_status.hump_max_mm  = 0.0f;
    g_status.humps        = 0u;
    g_status.obstacles    = 0u;
    g_status.barcodes_read = 0u;
    g_status.barcode      = 0;
    flag_clear(EVF_OBSTACLE_TRIGGER | EVF_BYPASS_COMPLETE | EVF_BYPASS_ABORT | EVF_LINE_LOST |
               EVF_LINE_FOUND | EVF_HUMP_START | EVF_HUMP_END);
    motion_speed_limit(1.0f);
    buzzer_request(BUZZ_START);
    (void)sys_set_state(SYS_LINE_FOLLOWING, false);
}

LOCAL void run_summary(void)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "t ");
    jb_uint(&jb, (hal_time_ms() - g_t_start) / 1000u);
    jb_raw(&jb, "s dist ");
    jb_fixed(&jb, g_status.dist_mm / 1000.0f, 2u);
    jb_raw(&jb, "m bc ");
    jb_uint(&jb, g_status.barcodes_read);
    jb_raw(&jb, " humps ");
    jb_uint(&jb, g_status.humps);
    jb_raw(&jb, " max ");
    jb_fixed(&jb, g_status.hump_max_mm, 1u);
    jb_raw(&jb, "mm obst ");
    jb_uint(&jb, g_status.obstacles);
    (void)post_event("MISSION_COMPLETE", text);       /* final summary (FR22) */
    log_line(text);
    buzzer_request(BUZZ_COMPLETE);
}

LOCAL void complete_run(void)
{
    motion_stop(SRC_NAV);
    (void)sys_set_state(SYS_MISSION_COMPLETE, false);
    run_summary();
}

/* TURN_SNAP (20 Nov rework). The car pivots in one go at SNAP_MM_S and the
   line sensors are watched for the whole turn once it has turned
   SNAP_GATE_FRAC of the angle (so a straight-on branch, crossed early,
   cannot stop it). It stops when the leading sensor (left one for a left
   turn) has seen the line and lost it again (the line is between the
   sensors), or when the trailing one sees it (just past). Gives up after
   SNAP_EXTRA_DEG beyond the angle and keeps the encoder result.
   Why: on the 20:17 run (barcode A, LEFT) the old turn did 45 deg on the
   encoders without looking, and the line passed both sensors during it (the
   car really turns ~1.5 x the encoder angle until TRACK_WIDTH_MM is
   calibrated: TURN_CAL); the slow part then swept 90 deg more and found
   nothing: ~135 deg on the encoders, a near U-turn. "Black" is the line
   follower's decision (the LEFT sensor has its own level, LINE_L_DARK_PCT). */
#define SNAP_MM_S        (50.0f)
#define SNAP_GATE_FRAC   (0.4f)
#define SNAP_EXTRA_DEG   (45.0f)

LOCAL bool line_black(bool b_left)
{
    bool lv = b_left ? hal_ir_left_level() : hal_ir_right_level();

    return lv == (g_params.line_do_black_high > 0.5f);
}

LOCAL float turned_deg(float h0)
{
    odometry_t o;
    float      d;

    odometry_get(&o);
    d = o.heading_deg - h0;
    return (d < 0.0f) ? -d : d;
}

LOCAL motion_result_t turn_on_line(float deg)
{
    float           dir  = (deg < 0.0f) ? -1.0f : 1.0f;
    float           mag  = (deg < 0.0f) ? -deg : deg;
    float           gate = SNAP_GATE_FRAC * mag;
    bool            b_seen = false;
    bool            b_arm_l = false;      /* a sensor counts only after it has read white */
    bool            b_arm_c = false;
    uint16_t        id;
    uint32_t        t0;
    float           h0;
    odometry_t      o;

    odometry_get(&o);
    h0 = o.heading_deg;
    id = motion_start_turn(dir * (mag + SNAP_EXTRA_DEG), SNAP_MM_S, SRC_NAV);
    t0 = hal_time_ms();
    for (;;)
    {
        motion_result_t r2;
        uint32_t        other;

        {
            bool bl = line_black(true);
            bool bc = line_black(false);

            b_arm_l = b_arm_l || !bl;
            b_arm_c = b_arm_c || !bc;
        }
        if (turned_deg(h0) >= gate)
        {
            bool b_lead  = (dir < 0.0f) ? (b_arm_l && line_black(true)) : (b_arm_c && line_black(false));
            bool b_trail = (dir < 0.0f) ? (b_arm_c && line_black(false)) : (b_arm_l && line_black(true));

            b_seen = b_seen || b_lead;
            if ((b_seen && !b_lead) || b_trail)
            {
                char   text[TLM_EVENT_DETAIL_LEN];
                jbuf_t jb;

                motion_stop(SRC_NAV);          /* the line is between the sensors */
                jb_init(&jb, text, sizeof(text));
                jb_raw(&jb, b_trail ? "stopped on the line (trailing sensor) after " : "stopped on the line after ");
                jb_fixed(&jb, turned_deg(h0), 0u);
                jb_raw(&jb, " deg (encoders)");
                (void)post_event("TURN_SNAP", text);
                return MOTION_OK;
            }
        }
        if (motion_wait(id, 0u, 10u, &r2, &other))
        {
            (void)post_event("TURN_SNAP", "line not seen, encoder angle kept");
            return (r2 == MOTION_STOPPED) ? MOTION_STOPPED : MOTION_OK;
        }
        if ((sys_state() != SYS_MANEUVER) || ((hal_time_ms() - t0) > 10000u))
        {
            motion_stop(SRC_NAV);
            return MOTION_STOPPED;
        }
    }
}

/* 20 Nov: TURN_CAL (IDLE, car on a straight line). Pivots left slowly up to
   CAL_MAX_DEG on the encoders and notes the encoder angle each time a line
   sensor turns black (after 5 white samples). A sensor's 1st and 3rd
   crossing are one real turn apart, so TRACK_WIDTH_MM is set to
   TRACK_WIDTH_MM x (that encoder angle) / 360 (turn_cal_track_mm). It runs
   as MANEUVER: E-stop, BTN2 and the pick-up stop all work. */
#define CAL_MAX_DEG      (720.0f)
#define CAL_ONSETS       (4u)

LOCAL void turn_cal(void)
{
    float      on_l[CAL_ONSETS];
    float      on_c[CAL_ONSETS];
    uint32_t   n_l = 0u, n_c = 0u, white_l = 0u, white_c = 0u;
    bool       b_l_prev = true, b_c_prev = true;
    float      h0;
    float      track;
    uint16_t   id;
    uint32_t   t0;
    odometry_t o;
    char       text[TLM_EVENT_DETAIL_LEN];
    jbuf_t     jb;

    (void)sys_set_state(SYS_MANEUVER, false);
    (void)post_event("TURN_CAL", "start: pivoting left on the line, counting line crossings");
    odometry_get(&o);
    h0 = o.heading_deg;
    id = motion_start_turn(-CAL_MAX_DEG, SNAP_MM_S, SRC_NAV);
    t0 = hal_time_ms();
    for (;;)
    {
        motion_result_t r2;
        uint32_t        other;
        bool            b_l = line_black(true);
        bool            b_c = line_black(false);
        float           t = turned_deg(h0);

        if (b_l && !b_l_prev && (white_l >= 5u) && (n_l < CAL_ONSETS))
        {
            on_l[n_l++] = t;
        }
        if (b_c && !b_c_prev && (white_c >= 5u) && (n_c < CAL_ONSETS))
        {
            on_c[n_c++] = t;
        }
        white_l = b_l ? 0u : (white_l + 1u);
        white_c = b_c ? 0u : (white_c + 1u);
        b_l_prev = b_l;
        b_c_prev = b_c;
        if ((n_l >= 3u) && (n_c >= 3u))
        {
            motion_stop(SRC_NAV);
            break;
        }
        if (motion_wait(id, 0u, 10u, &r2, &other))
        {
            break;
        }
        if ((sys_state() != SYS_MANEUVER) || ((hal_time_ms() - t0) > 40000u))
        {
            motion_stop(SRC_NAV);
            break;
        }
    }
    if (sys_state() == SYS_MANEUVER)
    {
        (void)sys_set_state(SYS_IDLE, false);
    }
    track = turn_cal_track_mm(g_params.track_width_mm, on_l, n_l, on_c, n_c);
    jb_init(&jb, text, sizeof(text));
    if ((track >= 50.0f) && (track <= 300.0f))
    {
        jb_raw(&jb, "TRACK_WIDTH_MM ");
        jb_fixed(&jb, g_params.track_width_mm, 0u);
        jb_raw(&jb, " -> ");
        jb_fixed(&jb, track, 0u);
        jb_raw(&jb, " (set; put it in robot_config.h). Crossings L ");
        jb_uint(&jb, n_l);
        jb_raw(&jb, " C ");
        jb_uint(&jb, n_c);
        g_params.track_width_mm = track;
    }
    else
    {
        jb_raw(&jb, "failed: crossings L ");
        jb_uint(&jb, n_l);
        jb_raw(&jb, " C ");
        jb_uint(&jb, n_c);
        jb_raw(&jb, " (need 3 of one). Is the car on a straight line?");
    }
    (void)post_event("TURN_CAL", text);
}

/* FR6 / FR8: barcode -> navigation command. At a junction the car first
   drives the sensor-to-axle distance, so it pivots on the crossing point. */
LOCAL void execute(char code, bool b_at_junction)
{
    nav_action_t    act = nav_map_get(code);      /* set over MQTT, never hard-coded */
    float           deg = nav_action_deg(act);
    motion_result_t res;
    char            text[TLM_EVENT_DETAIL_LEN];
    jbuf_t          jb;

    jb_init(&jb, text, sizeof(text));
    jb_char(&jb, code);
    jb_raw(&jb, " -> ");
    jb_raw(&jb, nav_action_name(act));
    if (act == ACT_NONE)
    {
        jb_raw(&jb, " (not in the MQTT map): straight on");
    }
    (void)post_event("NAV_ACTION", text);
    if (deg == 0.0f)
    {
        return;                                /* FORWARD or unmapped: keep following */
    }
    (void)sys_set_state(SYS_MANEUVER, false);
    motion_stop(SRC_NAV);
    res = MOTION_OK;
    if (b_at_junction && (g_params.ir_to_axle_mm > 0.0f))
    {
        res = motion_move(g_params.ir_to_axle_mm, g_params.bypass_mm_s);
    }
    if ((res == MOTION_OK) && (sys_state() == SYS_MANEUVER))
    {
        res = ((g_params.turn_snap > 0.5f) && b_at_junction) ? turn_on_line(deg)
                                                               : motion_turn(deg, g_params.turn_mm_s);
    }
    if (sys_state() != SYS_MANEUVER)
    {
        return;                                /* E-stop or error during the turn */
    }
    if (res == MOTION_OK)
    {
        (void)post_event("NAV", nav_action_name(act));
        (void)sys_set_state(SYS_LINE_FOLLOWING, false);
    }
    else
    {
        (void)post_event("NAV", "turn did not complete");
        flag_set(EVF_ERROR_REQ);
    }
}

/* 20 Nov: pivot calibration. The car pivots 360 deg as measured by the
   encoders with TRACK_WIDTH_MM, at TURN_MM_S, like the junction and bypass
   pivots. If it really turns A deg, set TRACK_WIDTH_MM to
   TRACK_WIDTH_MM x 360 / A (over-turn -> smaller value). It runs as
   MANEUVER, so E-stop, BTN2 and the pick-up stop all work during it. */
LOCAL void turn_test(void)
{
    char            text[TLM_EVENT_DETAIL_LEN];
    jbuf_t          jb;
    motion_result_t res;

    (void)sys_set_state(SYS_MANEUVER, false);
    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "start: 360 deg, TRACK_WIDTH_MM ");
    jb_fixed(&jb, g_params.track_width_mm, 0u);
    jb_raw(&jb, ", TURN_MM_S ");
    jb_fixed(&jb, g_params.turn_mm_s, 0u);
    (void)post_event("TURN_TEST", text);
    res = motion_turn(360.0f, g_params.turn_mm_s);
    motion_stop(SRC_NAV);
    if (sys_state() == SYS_MANEUVER)
    {
        (void)sys_set_state(SYS_IDLE, false);
    }
    (void)post_event("TURN_TEST", (res == MOTION_OK) ? "done: how far did it really turn?" : "stopped before 360 deg");
}

LOCAL void on_nav_event(const nav_event_t *p_ev)
{
    sys_state_t st = sys_state();

    switch (p_ev->type)
    {
        case NAV_BTN1:
        case NAV_START:
        case NAV_STOP:
            if ((st == SYS_IDLE) && (p_ev->type != NAV_STOP))
            {
                start_run();
            }
            else if (driving(st))
            {
                complete_run();
            }
            else if ((st == SYS_MISSION_COMPLETE) && (p_ev->type != NAV_STOP))
            {
                (void)sys_set_state(SYS_IDLE, false);
            }
            else
            {
                /* nothing to do in this state */
            }
            break;

        case NAV_BARCODE:
        case NAV_REMOTE_BARCODE:
            if (st == SYS_LINE_FOLLOWING)
            {
                if ((g_params.barcode_at_junction > 0.5f) && (p_ev->type == NAV_BARCODE))
                {
                    char text[TLM_EVENT_DETAIL_LEN];
                    jbuf_t jb;

                    g_pending = p_ev->code;
                    jb_init(&jb, text, sizeof(text));
                    jb_char(&jb, g_pending);
                    jb_raw(&jb, " stored for the next junction (");
                    jb_raw(&jb, nav_action_name(nav_map_get(g_pending)));
                    jb_char(&jb, ')');
                    (void)post_event("NAV", text);
                }
                else
                {
                    execute(p_ev->code, false);
                }
            }
            break;

        case NAV_JUNCTION:
            if ((st == SYS_LINE_FOLLOWING) && (g_pending != 0))
            {
                char code = g_pending;

                g_pending = 0;
                execute(code, true);
            }
            else if (st == SYS_LINE_FOLLOWING)
            {
                char   text[TLM_EVENT_DETAIL_LEN];
                jbuf_t jb;

                /* 19 Nov: shown on the dashboard; tsk_line drives straight across */
                jb_init(&jb, text, sizeof(text));
                jb_raw(&jb, (p_ev->code == 'L') ? "left black, right grey" :
                            ((p_ev->code == 'R') ? "right black, left grey" : "both sensors black"));
                jb_raw(&jb, ", no command: straight on for ");
                jb_fixed(&jb, g_params.line_junc_clear_mm, 0u);
                jb_raw(&jb, " mm");
                (void)post_event("JUNCTION", text);
            }
            else
            {
                /* not following */
            }
            break;

        case NAV_BARCODE_FAULT:
            (void)post_event("NAV", "barcode fault: going straight (C)");
            break;

        case NAV_TURN_TEST:
            if (st == SYS_IDLE)
            {
                turn_test();
            }
            break;

        case NAV_TURN_CAL:
            if (st == SYS_IDLE)
            {
                turn_cal();
            }
            break;

        case NAV_PICKUP:
            /* 20 Nov: TSK_LINE_BARCODE has already stopped the car and set
               MISSION COMPLETE (PICKED_UP event); only the summary is left */
            if (st == SYS_MISSION_COMPLETE)
            {
                run_summary();
            }
            break;

        default:
            break;
    }
}

void tsk_nav(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        app_msg_t  *p_msg = msg_recv(g_mbx_nav, 20);
        uint32_t    got = 0u;
        sys_state_t st;

        if (p_msg != NULL)
        {
            nav_event_t ev = p_msg->body.nav;

            msg_free(p_msg);
            on_nav_event(&ev);
        }
        /* obstacle profiles: kept for the record (published by the scan task) */
        while ((p_msg = msg_recv(g_mbx_obstacle, TMO_POL)) != NULL)
        {
            msg_free(p_msg);
        }

        st = sys_state();
        if (!driving(st))
        {
            g_pending = 0;
        }
        (void)flag_take(EVF_HUMP_START | EVF_HUMP_END | EVF_BYPASS_COMPLETE |
                        EVF_BYPASS_ABORT | EVF_LINE_LOST, &got);
        got |= flag_peek() & EVF_OBSTACLE_TRIGGER;   /* cleared only after the state change */

        /* hump speed limit (DR A2) */
        if ((got & EVF_HUMP_START) != 0u)
        {
            motion_speed_limit((float)HUMP_SPEED_PCT / 100.0f);
        }
        if ((got & EVF_HUMP_END) != 0u)
        {
            motion_speed_limit(1.0f);
        }

        if (st == SYS_LINE_FOLLOWING)
        {
            if ((got & EVF_OBSTACLE_TRIGGER) != 0u)
            {
                /* state first: TSK_LINE_BARCODE stops steering while the flag is set */
                (void)sys_set_state(SYS_OBSTACLE_AVOIDANCE, false);
                flag_clear(EVF_OBSTACLE_TRIGGER);
            }
            else if ((got & EVF_LINE_LOST) != 0u)
            {
                motion_stop(SRC_NAV);
                (void)sys_set_state(SYS_IDLE, false);
                buzzer_request(BUZZ_WARN);
                log_line("[nav] line lost: put the car back on the line and press BTN1");
            }
            else
            {
                /* keep following */
            }
        }
        else if (st == SYS_OBSTACLE_AVOIDANCE)
        {
            if ((got & EVF_BYPASS_COMPLETE) != 0u)
            {
                (void)sys_set_state(SYS_LINE_FOLLOWING, false);
            }
            else if ((got & EVF_BYPASS_ABORT) != 0u)
            {
                flag_set(EVF_ERROR_REQ);
            }
            else
            {
                /* bypass in progress */
            }
        }
        else
        {
            flag_clear(EVF_OBSTACLE_TRIGGER);       /* other states: flags are dropped */
        }
    }
}
