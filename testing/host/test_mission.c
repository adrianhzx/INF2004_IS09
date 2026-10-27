/** @file test_mission.c
 *  @brief Host tests for app_program/shared/mission_core.c (Phase 7).
 *         IDs H-MI-01.. A simulated car (motion executor with the Rev B 6.2
 *         acceptance rules, scanner, safety latch, buttons) is driven by
 *         the mission core through every S0-S7 transition. No kernel.
 */
/* NOLINTBEGIN(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_cal.h"
#include "app_ipc.h"
#include "mission_core.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, text)                                                  \
    do                                                                         \
    {                                                                          \
        if (cond)                                                              \
        {                                                                      \
            g_pass++;                                                          \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            g_fail++;                                                          \
            printf("FAIL %s: %s (line %d)\n", (id), (text), __LINE__);         \
        }                                                                      \
    } while (0)

/* ---- simulated car ------------------------------------------------------- */

#define SIM_MM_PER_TICK 4 /* 20 cm/s x 20 ms */
#define SIM_LOG_MAX 64U

typedef struct
{
    uint8_t type;
    uint8_t speed;
    int16_t param;
    bool    until_line;
    uint8_t seq;
} sim_cmd_t;

typedef struct
{
    /* motion executor */
    uint8_t   state; /* app_motion_state_t */
    sim_cmd_t active;
    uint32_t  cycles_left; /* ticks until the active command completes */
    uint32_t  cycles_per_cmd;
    bool      motion_valid;
    uint8_t   done_seq;
    uint8_t   done_result;
    int16_t   distance_mm;
    uint8_t   speed;
    bool      stuck;       /* BRAKE never reaches speed 0 */
    bool      reject_all;  /* executor rejects every drive command */
    bool      follow_drop; /* FOLLOW ends with TIMEOUT after 3 ticks */
    /* line */
    bool   on_line;
    int8_t line_error;
    /* scanner */
    uint32_t               scan_mode; /* last APP_SCAN_REQ_* seen */
    uint32_t               profile_ticks_left;
    bool                   profile_pending;
    bool                   scan_fail;
    app_obstacle_profile_t profile;
    uint16_t               side_range;
    /* safety */
    bool     latched;
    uint8_t  cause;
    uint16_t faults;
    bool     start_btn;
    bool     stop_btn;
    /* events queued for the next step */
    uint32_t pending_events;
    /* log of motion commands */
    sim_cmd_t log[SIM_LOG_MAX];
    uint32_t  n_log;
    uint32_t  scan_log[SIM_LOG_MAX];
    uint32_t  n_scan_log;
    uint32_t  tev_count[APP_TEV_COUNT];
    uint16_t  last_tev_payload[APP_TEV_COUNT];
} sim_t;

static void
sim_init (sim_t * p)
{
    memset(p, 0, sizeof(*p));
    p->state              = (uint8_t) APP_MOTION_BRAKED;
    p->cycles_per_cmd     = 5U;
    p->motion_valid       = true;
    p->on_line            = true;
    p->side_range         = APP_RANGE_CM_SENTINEL;
    p->profile.range_cm   = APP_RANGE_CM_SENTINEL;
    p->profile.closest_cm = APP_RANGE_CM_SENTINEL;
    p->profile.action     = (uint8_t) APP_AVOID_STOP;
}

static void
sim_latch (sim_t * p, uint8_t cause)
{
    if (!p->latched)
    {
        p->latched = true;
        p->cause   = cause;
        p->state   = (uint8_t) APP_MOTION_STOP_LATCHED;
        p->speed   = 0U;
        p->pending_events |= APP_EVT_SAFE_STOP;
    }
}

/** Apply the core's outputs to the car; queue the resulting events. */
static void
sim_apply (sim_t * p, mission_out_t const * p_out)
{
    uint32_t i = 0U;

    if ((uint8_t) APP_STOP_CAUSE_NONE != p_out->stop_request)
    {
        sim_latch(p, p_out->stop_request);
    }
    if (p_out->send_motion && !p->latched)
    {
        sim_cmd_t c;
        c.type       = p_out->mc_type;
        c.speed      = p_out->mc_speed_cm_s;
        c.param      = p_out->mc_param;
        c.until_line = p_out->mc_until_line;
        c.seq        = p_out->mc_seq;
        if (p->n_log < SIM_LOG_MAX)
        {
            p->log[p->n_log++] = c;
        }
        if ((uint8_t) APP_MC_BRAKE == c.type)
        {
            p->state       = (uint8_t) APP_MOTION_BRAKED;
            p->done_seq    = c.seq;
            p->done_result = (uint8_t) APP_MOTION_RESULT_OK;
            p->speed       = p->stuck ? 20U : 0U;
            p->pending_events |= APP_EVT_MOTION_DONE;
        }
        else if (((uint8_t) APP_MOTION_BRAKED != p->state) || p->reject_all)
        {
            p->pending_events |= APP_EVT_MOTION_REJECT;
        }
        else
        {
            p->active      = c;
            p->cycles_left = p->cycles_per_cmd;
            p->speed       = c.speed;
            if ((uint8_t) APP_MC_FOLLOW == c.type)
            {
                p->state       = (uint8_t) APP_MOTION_FOLLOWING;
                p->cycles_left = p->follow_drop ? 3U : 0xFFFFFFFFU;
            }
            else if ((uint8_t) APP_MC_MOVE == c.type)
            {
                p->state = (uint8_t) APP_MOTION_MOVING;
            }
            else
            {
                p->state = (uint8_t) APP_MOTION_TURNING;
            }
        }
    }
    if (0U != p_out->scan_req)
    {
        if (p->n_scan_log < SIM_LOG_MAX)
        {
            p->scan_log[p->n_scan_log++] = p_out->scan_req;
        }
        if (0U != (p_out->scan_req & APP_SCAN_REQ_PROFILE))
        {
            p->profile_pending    = true;
            p->profile_ticks_left = 3U;
        }
        p->scan_mode = p_out->scan_req;
    }
    if (p_out->clear_latch)
    {
        p->latched = false;
        p->cause   = (uint8_t) APP_STOP_CAUSE_NONE;
        p->state   = (uint8_t) APP_MOTION_BRAKED;
    }
    for (i = 0U; i < p_out->n_tev; i++)
    {
        uint8_t const t = p_out->tev[i].type;
        if (t < (uint8_t) APP_TEV_COUNT)
        {
            p->tev_count[t]++;
            p->last_tev_payload[t] = p_out->tev[i].payload;
        }
    }
}

/** Advance the car by one 20 ms tick. */
static void
sim_tick (sim_t * p)
{
    if (p->latched)
    {
        p->speed = 0U;
        return;
    }
    if (((uint8_t) APP_MOTION_MOVING == p->state) ||
        ((uint8_t) APP_MOTION_TURNING == p->state) ||
        ((uint8_t) APP_MOTION_FOLLOWING == p->state))
    {
        if ((uint8_t) APP_MOTION_TURNING != p->state)
        {
            p->distance_mm =
                (int16_t) (p->distance_mm + ((p->active.param < 0)
                                                 ? -SIM_MM_PER_TICK
                                                 : SIM_MM_PER_TICK));
        }
        if (p->active.until_line && p->on_line)
        {
            p->state       = (uint8_t) APP_MOTION_BRAKED;
            p->speed       = 0U;
            p->done_seq    = p->active.seq;
            p->done_result = (uint8_t) APP_MOTION_RESULT_OK;
            p->pending_events |= APP_EVT_MOTION_DONE;
        }
        else if (0U == p->cycles_left)
        {
            p->state       = (uint8_t) APP_MOTION_BRAKED;
            p->speed       = 0U;
            p->done_seq    = p->active.seq;
            p->done_result = p->active.until_line
                                 ? (uint8_t) APP_MOTION_RESULT_TIMEOUT
                                 : (uint8_t) APP_MOTION_RESULT_OK;
            if ((uint8_t) APP_MC_FOLLOW == p->active.type)
            {
                p->done_result = (uint8_t) APP_MOTION_RESULT_TIMEOUT;
            }
            p->pending_events |= APP_EVT_MOTION_DONE;
        }
        else
        {
            p->cycles_left--;
        }
    }
    if (p->profile_pending)
    {
        if (0U == p->profile_ticks_left)
        {
            p->profile_pending = false;
            p->pending_events |=
                p->scan_fail ? APP_EVT_SCAN_FAIL : APP_EVT_SCAN_DONE;
        }
        else
        {
            p->profile_ticks_left--;
        }
    }
}

/** Build the core input for this step. */
static void
sim_fill (sim_t * p, uint32_t extra_events, mission_in_t * p_in)
{
    memset(p_in, 0, sizeof(*p_in));
    p_in->events              = p->pending_events | extra_events | APP_EVT_TICK;
    p->pending_events         = 0U;
    p_in->boot_done           = true;
    p_in->motion_armed        = true;
    p_in->stop_latched        = p->latched;
    p_in->stop_cause          = p->cause;
    p_in->faults              = p->faults;
    p_in->start_pressed       = p->start_btn;
    p_in->stop_pressed        = p->stop_btn;
    p_in->motion_valid        = p->motion_valid;
    p_in->motion.speed_l_cm_s = p->speed;
    p_in->motion.speed_r_cm_s = p->speed;
    p_in->motion.distance_mm  = p->distance_mm;
    p_in->motion.motion_state = p->state;
    p_in->motion.done_seq     = p->done_seq;
    p_in->motion.done_result  = p->done_result;
    p_in->line.on_line        = p->on_line;
    p_in->line.line_error     = p->line_error;
    p_in->profile_valid       = true;
    p_in->profile             = p->profile;
    if (0U != (p->scan_mode & (APP_SCAN_REQ_SIDE_L | APP_SCAN_REQ_SIDE_R)))
    {
        p_in->profile.range_cm = p->side_range;
    }
}

/** One controller step with extra events, then one car tick. */
static void
step (mission_core_t * p_core, sim_t * p, uint32_t extra_events)
{
    mission_in_t  in;
    mission_out_t out;
    sim_fill(p, extra_events, &in);
    mission_core_step(p_core, &in, &out);
    if (NULL != getenv("MI_DEBUG"))
    {
        printf(
            "  [st=%u sub=%u ev=%05x mot=%u spd=%u dseq=%u/%u res=%u dist=%d]"
            " send=%u type=%u par=%d ul=%u seq=%u scan=%x stop=%u\n",
            p_core->ms.sys_state, p_core->sub, (uint32_t) in.events, p->state,
            p->speed, p->done_seq, p_core->await_seq, p->done_result,
            p->distance_mm, out.send_motion, out.mc_type, out.mc_param,
            out.mc_until_line, out.mc_seq, (uint32_t) out.scan_req,
            out.stop_request);
    }
    sim_apply(p, &out);
    sim_tick(p);
}

static void
run (mission_core_t * p_core, sim_t * p, uint32_t ticks)
{
    uint32_t i = 0U;
    for (i = 0U; i < ticks; i++)
    {
        step(p_core, p, 0U);
    }
}

static uint8_t
state_of (mission_core_t const * p_core)
{
    return p_core->ms.sys_state;
}

/** Step until the core reaches sub-step `sub` of its current state, the
 *  state changes, or the car latches. Returns true if `sub` was reached. */
static bool
run_until_sub (mission_core_t * p_core, sim_t * p, uint8_t sub, uint32_t max)
{
    uint8_t const st = state_of(p_core);
    uint32_t      i  = 0U;
    for (i = 0U; i < max; i++)
    {
        if ((p_core->sub == sub) && (state_of(p_core) == st))
        {
            return true;
        }
        if ((state_of(p_core) != st) || p->latched)
        {
            return false;
        }
        step(p_core, p, 0U);
    }
    return (p_core->sub == sub) && (state_of(p_core) == st);
}

/** Step until the system state equals `st` or the car latches. */
static bool
run_until_state (mission_core_t * p_core, sim_t * p, uint8_t st, uint32_t max)
{
    uint32_t i = 0U;
    for (i = 0U; i < max; i++)
    {
        if (state_of(p_core) == st)
        {
            return true;
        }
        if (p->latched && (st != (uint8_t) APP_SYS_S7_SAFE_STOP))
        {
            return false;
        }
        step(p_core, p, 0U);
    }
    return state_of(p_core) == st;
}

/** Last motion command of a given type in the log (NULL if none). */
static sim_cmd_t const *
last_cmd (sim_t const * p, uint8_t type)
{
    uint32_t i = p->n_log;
    while (i > 0U)
    {
        i--;
        if (p->log[i].type == type)
        {
            return &p->log[i];
        }
    }
    return NULL;
}

static bool
scan_seen (sim_t const * p, uint32_t req)
{
    uint32_t i = 0U;
    for (i = 0U; i < p->n_scan_log; i++)
    {
        if (0U != (p->scan_log[i] & req))
        {
            return true;
        }
    }
    return false;
}

/** Boot to S1 and press START: the car is in S2 following afterwards. */
static void
bring_to_s2 (mission_core_t * p_core, sim_t * p)
{
    mission_core_init(p_core);
    sim_init(p);
    run(p_core, p, 2U); /* S0 -> S1 */
    p->start_btn = true;
    run(p_core, p, 4U); /* debounce 40 ms = 2 ticks + edge */
    p->start_btn = false;
    run(p_core, p, 2U);
    p->n_log      = 0U;
    p->n_scan_log = 0U;
}

/* ---- tests --------------------------------------------------------------- */

static void
test_boot_start (void)
{
    mission_core_t core;
    sim_t          car;
    mission_in_t   in;
    mission_out_t  out;

    mission_core_init(&core);
    sim_init(&car);
    CHECK("H-MI-01", (uint8_t) APP_SYS_S0_BOOT_BRAKED == state_of(&core),
          "init: S0");
    run(&core, &car, 1U);
    CHECK("H-MI-02", (uint8_t) APP_SYS_S1_IDLE == state_of(&core),
          "boot ok + motion BRAKED -> S1 on the first tick");

    /* START while NOT armed: refused, no motion command */
    sim_fill(&car, 0U, &in);
    in.motion_armed  = false;
    in.start_pressed = true;
    mission_core_step(&core, &in, &out);
    sim_apply(&car, &out);
    in.events = APP_EVT_TICK;
    mission_core_step(&core, &in, &out);
    sim_apply(&car, &out);
    in.events = APP_EVT_TICK;
    mission_core_step(&core, &in, &out);
    sim_apply(&car, &out);
    CHECK("H-MI-03",
          ((uint8_t) APP_SYS_S1_IDLE == state_of(&core)) && (0U == car.n_log) &&
              (car.tev_count[APP_TEV_CMD_REJECTED] >= 1U),
          "START while motion disarmed: stays S1, CMD_REJECTED, no motion");

    /* armed START: debounced, one FOLLOW(V_NOM), watch on, S2 */
    bring_to_s2(&core, &car);
    CHECK("H-MI-04",
          ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)) &&
              ((uint8_t) APP_MOTION_FOLLOWING == car.state) &&
              (car.active.speed == APP_CAL_V_NOM_CM_S) &&
              scan_seen(&car, APP_SCAN_REQ_WATCH_ON) == false,
          "START -> S2 with FOLLOW(V_NOM) running");
    /* (scan log was cleared by bring_to_s2; check the mode instead) */
    CHECK("H-MI-05", 0U != (car.scan_mode & APP_SCAN_REQ_WATCH_ON),
          "forward watch requested on entering S2");

    /* a second START press while running does nothing */
    car.start_btn = true;
    run(&core, &car, 5U);
    car.start_btn = false;
    CHECK("H-MI-06",
          ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)) &&
              (0U == car.n_log),
          "START in S2 ignored");
}

static void
test_barcode_and_nav (void)
{
    mission_core_t    core;
    sim_t             car;
    sim_cmd_t const * p_cmd = NULL;

    bring_to_s2(&core, &car);

    /* invalid barcode: nothing pending */
    car.pending_events = 0U;
    {
        mission_in_t  in;
        mission_out_t out;
        sim_fill(&car, APP_EVT_BARCODE, &in);
        in.barcode_byte = app_barcode_pack(APP_BC_INV_NO_MATCH, 1U);
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
    }
    CHECK("H-MI-07", (uint8_t) APP_NAV_NONE == core.ms.pending_cmd,
          "invalid barcode leaves no pending command (FR7)");

    /* valid LEFT: pending, published */
    {
        mission_in_t  in;
        mission_out_t out;
        uint8_t const seq0 = core.ms.seq;
        sim_fill(&car, APP_EVT_BARCODE, &in);
        in.barcode_byte = app_barcode_pack(APP_BC_LEFT, 2U);
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
        CHECK(
            "H-MI-08",
            ((uint8_t) APP_NAV_LEFT == core.ms.pending_cmd) &&
                out.mission_changed && (core.ms.seq != seq0) &&
                ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)),
            "valid barcode -> pending LEFT, MissionStatus published, still S2");
    }
    /* a newer RIGHT replaces it */
    {
        mission_in_t  in;
        mission_out_t out;
        sim_fill(&car, APP_EVT_BARCODE, &in);
        in.barcode_byte = app_barcode_pack(APP_BC_RIGHT, 3U);
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
    }
    CHECK("H-MI-09", (uint8_t) APP_NAV_RIGHT == core.ms.pending_cmd,
          "a newer valid barcode replaces the pending command");

    /* junction with pending RIGHT -> S3: BRAKE, advance, TURN -90 until_line */
    car.n_log = 0U;
    step(&core, &car, APP_EVT_JUNCTION);
    CHECK("H-MI-10",
          ((uint8_t) APP_SYS_S3_EXECUTE_NAV == state_of(&core)) &&
              (car.n_log >= 1U) &&
              ((uint8_t) APP_MC_BRAKE == car.log[0].type) &&
              (0U != (car.scan_mode & APP_SCAN_REQ_WATCH_OFF)),
          "junction [pending] -> S3, BRAKE first (executor was FOLLOWING), "
          "watch off");
    (void) run_until_sub(&core, &car, (uint8_t) MC_NAV_EXECUTE, 20U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-11",
          (NULL != p_cmd) &&
              (p_cmd->param == (int16_t) APP_CAL_D_JUNC_ADVANCE_MM),
          "advance MOVE(D_JUNC_ADVANCE) before the pivot");
    run(&core, &car, 2U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-12",
          (NULL != p_cmd) && (p_cmd->param == -900) && p_cmd->until_line &&
              (p_cmd->speed == APP_CAL_V_TURN_CM_S),
          "RIGHT = TURN -90 deg until_line at V_TURN");
    /* the simulated car is on the line: the turn completes OK -> S2 */
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S2_LINE_FOLLOWING,
                           20U);
    run(&core, &car, 2U);
    CHECK("H-MI-13",
          ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)) &&
              ((uint8_t) APP_NAV_NONE == core.ms.pending_cmd) &&
              ((uint8_t) APP_MOTION_FOLLOWING == car.state) &&
              (car.tev_count[APP_TEV_NAV] >= 2U) &&
              (0U != (car.scan_mode & APP_SCAN_REQ_WATCH_ON)),
          "turn done -> FOLLOW, S2, pending cleared, watch on");

    /* junction with NO pending command: straight through (D-29) */
    car.n_log = 0U;
    step(&core, &car, APP_EVT_JUNCTION);
    run(&core, &car, 2U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-14",
          ((uint8_t) APP_SYS_S3_EXECUTE_NAV == state_of(&core)) &&
              (NULL != p_cmd) &&
              (p_cmd->param == (int16_t) APP_CAL_D_JUNC_CLEAR_MM) &&
              !p_cmd->until_line,
          "junction without a command: MOVE(D_JUNC_CLEAR) straight through");
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S2_LINE_FOLLOWING,
                           30U);
    CHECK("H-MI-15", (uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core),
          "straight crossing done -> S2");

    /* UTURN -> TURN 180 until_line */
    {
        mission_in_t  in;
        mission_out_t out;
        sim_fill(&car, APP_EVT_BARCODE, &in);
        in.barcode_byte = app_barcode_pack(APP_BC_UTURN, 4U);
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
    }
    car.n_log = 0U;
    step(&core, &car, APP_EVT_JUNCTION);
    (void) run_until_sub(&core, &car, (uint8_t) MC_NAV_EXECUTE, 20U);
    run(&core, &car, 2U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-16",
          (NULL != p_cmd) && (p_cmd->param == 1800) && p_cmd->until_line,
          "UTURN = TURN 180 deg until_line");
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S2_LINE_FOLLOWING,
                           30U);
    CHECK("H-MI-17", (uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core),
          "U-turn done -> S2");

    /* LINE_LOST and JUNCTION during the pivot are ignored */
    {
        mission_in_t  in;
        mission_out_t out;
        sim_fill(&car, APP_EVT_BARCODE, &in);
        in.barcode_byte = app_barcode_pack(APP_BC_LEFT, 5U);
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
    }
    car.on_line = false; /* the pivot leaves the line, as on the track */
    step(&core, &car, APP_EVT_JUNCTION);
    (void) run_until_sub(&core, &car, (uint8_t) MC_NAV_EXECUTE, 20U);
    run(&core, &car, 2U);
    step(&core, &car, APP_EVT_LINE_LOST | APP_EVT_JUNCTION);
    CHECK("H-MI-18", (uint8_t) APP_SYS_S3_EXECUTE_NAV == state_of(&core),
          "LINE_LOST / JUNCTION while pivoting do not leave S3");

    /* a pivot whose until_line never sees the line -> S5 search */
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S5_LINE_SEARCH, 30U);
    CHECK("H-MI-19",
          ((uint8_t) APP_SYS_S5_LINE_SEARCH == state_of(&core)) &&
              (car.tev_count[APP_TEV_SEARCH] >= 1U),
          "TURN until_line TIMEOUT -> S5 bounded search");
    car.on_line = true;
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S2_LINE_FOLLOWING,
                           30U);
    CHECK("H-MI-20", (uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core),
          "search finds the line -> S2");
}

static void
test_line_search (void)
{
    mission_core_t    core;
    sim_t             car;
    sim_cmd_t const * p_cmd = NULL;

    bring_to_s2(&core, &car);
    car.line_error = 40; /* line was right of centre */
    run(&core, &car, 2U);
    car.on_line = false;
    car.n_log   = 0U;
    step(&core, &car, APP_EVT_LINE_LOST);
    CHECK("H-MI-21",
          ((uint8_t) APP_SYS_S5_LINE_SEARCH == state_of(&core)) &&
              ((uint8_t) APP_MC_BRAKE == car.log[0].type),
          "LINE_LOST in S2 -> S5, brake first");
    run(&core, &car, 2U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-22",
          (NULL != p_cmd) && (p_cmd->param == -APP_CAL_SEARCH_ANGLE_DDEG) &&
              p_cmd->until_line,
          "first sweep turns toward the side the line was last seen (right)");
    (void) run_until_sub(&core, &car, (uint8_t) MC_SRCH_TURN_B, 30U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-23",
          (NULL != p_cmd) && (p_cmd->param == 2 * APP_CAL_SEARCH_ANGLE_DDEG) &&
              p_cmd->until_line,
          "second sweep: twice the angle the other way");
    (void) run_until_sub(&core, &car, (uint8_t) MC_SRCH_RECENTRE, 30U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-24",
          (NULL != p_cmd) && (p_cmd->param == -APP_CAL_SEARCH_ANGLE_DDEG) &&
              !p_cmd->until_line,
          "re-centre turn");
    (void) run_until_sub(&core, &car, (uint8_t) MC_SRCH_CREEP, 30U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-25",
          (NULL != p_cmd) && p_cmd->until_line &&
              (p_cmd->param == (int16_t) APP_CAL_D_SEARCH_CREEP_MM) &&
              (p_cmd->speed == APP_CAL_V_SEARCH_CM_S),
          "last resort: creep forward until_line at V_SEARCH");
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S7_SAFE_STOP, 30U);
    CHECK("H-MI-26",
          car.latched &&
              ((uint8_t) APP_STOP_CAUSE_LINE_SEARCH_TIMEOUT == car.cause) &&
              ((uint8_t) APP_SYS_S7_SAFE_STOP == state_of(&core)),
          "all sweeps fail -> SafeStop(LINE_SEARCH_TIMEOUT), S7 (FR25)");

    /* time bound: a search that drags on past T_LINE_SEARCH stops */
    bring_to_s2(&core, &car);
    car.on_line        = false;
    car.cycles_per_cmd = 1000U; /* turns never finish */
    step(&core, &car, APP_EVT_LINE_LOST);
    run(&core, &car, (APP_CAL_T_LINE_SEARCH_MS / 20U) + 5U);
    CHECK("H-MI-27",
          car.latched &&
              ((uint8_t) APP_STOP_CAUSE_LINE_SEARCH_TIMEOUT == car.cause),
          "T_LINE_SEARCH elapsed -> LINE_SEARCH_TIMEOUT");

    /* obstacle during the search: brake + NO_CLEARANCE (D-30) */
    bring_to_s2(&core, &car);
    car.on_line = false;
    step(&core, &car, APP_EVT_LINE_LOST);
    run(&core, &car, 2U);
    step(&core, &car, APP_EVT_OBSTACLE);
    CHECK("H-MI-28",
          car.latched && ((uint8_t) APP_STOP_CAUSE_NO_CLEARANCE == car.cause),
          "EVT_OBSTACLE in S5 -> NO_CLEARANCE stop");
}

/** Drive the bypass up to the profile result for a given action. */
static void
bypass_to_plan (mission_core_t * p_core, sim_t * p, uint8_t action,
                int8_t y_min, int8_t y_max, uint16_t range)
{
    p->profile.action     = action;
    p->profile.y_min_cm   = y_min;
    p->profile.y_max_cm   = y_max;
    p->profile.range_cm   = range;
    p->profile.closest_cm = range;
    p->n_log              = 0U;
    p->n_scan_log         = 0U;
    step(p_core, p, APP_EVT_OBSTACLE);
    run(p_core, p, 3U); /* brake, speed 0, REQ_PROFILE */
}

static void
test_bypass_full (void)
{
    mission_core_t    core;
    sim_t             car;
    sim_cmd_t const * p_cmd = NULL;
    uint32_t          i     = 0U;

    bring_to_s2(&core, &car);
    run(&core, &car, 60U); /* travel 240 mm so a reverse corridor exists */
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -6, 5, 30U);
    CHECK("H-MI-30",
          ((uint8_t) APP_SYS_S4_OBSTACLE_BYPASS == state_of(&core)) &&
              ((uint8_t) APP_MC_BRAKE == car.log[0].type) &&
              scan_seen(&car, APP_SCAN_REQ_WATCH_OFF) &&
              scan_seen(&car, APP_SCAN_REQ_PROFILE) &&
              (0U == core.ms.retry_count),
          "EVT_OBSTACLE -> S4: watch off, BRAKE, speed 0, then REQ_PROFILE "
          "(FR15, FR16)");
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A1_TURN, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK(
        "H-MI-31",
        (NULL != p_cmd) && (p_cmd->param == 900) && !p_cmd->until_line &&
            (core.sub == (uint8_t) MC_BYP_A1_TURN) && (core.offset_mm == 210U),
        "action TURN_LEFT: A1 = TURN +90; offset = (y_max + W/2 + M) = 210 mm");
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A2_OFFSET, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-32",
          (NULL != p_cmd) && (p_cmd->param == 210) &&
              (core.sub == (uint8_t) MC_BYP_A2_OFFSET),
          "A2 = MOVE lateral offset");
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A3_TURN, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-33",
          (NULL != p_cmd) && (p_cmd->param == -900) &&
              (core.sub == (uint8_t) MC_BYP_A3_TURN),
          "A3 = TURN -90 back to the heading");
    car.cycles_per_cmd = 1000U; /* the pass MOVE never finishes by itself */
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A4_PASS, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-34",
          (NULL != p_cmd) &&
              (p_cmd->param == (int16_t) APP_CAL_D_PASS_MAX_MM) &&
              (core.sub == (uint8_t) MC_BYP_A4_PASS) &&
              scan_seen(&car, APP_SCAN_REQ_SIDE_R),
          "A4 = MOVE D_PASS_MAX with RIGHT side ranging (obstacle is right of "
          "a left bypass)");
    /* obstacle beside us for a while, then free: free readings before
       D_PASS_MIN must not count */
    car.side_range = 30U;
    run(&core, &car, 10U);
    car.side_range = 100U;
    for (i = 0U; i < 40U; i++)
    {
        step(&core, &car, 0U);
        if (core.sub != (uint8_t) MC_BYP_A4_PASS)
        {
            break;
        }
    }
    car.cycles_per_cmd = 5U;
    run(&core, &car, 1U); /* BRAKE pre-empted the pass; now the extra MOVE */
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-35",
          (core.sub == (uint8_t) MC_BYP_A4_EXTRA) && (NULL != p_cmd) &&
              (p_cmd->param == (int16_t) APP_CAL_D_PASS_EXTRA_MM) &&
              ((car.distance_mm - 0) > 0),
          "side free for SIDE_FREE_COUNT readings after D_PASS_MIN -> one more "
          "car length");
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A5_TURN, 30U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-36",
          (core.sub == (uint8_t) MC_BYP_A5_TURN) && (NULL != p_cmd) &&
              (p_cmd->param == -900) && scan_seen(&car, APP_SCAN_REQ_IDLE),
          "A5 = TURN -90 toward the line, side ranging idle");
    car.on_line = false;
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A5_MOVE, 30U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-37",
          (core.sub == (uint8_t) MC_BYP_A5_MOVE) && (NULL != p_cmd) &&
              p_cmd->until_line &&
              (p_cmd->param == (int16_t) (210U + APP_CAL_D_RETURN_MARGIN_MM)) &&
              (p_cmd->speed == APP_CAL_V_SEARCH_CM_S),
          "A5 = MOVE until_line, bounded by offset + margin, slow");
    run(&core, &car, 2U);
    car.on_line = true; /* line reached */
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A6_ALIGN, 30U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK("H-MI-38",
          (core.sub == (uint8_t) MC_BYP_A6_ALIGN) && (NULL != p_cmd) &&
              (p_cmd->param == 900) && p_cmd->until_line,
          "A6 = TURN +90 until_line to align");
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S2_LINE_FOLLOWING,
                           30U);
    run(&core, &car, 1U);
    CHECK("H-MI-39",
          ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)) &&
              ((uint8_t) APP_MOTION_FOLLOWING == car.state) &&
              (0U != (car.scan_mode & APP_SCAN_REQ_WATCH_ON)) &&
              (0U == core.ms.retry_count) && !car.latched,
          "bypass complete + line found -> S2 FOLLOW, watch on (FR18, FR19)");
    CHECK("H-MI-40", car.tev_count[APP_TEV_BYPASS] >= 9U,
          "every bypass step reported in telemetry");
}

static void
test_bypass_variants (void)
{
    mission_core_t    core;
    sim_t             car;
    sim_cmd_t const * p_cmd = NULL;

    /* CONTINUE: nothing in the way after all -> back to S2 */
    bring_to_s2(&core, &car);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_CONTINUE, 0, 0, 60U);
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S2_LINE_FOLLOWING,
                           30U);
    CHECK("H-MI-41",
          ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)) &&
              !car.latched,
          "action CONTINUE -> resume line following");

    /* RIGHT bypass mirrors the turns and uses the LEFT side ranging */
    bring_to_s2(&core, &car);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_RIGHT, -4, 9, 30U);
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A1_TURN, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_TURN);
    CHECK(
        "H-MI-42",
        (NULL != p_cmd) && (p_cmd->param == -900) && (core.offset_mm == 200U),
        "TURN_RIGHT: A1 = TURN -90; offset from y_min = (4 + 8 + 8) = 200 mm");
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A4_PASS, 40U);
    run(&core, &car, 1U);
    CHECK("H-MI-43",
          (core.sub == (uint8_t) MC_BYP_A4_PASS) &&
              scan_seen(&car, APP_SCAN_REQ_SIDE_L),
          "right bypass ranges on the LEFT side");

    /* STOP with a known-clear corridor: reverse and rescan, bounded */
    bring_to_s2(&core, &car);
    run(&core, &car, 60U); /* 240 mm travelled > D_REVERSE + margin */
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_STOP, -20, 20, 30U);
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_REVERSE, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-44",
          (NULL != p_cmd) &&
              (p_cmd->param == -(int16_t) APP_CAL_D_REVERSE_MM) &&
              (p_cmd->speed == APP_CAL_V_REVERSE_CM_S) &&
              (1U == core.ms.retry_count) && !car.latched,
          "no valid side -> MOVE(-D_REVERSE, V_REVERSE), retry_count 1");
    car.n_scan_log = 0U;
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_PROFILE, 20U);
    run(&core, &car, 1U);
    CHECK("H-MI-45",
          scan_seen(&car, APP_SCAN_REQ_PROFILE) &&
              ((uint8_t) APP_SYS_S4_OBSTACLE_BYPASS == state_of(&core)),
          "after the reverse the profile is requested again");
    /* the corridor behind is now used up (reference moved): second STOP
       cannot retry even though retry_count < N_RETRY_MAX */
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S7_SAFE_STOP, 30U);
    CHECK("H-MI-46",
          car.latched && ((uint8_t) APP_STOP_CAUSE_NO_CLEARANCE == car.cause),
          "no proven-clear corridor -> SafeStop(NO_CLEARANCE)");

    /* retry limit: travel far before each scan so the corridor is clear */
    bring_to_s2(&core, &car);
    run(&core, &car, 60U);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_STOP, -20, 20, 30U);
    {
        uint32_t n = 0U;
        for (n = 0U; (n < 120U) && !car.latched; n++)
        {
            step(&core, &car, 0U);
            if ((uint8_t) MC_BYP_PROFILE == core.sub)
            {
                /* pretend the car drove on before each rescan */
                car.distance_mm = (int16_t) (car.distance_mm + 1000);
            }
        }
    }
    CHECK("H-MI-47",
          car.latched && ((uint8_t) APP_STOP_CAUSE_NO_CLEARANCE == car.cause) &&
              (core.ms.retry_count == APP_CAL_N_RETRY_MAX),
          "N_RETRY_MAX reverse-and-rescan attempts, then NO_CLEARANCE");

    /* REVERSE action (pivot guard failed) uses the same retry path */
    bring_to_s2(&core, &car);
    run(&core, &car, 60U);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_REVERSE, -5, 5, 10U);
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_REVERSE, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-48",
          (NULL != p_cmd) && (p_cmd->param < 0) && (1U == core.ms.retry_count),
          "action REVERSE -> reverse and rescan");

    /* TURN action but the forward range is inside the swept radius */
    bring_to_s2(&core, &car);
    run(&core, &car, 60U);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -5, 5, 12U);
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_REVERSE, 20U);
    run(&core, &car, 1U);
    p_cmd = last_cmd(&car, (uint8_t) APP_MC_MOVE);
    CHECK("H-MI-49",
          (NULL != p_cmd) && (p_cmd->param < 0) &&
              (NULL == last_cmd(&car, (uint8_t) APP_MC_TURN)),
          "post-brake clearance guard: D_front < R_swept + M_clear -> no "
          "pivot, reverse first");

    /* SCAN_FAIL -> SCAN_TIMEOUT (D-22) */
    bring_to_s2(&core, &car);
    car.scan_fail = true;
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -5, 5, 30U);
    run(&core, &car, 6U);
    CHECK("H-MI-50",
          car.latched && ((uint8_t) APP_STOP_CAUSE_SCAN_TIMEOUT == car.cause),
          "EVT_SCAN_FAIL -> SafeStop(SCAN_TIMEOUT)");

    /* no scan answer at all -> SCAN_TIMEOUT after T_SCAN_TIMEOUT + margin */
    bring_to_s2(&core, &car);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -5, 5, 30U);
    car.profile_pending = false; /* scanner never answers */
    run(&core, &car,
        ((APP_CAL_T_SCAN_TIMEOUT_MS + APP_CAL_T_SCAN_WAIT_MARGIN_MS) / 20U) +
            3U);
    CHECK("H-MI-51",
          car.latched && ((uint8_t) APP_STOP_CAUSE_SCAN_TIMEOUT == car.cause),
          "scanner silent -> SCAN_TIMEOUT");

    /* SIDE_UNSAFE during action 4 -> brake + NO_CLEARANCE */
    bring_to_s2(&core, &car);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -5, 5, 30U);
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A4_PASS, 40U);
    run(&core, &car, 1U);
    CHECK("H-MI-52", core.sub == (uint8_t) MC_BYP_A4_PASS,
          "(reached action 4)");
    car.n_log = 0U;
    step(&core, &car, APP_EVT_SIDE_UNSAFE);
    CHECK("H-MI-53",
          car.latched && ((uint8_t) APP_STOP_CAUSE_NO_CLEARANCE == car.cause),
          "EVT_SIDE_UNSAFE while passing -> NO_CLEARANCE stop");

    /* speed never reaches zero after BRAKE -> CRITICAL_RUNTIME */
    bring_to_s2(&core, &car);
    car.stuck = true;
    step(&core, &car, APP_EVT_OBSTACLE);
    run(&core, &car, (APP_CAL_T_SPEED_ZERO_MS / 20U) + 3U);
    CHECK("H-MI-54",
          car.latched &&
              ((uint8_t) APP_STOP_CAUSE_CRITICAL_RUNTIME == car.cause),
          "BRAKE without speed 0 within T_SPEED_ZERO -> CRITICAL_RUNTIME");

    /* whole-bypass time bound */
    bring_to_s2(&core, &car);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -5, 5, 30U);
    car.cycles_per_cmd = 100000U; /* A1 never completes */
    run(&core, &car, (APP_CAL_T_BYPASS_MAX_MS / 20U) + 3U);
    CHECK("H-MI-55",
          car.latched && ((uint8_t) APP_STOP_CAUSE_NO_CLEARANCE == car.cause),
          "T_BYPASS_MAX elapsed -> NO_CLEARANCE");

    /* action 5 never finds the line -> S5 search (FR19), not a stop */
    bring_to_s2(&core, &car);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -5, 5, 30U);
    (void) run_until_sub(&core, &car, (uint8_t) MC_BYP_A5_MOVE, 60U);
    car.on_line = false;
    (void) run_until_state(&core, &car, (uint8_t) APP_SYS_S5_LINE_SEARCH, 40U);
    CHECK("H-MI-56",
          ((uint8_t) APP_SYS_S5_LINE_SEARCH == state_of(&core)) && !car.latched,
          "A5 MOVE until_line TIMEOUT -> S5 bounded search");

    /* obstacle in S3 -> NO_CLEARANCE (D-30) */
    bring_to_s2(&core, &car);
    step(&core, &car, APP_EVT_JUNCTION);
    run(&core, &car, 2U);
    step(&core, &car, APP_EVT_OBSTACLE);
    CHECK("H-MI-57",
          car.latched && ((uint8_t) APP_STOP_CAUSE_NO_CLEARANCE == car.cause),
          "EVT_OBSTACLE in S3 -> NO_CLEARANCE stop");

    /* a barcode read during the bypass is kept for the next junction */
    bring_to_s2(&core, &car);
    bypass_to_plan(&core, &car, (uint8_t) APP_AVOID_TURN_LEFT, -5, 5, 30U);
    {
        mission_in_t  in;
        mission_out_t out;
        sim_fill(&car, APP_EVT_BARCODE, &in);
        in.barcode_byte = app_barcode_pack(APP_BC_STRAIGHT, 6U);
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
    }
    CHECK("H-MI-58",
          ((uint8_t) APP_NAV_STRAIGHT == core.ms.pending_cmd) &&
              ((uint8_t) APP_SYS_S4_OBSTACLE_BYPASS == state_of(&core)),
          "barcode during S4 stored as pending, S4 continues");
}

static void
test_safety_and_recovery (void)
{
    mission_core_t core;
    sim_t          car;
    uint8_t const  states[4] = {
        (uint8_t) APP_SYS_S2_LINE_FOLLOWING, (uint8_t) APP_SYS_S3_EXECUTE_NAV,
        (uint8_t) APP_SYS_S4_OBSTACLE_BYPASS, (uint8_t) APP_SYS_S5_LINE_SEARCH};
    uint32_t i  = 0U;
    bool     ok = true;

    /* EVT_SAFE_STOP from every mission state -> S7, scanner idle */
    for (i = 0U; i < 4U; i++)
    {
        bring_to_s2(&core, &car);
        if (states[i] == (uint8_t) APP_SYS_S3_EXECUTE_NAV)
        {
            step(&core, &car, APP_EVT_JUNCTION);
        }
        else if (states[i] == (uint8_t) APP_SYS_S4_OBSTACLE_BYPASS)
        {
            step(&core, &car, APP_EVT_OBSTACLE);
        }
        else if (states[i] == (uint8_t) APP_SYS_S5_LINE_SEARCH)
        {
            car.on_line = false;
            step(&core, &car, APP_EVT_LINE_LOST);
        }
        else
        {
            /* S2 */
        }
        run(&core, &car, 2U);
        if (state_of(&core) != states[i])
        {
            ok = false;
        }
        car.n_scan_log = 0U;
        sim_latch(&car, (uint8_t) APP_STOP_CAUSE_COLLISION);
        step(&core, &car, 0U);
        if ((state_of(&core) != (uint8_t) APP_SYS_S7_SAFE_STOP) ||
            !scan_seen(&car, APP_SCAN_REQ_IDLE))
        {
            ok = false;
        }
        /* nothing drives while latched */
        car.n_log = 0U;
        run(&core, &car, 10U);
        if (0U != car.n_log)
        {
            ok = false;
        }
    }
    CHECK("H-MI-60", ok,
          "EVT_SAFE_STOP from S2/S3/S4/S5 -> S7, scanner idle, no motion sent");

    /* recovery guard: START held T_HOLD with speed 0 and cause cleared */
    bring_to_s2(&core, &car);
    sim_latch(&car, (uint8_t) APP_STOP_CAUSE_COLLISION);
    car.faults = APP_FAULT_IMU_FAULT; /* initiating condition still present */
    step(&core, &car, 0U);
    car.start_btn = true;
    run(&core, &car, (APP_CAL_T_HOLD_MS / 20U) + 5U);
    CHECK("H-MI-61",
          car.latched && ((uint8_t) APP_SYS_S7_SAFE_STOP == state_of(&core)),
          "START held but IMU_FAULT still set: stays latched");
    car.faults    = 0U;
    car.start_btn = false;
    run(&core, &car, 2U);
    car.start_btn = true;
    run(&core, &car, (APP_CAL_T_HOLD_MS / 20U) - 2U);
    CHECK("H-MI-62", car.latched, "START held < T_HOLD: still latched");
    run(&core, &car, 4U);
    CHECK("H-MI-63",
          !car.latched && (car.tev_count[APP_TEV_RECOVERY] == 1U) &&
              (((uint8_t) APP_SYS_S0_BOOT_BRAKED == state_of(&core)) ||
               ((uint8_t) APP_SYS_S1_IDLE == state_of(&core))),
          "START held >= T_HOLD, cause cleared, speed 0 -> latch cleared, S0");
    run(&core, &car, 3U);
    CHECK("H-MI-64",
          ((uint8_t) APP_SYS_S1_IDLE == state_of(&core)) &&
              ((uint8_t) APP_NAV_NONE == core.ms.pending_cmd) &&
              (0U == car.n_log),
          "then S1, pending cleared, no automatic restart while START still "
          "held");
    car.start_btn = false;
    run(&core, &car, 2U);
    car.start_btn = true;
    run(&core, &car, 4U);
    CHECK("H-MI-65", (uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core),
          "a fresh START press after recovery starts a new run");

    /* operator STOP: cleared only once the button is released */
    sim_latch(&car, (uint8_t) APP_STOP_CAUSE_OPERATOR_STOP);
    car.stop_btn = true;
    step(&core, &car, 0U);
    run(&core, &car, (APP_CAL_T_HOLD_MS / 20U) + 5U);
    CHECK("H-MI-66", car.latched, "STOP still pressed: no recovery");
    car.stop_btn  = false;
    car.start_btn = false;
    run(&core, &car, 2U);
    car.start_btn = true;
    run(&core, &car, (APP_CAL_T_HOLD_MS / 20U) + 3U);
    CHECK("H-MI-67", !car.latched, "STOP released + START held -> recovered");

    /* CRITICAL_RUNTIME never recovers in software */
    sim_latch(&car, (uint8_t) APP_STOP_CAUSE_CRITICAL_RUNTIME);
    car.start_btn = false;
    run(&core, &car, 2U);
    car.start_btn = true;
    run(&core, &car, (APP_CAL_T_HOLD_MS / 20U) + 10U);
    CHECK("H-MI-68", car.latched, "CRITICAL_RUNTIME needs a reboot");

    /* remote commands */
    bring_to_s2(&core, &car);
    {
        mission_in_t  in;
        mission_out_t out;
        sim_fill(&car, APP_EVT_REMOTE_CMD, &in);
        in.remote_start = true;
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
        CHECK("H-MI-69",
              (car.tev_count[APP_TEV_CMD_REJECTED] >= 1U) &&
                  ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)),
              "remote START outside S1 rejected");
        sim_fill(&car, APP_EVT_REMOTE_CMD, &in);
        in.remote_stop = true;
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        CHECK("H-MI-70",
              car.latched &&
                  ((uint8_t) APP_STOP_CAUSE_OPERATOR_STOP == car.cause),
              "remote STOP -> SafeStop(OPERATOR_STOP) (D-01)");
    }
    mission_core_init(&core);
    sim_init(&car);
    run(&core, &car, 2U);
    {
        mission_in_t  in;
        mission_out_t out;
        sim_fill(&car, APP_EVT_REMOTE_CMD, &in);
        in.remote_start = true;
        mission_core_step(&core, &in, &out);
        sim_apply(&car, &out);
        sim_tick(&car);
        run(&core, &car, 2U);
        CHECK("H-MI-71",
              ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)) &&
                  ((uint8_t) APP_MOTION_FOLLOWING == car.state),
              "remote START in S1 starts the run (FR23)");
    }

    /* executor rejects twice -> CRITICAL_RUNTIME; once -> retried */
    bring_to_s2(&core, &car);
    car.reject_all = true;
    step(&core, &car, APP_EVT_JUNCTION);
    run(&core, &car, 8U);
    CHECK("H-MI-72",
          car.latched &&
              ((uint8_t) APP_STOP_CAUSE_CRITICAL_RUNTIME == car.cause),
          "repeated MOTION_REJECT of a command -> CRITICAL_RUNTIME");

    /* FOLLOW dropped by the executor is re-issued */
    bring_to_s2(&core, &car);
    car.follow_drop = true;
    car.cycles_left = 3U; /* the running FOLLOW ends with TIMEOUT */
    car.n_log       = 0U;
    run(&core, &car, 6U);
    CHECK("H-MI-73",
          (NULL != last_cmd(&car, (uint8_t) APP_MC_FOLLOW)) &&
              ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)),
          "FOLLOW reported done (dropped) -> FOLLOW sent again, still S2");

    /* hump events change nothing in the mission */
    bring_to_s2(&core, &car);
    car.n_log = 0U;
    step(&core, &car, APP_EVT_HUMP_START);
    run(&core, &car, 3U);
    step(&core, &car, APP_EVT_HUMP_END);
    CHECK("H-MI-74",
          ((uint8_t) APP_SYS_S2_LINE_FOLLOWING == state_of(&core)) &&
              (0U == car.n_log),
          "EVT_HUMP_START/END: concurrent monitor, no mission transition");

    /* offset helper clamps */
    {
        app_obstacle_profile_t p;
        memset(&p, 0, sizeof(p));
        p.y_max_cm = 60;
        p.y_min_cm = -1;
        CHECK(
            "H-MI-75",
            (mission_core_offset_mm(&p, true) == APP_CAL_Y_PATH_MAX_CM * 10U) &&
                (mission_core_offset_mm(&p, false) == 170U),
            "offset clamps at Y_PATH_MAX; right offset from y_min");
    }

    /* MissionStatus word round trip carries state, pending and retries */
    bring_to_s2(&core, &car);
    core.ms.pending_cmd = (uint8_t) APP_NAV_UTURN;
    core.ms.retry_count = 3U;
    {
        app_mission_status_t v;
        app_mission_unpack(mission_core_status(&core), &v);
        CHECK("H-MI-76",
              (v.sys_state == (uint8_t) APP_SYS_S2_LINE_FOLLOWING) &&
                  (v.pending_cmd == (uint8_t) APP_NAV_UTURN) &&
                  (v.retry_count == 3U),
              "MissionStatus pack/unpack of the core view");
    }
}

int
main (void)
{
    test_boot_start();
    test_barcode_and_nav();
    test_line_search();
    test_bypass_full();
    test_bypass_variants();
    test_safety_and_recovery();
    printf("mission host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
/* NOLINTEND(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */
