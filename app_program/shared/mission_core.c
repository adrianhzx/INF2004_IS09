/** @file mission_core.c
 *  @brief Pure mission statechart, see mission_core.h. Integer only.
 */
#include "mission_core.h"

#include "app_cal.h"
#include "app_ipc.h"

#define MC_TICK_MS 20U
#define MC_MS_TO_TICKS(ms) (((ms) + MC_TICK_MS - 1U) / MC_TICK_MS)
#define MC_SEQ_MASK 0x0FU
#define MC_SEQ_NONE 0xFFU
#define MC_MM_PER_CM 10U
#define MC_DDEG_90 900 /* 0.1 deg units */
#define MC_DDEG_180 1800
#define MC_HALF 2U
#define MC_REJECT_MAX 2U /* rejections before a critical stop */
#define MC_PAYLOAD_SHIFT 8U
#define MC_NAV_START 0U
#define MC_NAV_DONE 1U
#define MC_NAV_LOST 2U
#define MC_SEARCH_START 0U
#define MC_SEARCH_FOUND 1U
#define MC_SEARCH_FAIL 2U
#define MC_BYP_DETAIL_LEFT 1U
#define MC_BYP_DETAIL_RIGHT 2U
#define MC_BYP_DETAIL_RETRY_SHIFT 4U
#define MC_REJECT_NOT_ARMED 0x0001U
#define MC_REJECT_REMOTE_STATE 0x0100U
#define MC_TICKS_SAT 0xFFFFU
#define MC_PAYLOAD_MASK 0xFFFFU

/* ---- small helpers ------------------------------------------------------ */

/** Telemetry payload hi<<8 | lo, computed unsigned (no int promotion). */
static uint16_t
payload (uint32_t hi, uint32_t lo)
{
    return (uint16_t) (((hi << MC_PAYLOAD_SHIFT) | lo) & MC_PAYLOAD_MASK);
}

static void
post_tev (mission_out_t * p_out, uint8_t type, uint16_t payload)
{
    if (p_out->n_tev < MISSION_TEV_MAX)
    {
        p_out->tev[p_out->n_tev].type    = type;
        p_out->tev[p_out->n_tev].payload = payload;
        p_out->n_tev++;
    }
}

static void
publish (mission_core_t * p_core, mission_out_t * p_out)
{
    p_core->ms.seq         = (uint8_t) ((p_core->ms.seq + 1U) & MC_SEQ_MASK);
    p_out->mission_changed = true;
}

static void
set_state (mission_core_t * p_core, mission_out_t * p_out,
           app_sys_state_t new_state)
{
    if (p_core->ms.sys_state != (uint8_t) new_state)
    {
        p_core->ms.sys_state   = (uint8_t) new_state;
        p_core->sub            = 0U;
        p_core->ticks_in_state = 0U;
        p_core->ticks_in_sub   = 0U;
        publish(p_core, p_out);
        post_tev(p_out, (uint8_t) APP_TEV_SYS_STATE, (uint16_t) new_state);
    }
}

static void
set_sub (mission_core_t * p_core, uint8_t sub)
{
    p_core->sub          = sub;
    p_core->ticks_in_sub = 0U;
}

static void
request_stop (mission_out_t * p_out, app_stop_cause_t cause)
{
    if ((uint8_t) APP_STOP_CAUSE_NONE == p_out->stop_request)
    {
        p_out->stop_request = (uint8_t) cause;
    }
}

static void
request_scan (mission_out_t * p_out, uint32_t req)
{
    p_out->scan_req |= req;
}

static void
watch (mission_core_t * p_core, mission_out_t * p_out, bool on)
{
    if (on != p_core->watch_on)
    {
        p_core->watch_on = on;
        request_scan(p_out,
                     on ? APP_SCAN_REQ_WATCH_ON : APP_SCAN_REQ_WATCH_OFF);
    }
}

static bool
in_state (mission_core_t const * p_core, app_sys_state_t s)
{
    return p_core->ms.sys_state == (uint8_t) s;
}

static bool
motion_stopped (mission_in_t const * p_in)
{
    return p_in->motion_valid && (0U == p_in->motion.speed_l_cm_s) &&
           (0U == p_in->motion.speed_r_cm_s) &&
           (((uint8_t) APP_MOTION_BRAKED == p_in->motion.motion_state) ||
            ((uint8_t) APP_MOTION_STOP_LATCHED == p_in->motion.motion_state));
}

/* ---- motion sequencing --------------------------------------------------- */

/** Queue the next drive command; emitted by motion_service(). */
static void
want_motion (mission_core_t * p_core, app_motion_cmd_type_t type,
             uint8_t speed_cm_s, int16_t param, bool until_line)
{
    p_core->want_pending    = true;
    p_core->want_type       = (uint8_t) type;
    p_core->want_speed      = speed_cm_s;
    p_core->want_param      = param;
    p_core->want_until_line = until_line;
    p_core->awaiting        = false;
    p_core->rejects         = 0U;
}

static void
emit_motion (mission_core_t * p_core, mission_in_t const * p_in,
             mission_out_t * p_out, uint8_t type, uint8_t speed, int16_t param,
             bool until_line)
{
    p_core->motion_seq    = (uint8_t) ((p_core->motion_seq + 1U) & MC_SEQ_MASK);
    p_out->send_motion    = true;
    p_out->mc_type        = type;
    p_out->mc_speed_cm_s  = speed;
    p_out->mc_param       = param;
    p_out->mc_until_line  = until_line;
    p_out->mc_seq         = p_core->motion_seq;
    p_core->emit_done_seq = p_in->motion.done_seq;
}

/** Send the queued command when the executor can take it (BRAKED), else a
 *  BRAKE first. One record per step. */
static void
motion_service (mission_core_t * p_core, mission_in_t const * p_in,
                mission_out_t * p_out)
{
    if (!p_core->want_pending || p_out->send_motion || p_in->stop_latched)
    {
        return;
    }
    if (((uint8_t) APP_MC_BRAKE == p_core->want_type) ||
        (p_in->motion_valid &&
         ((uint8_t) APP_MOTION_BRAKED == p_in->motion.motion_state)))
    {
        emit_motion(p_core, p_in, p_out, p_core->want_type, p_core->want_speed,
                    p_core->want_param, p_core->want_until_line);
        p_core->await_seq    = p_core->motion_seq;
        p_core->awaiting     = true;
        p_core->want_pending = false;
    }
    else
    {
        emit_motion(p_core, p_in, p_out, (uint8_t) APP_MC_BRAKE, 0U, 0, false);
    }
}

/** True once the awaited command reported DONE; *p_result receives it. */
static bool
motion_done (mission_core_t * p_core, mission_in_t const * p_in,
             uint8_t * p_result)
{
    bool const evt = (0U != (p_in->events & APP_EVT_MOTION_DONE));

    if (!p_core->awaiting || !p_in->motion_valid)
    {
        return false;
    }
    if ((p_in->motion.done_seq == p_core->await_seq) &&
        (evt || (p_in->motion.done_seq != p_core->emit_done_seq)))
    {
        p_core->awaiting = false;
        *p_result        = p_in->motion.done_result;
        return true;
    }
    return false;
}

/** A rejection of the awaited command: retry once, then critical stop. */
static void
motion_rejected (mission_core_t * p_core, mission_in_t const * p_in,
                 mission_out_t * p_out)
{
    if (!p_core->awaiting || (0U == (p_in->events & APP_EVT_MOTION_REJECT)))
    {
        return;
    }
    p_core->rejects++;
    if (p_core->rejects >= MC_REJECT_MAX)
    {
        request_stop(p_out, APP_STOP_CAUSE_CRITICAL_RUNTIME);
        return;
    }
    /* re-queue the same command; the sequencer brakes first if needed */
    p_core->want_pending = true;
    p_core->awaiting     = false;
}

/* ---- transitions shared by several states -------------------------------- */

static void
enter_follow (mission_core_t * p_core, mission_in_t const * p_in,
              mission_out_t * p_out)
{
    want_motion(p_core, APP_MC_FOLLOW, APP_CAL_V_NOM_CM_S, 0, false);
    p_core->dist_ref_mm = p_in->motion_valid ? p_in->motion.distance_mm : 0;
    watch(p_core, p_out, true);
    set_state(p_core, p_out, APP_SYS_S2_LINE_FOLLOWING);
}

static void
enter_search (mission_core_t * p_core, mission_out_t * p_out)
{
    p_core->search_dir = (p_core->last_err_sign < 0) ? (int8_t) -1 : (int8_t) 1;
    p_core->stats_searches++;
    watch(p_core, p_out, true); /* D-30: forward watch stays on in S5 */
    set_state(p_core, p_out, APP_SYS_S5_LINE_SEARCH);
    set_sub(p_core, (uint8_t) MC_SRCH_TURN_A);
    /* + param = left (CCW); line right of centre (+err) needs a right turn */
    want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S,
                (int16_t) (-(int16_t) p_core->search_dir *
                           (int16_t) APP_CAL_SEARCH_ANGLE_DDEG),
                true);
    post_tev(p_out, (uint8_t) APP_TEV_SEARCH,
             payload((uint32_t) MC_SRCH_TURN_A, (uint32_t) MC_SEARCH_START));
}

static void
enter_nav (mission_core_t * p_core, mission_out_t * p_out, uint8_t cmd,
           bool implicit)
{
    p_core->nav_cmd      = cmd;
    p_core->nav_implicit = implicit;
    p_core->stats_nav++;
    watch(p_core, p_out, false); /* the sensor sweeps the obstacle field */
    set_state(p_core, p_out, APP_SYS_S3_EXECUTE_NAV);
    post_tev(p_out, (uint8_t) APP_TEV_NAV,
             payload((uint32_t) cmd, (uint32_t) MC_NAV_START));
    if (((uint8_t) APP_NAV_STRAIGHT == cmd) ||
        (0U == APP_CAL_D_JUNC_ADVANCE_MM))
    {
        set_sub(p_core, (uint8_t) MC_NAV_EXECUTE);
    }
    else
    {
        set_sub(p_core, (uint8_t) MC_NAV_ADVANCE);
        want_motion(p_core, APP_MC_MOVE, APP_CAL_V_NOM_CM_S,
                    (int16_t) APP_CAL_D_JUNC_ADVANCE_MM, false);
        return;
    }
    if ((uint8_t) APP_NAV_STRAIGHT == cmd)
    {
        want_motion(p_core, APP_MC_MOVE, APP_CAL_V_NOM_CM_S,
                    (int16_t) APP_CAL_D_JUNC_CLEAR_MM, false);
    }
    else if ((uint8_t) APP_NAV_LEFT == cmd)
    {
        want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, MC_DDEG_90, true);
    }
    else if ((uint8_t) APP_NAV_RIGHT == cmd)
    {
        want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, -MC_DDEG_90,
                    true);
    }
    else
    {
        want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, MC_DDEG_180,
                    true);
    }
}

static void
enter_bypass (mission_core_t * p_core, mission_out_t * p_out, bool retry)
{
    if (!retry)
    {
        p_core->ms.retry_count = 0U;
        p_core->stats_bypasses++;
        set_state(p_core, p_out, APP_SYS_S4_OBSTACLE_BYPASS);
    }
    watch(p_core, p_out, false);
    set_sub(p_core, (uint8_t) MC_BYP_BRAKE);
    want_motion(p_core, APP_MC_BRAKE, 0U, 0, false);
    post_tev(
        p_out, (uint8_t) APP_TEV_BYPASS,
        payload((uint32_t) MC_BYP_BRAKE, (uint32_t) p_core->ms.retry_count
                                             << MC_BYP_DETAIL_RETRY_SHIFT));
}

static void
bypass_step_tev (mission_core_t const * p_core, mission_out_t * p_out,
                 uint8_t step)
{
    post_tev(
        p_out, (uint8_t) APP_TEV_BYPASS,
        payload((uint32_t) step, (p_core->bypass_left ? MC_BYP_DETAIL_LEFT
                                                      : MC_BYP_DETAIL_RIGHT) |
                                     ((uint32_t) p_core->ms.retry_count
                                      << MC_BYP_DETAIL_RETRY_SHIFT)));
}

/* ---- barcode handling (any RUNNING state) -------------------------------- */

static void
take_barcode (mission_core_t * p_core, mission_in_t const * p_in,
              mission_out_t * p_out)
{
    app_barcode_result_t const r   = app_barcode_result(p_in->barcode_byte);
    uint8_t                    nav = (uint8_t) APP_NAV_NONE;

    if (0U == (p_in->events & APP_EVT_BARCODE))
    {
        return;
    }
    switch (r)
    {
        case APP_BC_LEFT:
            nav = (uint8_t) APP_NAV_LEFT;
            break;
        case APP_BC_RIGHT:
            nav = (uint8_t) APP_NAV_RIGHT;
            break;
        case APP_BC_STRAIGHT:
            nav = (uint8_t) APP_NAV_STRAIGHT;
            break;
        case APP_BC_UTURN:
            nav = (uint8_t) APP_NAV_UTURN;
            break;
        case APP_BC_NONE:
        case APP_BC_INV_NO_MATCH:
        case APP_BC_INV_OVERFLOW:
        default:
            nav = (uint8_t) APP_NAV_NONE; /* FR7: reported by the decoder */
            break;
    }
    if (((uint8_t) APP_NAV_NONE != nav) && (nav != p_core->ms.pending_cmd))
    {
        /* a newer valid code replaces an older pending one */
        p_core->ms.pending_cmd = nav;
        publish(p_core, p_out);
    }
}

/* ---- S0 / S1 / S7 ------------------------------------------------------ */

static bool
initiating_condition_cleared (mission_in_t const * p_in)
{
    uint16_t const faults = p_in->faults;
    switch ((app_stop_cause_t) p_in->stop_cause)
    {
        case APP_STOP_CAUSE_OPERATOR_STOP:
            return !p_in->stop_pressed;
        case APP_STOP_CAUSE_COLLISION:
            return 0U == (faults & APP_FAULT_IMU_FAULT);
        case APP_STOP_CAUSE_STALL:
            return 0U ==
                   (faults & (APP_FAULT_MOTOR_STALL | APP_FAULT_ENCODER_FAULT));
        case APP_STOP_CAUSE_STALE_LINE:
            return 0U == (faults & APP_FAULT_STALE_LINE);
        case APP_STOP_CAUSE_STALE_MOTION:
            return 0U == (faults & APP_FAULT_STALE_MOTION);
        case APP_STOP_CAUSE_CTRL_HEARTBEAT:
            return 0U == (faults & APP_FAULT_CTRL_HEARTBEAT);
        case APP_STOP_CAUSE_SCAN_TIMEOUT:
            return 0U == (faults & APP_FAULT_SCAN_TIMEOUT);
        case APP_STOP_CAUSE_NO_CLEARANCE: /* operator must move the car */
        case APP_STOP_CAUSE_LINE_SEARCH_TIMEOUT:
            return true;
        case APP_STOP_CAUSE_CRITICAL_RUNTIME: /* needs a reboot */
        case APP_STOP_CAUSE_REMOTE_STOP:
        case APP_STOP_CAUSE_NONE:
        case APP_STOP_CAUSE_COUNT:
        default:
            return false;
    }
}

static void
start_run (mission_core_t * p_core, mission_in_t const * p_in,
           mission_out_t * p_out)
{
    if (p_in->motion_armed)
    {
        p_core->ms.pending_cmd = (uint8_t) APP_NAV_NONE;
        p_core->ms.retry_count = 0U;
        enter_follow(p_core, p_in, p_out);
    }
    else
    {
        /* Fail closed: calibration not signed (HV-3/HV-4). */
        post_tev(p_out, (uint8_t) APP_TEV_CMD_REJECTED, MC_REJECT_NOT_ARMED);
    }
}

static void
s7_tick (mission_core_t * p_core, mission_in_t const * p_in,
         mission_out_t * p_out)
{
    if (p_in->start_pressed)
    {
        p_core->s7_hold_ticks++;
    }
    else
    {
        p_core->s7_hold_ticks = 0U;
    }
    if (motion_stopped(p_in) && initiating_condition_cleared(p_in) &&
        (p_core->s7_hold_ticks >= MC_MS_TO_TICKS(APP_CAL_T_HOLD_MS)))
    {
        p_out->clear_latch       = true;
        p_core->s7_hold_ticks    = 0U;
        p_core->start_armed_edge = false; /* the held START is consumed */
        p_core->ms.pending_cmd   = (uint8_t) APP_NAV_NONE;
        p_core->ms.retry_count   = 0U;
        p_core->want_pending     = false;
        p_core->awaiting         = false;
        post_tev(p_out, (uint8_t) APP_TEV_RECOVERY, 0U);
        set_state(p_core, p_out, APP_SYS_S0_BOOT_BRAKED);
    }
}

/* ---- S2 LINE FOLLOWING --------------------------------------------------- */

static void
s2_step (mission_core_t * p_core, mission_in_t const * p_in,
         mission_out_t * p_out)
{
    uint8_t result = 0U;

    if (0U != (p_in->events & APP_EVT_OBSTACLE))
    {
        enter_bypass(p_core, p_out, false);
        return;
    }
    if (0U != (p_in->events & APP_EVT_JUNCTION))
    {
        uint8_t const cmd = p_core->ms.pending_cmd;
        if ((uint8_t) APP_NAV_NONE != cmd)
        {
            enter_nav(p_core, p_out, cmd, false);
        }
        else
        {
            enter_nav(p_core, p_out, (uint8_t) APP_NAV_STRAIGHT,
                      true); /* D-29 */
        }
        return;
    }
    if (0U != (p_in->events & APP_EVT_LINE_LOST))
    {
        enter_search(p_core, p_out);
        return;
    }
    /* FOLLOW never completes by itself (D-31); a DONE here means the
       executor dropped it (ABORTED / TIMEOUT): request it again. */
    if (motion_done(p_core, p_in, &result))
    {
        want_motion(p_core, APP_MC_FOLLOW, APP_CAL_V_NOM_CM_S, 0, false);
    }
}

/* ---- S3 EXECUTE NAV ------------------------------------------------------ */

static void
nav_finished (mission_core_t * p_core, mission_in_t const * p_in,
              mission_out_t * p_out, uint8_t result)
{
    uint8_t const cmd = p_core->nav_cmd;

    if (!p_core->nav_implicit && (cmd == p_core->ms.pending_cmd))
    {
        p_core->ms.pending_cmd = (uint8_t) APP_NAV_NONE; /* consumed */
        publish(p_core, p_out);
    }
    if ((uint8_t) APP_MOTION_RESULT_OK == result)
    {
        post_tev(p_out, (uint8_t) APP_TEV_NAV,
                 payload((uint32_t) cmd, (uint32_t) MC_NAV_DONE));
        enter_follow(p_core, p_in, p_out);
    }
    else
    {
        /* until_line never saw the line (TIMEOUT): bounded search (FR2) */
        post_tev(p_out, (uint8_t) APP_TEV_NAV,
                 payload((uint32_t) cmd, (uint32_t) MC_NAV_LOST));
        enter_search(p_core, p_out);
    }
}

static void
s3_step (mission_core_t * p_core, mission_in_t const * p_in,
         mission_out_t * p_out)
{
    uint8_t result = 0U;

    if (0U != (p_in->events & APP_EVT_OBSTACLE))
    {
        /* D-30: not handled by the bypass in S3; fail closed */
        want_motion(p_core, APP_MC_BRAKE, 0U, 0, false);
        request_stop(p_out, APP_STOP_CAUSE_NO_CLEARANCE);
        return;
    }
    if (!motion_done(p_core, p_in, &result))
    {
        return; /* LINE_LOST / JUNCTION during a pivot are expected: ignored */
    }
    if ((uint8_t) MC_NAV_ADVANCE == p_core->sub)
    {
        uint8_t const cmd = p_core->nav_cmd;
        set_sub(p_core, (uint8_t) MC_NAV_EXECUTE);
        if ((uint8_t) APP_NAV_LEFT == cmd)
        {
            want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, MC_DDEG_90,
                        true);
        }
        else if ((uint8_t) APP_NAV_RIGHT == cmd)
        {
            want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, -MC_DDEG_90,
                        true);
        }
        else
        {
            want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, MC_DDEG_180,
                        true);
        }
        return;
    }
    nav_finished(p_core, p_in, p_out, result);
}

/* ---- S4 OBSTACLE BYPASS -------------------------------------------------- */

uint16_t
mission_core_offset_mm (app_obstacle_profile_t const * p_profile, bool left)
{
    int32_t const half_car = (int32_t) (APP_CAL_W_CAR_CM / MC_HALF);
    int32_t const edge =
        left ? (int32_t) p_profile->y_max_cm : -(int32_t) p_profile->y_min_cm;
    int32_t off_cm = edge + half_car + (int32_t) APP_CAL_M_CLEAR_CM;

    if (off_cm < 0)
    {
        off_cm = 0;
    }
    if (off_cm > (int32_t) APP_CAL_Y_PATH_MAX_CM)
    {
        off_cm = (int32_t) APP_CAL_Y_PATH_MAX_CM;
    }
    return (uint16_t) ((uint32_t) off_cm * MC_MM_PER_CM);
}

/** Retry (reverse and rescan) or NO_CLEARANCE, per Rev B 6.3. */
static void
bypass_no_path (mission_core_t * p_core, mission_in_t const * p_in,
                mission_out_t * p_out)
{
    int32_t const travelled = p_in->motion_valid
                                  ? ((int32_t) p_in->motion.distance_mm -
                                     (int32_t) p_core->dist_ref_mm)
                                  : 0;
    int32_t const needed =
        (int32_t) (APP_CAL_D_REVERSE_MM + APP_CAL_D_REVERSE_MARGIN_MM);

    if ((p_core->ms.retry_count < APP_CAL_N_RETRY_MAX) && (travelled >= needed))
    {
        p_core->ms.retry_count++;
        publish(p_core, p_out);
        set_sub(p_core, (uint8_t) MC_BYP_REVERSE);
        want_motion(p_core, APP_MC_MOVE, APP_CAL_V_REVERSE_CM_S,
                    (int16_t) (-(int16_t) APP_CAL_D_REVERSE_MM), false);
        bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_REVERSE);
    }
    else
    {
        request_stop(p_out, APP_STOP_CAUSE_NO_CLEARANCE);
    }
}

static void
bypass_plan (mission_core_t * p_core, mission_in_t const * p_in,
             mission_out_t * p_out)
{
    app_obstacle_profile_t const * p = &p_in->profile;
    uint32_t const min_front         = APP_CAL_R_SWEPT_CM + APP_CAL_M_CLEAR_CM;

    if (!p_in->profile_valid)
    {
        request_stop(p_out, APP_STOP_CAUSE_SCAN_TIMEOUT);
        return;
    }
    switch ((app_avoid_action_t) p->action)
    {
        case APP_AVOID_CONTINUE:
            enter_follow(p_core, p_in, p_out);
            break;
        case APP_AVOID_TURN_LEFT:
        case APP_AVOID_TURN_RIGHT:
            /* post-brake clearance guard before the first pivot */
            if ((APP_RANGE_CM_SENTINEL != p->range_cm) &&
                ((uint32_t) p->range_cm < min_front))
            {
                bypass_no_path(p_core, p_in, p_out);
                break;
            }
            p_core->bypass_left = ((uint8_t) APP_AVOID_TURN_LEFT == p->action);
            p_core->offset_mm = mission_core_offset_mm(p, p_core->bypass_left);
            set_sub(p_core, (uint8_t) MC_BYP_A1_TURN);
            want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S,
                        p_core->bypass_left ? MC_DDEG_90 : -MC_DDEG_90, false);
            bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A1_TURN);
            break;
        case APP_AVOID_REVERSE:
        case APP_AVOID_STOP:
        default:
            bypass_no_path(p_core, p_in, p_out);
            break;
    }
}

/** Action 4: side ranging while moving past; true when the side is free. */
static bool
bypass_side_free (mission_core_t * p_core, mission_in_t const * p_in)
{
    uint16_t const r     = p_in->profile.range_cm;
    int32_t const  moved = p_in->motion_valid
                               ? ((int32_t) p_in->motion.distance_mm -
                                 (int32_t) p_core->dist_ref_mm)
                               : 0;

    if (!p_in->profile_valid)
    {
        return false;
    }
    if ((APP_RANGE_CM_SENTINEL != r) && (r <= APP_CAL_R_SIDE_FREE_CM))
    {
        p_core->side_free_count = 0U;
        return false;
    }
    if (p_core->side_free_count < APP_CAL_SIDE_FREE_COUNT)
    {
        p_core->side_free_count++;
    }
    return (p_core->side_free_count >= APP_CAL_SIDE_FREE_COUNT) &&
           (moved >= (int32_t) APP_CAL_D_PASS_MIN_MM);
}

/** S4 actions 1-4: turn out, offset, turn back, pass with side ranging. */
static void
s4_leg_out (mission_core_t * p_core, mission_in_t const * p_in,
            mission_out_t * p_out, bool done)
{
    bool const    left = p_core->bypass_left;
    int16_t const back = left ? (int16_t) -MC_DDEG_90 : (int16_t) MC_DDEG_90;

    switch ((mc_bypass_step_t) p_core->sub)
    {
        case MC_BYP_A1_TURN:
            if (done)
            {
                set_sub(p_core, (uint8_t) MC_BYP_A2_OFFSET);
                want_motion(p_core, APP_MC_MOVE, APP_CAL_V_NOM_CM_S,
                            (int16_t) p_core->offset_mm, false);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A2_OFFSET);
            }
            break;
        case MC_BYP_A2_OFFSET:
            if (done)
            {
                set_sub(p_core, (uint8_t) MC_BYP_A3_TURN);
                want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, back,
                            false);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A3_TURN);
            }
            break;
        case MC_BYP_A3_TURN:
            if (done)
            {
                set_sub(p_core, (uint8_t) MC_BYP_A4_PASS);
                p_core->side_free_count = 0U;
                p_core->dist_ref_mm     = p_in->motion.distance_mm;
                request_scan(p_out,
                             left ? APP_SCAN_REQ_SIDE_R : APP_SCAN_REQ_SIDE_L);
                want_motion(p_core, APP_MC_MOVE, APP_CAL_V_NOM_CM_S,
                            (int16_t) APP_CAL_D_PASS_MAX_MM, false);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A4_PASS);
            }
            break;
        case MC_BYP_A4_PASS:
            if (p_core->awaiting && bypass_side_free(p_core, p_in))
            {
                /* obstacle passed: drive one more car length, then turn */
                set_sub(p_core, (uint8_t) MC_BYP_A4_EXTRA);
                want_motion(p_core, APP_MC_MOVE, APP_CAL_V_NOM_CM_S,
                            (int16_t) APP_CAL_D_PASS_EXTRA_MM, false);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A4_EXTRA);
            }
            else if (done)
            {
                /* D_PASS_MAX reached without a free side: bounded, go on */
                set_sub(p_core, (uint8_t) MC_BYP_A5_TURN);
                request_scan(p_out, APP_SCAN_REQ_IDLE);
                want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, back,
                            false);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A5_TURN);
            }
            else
            {
                /* passing */
            }
            break;
        case MC_BYP_A4_EXTRA:
            if (done)
            {
                set_sub(p_core, (uint8_t) MC_BYP_A5_TURN);
                request_scan(p_out, APP_SCAN_REQ_IDLE);
                want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, back,
                            false);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A5_TURN);
            }
            break;
        case MC_BYP_A5_TURN:
        case MC_BYP_A5_MOVE:
        case MC_BYP_A6_ALIGN:
        case MC_BYP_BRAKE:
        case MC_BYP_PROFILE:
        case MC_BYP_PLAN:
        case MC_BYP_REVERSE:
        case MC_BYP_STEP_COUNT:
        default:
            break; /* handled by s4_wait_steps */
    }
}

/** S4 actions 5-6: turn toward the line, creep until_line, align. */
static void
s4_leg_back (mission_core_t * p_core, mission_in_t const * p_in,
             mission_out_t * p_out, bool done, uint8_t result)
{
    bool const    left = p_core->bypass_left;
    int16_t const fwd  = left ? (int16_t) MC_DDEG_90 : (int16_t) -MC_DDEG_90;

    switch ((mc_bypass_step_t) p_core->sub)
    {
        case MC_BYP_A5_TURN:
            if (done)
            {
                set_sub(p_core, (uint8_t) MC_BYP_A5_MOVE);
                want_motion(
                    p_core, APP_MC_MOVE, APP_CAL_V_SEARCH_CM_S,
                    (int16_t) (p_core->offset_mm + APP_CAL_D_RETURN_MARGIN_MM),
                    true);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A5_MOVE);
            }
            break;
        case MC_BYP_A5_MOVE:
            if (done)
            {
                if ((uint8_t) APP_MOTION_RESULT_OK == result)
                {
                    set_sub(p_core, (uint8_t) MC_BYP_A6_ALIGN);
                    want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S, fwd,
                                true);
                    bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_A6_ALIGN);
                }
                else
                {
                    enter_search(p_core, p_out); /* FR19: bounded search */
                }
            }
            break;
        case MC_BYP_A6_ALIGN:
            if (done)
            {
                if ((uint8_t) APP_MOTION_RESULT_OK == result)
                {
                    p_core->ms.retry_count = 0U;
                    publish(p_core, p_out);
                    enter_follow(p_core, p_in, p_out);
                }
                else
                {
                    enter_search(p_core, p_out);
                }
            }
            break;
        case MC_BYP_A1_TURN:
        case MC_BYP_A2_OFFSET:
        case MC_BYP_A3_TURN:
        case MC_BYP_A4_PASS:
        case MC_BYP_A4_EXTRA:
        case MC_BYP_BRAKE:
        case MC_BYP_PROFILE:
        case MC_BYP_PLAN:
        case MC_BYP_REVERSE:
        case MC_BYP_STEP_COUNT:
        default:
            break; /* handled by s4_wait_steps */
    }
}

/** S4 six actions (Rev B 6.3), one motion command per step. */
static void
s4_action_steps (mission_core_t * p_core, mission_in_t const * p_in,
                 mission_out_t * p_out, bool done, uint8_t result)
{
    if (p_core->sub <= (uint8_t) MC_BYP_A4_EXTRA)
    {
        s4_leg_out(p_core, p_in, p_out, done);
    }
    else
    {
        s4_leg_back(p_core, p_in, p_out, done, result);
    }
}

/** S4 steps that wait for the executor or the scanner (BRAKE, PROFILE,
 *  PLAN, REVERSE). */
static void
s4_wait_steps (mission_core_t * p_core, mission_in_t const * p_in,
               mission_out_t * p_out, bool done, uint8_t result)
{
    switch ((mc_bypass_step_t) p_core->sub)
    {
        case MC_BYP_BRAKE:
            if (motion_stopped(p_in) && !p_core->want_pending)
            {
                set_sub(p_core, (uint8_t) MC_BYP_PROFILE);
                request_scan(p_out, APP_SCAN_REQ_PROFILE);
                bypass_step_tev(p_core, p_out, (uint8_t) MC_BYP_PROFILE);
            }
            else if (p_core->ticks_in_sub >
                     MC_MS_TO_TICKS(APP_CAL_T_SPEED_ZERO_MS))
            {
                request_stop(p_out, APP_STOP_CAUSE_CRITICAL_RUNTIME);
            }
            else
            {
                /* waiting for speed 0 */
            }
            break;
        case MC_BYP_PROFILE:
            if (0U != (p_in->events & APP_EVT_SCAN_FAIL))
            {
                request_stop(p_out, APP_STOP_CAUSE_SCAN_TIMEOUT); /* D-22 */
            }
            else if (0U != (p_in->events & APP_EVT_SCAN_DONE))
            {
                set_sub(p_core, (uint8_t) MC_BYP_PLAN);
                bypass_plan(p_core, p_in, p_out);
            }
            else if (p_core->ticks_in_sub >
                     MC_MS_TO_TICKS(APP_CAL_T_SCAN_TIMEOUT_MS +
                                    APP_CAL_T_SCAN_WAIT_MARGIN_MS))
            {
                request_stop(p_out, APP_STOP_CAUSE_SCAN_TIMEOUT);
            }
            else
            {
                /* scanning */
            }
            break;
        case MC_BYP_PLAN:
            break; /* transient: bypass_plan() always moves on */
        case MC_BYP_REVERSE:
            if (done)
            {
                /* corridor behind is now used up: rescan from here */
                p_core->dist_ref_mm = p_in->motion.distance_mm;
                enter_bypass(p_core, p_out, true);
            }
            break;
        case MC_BYP_A1_TURN:
        case MC_BYP_A2_OFFSET:
        case MC_BYP_A3_TURN:
        case MC_BYP_A4_PASS:
        case MC_BYP_A4_EXTRA:
        case MC_BYP_A5_TURN:
        case MC_BYP_A5_MOVE:
        case MC_BYP_A6_ALIGN:
            s4_action_steps(p_core, p_in, p_out, done, result);
            break;
        case MC_BYP_STEP_COUNT:
        default:
            request_stop(p_out, APP_STOP_CAUSE_CRITICAL_RUNTIME);
            break;
    }
}

static void
s4_step (mission_core_t * p_core, mission_in_t const * p_in,
         mission_out_t * p_out)
{
    uint8_t    result = 0U;
    bool const done   = motion_done(p_core, p_in, &result);

    if (p_core->ticks_in_state > MC_MS_TO_TICKS(APP_CAL_T_BYPASS_MAX_MS))
    {
        request_stop(p_out, APP_STOP_CAUSE_NO_CLEARANCE);
        return;
    }
    if (0U != (p_in->events & APP_EVT_SIDE_UNSAFE))
    {
        want_motion(p_core, APP_MC_BRAKE, 0U, 0, false);
        request_stop(p_out, APP_STOP_CAUSE_NO_CLEARANCE);
        return;
    }
    if (done && ((uint8_t) APP_MOTION_RESULT_OK != result) &&
        ((uint8_t) MC_BYP_A5_MOVE != p_core->sub) &&
        ((uint8_t) MC_BYP_A6_ALIGN != p_core->sub) &&
        ((uint8_t) MC_BYP_A4_PASS != p_core->sub))
    {
        /* a bypass movement that did not complete leaves the car in an
           unknown pose: fail closed */
        request_stop(p_out, APP_STOP_CAUSE_NO_CLEARANCE);
        return;
    }

    s4_wait_steps(p_core, p_in, p_out, done, result);
}

/* ---- S5 LINE SEARCH ------------------------------------------------------ */

static void
search_found (mission_core_t * p_core, mission_in_t const * p_in,
              mission_out_t * p_out)
{
    post_tev(p_out, (uint8_t) APP_TEV_SEARCH,
             payload((uint32_t) p_core->sub, (uint32_t) MC_SEARCH_FOUND));
    enter_follow(p_core, p_in, p_out);
}

static void
s5_step (mission_core_t * p_core, mission_in_t const * p_in,
         mission_out_t * p_out)
{
    uint8_t       result = 0U;
    int16_t const angle  = (int16_t) APP_CAL_SEARCH_ANGLE_DDEG;
    int16_t const dir    = (int16_t) p_core->search_dir;

    if (0U != (p_in->events & APP_EVT_OBSTACLE))
    {
        want_motion(p_core, APP_MC_BRAKE, 0U, 0, false); /* D-30 */
        request_stop(p_out, APP_STOP_CAUSE_NO_CLEARANCE);
        return;
    }
    if (p_core->ticks_in_state > MC_MS_TO_TICKS(APP_CAL_T_LINE_SEARCH_MS))
    {
        post_tev(p_out, (uint8_t) APP_TEV_SEARCH,
                 payload((uint32_t) p_core->sub, (uint32_t) MC_SEARCH_FAIL));
        request_stop(p_out, APP_STOP_CAUSE_LINE_SEARCH_TIMEOUT);
        return;
    }
    if (!motion_done(p_core, p_in, &result))
    {
        return;
    }
    if ((uint8_t) APP_MOTION_RESULT_OK == result)
    {
        if ((uint8_t) MC_SRCH_RECENTRE == p_core->sub)
        {
            /* the re-centring turn is not an until_line move: creep next */
            set_sub(p_core, (uint8_t) MC_SRCH_CREEP);
            want_motion(p_core, APP_MC_MOVE, APP_CAL_V_SEARCH_CM_S,
                        (int16_t) APP_CAL_D_SEARCH_CREEP_MM, true);
        }
        else
        {
            search_found(p_core, p_in, p_out);
        }
        return;
    }
    /* TIMEOUT = the bound was reached without the line: next sweep */
    switch ((mc_search_step_t) p_core->sub)
    {
        case MC_SRCH_TURN_A:
            set_sub(p_core, (uint8_t) MC_SRCH_TURN_B);
            want_motion(
                p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S,
                (int16_t) (dir * (int16_t) (MC_HALF * (uint16_t) angle)), true);
            break;
        case MC_SRCH_TURN_B:
            set_sub(p_core, (uint8_t) MC_SRCH_RECENTRE);
            want_motion(p_core, APP_MC_TURN, APP_CAL_V_TURN_CM_S,
                        (int16_t) (-dir * angle), false);
            break;
        case MC_SRCH_CREEP:
        case MC_SRCH_RECENTRE:
        case MC_SRCH_STEP_COUNT:
        default:
            post_tev(
                p_out, (uint8_t) APP_TEV_SEARCH,
                payload((uint32_t) p_core->sub, (uint32_t) MC_SEARCH_FAIL));
            request_stop(p_out, APP_STOP_CAUSE_LINE_SEARCH_TIMEOUT);
            break;
    }
}

/* ---- public -------------------------------------------------------------- */

void
mission_core_init (mission_core_t * p_core)
{
    uint8_t * p = (uint8_t *) p_core;
    uint32_t  i = 0U;

    for (i = 0U; i < sizeof(*p_core); i++)
    {
        p[i] = 0U;
    }
    p_core->ms.sys_state   = (uint8_t) APP_SYS_S0_BOOT_BRAKED;
    p_core->ms.pending_cmd = (uint8_t) APP_NAV_NONE;
    p_core->await_seq      = MC_SEQ_NONE;
    p_core->search_dir     = 1;
    p_core->last_err_sign  = 1;
}

uint32_t
mission_core_status (mission_core_t const * p_core)
{
    return app_mission_pack(&p_core->ms);
}

/** Per-tick bookkeeping: timers, START debounce, last line side. */
static void
tick_update (mission_core_t * p_core, mission_in_t const * p_in)
{
    p_core->ticks_in_state++;
    p_core->ticks_in_sub++;
    /* START debounce: pressed for >= T_START_DEBOUNCE, once per press */
    if (p_in->start_pressed)
    {
        if (p_core->start_held_ticks < MC_TICKS_SAT)
        {
            p_core->start_held_ticks++;
        }
    }
    else
    {
        p_core->start_held_ticks = 0U;
        p_core->start_armed_edge = true;
    }
    if (0 != p_in->line.line_error)
    {
        p_core->last_err_sign =
            (p_in->line.line_error < 0) ? (int8_t) -1 : (int8_t) 1;
    }
}

/** Safety, remote STOP and barcode handling common to every state. */
static void
common_step (mission_core_t * p_core, mission_in_t const * p_in,
             mission_out_t * p_out)
{
    /* Safety first: the latch (or its event) always wins. */
    if ((0U != (p_in->events & APP_EVT_SAFE_STOP)) || p_in->stop_latched)
    {
        if (!in_state(p_core, APP_SYS_S7_SAFE_STOP))
        {
            p_core->s7_hold_ticks = 0U;
            p_core->want_pending  = false;
            p_core->awaiting      = false;
            watch(p_core, p_out, false);
            request_scan(p_out, APP_SCAN_REQ_IDLE);
            set_state(p_core, p_out, APP_SYS_S7_SAFE_STOP);
        }
    }

    /* Remote STOP: D-01, validated frame on the controller path. */
    if (p_in->remote_stop)
    {
        request_stop(p_out, APP_STOP_CAUSE_OPERATOR_STOP);
    }

    /* A valid barcode is stored in every RUNNING state. */
    if (in_state(p_core, APP_SYS_S2_LINE_FOLLOWING) ||
        in_state(p_core, APP_SYS_S3_EXECUTE_NAV) ||
        in_state(p_core, APP_SYS_S4_OBSTACLE_BYPASS) ||
        in_state(p_core, APP_SYS_S5_LINE_SEARCH))
    {
        take_barcode(p_core, p_in, p_out);
        motion_rejected(p_core, p_in, p_out);
    }
}

/** The state dispatch. Returns true if a START was acted on. */
static bool
dispatch (mission_core_t * p_core, mission_in_t const * p_in,
          mission_out_t * p_out, bool tick)
{
    bool started = false;

    switch ((app_sys_state_t) p_core->ms.sys_state)
    {
        case APP_SYS_S0_BOOT_BRAKED:
            if (tick && p_in->boot_done && !p_in->stop_latched &&
                p_in->motion_valid &&
                ((uint8_t) APP_MOTION_BRAKED == p_in->motion.motion_state))
            {
                set_state(p_core, p_out, APP_SYS_S1_IDLE);
            }
            break;
        case APP_SYS_S1_IDLE:
            if (p_in->remote_start)
            {
                started = true;
                start_run(p_core, p_in, p_out);
            }
            else if (tick && p_core->start_armed_edge &&
                     (p_core->start_held_ticks >=
                      MC_MS_TO_TICKS(APP_CAL_T_START_DEBOUNCE_MS)))
            {
                p_core->start_armed_edge = false;
                start_run(p_core, p_in, p_out);
            }
            else
            {
                /* idle */
            }
            break;
        case APP_SYS_S2_LINE_FOLLOWING:
            s2_step(p_core, p_in, p_out);
            break;
        case APP_SYS_S3_EXECUTE_NAV:
            s3_step(p_core, p_in, p_out);
            break;
        case APP_SYS_S4_OBSTACLE_BYPASS:
            s4_step(p_core, p_in, p_out);
            break;
        case APP_SYS_S5_LINE_SEARCH:
            s5_step(p_core, p_in, p_out);
            break;
        case APP_SYS_S6_RUN_COMPLETE:
            break; /* D-28: unreachable on a loop course; stays braked */
        case APP_SYS_S7_SAFE_STOP:
            if (tick)
            {
                s7_tick(p_core, p_in, p_out);
            }
            break;
        default:
            request_stop(p_out, APP_STOP_CAUSE_CRITICAL_RUNTIME);
            break;
    }
    return started;
}

void
mission_core_step (mission_core_t * p_core, mission_in_t const * p_in,
                   mission_out_t * p_out)
{
    bool const tick    = (0U != (p_in->events & APP_EVT_TICK));
    bool       started = false; /* a START was acted on this step */

    p_out->send_motion     = false;
    p_out->mc_type         = (uint8_t) APP_MC_BRAKE;
    p_out->mc_speed_cm_s   = 0U;
    p_out->mc_param        = 0;
    p_out->mc_until_line   = false;
    p_out->mc_seq          = 0U;
    p_out->scan_req        = 0U;
    p_out->stop_request    = (uint8_t) APP_STOP_CAUSE_NONE;
    p_out->clear_latch     = false;
    p_out->mission_changed = false;
    p_out->n_tev           = 0U;

    if (tick)
    {
        tick_update(p_core, p_in);
    }
    common_step(p_core, p_in, p_out);
    started = dispatch(p_core, p_in, p_out, tick);

    if (p_in->remote_start && !started)
    {
        /* FR23 / D-02: a remote START is acted on only in S1 */
        post_tev(p_out, (uint8_t) APP_TEV_CMD_REJECTED,
                 (uint16_t) (MC_REJECT_REMOTE_STATE |
                             (uint16_t) APP_CMD_START_STATE));
    }

    motion_service(p_core, p_in, p_out);
}
