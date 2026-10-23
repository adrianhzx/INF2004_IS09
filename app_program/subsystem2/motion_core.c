/** @file motion_core.c
 *  @brief Pure motion executor implementation. See motion_core.h.
 */

#include "motion_core.h"

#include "../shared/app_cal.h"
#include "../shared/app_line.h"
#include "../shared/app_pack.h"

#define MC_CYCLE_MS 10U
#define MC_MS_TO_CYCLES(ms) (((ms) + MC_CYCLE_MS - 1U) / MC_CYCLE_MS)
#define MC_Q8 256
#define MC_PI_NUM 355U /* pi ~ 355/113 */
#define MC_PI_DEN 113U
#define MC_UM_PER_MM 1000U
#define MC_US_PER_32US 32U
#define MC_CM_PER_UM_X 100U /* um/us -> cm/s multiplier */
#define MC_TENTH_DEG_TURN 3600U
#define MC_PCT 100U
#define MC_INT16_MAX 32767
#define MC_INT16_MIN (-32768)
#define MC_DIST_UM_MAX (2000000000) /* int32 saturation for run distance */
#define MC_IDLE_SAT 0xFFFFU

/* ---- geometry ------------------------------------------------------------ */

uint32_t
motion_core_um_per_tick (void)
{
    /* circumference_um / ticks_per_rev with pi = 355/113 */
    return (APP_CAL_WHEEL_DIA_MM * MC_UM_PER_MM * MC_PI_NUM) /
           (MC_PI_DEN * APP_CAL_TICKS_PER_REV);
}

uint32_t
motion_core_turn_ticks (uint16_t angle_tenth_deg)
{
    /* wheel path for an in-place pivot: pi * track * angle / 360.
       Ordered to stay inside uint32: track(<=1000) * 355 * angle(<=32767)
       < 1.2e10 would overflow, so angle is limited to 3600 (one turn) per
       command; the controller never asks for more than 180 deg. */
    uint32_t const angle        = (angle_tenth_deg > MC_TENTH_DEG_TURN)
                                      ? MC_TENTH_DEG_TURN
                                      : angle_tenth_deg;
    uint32_t const path_mm_x100 = (APP_CAL_TRACK_WIDTH_MM * MC_PI_NUM * angle) /
                                  (MC_PI_DEN * (MC_TENTH_DEG_TURN / MC_PCT));
    uint32_t const path_um = path_mm_x100 * (MC_UM_PER_MM / MC_PCT);
    return path_um / motion_core_um_per_tick();
}

uint8_t
motion_core_speed_from_interval (uint16_t interval_32us)
{
    uint32_t const interval_us = (uint32_t) interval_32us * MC_US_PER_32US;
    uint32_t       cm_s        = 0U;

    if (0U == interval_us)
    {
        return 0U;
    }
    /* um per tick / us per tick = um/us = m/s ; x100 = cm/s */
    cm_s = (motion_core_um_per_tick() * MC_CM_PER_UM_X) / interval_us;
    return (uint8_t) app_pack_sat_u(cm_s, APP_MSA_SPEED_BITS, NULL);
}

/* ---- helpers ------------------------------------------------------------- */

static int32_t
clamp_i32 (int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

static int32_t
abs_i32 (int32_t value)
{
    return (value < 0) ? -value : value;
}

/** Speed loop for one wheel on magnitudes. Returns PWM % 0..PWM_MAX. */
static int32_t
speed_control (int32_t set_cm_s, int32_t meas_cm_s, int32_t * p_i_q8)
{
    int32_t const err    = set_cm_s - meas_cm_s;
    int32_t       out_q8 = 0;

    *p_i_q8 = clamp_i32(*p_i_q8 + ((int32_t) APP_CAL_SPEED_KI_Q8 * err),
                        -APP_CAL_SPEED_I_MAX_Q8, APP_CAL_SPEED_I_MAX_Q8);
    out_q8  = ((int32_t) APP_CAL_FF_Q8_PER_CM_S * set_cm_s) +
             ((int32_t) APP_CAL_SPEED_KP_Q8 * err) + *p_i_q8;
    return clamp_i32(out_q8 / MC_Q8, 0, (int32_t) APP_CAL_PWM_MAX_PCT);
}

/** Final per-wheel limiter: magnitude ceiling and stiction floor. */
static int8_t
limit_pwm (int32_t pwm_pct)
{
    int32_t const mag =
        clamp_i32(abs_i32(pwm_pct), 0, (int32_t) APP_CAL_PWM_MAX_PCT);
    int32_t out = mag;

    if ((mag > 0) && (mag < (int32_t) APP_CAL_PWM_MIN_PCT))
    {
        out = (int32_t) APP_CAL_PWM_MIN_PCT;
    }
    return (int8_t) ((pwm_pct < 0) ? -out : out); /* |out| <= 100 fits int8 */
}

static void
reset_command (motion_core_t * p_core)
{
    p_core->move_ticks_l      = 0;
    p_core->move_ticks_r      = 0;
    p_core->i_l_q8            = 0;
    p_core->i_r_q8            = 0;
    p_core->cmd_cycles        = 0U;
    p_core->stall_cycles      = 0U;
    p_core->line_stale_cycles = 0U;
}

static void
finish (motion_core_t * p_core, motion_out_t * p_out, uint8_t result)
{
    p_core->done_seq    = p_core->active_seq;
    p_core->done_result = result;
    p_core->state       = (uint8_t) APP_MOTION_BRAKED;
    p_core->cmd_type    = (uint8_t) APP_MC_BRAKE;
    p_out->done_event   = true;
    reset_command(p_core);
}

/** Enter STOP_LATCHED now: brake, flush, report ABORTED for an active command.
 */
static void
latch_now (motion_core_t * p_core, motion_out_t * p_out)
{
    if ((uint8_t) APP_MOTION_STOP_LATCHED != p_core->state)
    {
        if ((uint8_t) APP_MOTION_BRAKED != p_core->state)
        {
            p_core->done_seq    = p_core->active_seq;
            p_core->done_result = (uint8_t) APP_MOTION_RESULT_ABORTED;
            p_out->done_event   = true;
        }
        p_core->state    = (uint8_t) APP_MOTION_STOP_LATCHED;
        p_core->cmd_type = (uint8_t) APP_MC_BRAKE;
        reset_command(p_core);
    }
    p_out->flush_queue = true;
}

/* ---- odometry ------------------------------------------------------------ */

static void
odometry (motion_core_t * p_core, motion_in_t const * p_in, int32_t * p_dl,
          int32_t * p_dr)
{
    int32_t dl = 0;
    int32_t dr = 0;

    if (p_core->have_ticks)
    {
        /* wrap-safe: uint16 subtraction then explicit sign decode */
        dl = app_pack_u_to_s(
            app_motion_enc_delta(p_in->ticks_l, p_core->last_ticks_l),
            APP_ENC_TICKS_BITS);
        dr = app_pack_u_to_s(
            app_motion_enc_delta(p_in->ticks_r, p_core->last_ticks_r),
            APP_ENC_TICKS_BITS);
    }
    p_core->have_ticks   = true;
    p_core->last_ticks_l = p_in->ticks_l;
    p_core->last_ticks_r = p_in->ticks_r;

    p_core->move_ticks_l += dl;
    p_core->move_ticks_r += dr;
    {
        /* average of both wheels, saturating int32 micrometres */
        int32_t const step_um =
            ((dl + dr) * (int32_t) motion_core_um_per_tick()) / 2;
        p_core->run_dist_um = clamp_i32(p_core->run_dist_um + step_um,
                                        -MC_DIST_UM_MAX, MC_DIST_UM_MAX);
    }

    if (0 == dl)
    {
        if (p_core->idle_cycles_l < MC_IDLE_SAT)
        {
            p_core->idle_cycles_l++;
        }
    }
    else
    {
        p_core->idle_cycles_l = 0U;
        p_core->dir_l_reverse = (dl < 0);
    }
    if (0 == dr)
    {
        if (p_core->idle_cycles_r < MC_IDLE_SAT)
        {
            p_core->idle_cycles_r++;
        }
    }
    else
    {
        p_core->idle_cycles_r = 0U;
        p_core->dir_r_reverse = (dr < 0);
    }
    p_core->speed_l_cm_s =
        (p_core->idle_cycles_l >= MC_MS_TO_CYCLES(APP_CAL_ENC_STOP_MS))
            ? 0U
            : motion_core_speed_from_interval(p_in->interval_l_32us);
    p_core->speed_r_cm_s =
        (p_core->idle_cycles_r >= MC_MS_TO_CYCLES(APP_CAL_ENC_STOP_MS))
            ? 0U
            : motion_core_speed_from_interval(p_in->interval_r_32us);
    *p_dl = dl;
    *p_dr = dr;
}

/* ---- public ----------------------------------------------------------------
 */

void
motion_core_init (motion_core_t * p_core)
{
    uint8_t * p_bytes =
        (uint8_t *) p_core; /* plain-old-data struct, zero is a valid state */
    uint32_t i = 0U;
    for (i = 0U; i < sizeof(motion_core_t); i++)
    {
        p_bytes[i] = 0U;
    }
    p_core->state    = (uint8_t) APP_MOTION_BRAKED;
    p_core->cmd_type = (uint8_t) APP_MC_BRAKE;
}

/* ---- step helpers ----------------------------------------------------------
 */

static void
clear_outputs (motion_out_t * p_out)
{
    p_out->pwm_l_pct    = 0;
    p_out->pwm_r_pct    = 0;
    p_out->flush_queue  = false;
    p_out->done_event   = false;
    p_out->reject_event = false;
    p_out->reject_seq   = 0U;
    p_out->stop_request = APP_STOP_CAUSE_NONE;
    p_out->fault_set    = 0U;
}

static bool
is_driving (motion_core_t const * p_core)
{
    return ((uint8_t) APP_MOTION_FOLLOWING == p_core->state) ||
           ((uint8_t) APP_MOTION_MOVING == p_core->state) ||
           ((uint8_t) APP_MOTION_TURNING == p_core->state);
}

/** Step 1: latch handling. */
static void
latch_step (motion_core_t * p_core, motion_in_t const * p_in,
            motion_out_t * p_out)
{
    if (p_in->stop_latched)
    {
        latch_now(p_core, p_out);
        if (p_in->has_cmd)
        {
            app_motion_cmd_t cmd;
            app_motion_cmd_unpack(p_in->cmd_record, &cmd);
            p_out->reject_event = true;
            p_out->reject_seq   = cmd.seq;
        }
    }
    else if ((uint8_t) APP_MOTION_STOP_LATCHED == p_core->state)
    {
        /* Guarded recovery: the controller cleared the latch -> BRAKED. */
        p_core->state = (uint8_t) APP_MOTION_BRAKED;
        reset_command(p_core);
    }
    else
    {
        /* normal cycle */
    }
}

/** Step 2: freshness counters (always) and hazard guards (while driving). */
static void
guard_step (motion_core_t * p_core, motion_in_t const * p_in,
            app_line_state_t const * p_line, int32_t dl, int32_t dr,
            motion_out_t * p_out)
{
    if (p_in->ctrl_heartbeat == p_core->last_heartbeat)
    {
        if (p_core->hb_stale_cycles < MC_IDLE_SAT)
        {
            p_core->hb_stale_cycles++;
        }
    }
    else
    {
        p_core->hb_stale_cycles = 0U;
        p_core->last_heartbeat  = p_in->ctrl_heartbeat;
    }
    if (p_line->seq == p_core->last_line_seq)
    {
        if (p_core->line_stale_cycles < MC_IDLE_SAT)
        {
            p_core->line_stale_cycles++;
        }
    }
    else
    {
        p_core->line_stale_cycles = 0U;
        p_core->last_line_seq     = p_line->seq;
    }
    p_core->glitches_seen = p_in->glitches;

    if (!is_driving(p_core))
    {
        p_core->stall_cycles = 0U;
        return;
    }
    {
        bool const line_needed =
            ((uint8_t) APP_MOTION_FOLLOWING == p_core->state) ||
            (((uint8_t) APP_MOTION_TURNING == p_core->state) &&
             p_core->until_line);
        if (p_core->hb_stale_cycles >=
            MC_MS_TO_CYCLES(APP_CAL_T_CTRL_HEARTBEAT_MS))
        {
            p_out->fault_set |= APP_FAULT_CTRL_HEARTBEAT;
            p_out->stop_request = APP_STOP_CAUSE_CTRL_HEARTBEAT;
        }
        else if (line_needed && (p_core->line_stale_cycles >=
                                 MC_MS_TO_CYCLES(APP_CAL_T_STALE_LINE_MS)))
        {
            p_out->fault_set |= APP_FAULT_STALE_LINE;
            p_out->stop_request = APP_STOP_CAUSE_STALE_LINE;
        }
        else if (p_in->glitches >= APP_CAL_ENC_GLITCH_MAX)
        {
            p_out->fault_set |= APP_FAULT_ENCODER_FAULT;
            p_out->stop_request = APP_STOP_CAUSE_STALL;
        }
        else
        {
            /* stall: demand above the watch level and no edge on either wheel
             */
            if ((p_core->last_pwm_mag >= (uint8_t) APP_CAL_PWM_STALL_PCT) &&
                (0 == dl) && (0 == dr))
            {
                p_core->stall_cycles++;
            }
            else
            {
                p_core->stall_cycles = 0U;
            }
            if (p_core->stall_cycles >= MC_MS_TO_CYCLES(APP_CAL_ENC_STALL_MS))
            {
                p_out->fault_set |= APP_FAULT_MOTOR_STALL;
                p_out->stop_request = APP_STOP_CAUSE_STALL;
            }
        }
    }
    if (APP_STOP_CAUSE_NONE != p_out->stop_request)
    {
        /* brake in this very cycle; the task calls SafeStop with the cause */
        latch_now(p_core, p_out);
    }
}

/** Step 3: at most one command per cycle. */
static void
command_step (motion_core_t * p_core, motion_in_t const * p_in,
              app_line_state_t const * p_line, motion_out_t * p_out)
{
    app_motion_cmd_t cmd;

    if (!p_in->has_cmd || ((uint8_t) APP_MOTION_STOP_LATCHED == p_core->state))
    {
        return;
    }
    app_motion_cmd_unpack(p_in->cmd_record, &cmd);
    if ((uint8_t) APP_MC_BRAKE == cmd.type)
    {
        /* BRAKE pre-empts every mode and completes immediately */
        p_core->active_seq = cmd.seq;
        finish(p_core, p_out, (uint8_t) APP_MOTION_RESULT_OK);
    }
    else if (((uint8_t) APP_MOTION_BRAKED != p_core->state) ||
             !p_in->motion_armed || (0U == cmd.speed_cm_s))
    {
        p_out->reject_event = true;
        p_out->reject_seq   = cmd.seq;
    }
    else
    {
        reset_command(p_core);
        p_core->active_seq     = cmd.seq;
        p_core->cmd_type       = cmd.type;
        p_core->speed_set_cm_s = cmd.speed_cm_s;
        p_core->param          = cmd.param;
        p_core->until_line     = cmd.until_line;
        p_core->last_line_seq =
            p_line->seq; /* fresh start for the stale check */
        if ((uint8_t) APP_MC_FOLLOW == cmd.type)
        {
            p_core->state = (uint8_t) APP_MOTION_FOLLOWING;
        }
        else if ((uint8_t) APP_MC_MOVE == cmd.type)
        {
            p_core->state = (uint8_t) APP_MOTION_MOVING;
        }
        else
        {
            p_core->state = (uint8_t) APP_MOTION_TURNING;
        }
    }
}

/** TURNING control; returns false when the turn finished this cycle. */
static bool
turn_control (motion_core_t * p_core, app_line_state_t const * p_line,
              int32_t progress, int32_t base_l, int32_t base_r,
              int32_t * p_pwm_l, int32_t * p_pwm_r, motion_out_t * p_out)
{
    uint16_t const angle  = (uint16_t) abs_i32(p_core->param);
    int32_t const  target = (int32_t) motion_core_turn_ticks(angle);
    int32_t const  sign   = (p_core->param < 0) ? -1 : 1; /* + = left (CCW) */

    if (p_core->until_line)
    {
        if ((progress * (int32_t) MC_PCT >=
             target * (int32_t) APP_CAL_TURN_UNTIL_LINE_MIN_PCT) &&
            p_line->on_line)
        {
            finish(p_core, p_out, (uint8_t) APP_MOTION_RESULT_OK);
            return false;
        }
        if (progress * (int32_t) MC_PCT >=
            target * (int32_t) APP_CAL_TURN_UNTIL_LINE_MAX_PCT)
        {
            finish(p_core, p_out, (uint8_t) APP_MOTION_RESULT_TIMEOUT);
            return false;
        }
    }
    else if (progress >= target)
    {
        finish(p_core, p_out, (uint8_t) APP_MOTION_RESULT_OK);
        return false;
    }
    else
    {
        /* keep turning */
    }
    *p_pwm_l = -sign * base_l; /* left turn: left wheel backward */
    *p_pwm_r = sign * base_r;
    return true;
}

/** Step 4: control for the active mode. */
static void
control_step (motion_core_t * p_core, app_line_state_t const * p_line,
              int32_t * p_pwm_l, int32_t * p_pwm_r, motion_out_t * p_out)
{
    int32_t const set = (int32_t) p_core->speed_set_cm_s;
    int32_t const base_l =
        speed_control(set, (int32_t) p_core->speed_l_cm_s, &p_core->i_l_q8);
    int32_t const base_r =
        speed_control(set, (int32_t) p_core->speed_r_cm_s, &p_core->i_r_q8);
    int32_t const progress =
        (abs_i32(p_core->move_ticks_l) + abs_i32(p_core->move_ticks_r)) / 2;

    p_core->cmd_cycles++;
    if (p_core->cmd_cycles >= MC_MS_TO_CYCLES(APP_CAL_T_MOTION_TIMEOUT_MS))
    {
        finish(p_core, p_out, (uint8_t) APP_MOTION_RESULT_TIMEOUT);
    }
    else if ((uint8_t) APP_MOTION_FOLLOWING == p_core->state)
    {
        int32_t const steer =
            ((int32_t) p_line->steer * (int32_t) APP_CAL_STEER_GAIN_Q8) / MC_Q8;
        *p_pwm_l = base_l +
                   steer; /* + steer = line to the right -> left wheel faster */
        *p_pwm_r = base_r - steer;
    }
    else if ((uint8_t) APP_MOTION_MOVING == p_core->state)
    {
        int32_t const target =
            ((int32_t) abs_i32(p_core->param) * (int32_t) MC_UM_PER_MM) /
            (int32_t) motion_core_um_per_tick();
        if (progress >= target)
        {
            finish(p_core, p_out, (uint8_t) APP_MOTION_RESULT_OK);
        }
        else
        {
            int32_t const diff =
                p_core->move_ticks_l - p_core->move_ticks_r; /* signed */
            int32_t const corr =
                (diff * (int32_t) APP_CAL_STRAIGHT_KP_Q8) / MC_Q8;
            int32_t const sign = (p_core->param < 0) ? -1 : 1;
            /* in reverse both diff and the wheels are negative: same correction
             */
            *p_pwm_l = sign * (base_l - (sign * corr));
            *p_pwm_r = sign * (base_r + (sign * corr));
        }
    }
    else
    {
        (void) turn_control(p_core, p_line, progress, base_l, base_r, p_pwm_l,
                            p_pwm_r, p_out);
    }
}

/** Step 5: limiter, stall-watch memory and the status view. */
static void
status_step (motion_core_t * p_core, int32_t pwm_l, int32_t pwm_r,
             motion_out_t * p_out)
{
    if (!is_driving(p_core))
    {
        pwm_l = 0;
        pwm_r = 0;
    }
    p_out->pwm_l_pct = limit_pwm(pwm_l);
    p_out->pwm_r_pct = limit_pwm(pwm_r);
    {
        int32_t const ml     = abs_i32((int32_t) p_out->pwm_l_pct);
        int32_t const mr     = abs_i32((int32_t) p_out->pwm_r_pct);
        p_core->last_pwm_mag = (uint8_t) ((ml > mr) ? ml : mr); /* <= 100 */
    }
    p_core->cycle_tag =
        (uint8_t) ((p_core->cycle_tag + 1U) & app_pack_mask(APP_MSA_TAG_BITS));
    {
        int32_t const dist_mm = p_core->run_dist_um / (int32_t) MC_UM_PER_MM;
        p_out->status.speed_l_cm_s = p_core->speed_l_cm_s;
        p_out->status.speed_r_cm_s = p_core->speed_r_cm_s;
        p_out->status.distance_mm =
            (int16_t) clamp_i32(dist_mm, MC_INT16_MIN, MC_INT16_MAX);
        p_out->status.cycle_tag     = p_core->cycle_tag;
        p_out->status.motion_state  = p_core->state;
        p_out->status.done_seq      = p_core->done_seq;
        p_out->status.active_seq    = p_core->active_seq;
        p_out->status.dir_l_reverse = p_core->dir_l_reverse;
        p_out->status.dir_r_reverse = p_core->dir_r_reverse;
        p_out->status.done_result   = p_core->done_result;
    }
}

void
motion_core_step (motion_core_t * p_core, motion_in_t const * p_in,
                  motion_out_t * p_out)
{
    int32_t          dl    = 0;
    int32_t          dr    = 0;
    int32_t          pwm_l = 0;
    int32_t          pwm_r = 0;
    app_line_state_t line;

    clear_outputs(p_out);
    odometry(p_core, p_in, &dl, &dr);
    app_line_unpack(p_in->line_word, &line);

    latch_step(p_core, p_in, p_out);                /* 1. STOP_LATCH first */
    guard_step(p_core, p_in, &line, dl, dr, p_out); /* 2. hazards */
    command_step(p_core, p_in, &line, p_out);       /* 3. one command */
    if (is_driving(p_core))
    {
        control_step(p_core, &line, &pwm_l, &pwm_r, p_out); /* 4. control */
    }
    status_step(p_core, pwm_l, pwm_r, p_out); /* 5. outputs */
}
