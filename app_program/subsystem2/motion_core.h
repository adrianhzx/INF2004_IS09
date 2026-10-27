/** @file motion_core.h
 *  @brief Pure motion executor (Rev B 6.2): BRAKED / FOLLOWING / MOVING /
 *         TURNING / STOP_LATCHED, odometry, speed and straightness control.
 *         Owner: Buddy 2 (S Reuben Kanna). No kernel or hardware calls, so
 *         the same code runs in testing/host/test_motion.c.
 *
 *  One call to motion_core_step() per 10 ms cycle. Order inside a step
 *  (Rev B "Safety ordering"): STOP_LATCH first, then heartbeat / stale-line /
 *  encoder-stall guards, then at most one dequeued command, then control.
 *  The caller (task_motion.c) owns the pins: it applies the returned PWM
 *  demands and publishes the MotionStatus words.
 *
 *  Arithmetic: 32-bit integer, Q8 fixed point for gains. No floating point.
 */

#ifndef MOTION_CORE_H
#define MOTION_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "../shared/app_data.h"
#include "../shared/app_motion.h"

/** Per-cycle inputs read by the task from ISR words and blackboards. */
typedef struct
{
    bool     stop_latched; /* g_stop_latch == SET */
    bool     motion_armed; /* APP_CAL_MOTION_ARMED */
    uint16_t ticks_l;      /* EncoderIsr counters (wrap) */
    uint16_t ticks_r;
    uint16_t interval_l_32us; /* last edge interval, 32 us units, 0 = none */
    uint16_t interval_r_32us;
    uint8_t  glitches;       /* invalid quadrature transitions (saturating) */
    bool     has_cmd;        /* a command record was dequeued this cycle */
    uint32_t cmd_record;     /* packed MotionCmd */
    uint32_t line_word;      /* g_line_state */
    uint8_t  ctrl_heartbeat; /* g_ctrl_heartbeat */
} motion_in_t;

/** Per-cycle outputs. */
typedef struct
{
    int8_t  pwm_l_pct; /* -100..100, sign = direction, 0 = brake */
    int8_t  pwm_r_pct;
    bool    flush_queue;  /* task must drain MBF_MOTION_CMD now */
    bool    done_event;   /* raise EVT_MOTION_DONE (status.done_seq/result) */
    bool    reject_event; /* raise EVT_MOTION_REJECT (reject_seq) */
    uint8_t reject_seq;
    app_stop_cause_t    stop_request; /* NONE, or cause to pass to SafeStop */
    uint16_t            fault_set;    /* fault bits to set this cycle */
    app_motion_status_t status;       /* to be packed and published */
} motion_out_t;

typedef struct
{
    /* mode */
    uint8_t state; /* app_motion_state_t */
    uint8_t active_seq;
    uint8_t cmd_type;
    uint8_t speed_set_cm_s;
    int16_t param;
    bool    until_line;
    uint8_t done_seq;
    uint8_t done_result;
    uint8_t cycle_tag;
    /* odometry */
    bool     have_ticks;
    uint16_t last_ticks_l;
    uint16_t last_ticks_r;
    int32_t  run_dist_um;  /* signed run distance, micrometres (int32) */
    int32_t  move_ticks_l; /* ticks since the current command started */
    int32_t  move_ticks_r;
    uint16_t idle_cycles_l; /* cycles without an edge */
    uint16_t idle_cycles_r;
    uint8_t  speed_l_cm_s;
    uint8_t  speed_r_cm_s;
    bool     dir_l_reverse;
    bool     dir_r_reverse;
    /* control */
    int32_t  i_l_q8; /* integral terms */
    int32_t  i_r_q8;
    uint32_t cmd_cycles;     /* cycles in the current command */
    uint32_t stall_cycles_l; /* per wheel (D-33): one dead encoder is a stall */
    uint32_t stall_cycles_r;
    uint8_t  last_pwm_mag_l; /* |PWM| applied last cycle (stall watch) */
    uint8_t  last_pwm_mag_r;
    uint8_t  last_line_seq;
    uint16_t line_stale_cycles;
    uint8_t  last_heartbeat;
    uint16_t hb_stale_cycles;
    uint8_t  glitches_seen;
} motion_core_t;

void motion_core_init (motion_core_t * p_core);

/** One 10 ms cycle. */
void motion_core_step (motion_core_t * p_core, motion_in_t const * p_in,
                       motion_out_t * p_out);

/* ---- geometry helpers (exposed for tests and calibration printouts) ---- */
/** Distance per encoder tick in micrometres (wheel circumference / ticks). */
uint32_t motion_core_um_per_tick (void);
/** Encoder ticks per wheel for an in-place pivot of |angle| 0.1 deg units. */
uint32_t motion_core_turn_ticks (uint16_t angle_tenth_deg);
/** Speed in cm/s from an inter-edge interval in 32 us units (0 -> 0). */
uint8_t motion_core_speed_from_interval (uint16_t interval_32us);

#endif /* MOTION_CORE_H */
