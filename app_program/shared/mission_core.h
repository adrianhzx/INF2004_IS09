/** @file mission_core.h
 *  @brief Pure mission statechart (Rev B section 5 and 6.3): S0 BOOT,
 *         S1 IDLE, S2 LINE FOLLOWING, S3 EXECUTE NAV, S4 OBSTACLE BYPASS,
 *         S5 LINE SEARCH, S6 RUN COMPLETE, S7 SAFE STOP. One call to
 *         mission_core_step() per controller wake-up (20 ms tick or event).
 *         No kernel or hardware calls, so the same code runs in
 *         testing/host/test_mission.c against a simulated car.
 *
 *  Ownership: the core is the single writer of MissionStatus (system_state,
 *  pending_cmd, retry_count) and the only source of motion commands and scan
 *  requests. It never touches STOP_LATCH itself: it asks the task to call
 *  app_safe_stop_request(cause) or app_safety_controller_clear() through the
 *  output record, so the ISR-safe helpers stay the only writers.
 *
 *  Motion sequencing: MotionControlTask rejects a drive command unless it is
 *  BRAKED (Rev B 6.2), so the core queues one wanted command and emits a
 *  BRAKE first whenever the executor is still driving. At most one motion
 *  record is sent per step.
 *
 *  Decisions recorded for this phase: D-28 (no end-of-track on a loop
 *  course: S6 has no producer), D-29 (junction without a pending command is
 *  crossed straight), D-30 (obstacle during S3/S5 brakes and latches
 *  NO_CLEARANCE), D-31 (FOLLOW has no lifetime timeout; MOVE honours
 *  until_line).
 */
#ifndef MISSION_CORE_H
#define MISSION_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_barcode.h"
#include "app_data.h"
#include "app_line.h"
#include "app_motion.h"
#include "app_obstacle.h"
#include "app_telemetry.h"

/** Inputs gathered by the task before each step. */
typedef struct
{
    uint32_t events; /* APP_EVT_* bits delivered by this wake-up */
    bool     boot_done;
    bool     motion_armed; /* APP_CAL_MOTION_ARMED */
    bool     stop_latched;
    uint8_t  stop_cause; /* app_stop_cause_t */
    uint16_t faults;
    bool     start_pressed; /* raw button level (debounced here) */
    bool     stop_pressed;
    bool     remote_start; /* validated remote frames seen this step */
    bool     remote_stop;
    bool     motion_valid; /* app_motion_read() returned coherent */
    app_motion_status_t    motion;
    app_line_state_t       line;
    uint8_t                barcode_byte;
    bool                   profile_valid; /* s5_profile_copy() succeeded */
    app_obstacle_profile_t profile;
} mission_in_t;

#define MISSION_TEV_MAX 4U

typedef struct
{
    uint8_t  type; /* app_telem_event_t */
    uint16_t payload;
} mission_tev_t;

/** Outputs the task must act on, in this order: stop_request, motion,
 *  scan_req, clear_latch, mission publish, telemetry events. */
typedef struct
{
    bool     send_motion;
    uint8_t  mc_type; /* app_motion_cmd_type_t */
    uint8_t  mc_speed_cm_s;
    int16_t  mc_param;
    bool     mc_until_line;
    uint8_t  mc_seq;
    uint32_t scan_req;     /* APP_SCAN_REQ_* bits, 0 = none */
    uint8_t  stop_request; /* app_stop_cause_t, NONE = none */
    bool     clear_latch;  /* S7 guard met: call app_safety_controller_clear */
    bool     mission_changed; /* publish mission_core_status() */
    uint8_t  n_tev;
    mission_tev_t tev[MISSION_TEV_MAX];
} mission_out_t;

/** S4 sub-steps (published in telemetry as APP_TEV_BYPASS payload). */
typedef enum
{
    MC_BYP_BRAKE      = 0,  /* BRAKE sent, waiting for speed 0 */
    MC_BYP_PROFILE    = 1,  /* REQ_PROFILE sent, waiting for SCAN_DONE/FAIL */
    MC_BYP_PLAN       = 2,  /* profile read, action chosen */
    MC_BYP_REVERSE    = 3,  /* MOVE(-D_REVERSE) for a retry */
    MC_BYP_A1_TURN    = 4,  /* action 1: TURN 90 toward the bypass side */
    MC_BYP_A2_OFFSET  = 5,  /* action 2: MOVE lateral offset */
    MC_BYP_A3_TURN    = 6,  /* action 3: TURN 90 back to the heading */
    MC_BYP_A4_PASS    = 7,  /* action 4: MOVE past with side ranging */
    MC_BYP_A4_EXTRA   = 8,  /* action 4: clear the obstacle by a car length */
    MC_BYP_A5_TURN    = 9,  /* action 5: TURN 90 toward the line */
    MC_BYP_A5_MOVE    = 10, /* action 5: MOVE until_line */
    MC_BYP_A6_ALIGN   = 11, /* action 6: TURN until_line */
    MC_BYP_STEP_COUNT = 12
} mc_bypass_step_t;

/** S5 sub-steps. */
typedef enum
{
    MC_SRCH_TURN_A     = 0, /* toward the side the line was last seen */
    MC_SRCH_TURN_B     = 1, /* the other way, twice the angle */
    MC_SRCH_RECENTRE   = 2, /* back to the original heading */
    MC_SRCH_CREEP      = 3, /* MOVE forward until_line */
    MC_SRCH_STEP_COUNT = 4
} mc_search_step_t;

/** S3 sub-steps. */
typedef enum
{
    MC_NAV_ADVANCE    = 0, /* optional MOVE D_JUNC_ADVANCE before a pivot */
    MC_NAV_EXECUTE    = 1, /* the TURN / MOVE itself */
    MC_NAV_STEP_COUNT = 2
} mc_nav_step_t;

/** Controller-private state. */
typedef struct
{
    app_mission_status_t ms;  /* published MissionStatus view */
    uint8_t              sub; /* sub-step of S3 / S4 / S5 */
    uint32_t ticks_in_state;  /* 20 ms ticks since the last state change */
    uint32_t ticks_in_sub;
    /* buttons */
    uint32_t start_held_ticks;
    bool     start_armed_edge;
    uint32_t s7_hold_ticks;
    /* motion sequencing */
    uint8_t motion_seq;    /* last seq emitted (4 bit) */
    uint8_t await_seq;     /* seq of the command whose DONE we wait for */
    bool    awaiting;      /* await_seq is valid */
    uint8_t emit_done_seq; /* motion.done_seq when the command was sent */
    bool    want_pending;
    uint8_t want_type;
    uint8_t want_speed;
    int16_t want_param;
    bool    want_until_line;
    uint8_t rejects; /* consecutive rejections of the awaited command */
    /* mission data */
    bool     nav_implicit; /* S3 entered by a junction with no command */
    uint8_t  nav_cmd;      /* app_nav_cmd_t being executed */
    int16_t  dist_ref_mm;  /* run distance reference: S2 entry, pass start */
    bool     bypass_left;  /* chosen side */
    uint16_t offset_mm;    /* lateral path offset */
    uint8_t  side_free_count;
    int8_t   search_dir; /* +1 = line was last right of centre */
    int8_t   last_err_sign;
    bool     watch_on; /* forward watch currently requested on */
    /* statistics for tests and telemetry */
    uint32_t stats_bypasses;
    uint32_t stats_nav;
    uint32_t stats_searches;
} mission_core_t;

void mission_core_init (mission_core_t * p_core);

/** One controller step. Clears *p_out first. */
void mission_core_step (mission_core_t * p_core, mission_in_t const * p_in,
                        mission_out_t * p_out);

/** Current packed MissionStatus word (single writer: the core). */
uint32_t mission_core_status (mission_core_t const * p_core);

/** Lateral bypass offset (mm) for a side, from the profile (Rev B 6.3). */
uint16_t mission_core_offset_mm (app_obstacle_profile_t const * p_profile,
                                 bool                           left);

#endif /* MISSION_CORE_H */
