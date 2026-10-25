/** @file scan_core.h
 *  @brief Pure obstacle-scan algorithm (Rev B 6.3): servo angle index <->
 *         pulse and settle time, forward-watch confirmation, coarse scan plan
 *         and fine window selection, polar -> lateral geometry with beam
 *         correction, ObstacleProfile construction (y_min, y_max, width,
 *         closest, recommended action), side-range confirmation.
 *         Owner: Buddy 5 (Ho Zhi Xian Adrian). No kernel or hardware calls.
 *
 *  Angle index 0..36 is 5 deg steps from -90 deg (RIGHT) through 18 (ahead)
 *  to +90 deg (LEFT). Lateral y is positive to the LEFT, as the Rev B side
 *  validity rules read (LEFT valid when y_path >= y_max + W_car/2 + M_clear).
 *  Ranges are in cm; APP_RANGE_CM_SENTINEL means "no echo / beyond range".
 */

#ifndef SCAN_CORE_H
#define SCAN_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "../shared/app_obstacle.h"

#define SC_ANGLE_COUNT (APP_SCAN_ANGLE_IDX_MAX + 1U) /* 37 */
#define SC_ANGLE_AHEAD 18U
#define SC_ANGLE_LEFT 36U
#define SC_ANGLE_RIGHT 0U
#define SC_DEG_PER_IDX 5U

/** scan_state field values (3 bits). */
typedef enum
{
    SC_STATE_IDLE     = 0,
    SC_STATE_WATCH    = 1,
    SC_STATE_COARSE   = 2,
    SC_STATE_FINE     = 3,
    SC_STATE_SIDE     = 4,
    SC_STATE_FAIL     = 5,
    SC_STATE_DISABLED = 6 /* HV-5 / HV-7 switches off */
} sc_state_t;

/** One scan's samples: cm per angle index, sentinel = none/too far. */
typedef struct
{
    uint16_t range_cm[SC_ANGLE_COUNT];
    uint64_t valid_mask; /* bit i: angle i was ranged in this scan */
} sc_samples_t;

/** Forward-watch confirmation state (two consecutive inside D_STOP). */
typedef struct
{
    uint8_t inside_count;
    bool    reported; /* EVT_OBSTACLE raised; re-armed when range clears */
    uint8_t no_echo_count;
} sc_watch_t;

/* ---- servo -------------------------------------------------------------- */
uint32_t scan_core_servo_us (uint32_t angle_idx);
/** Settle time for a move, scaled from APP_CAL_T_SERVO_SETTLE_MS per 30 deg. */
uint32_t scan_core_settle_ms (uint32_t from_idx, uint32_t to_idx);

/* ---- geometry ----------------------------------------------------------- */
/** sin / cos of the beam angle for an index, Q15 (sin positive to the LEFT). */
int32_t scan_core_sin_q15 (uint32_t angle_idx);
int32_t scan_core_cos_q15 (uint32_t angle_idx);
/** Lateral coordinate of a sample, cm, clamped to the int7 field. */
int8_t scan_core_lateral_cm (uint16_t range_cm, uint32_t angle_idx);

/* ---- forward watch ---------------------------------------------------- */
void scan_core_watch_init (sc_watch_t * p_w);
/** One 100 ms ranging. Returns true when EVT_OBSTACLE must be raised
 *  (two consecutive readings at or inside D_STOP, once per approach).
 *  *p_no_echo_fault reports whether the NO_ECHO fault should be set. */
bool scan_core_watch_step (sc_watch_t * p_w, uint16_t range_cm,
                           bool * p_no_echo_fault);

/* ---- scans -------------------------------------------------------------- */
void scan_core_samples_init (sc_samples_t * p_s);
void scan_core_samples_set (sc_samples_t * p_s, uint32_t angle_idx,
                            uint16_t range_cm);
/** Coarse plan: indices 6..30 step 3 (-60..+60 deg, 15 deg). Returns count. */
uint32_t scan_core_coarse_plan (uint8_t * p_idx, uint32_t max);
/** Fine window from the coarse samples: 5 deg steps across the span of
 *  samples inside APP_CAL_R_PROFILE_CM, widened by one coarse step each
 *  side and clamped. Returns count (0 when nothing is inside). */
uint32_t scan_core_fine_plan (sc_samples_t const * p_coarse, uint8_t * p_idx,
                              uint32_t max);
/** Build the profile from the (coarse + fine) samples. */
void scan_core_build_profile (sc_samples_t const * p_s, uint16_t forward_cm,
                              app_obstacle_profile_t * p_out);

/* ---- side ranging ----------------------------------------------------- */
/** Two consecutive side readings inside APP_CAL_D_SIDE_UNSAFE_CM -> true. */
bool scan_core_side_step (uint8_t * p_count, uint16_t range_cm);

#endif /* SCAN_CORE_H */
