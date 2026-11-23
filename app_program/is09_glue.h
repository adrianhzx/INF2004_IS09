/**
 * @file    is09_glue.h
 * @brief   Adapters between the base tasks and the IS09 modules.
 *
 * The IS09 modules (app_program/is09/, copied unchanged from the IS09 repo)
 * are pure algorithms with no kernel or hardware calls:
 *   - subsystem3/barcode_core  Code 39 decoder on edge times (Buddy 3);
 *   - subsystem3/line_core     analog line-sensor calibration and
 *                              normalisation (Buddy 3);
 *   - subsystem4/imu_core      pitch, hump episodes, collision, motion
 *                              class (Buddy 4);
 *   - subsystem5/scan_core     ultrasonic forward watch, scan plans and
 *                              obstacle profile (Buddy 5).
 * The is09_*.c adapters own their state and expose plain C types only, so
 * this header can be included by micro T-Kernel task code and by Pico SDK
 * driver code alike. The IS09 headers are never included by the base
 * tasks: several names (hump_state_t, BC_MAX_ELEMENTS) exist in both.
 */
#ifndef IS09_GLUE_H
#define IS09_GLUE_H

#include <stdbool.h>
#include <stdint.h>

/* ======================================================================== */
/* Line sensors (is09_line.c): IS09 line_core turns the two analog sensors  */
/* into darkness 0..100; a sensor is "black" at or above dark_pct and stays */
/* black until it falls below dark_pct - hys_pct.                           */
/* ======================================================================== */
void is09_line_init(void);
/* One 10 ms step with the two raw 12-bit samples (LEFT, CENTRE). */
void is09_line_update(uint32_t adc_left, uint32_t adc_centre, uint32_t dark_pct, uint32_t hys_pct);
/* 20 Nov: black level for the LEFT sensor only (the IS09 car's LEFT sensor
   reads about half as dark as CENTRE on the line). 0 = dark_pct, as before. */
void is09_line_set_left_dark(uint32_t dark_pct);
bool is09_line_black_left(void);
bool is09_line_black_right(void);           /* the IS09 CENTRE sensor */

typedef struct
{
    uint16_t raw_l;
    uint16_t raw_c;
    uint8_t  n_l;                           /* 0 white .. 100 black */
    uint8_t  n_c;
    uint16_t ref_white;                     /* running references (raw ADC) */
    uint16_t ref_black;
    bool     calibrated;                    /* span >= APP_CAL_IR_MIN_CONTRAST */
} is09_line_diag_t;
void is09_line_diag(is09_line_diag_t *p_d);

/* ======================================================================== */
/* Barcode (is09_barcode.c): IS09 BarcodeEdgeIsr + barcode_core, polled by  */
/* TSK_LINE_BARCODE every 10 ms instead of a task of its own.               */
/* ======================================================================== */
void is09_bc_init(void);
/* GPIO interrupt (hal_isr.c): one edge on the barcode pin. */
void is09_bc_isr_edge(bool b_level, uint32_t stamp_us);
/* ARM (start looking for a quiet zone) or DISARM (edges ignored). */
void is09_bc_arm(bool b_arm, uint32_t now_us);

typedef struct
{
    bool    b_done;      /* a capture of APP_CAL_BC_N_MIN+ widths ended */
    bool    b_letter;    /* ... and it decoded to a Code 39 character */
    char    letter;      /* '0'..'9', 'A'..'Z' or a symbol */
    bool    b_reversed;  /* read backwards */
    bool    b_overflow;  /* edge ring or element store overflowed */
    uint8_t count;       /* widths in the capture */
} is09_bc_result_t;

/* Drains the edge ring into barcode_core and ends a capture after the IS09
   quiet zone (APP_CAL_T_QUIET_MS without an edge). Returns true when
   *p_res holds a finished capture. Call every 10 ms. */
bool is09_bc_service(uint32_t now_us, is09_bc_result_t *p_res);

/* Diagnostics in the IS09 serial format (subsystem1.c, 10 Nov):
   "cap=N n=X.Yms w=X.Yms stray=S pat=<9 1 9 1 9>[ R]" and
   "widths #K (N, ms): bX.Y sX.Y ...". The capture number K increases by one
   per capture that reached the quiet zone (strays included). */
uint32_t is09_bc_capture_count(void);
void     is09_bc_capture_text(char *p_buf, uint32_t size);
void     is09_bc_widths_text(char *p_buf, uint32_t size);
/* "pin=P st=STATE edges=E results=R" */
void     is09_bc_health_text(char *p_buf, uint32_t size, bool b_pin);

/* ======================================================================== */
/* IMU (is09_imu.c): IS09 imu_core, one step every 10 ms.                    */
/* ======================================================================== */
typedef struct
{
    int16_t x_mg;              /* raw sensor axes, mg */
    int16_t y_mg;
    int16_t z_mg;
    bool    b_sample_valid;    /* false: the sensor is faulty this cycle */
    bool    b_motion_fresh;    /* odometry is being updated */
    bool    b_moving;          /* either wheel turning */
    bool    b_turning;         /* a commanded pivot turn */
    uint8_t speed_cm_s;        /* mean wheel speed */
    int16_t dist_delta_mm;     /* distance since the last step */
    bool    b_stop_latched;    /* E-stop latched */
    int16_t a_comp_mg;         /* Port: forward acceleration from the wheel
                                  encoders, removed from the forward axis so
                                  speeding up does not read as nose-up (0 = off) */
} is09_imu_in_t;

/* IS09 app_motion_class_t (app_data.h), same values */
typedef enum
{
    IS09_CLASS_STATIONARY   = 0,
    IS09_CLASS_ACCELERATING = 1,
    IS09_CLASS_TURNING      = 2,
    IS09_CLASS_CLIMBING     = 3,
    IS09_CLASS_DESCENDING   = 4,
    IS09_CLASS_IMPACT       = 5,
    IS09_CLASS_CRUISING     = 6
} is09_motion_class_t;

/* IS09 app_hump_quality_t (app_data.h), same values */
#define IS09_HUMP_Q_NORMAL    (0u)
#define IS09_HUMP_Q_REJECTED  (1u)
#define IS09_HUMP_Q_DEGRADED  (2u)
#define IS09_HUMP_Q_INVALID   (3u)

typedef struct
{
    float   pitch_deg;         /* nose up = + */
    bool    b_hump_active;
    uint8_t height_mm;         /* live peak while active, else the last one */
    uint8_t max_height_mm;     /* run maximum */
    uint8_t motion_class;      /* is09_motion_class_t */
    bool    b_hump_start;      /* events of this step */
    bool    b_hump_end;
    uint8_t end_quality;       /* IS09_HUMP_Q_* of the episode that ended */
    uint8_t end_height_mm;
    bool    b_collision;
    bool    b_class_changed;
} is09_imu_out_t;

void is09_imu_init(void);
void is09_imu_step(const is09_imu_in_t *p_in, is09_imu_out_t *p_out);

/* Run-time pitch thresholds (degrees): a hump starts above on_deg for
   APP_CAL_T_THETA_CONFIRM_MS and counts as level below off_deg. */
void is09_imu_set_thresholds(float on_deg, float off_deg);

/* What the hump detector itself sees (slow, zeroed pitch), with its highest
   and lowest values since the previous call (one caller: telemetry). */
typedef struct
{
    float   dp_deg;            /* detector pitch now */
    float   dp_hi_deg;         /* since the previous call */
    float   dp_lo_deg;
    float   on_deg;            /* thresholds in use */
    float   off_deg;
    uint8_t hump_state;        /* 0 FLAT, 1 CLIMBING, 2 DESCENDING */
    uint8_t last_quality;      /* IS09_HUMP_Q_* of the last episode */
    uint8_t last_height_mm;
    uint8_t max_height_mm;
    uint8_t episodes;          /* started, modulo 256 */
} is09_imu_diag_t;
void is09_imu_diag(is09_imu_diag_t *p_d);
/* The sensor axes as IS09 imu_core maps them (forward, lateral, up: a
   nose-up tilt reads + forward, level reads + up), for hump2.c. */
void is09_imu_map_axes(int16_t x_mg, int16_t y_mg, int16_t z_mg,
                       int16_t *p_fwd_mg, int16_t *p_lat_mg, int16_t *p_up_mg);
const char *is09_imu_class_name(uint8_t motion_class);
const char *is09_imu_quality_name(uint8_t quality);

/* ======================================================================== */
/* Ultrasonic (is09_scan.c): IS09 scan_core watch, scan plans and profile.   */
/* Angles here are base robot angles: 0 = ahead, + = RIGHT.                 */
/* ======================================================================== */
#define IS09_RANGE_NONE   (511u)    /* APP_RANGE_CM_SENTINEL: no echo / too far */
#define IS09_SCAN_MAX     (37u)     /* angle indices 0..36 (5 deg steps) */

/* IS09 app_avoid_action_t */
#define IS09_AVOID_STOP        (0u)
#define IS09_AVOID_CONTINUE    (1u)
#define IS09_AVOID_TURN_LEFT   (2u)
#define IS09_AVOID_TURN_RIGHT  (3u)
#define IS09_AVOID_REVERSE     (4u)

/* IS09 hardware gate APP_CAL_ECHO_SHIFTER_VERIFIED (HV-5): false = never
   pulse TRIG (the ECHO line has not been measured <= 3.3 V). */
bool     is09_us_enabled(void);
uint16_t is09_us_cm(float cm);          /* base ping (< 0 = none) -> IS09 cm / sentinel */
float    is09_scan_deg(uint32_t idx);   /* angle index -> robot deg (+ right) */

/* Forward watch: two consecutive readings at or inside APP_CAL_D_STOP_CM,
   once per approach (re-armed above D_STOP + hysteresis). */
void is09_watch_reset(void);
bool is09_watch_step(uint16_t range_cm, bool *p_no_echo_fault);
uint32_t is09_watch_stop_cm(void);

/* Profile scan: coarse plan (-60..+60 deg in 15 deg steps), then a fine
   plan (5 deg steps) around what the coarse sweep found. */
void     is09_scan_begin(void);
uint32_t is09_scan_coarse_plan(uint8_t *p_idx, uint32_t max);
uint32_t is09_scan_fine_plan(uint8_t *p_idx, uint32_t max);
void     is09_scan_set(uint32_t idx, uint16_t range_cm);
uint16_t is09_scan_get(uint32_t idx);   /* IS09_RANGE_NONE if not sampled */
bool     is09_scan_sampled(uint32_t idx);

typedef struct
{
    uint16_t range_cm;         /* straight ahead */
    uint16_t closest_cm;
    float    closest_deg;      /* robot deg (+ right) */
    uint8_t  width_cm;
    float    left_edge_cm;     /* lateral, + = RIGHT (base frame) */
    float    right_edge_cm;
    uint8_t  action;           /* IS09_AVOID_* */
} is09_profile_t;
void        is09_scan_profile(is09_profile_t *p_out);
const char *is09_action_name(uint8_t action);

#endif /* IS09_GLUE_H */
