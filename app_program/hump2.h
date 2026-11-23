/**
 * @file    hump2.h
 * @brief   Second hump detector: the base hump profiler (imu_logic.c,
 *          DR 6.3), running beside IS09 imu_core to measure the hump height
 *          (19 Nov). It only measures: IS09 imu_core still drives the hump
 *          signals (speed limit, EVF_HUMP_*), the collision test and the
 *          motion class.
 *
 * base logic, kept (every 20 ms, from two 10 ms samples):
 *   pitch  = atan2(a_fwd - a_fwd_encoders, sqrt(a_lat^2 + a_up^2)) - theta0,
 *            EMA filtered (alpha 0.3). theta0 (mounting offset) is the mean
 *            of 50 samples (1 s) while IDLE and standing still.
 *   states FLAT -> ASCENT (pitch >= start for 3 samples) -> CREST (pitch
 *            below level with h > min height) -> DESCENT (pitch <= -start for
 *            3 samples) -> end (|pitch| <= level for 5 samples).
 * Changed on 19 Nov after the first car run, where vibration spikes of
 * 60-80 ms gave a start/end pair each and a real climb was cut short:
 *   - ASCENT ends only after 3 samples below level (base: 1 sample);
 *   - one event per hump, at its end (HUMP2_END), and only for a whole
 *     hump: a climb with sum above the min height, both the climb and the
 *     drop at least min_climb_mm long. Everything else (vibration spikes,
 *     a climb with no drop within max_len_mm, the jolt when the car stops)
 *     is only counted ("rejected").
 * Second run (20:25): the real hump climbed only 2-4.4 deg and dropped to
 * -5 deg, so the defaults are now start 2.0, level 1.0, min sum 0.5 mm.
 * Third run (20:42): the start jolt (+6 to +9.5 deg in the first 15 mm)
 * was counted as a hump, so no climb may start in the first start_blank_ms
 * of driving (default 500 ms, about 40 mm).
 *   height h += sin(pitch) x ds (encoder distance), the peak is reported.
 *            A climb that never gets above the min height is a "false slope";
 *            an episode longer than 1500 mm is dropped.
 * Added for the 1 cm hump (see docs/IS09_PORT_NOTES.md, "Hump height"):
 *   tilt height = L x (sin(highest pitch) + sin(-lowest pitch)) / 2, with L
 *            the distance from the drive axle to the caster. For a hump
 *            shorter than about 2 x L this is the peak height; the base sum
 *            gives the mean height under the car (too low on a short hump).
 *            Averaging the climb and the drop cancels a constant pitch error.
 *
 * Pure logic, no RTOS calls: tested on the PC.
 */
#ifndef HUMP2_H
#define HUMP2_H

#include <stdbool.h>
#include <stdint.h>

#define HUMP2_TRACE_LEN  (32u)    /* 640 ms of 20 ms samples */
#define HUMP2_TRACE_MAX  (14u)    /* sent per frame at most (frames are ~110-220 ms) */

typedef enum
{
    HUMP2_FLAT = 0,
    HUMP2_ASCENT,
    HUMP2_CREST,
    HUMP2_DESCENT
} hump2_state_t;

typedef struct
{
    float    start_deg;       /* base: 5.0 */
    float    level_deg;       /* base: 3.0 */
    float    min_height_mm;   /* base: 15 */
    float    min_climb_mm;    /* the climb must also last this far (added 19 Nov: spikes are 2-11 mm) */
    uint32_t start_blank_ms;  /* no climb may start this soon after the car starts driving (start jolt) */
    float    max_len_mm;      /* base: 1500 */
    float    alpha;           /* pitch EMA, base: 0.3 at 20 ms */
    float    wheelbase_mm;    /* L: drive axle to caster contact */
    uint32_t start_samples;   /* base: 3 (60 ms) */
    uint32_t end_samples;     /* base: 5 (100 ms) */
    bool     b_accel_comp;    /* subtract the encoder acceleration */
} hump2_par_t;

/* inputs of one 10 ms cycle, sensor axes already mapped to the car */
typedef struct
{
    int16_t fwd_mg;
    int16_t lat_mg;
    int16_t up_mg;
    bool    b_sample_valid;
    float   a_fwd_g;          /* forward acceleration from the encoders */
    float   ds_mm;            /* distance moved since the last cycle */
    bool    b_learn_offset;   /* IDLE and standing still */
    bool    b_driving;        /* LINE_FOLLOWING / OBSTACLE_AVOIDANCE / MANEUVER */
} hump2_in_t;

/* events of one cycle. A climb that never reaches the min height (a vibration
 * spike) gives no event at all; it is only counted in n_rejected. */
typedef struct
{
    bool  b_start;            /* a climb confirmed (first crest): height so far */
    bool  b_peak;             /* CREST reached (also again after a double hump) */
    bool  b_end;              /* a whole hump: climb, crest and drop seen (always valid) */
    bool  b_valid;            /* = b_end (kept for the callers) */
    bool  b_too_long;         /* unused: a climb without a drop is only counted */
    float height_mm;          /* sum of sin(pitch) x ds (base) */
    float tilt_mm;            /* L x sin(pitch) */
    float len_mm;
} hump2_ev_t;

typedef struct
{
    /* base hump_ctx_t */
    hump2_state_t state;
    uint32_t n_up;
    uint32_t n_down;
    uint32_t n_level;
    uint32_t n_below;         /* samples below level in ASCENT (debounce, 19 Nov) */
    bool     b_confirmed;     /* the climb reached the min height and length */
    float    desc_from_mm;    /* len_mm when DESCENT began */
    uint32_t drive_ms;        /* time driving (20 ms steps, saturates) */
    float    h_mm;
    float    peak_mm;
    float    len_mm;
    /* pitch */
    float    pitch_deg;
    float    theta0_deg;
    float    off_sum;
    uint32_t off_n;
    bool     b_offset;        /* theta0 learnt at least once */
    /* 20 ms decimation */
    uint32_t sub;
    int32_t  fwd_sum;
    int32_t  lat_sum;
    int32_t  up_sum;
    uint32_t n_sum;
    float    ds_sum;
    /* tilt height */
    float    p_max;
    float    p_min;
    /* results */
    float    last_h_mm;
    float    last_tilt_mm;
    float    last_len_mm;
    float    max_h_mm;
    float    max_tilt_mm;
    uint32_t n_valid;
    uint32_t n_rejected;
    /* pitch extremes since the last hump2_diag() */
    float    p_hi;
    float    p_lo;
    bool     b_range_reset;
    /* every 20 ms pitch, 0.1 deg, for the dashboard trace */
    int16_t  trace[HUMP2_TRACE_LEN];
    uint32_t trace_n;         /* samples written in total */
} hump2_t;

void hump2_init(hump2_t *p_h);
void hump2_new_run(hump2_t *p_h);          /* clears the results and counters */
/* one 10 ms cycle; returns true on the cycles the base logic ran (every 2nd) */
bool hump2_step(hump2_t *p_h, const hump2_par_t *p_par,
                    const hump2_in_t *p_in, hump2_ev_t *p_ev);
/* live tilt height of the episode in progress (0 when FLAT) */
float hump2_live_tilt_mm(const hump2_t *p_h, const hump2_par_t *p_par);
const char *hump2_state_name(hump2_state_t st);

/* the instance run by TSK_TERRAIN_IMU, read by the telemetry task */
extern hump2_t     g_hump2;
extern hump2_par_t g_hump2_par;

typedef struct
{
    float    pitch_deg;
    float    p_hi;            /* since the previous call */
    float    p_lo;
    float    theta0_deg;
    uint8_t  state;
    float    h_mm;
    float    peak_mm;
    float    tilt_mm;         /* live */
    float    last_h_mm;
    float    last_tilt_mm;
    float    max_h_mm;
    float    max_tilt_mm;
    uint32_t n_valid;
    uint32_t n_rejected;
} hump2_diag_t;
void hump2_diag(hump2_diag_t *p_d);         /* one caller: telemetry */
/* the 20 ms pitch samples (0.1 deg) written since *p_rd, oldest first, at
 * most max (the newest are kept); *p_rd is moved on. One caller: telemetry. */
uint32_t hump2_trace(int16_t *p_out, uint32_t max, uint32_t *p_rd);

#endif /* HUMP2_H */
