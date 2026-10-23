/** @file imu_core.h
 *  @brief Pure IMU algorithm (Rev B 6.4 "Hump Detector and Peak Estimator"):
 *         axis mapping and low-pass filter, pitch in 0.5 deg units,
 *         FLAT / CLIMBING / DESCENDING hump episodes with NORMAL / REJECTED /
 *         DEGRADED / INVALID terminal qualities and exactly one END per
 *         START, peak height by distance x sin(pitch) with K_CAL, run
 *         maximum, collision spike detection, six-class motion classifier.
 *         Owner: Buddy 4 (Wong Wei Qi). No kernel or hardware calls; one
 *         call per 10 ms cycle.
 *
 *  Sign conventions (HV-12 confirms them on the real mounting):
 *  forward axis positive towards the nose, up axis positive away from the
 *  floor; at rest and level the up axis reads about +1000 mg. Nose-up pitch
 *  is positive: pitch = atan2(a_fwd, a_up).
 */

#ifndef IMU_CORE_H
#define IMU_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "../shared/app_imu.h"

typedef enum
{
    HUMP_FLAT       = 0,
    HUMP_CLIMBING   = 1,
    HUMP_DESCENDING = 2
} hump_state_t;

/** One cycle of inputs, assembled by the task. */
typedef struct
{
    int16_t a_fwd_mg;      /* raw forward acceleration, mg (after axis map) */
    int16_t a_lat_mg;      /* raw lateral, mg */
    int16_t a_up_mg;       /* raw up, mg */
    bool    sample_valid;  /* false: sensor read failed this cycle */
    bool    motion_fresh;  /* MotionStatus changed within T_STALE_MOTION */
    bool    moving;        /* either wheel speed > 0 */
    bool    turning;       /* MotionStatus state TURNING */
    uint8_t speed_cm_s;    /* mean wheel speed */
    int16_t dist_delta_mm; /* MotionStatus distance change since last cycle */
    bool    stop_latched;  /* STOP_LATCH byte */
} imu_in_t;

typedef struct
{
    app_imu_state_t state;  /* to pack and publish */
    bool    evt_hump_start; /* raise EVT_HUMP_START (seq = state.episode_seq) */
    bool    evt_hump_end;   /* raise EVT_HUMP_END once per episode */
    uint8_t end_seq;        /* the episode that ended */
    uint8_t end_quality;    /* app_hump_quality_t of that episode */
    uint8_t end_height_mm;  /* its reported peak (0 for REJECTED/INVALID) */
    bool    collision;      /* call app_safe_stop_request(COLLISION) */
    bool    class_changed;  /* post APP_TEV_MOTION_CLASS */
    bool    height_sat;     /* a height field saturated at 127 mm */
} imu_out_t;

typedef struct
{
    int32_t  lp_fwd; /* low-pass accelerations, mg x 2^APP_CAL_IMU_LP_SHIFT */
    int32_t  lp_lat;
    int32_t  lp_up;
    bool     lp_seeded;
    int8_t   pitch_half_deg;
    uint8_t  hump_state;       /* hump_state_t, 2 bits used */
    uint8_t  theta_on_cycles;  /* consecutive pitch > theta_ON */
    uint8_t  theta_neg_cycles; /* consecutive pitch < -theta_ON */
    uint16_t flat_cycles;      /* consecutive |pitch| < theta_OFF */
    uint16_t episode_cycles;
    uint16_t episode_dist_mm;
    int32_t  height_q8; /* signed running height, mm x 256 */
    int32_t  peak_q8;   /* episode maximum */
    uint8_t  seq;       /* episode sequence 0..15 */
    uint8_t  last_quality;
    uint8_t  last_height_mm;
    uint8_t  max_height_mm;
    uint8_t  motion_class;
    uint8_t  class_candidate;
    uint8_t  class_cycles;
    uint16_t impact_cycles; /* IMPACT class hold-down */
    uint8_t  coll_cycles;   /* consecutive spike samples */
    uint16_t speed_lp_q4;   /* low-pass mean speed, cm/s x 16 */
    uint32_t stats_episodes;
    uint32_t stats_collisions;
} imu_core_t;

void imu_core_init (imu_core_t * p_core);

/** One 10 ms step. */
void imu_core_step (imu_core_t * p_core, imu_in_t const * p_in,
                    imu_out_t * p_out);

/** Integer atan2 in 0.5 deg units, -360..360 (exposed for tests). */
int16_t imu_core_atan2_half_deg (int32_t y, int32_t x);

/** Integer square root (exposed for tests). */
uint32_t imu_core_isqrt (uint32_t v);

/** Map the three raw sensor axes (mg) onto forward / lateral / up using the
 *  app_cal.h axis and sign constants (exposed for tests). */
void imu_core_map_axes (int16_t x_mg, int16_t y_mg, int16_t z_mg,
                        imu_in_t * p_in);

#endif /* IMU_CORE_H */
