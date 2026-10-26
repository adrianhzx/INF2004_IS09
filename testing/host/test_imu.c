/** @file test_imu.c
 *  @brief Host tests for subsystem 4 imu_core (H-IM-*): integer maths,
 *         axis mapping, pitch filter, hump episodes with every terminal
 *         quality, run maximum, saturation, collision spike, motion classes.
 *         The "car" is a kinematic script: pitch profile + 2 mm per 10 ms
 *         cycle (20 cm/s); gravity is resolved onto the sensor axes with libm.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_cal.h"
#include "host_port.h"
#include "imu_core.h"

/* NOLINTBEGIN(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, desc)                                                  \
    do                                                                         \
    {                                                                          \
        if (cond)                                                              \
        {                                                                      \
            g_pass++;                                                          \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            g_fail++;                                                          \
            printf("FAIL %s %s (%s:%d)\n", id, desc, __FILE__, __LINE__);      \
        }                                                                      \
    } while (0)

#define G_MG 1000.0
#define T_PI 3.14159265358979323846

typedef struct
{
    uint32_t starts;
    uint32_t ends;
    uint8_t  last_end_seq;
    uint8_t  last_end_quality;
    uint8_t  last_end_height;
    uint32_t collisions;
    uint32_t class_changes;
    bool     any_sat;
} tally_t;

static void
tally_reset (tally_t * p_t)
{
    memset(p_t, 0, sizeof(*p_t));
}

static void
tally_add (tally_t * p_t, imu_out_t const * p_o)
{
    p_t->starts += p_o->evt_hump_start ? 1U : 0U;
    if (p_o->evt_hump_end)
    {
        p_t->ends++;
        p_t->last_end_seq     = p_o->end_seq;
        p_t->last_end_quality = p_o->end_quality;
        p_t->last_end_height  = p_o->end_height_mm;
    }
    p_t->collisions += p_o->collision ? 1U : 0U;
    p_t->class_changes += p_o->class_changed ? 1U : 0U;
    p_t->any_sat = p_t->any_sat || p_o->height_sat;
}

/** Inputs for a car moving at @p speed_cm_s on a slope of @p pitch_deg. */
static void
make_in (imu_in_t * p_in, double pitch_deg, uint8_t speed_cm_s)
{
    double const rad = pitch_deg * T_PI / 180.0;
    memset(p_in, 0, sizeof(*p_in));
    p_in->a_fwd_mg     = (int16_t) lround(G_MG * sin(rad));
    p_in->a_up_mg      = (int16_t) lround(G_MG * cos(rad));
    p_in->a_lat_mg     = 0;
    p_in->sample_valid = true;
    p_in->motion_fresh = true;
    p_in->moving       = (speed_cm_s > 0U);
    p_in->turning      = false;
    p_in->speed_cm_s   = speed_cm_s;
    p_in->dist_delta_mm =
        (int16_t) (speed_cm_s / 10U); /* cm/s -> mm per 10 ms */
    p_in->stop_latched = false;
}

static void
run_cycles (imu_core_t * p_core, imu_in_t const * p_in, uint32_t n,
            tally_t * p_t, imu_out_t * p_last)
{
    uint32_t i = 0U;
    for (i = 0U; i < n; i++)
    {
        imu_core_step(p_core, p_in, p_last);
        tally_add(p_t, p_last);
    }
}

/* ---- maths --------------------------------------------------------------- */

static void
test_maths (void)
{
    int32_t  deg     = 0;
    int32_t  worst   = 0;
    uint32_t v       = 0U;
    bool     sqrt_ok = true;
    imu_in_t in;

    for (v = 0U; v < 5000U; v += 7U)
    {
        uint32_t const r = imu_core_isqrt(v * v + (v / 3U));
        sqrt_ok          = sqrt_ok && (r == v);
    }
    CHECK("H-IM-01",
          sqrt_ok && (65535U == imu_core_isqrt(0xFFFFFFFFU)) &&
              (0U == imu_core_isqrt(0U)),
          "integer sqrt exact (floor)");

    for (deg = -179; deg <= 180; deg++)
    {
        double const  rad = (double) deg * T_PI / 180.0;
        int32_t const x   = (int32_t) lround(1000.0 * cos(rad));
        int32_t const y   = (int32_t) lround(1000.0 * sin(rad));
        int32_t const got = imu_core_atan2_half_deg(y, x);
        int32_t       err = got - (2 * deg);
        err               = (err < 0) ? -err : err;
        if (err > 360)
        {
            err = 720 - err; /* wrap at +-180 */
        }
        if (err > worst)
        {
            worst = err;
        }
    }
    CHECK("H-IM-02", worst <= 1,
          "atan2 within 0.5 deg over the full circle (1000 mg vectors)");
    CHECK("H-IM-03", 0 == imu_core_atan2_half_deg(0, 0), "atan2(0,0) = 0");
    CHECK("H-IM-04",
          (imu_core_atan2_half_deg(174000, 985000) >= 19) &&
              (imu_core_atan2_half_deg(174000, 985000) <= 21),
          "large inputs are prescaled (10 deg -> 20 half-deg)");
    CHECK("H-IM-05", 180 == imu_core_atan2_half_deg(500, 0),
          "atan2(+y, 0) = 90 deg");
    CHECK("H-IM-06", -180 == imu_core_atan2_half_deg(-500, 0),
          "atan2(-y, 0) = -90 deg");

    imu_core_map_axes(100, 200, 300, &in);
    CHECK("H-IM-07",
          (100 == in.a_fwd_mg) && (300 == in.a_up_mg) && (200 == in.a_lat_mg),
          "default axis map: X forward, Z up, Y lateral");
}

/* ---- pitch and filter -------------------------------------------------- */

static void
test_pitch (void)
{
    imu_core_t core;
    imu_in_t   in;
    imu_out_t  out;
    tally_t    t;

    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 0.0, 0U);
    run_cycles(&core, &in, 5U, &t, &out);
    CHECK(
        "H-IM-08",
        (0 == out.state.pitch_half_deg) && !out.state.hump_active &&
            ((uint8_t) APP_MOTION_CLASS_STATIONARY == out.state.motion_class) &&
            ((uint8_t) APP_HUMP_Q_INVALID == out.state.quality),
        "level at rest: pitch 0, STATIONARY, quality INVALID before any "
        "episode");
    make_in(&in, 10.0, 0U);
    run_cycles(&core, &in, 1U, &t, &out);
    CHECK("H-IM-09",
          (out.state.pitch_half_deg > 0) && (out.state.pitch_half_deg < 20),
          "filter: one sample moves pitch part-way (alpha 1/4)");
    run_cycles(&core, &in, 30U, &t, &out);
    CHECK("H-IM-10",
          (out.state.pitch_half_deg >= 19) && (out.state.pitch_half_deg <= 21),
          "settled: 10 deg -> 20");
    make_in(&in, -70.0, 0U);
    run_cycles(&core, &in, 40U, &t, &out);
    CHECK("H-IM-11", -127 == out.state.pitch_half_deg,
          "pitch clamps at -63.5 deg (int8 field)");
    CHECK("H-IM-12", 0U == t.starts, "no episode while not moving");

    /* invalid sample: hold the filter, publish pitch 0 and INVALID */
    make_in(&in, 10.0, 0U);
    run_cycles(&core, &in, 30U, &t, &out);
    in.sample_valid = false;
    run_cycles(&core, &in, 1U, &t, &out);
    CHECK("H-IM-13",
          (0 == out.state.pitch_half_deg) &&
              ((uint8_t) APP_HUMP_Q_INVALID == out.state.quality),
          "invalid sample -> pitch 0, quality INVALID");
    in.sample_valid = true;
    run_cycles(&core, &in, 1U, &t, &out);
    CHECK("H-IM-14",
          (out.state.pitch_half_deg >= 19) && (out.state.pitch_half_deg <= 21),
          "filter state kept across an invalid sample");
}

/* ---- hump episodes --------------------------------------------------------
 */

/** Drive a trapezoid hump: up_cycles at +slope, crest, down_cycles at -slope.
 */
static void
drive_hump (imu_core_t * p_core, double slope_deg, uint32_t up_cycles,
            uint32_t crest_cycles, uint32_t down_cycles, tally_t * p_t,
            imu_out_t * p_out)
{
    imu_in_t in;
    make_in(&in, slope_deg, 20U);
    run_cycles(p_core, &in, up_cycles, p_t, p_out);
    make_in(&in, 0.0, 20U);
    run_cycles(p_core, &in, crest_cycles, p_t, p_out);
    make_in(&in, -slope_deg, 20U);
    run_cycles(p_core, &in, down_cycles, p_t, p_out);
    make_in(&in, 0.0, 20U);
    run_cycles(p_core, &in, 40U, p_t, p_out);
}

static void
test_hump_normal (void)
{
    imu_core_t core;
    imu_in_t   in;
    imu_out_t  out;
    tally_t    t;
    uint32_t   i              = 0U;
    uint32_t   start_at       = 0U;
    uint8_t    cls_seen_climb = 0U;
    uint8_t    cls_seen_desc  = 0U;

    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 0.0, 20U);
    run_cycles(&core, &in, 20U, &t, &out);
    CHECK("H-IM-15",
          (uint8_t) APP_MOTION_CLASS_CRUISING == out.state.motion_class,
          "steady 20 cm/s -> CRUISING");

    /* 15 deg ramp, 100 cycles = 200 mm along the slope -> 51.8 mm rise */
    make_in(&in, 15.0, 20U);
    for (i = 0U; i < 100U; i++)
    {
        imu_core_step(&core, &in, &out);
        tally_add(&t, &out);
        if (out.evt_hump_start)
        {
            start_at = i + 1U;
        }
        if ((uint8_t) APP_MOTION_CLASS_CLIMBING == out.state.motion_class)
        {
            cls_seen_climb = 1U;
        }
    }
    CHECK("H-IM-16", (1U == t.starts) && (start_at >= 5U) && (start_at <= 12U),
          "EVT_HUMP_START after theta_ON held 50 ms (filter + confirm)");
    CHECK("H-IM-17",
          out.state.hump_active && (1U == out.state.episode_seq) &&
              (out.state.height_mm > 30U),
          "active episode: seq 1, live peak published");
    CHECK("H-IM-18", 1U == cls_seen_climb,
          "motion class CLIMBING during the climb");

    make_in(&in, 0.0, 20U);
    run_cycles(&core, &in, 5U, &t, &out); /* crest */
    make_in(&in, -15.0, 20U);
    for (i = 0U; i < 100U; i++)
    {
        imu_core_step(&core, &in, &out);
        tally_add(&t, &out);
        if ((uint8_t) APP_MOTION_CLASS_DESCENDING == out.state.motion_class)
        {
            cls_seen_desc = 1U;
        }
    }
    CHECK("H-IM-19",
          (1U == cls_seen_desc) && out.state.hump_active && (0U == t.ends),
          "DESCENDING, still active, no END before settling");
    make_in(&in, 0.0, 20U);
    for (i = 0U; i < 40U; i++)
    {
        imu_core_step(&core, &in, &out);
        tally_add(&t, &out);
        if (out.evt_hump_end)
        {
            CHECK("H-IM-20", (i >= 4U) && (i <= 14U),
                  "END once |pitch| < theta_OFF for 50 ms");
        }
    }
    CHECK("H-IM-21",
          (1U == t.ends) && (1U == t.last_end_seq) &&
              ((uint8_t) APP_HUMP_Q_NORMAL == t.last_end_quality),
          "exactly one END, same seq, NORMAL");
    CHECK("H-IM-22", (t.last_end_height >= 45U) && (t.last_end_height <= 58U),
          "peak 51.8 mm estimated within +-6 mm (filter lag)");
    CHECK("H-IM-23",
          (out.state.max_height_mm == t.last_end_height) &&
              (out.state.height_mm == t.last_end_height) &&
              !out.state.hump_active &&
              ((uint8_t) APP_HUMP_Q_NORMAL == out.state.quality),
          "published: max = height = result, inactive, quality NORMAL");
    CHECK("H-IM-24",
          (uint8_t) APP_MOTION_CLASS_CRUISING == out.state.motion_class,
          "back to CRUISING after the hump");

    /* second, smaller hump: seq 2, run maximum retained */
    drive_hump(&core, 8.0, 100U, 5U, 100U, &t, &out);
    CHECK("H-IM-25",
          (2U == t.starts) && (2U == t.ends) && (2U == t.last_end_seq) &&
              ((uint8_t) APP_HUMP_Q_NORMAL == t.last_end_quality),
          "second episode seq 2 NORMAL");
    CHECK("H-IM-26",
          (t.last_end_height >= 22U) && (t.last_end_height <= 34U) &&
              (out.state.max_height_mm > t.last_end_height),
          "smaller hump (27.8 mm) does not lower the run maximum (FR11)");
    /* larger hump raises it */
    drive_hump(&core, 20.0, 100U, 5U, 100U, &t, &out);
    CHECK("H-IM-27",
          (3U == t.last_end_seq) &&
              (out.state.max_height_mm == t.last_end_height) &&
              (t.last_end_height >= 60U),
          "larger hump (68 mm) becomes the run maximum");
    CHECK("H-IM-28", 3U == core.stats_episodes, "episode statistics");
}

static void
test_hump_terminals (void)
{
    imu_core_t core;
    imu_in_t   in;
    imu_out_t  out;
    tally_t    t;
    uint32_t   i = 0U;

    /* REJECTED: false trigger, back to flat before any descent */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 15.0, 20U);
    run_cycles(&core, &in, 15U, &t, &out);
    CHECK("H-IM-29", (1U == t.starts) && out.state.hump_active, "trigger");
    make_in(&in, 0.0, 20U);
    for (i = 0U; i < 60U; i++)
    {
        imu_core_step(&core, &in, &out);
        tally_add(&t, &out);
        if (out.evt_hump_end)
        {
            CHECK("H-IM-30", (i >= 30U) && (i <= 40U),
                  "REJECTED after T_ABORT (300 ms) in the flat band");
        }
    }
    CHECK("H-IM-31",
          (1U == t.ends) &&
              ((uint8_t) APP_HUMP_Q_REJECTED == t.last_end_quality) &&
              (0U == t.last_end_height) && (0U == out.state.max_height_mm) &&
              !out.state.hump_active,
          "REJECTED: height 0, max untouched, one END");

    /* REJECTED: climb that never descends within D_HUMP_MAX */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 15.0, 20U); /* 2 mm per cycle -> 800 mm at ~400 cycles */
    for (i = 0U; i < 450U; i++)
    {
        imu_core_step(&core, &in, &out);
        tally_add(&t, &out);
        if (out.evt_hump_end)
        {
            CHECK("H-IM-32", (i >= 395U) && (i <= 415U),
                  "distance bound closes the episode");
        }
    }
    CHECK("H-IM-33",
          (1U == t.ends) &&
              ((uint8_t) APP_HUMP_Q_REJECTED == t.last_end_quality),
          "no descent within D_HUMP_MAX -> REJECTED");

    /* DEGRADED: descends but never settles -> time/distance bound keeps the
     * peak */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 15.0, 20U);
    run_cycles(&core, &in, 100U, &t, &out);
    make_in(&in, -15.0, 20U);
    run_cycles(&core, &in, 400U, &t, &out);
    CHECK(
        "H-IM-34",
        (1U == t.ends) &&
            ((uint8_t) APP_HUMP_Q_DEGRADED == t.last_end_quality) &&
            (t.last_end_height >= 45U) && (0U == out.state.max_height_mm) &&
            ((uint8_t) APP_HUMP_Q_DEGRADED == out.state.quality),
        "never settled -> DEGRADED with the peak; hump_max not updated (D-17)");

    /* INVALID: STOP_LATCH during CLIMBING */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 15.0, 20U);
    run_cycles(&core, &in, 50U, &t, &out);
    in.stop_latched = true;
    run_cycles(&core, &in, 1U, &t, &out);
    CHECK("H-IM-35",
          (1U == t.ends) &&
              ((uint8_t) APP_HUMP_Q_INVALID == t.last_end_quality) &&
              (0U == t.last_end_height) && !out.state.hump_active &&
              (0U == out.state.max_height_mm),
          "STOP_LATCH -> INVALID at once, peak discarded");
    run_cycles(&core, &in, 20U, &t, &out);
    CHECK("H-IM-36", (1U == t.starts) && (1U == t.ends),
          "no new episode while latched");

    /* INVALID: stale MotionStatus during DESCENDING */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 15.0, 20U);
    run_cycles(&core, &in, 100U, &t, &out);
    make_in(&in, -15.0, 20U);
    run_cycles(&core, &in, 30U, &t, &out);
    CHECK("H-IM-37",
          (uint8_t) APP_MOTION_CLASS_DESCENDING == out.state.motion_class,
          "in DESCENDING");
    in.motion_fresh = false;
    run_cycles(&core, &in, 1U, &t, &out);
    CHECK("H-IM-38",
          (1U == t.ends) &&
              ((uint8_t) APP_HUMP_Q_INVALID == t.last_end_quality),
          "stale motion -> INVALID");

    /* INVALID: sensor read failure mid-episode */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 15.0, 20U);
    run_cycles(&core, &in, 50U, &t, &out);
    in.sample_valid = false;
    run_cycles(&core, &in, 1U, &t, &out);
    CHECK("H-IM-39",
          (1U == t.ends) &&
              ((uint8_t) APP_HUMP_Q_INVALID == t.last_end_quality),
          "sensor fault -> INVALID");

    /* no start without fresh motion data */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 15.0, 20U);
    in.motion_fresh = false;
    run_cycles(&core, &in, 50U, &t, &out);
    CHECK("H-IM-40", 0U == t.starts, "no episode on stale motion data");

    /* saturation: a 40 deg, 400 mm climb rises ~257 mm -> clipped, DEGRADED */
    tally_reset(&t);
    imu_core_init(&core);
    drive_hump(&core, 40.0, 150U, 5U, 150U, &t, &out);
    CHECK("H-IM-41",
          (1U == t.ends) && t.any_sat && (127U == t.last_end_height) &&
              ((uint8_t) APP_HUMP_Q_DEGRADED == t.last_end_quality) &&
              (0U == out.state.max_height_mm),
          "height > 127 mm saturates: reported 127 as DEGRADED, never as fact");

    /* seq wraps in 4 bits */
    tally_reset(&t);
    imu_core_init(&core);
    for (i = 0U; i < 18U; i++)
    {
        drive_hump(&core, 10.0, 40U, 3U, 40U, &t, &out);
    }
    CHECK("H-IM-42",
          (18U == t.starts) && (18U == t.ends) && (2U == t.last_end_seq) &&
              (out.state.episode_seq == 2U),
          "18 episodes: seq wraps modulo 16 (-> 2)");
}

/* ---- collision and classes --------------------------------------------- */

static void
test_collision_and_class (void)
{
    imu_core_t core;
    imu_in_t   in;
    imu_out_t  out;
    tally_t    t;
    uint32_t   i = 0U;

    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 0.0, 20U);
    run_cycles(&core, &in, 20U, &t, &out);
    /* one spike sample: not enough */
    in.a_fwd_mg = -3000;
    run_cycles(&core, &in, 1U, &t, &out);
    in.a_fwd_mg = 0;
    run_cycles(&core, &in, 5U, &t, &out);
    CHECK("H-IM-43", 0U == t.collisions, "single-sample spike is ignored");
    /* two consecutive samples */
    in.a_fwd_mg = -3000;
    run_cycles(&core, &in, 2U, &t, &out);
    CHECK("H-IM-44",
          (1U == t.collisions) &&
              ((uint8_t) APP_MOTION_CLASS_IMPACT == out.state.motion_class),
          "2 spike samples (20 ms) -> collision once, class IMPACT at once");
    in.a_fwd_mg = 0;
    run_cycles(&core, &in, 30U, &t, &out);
    CHECK("H-IM-45",
          (uint8_t) APP_MOTION_CLASS_IMPACT == out.state.motion_class,
          "IMPACT held 500 ms");
    run_cycles(&core, &in, 30U, &t, &out);
    CHECK("H-IM-46",
          (uint8_t) APP_MOTION_CLASS_CRUISING == out.state.motion_class,
          "IMPACT released after the hold");
    CHECK("H-IM-47", 1U == core.stats_collisions, "collision counted once");

    /* spike at rest: no stop (car handled on the bench) */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 0.0, 0U);
    run_cycles(&core, &in, 10U, &t, &out);
    in.a_fwd_mg = 3000;
    run_cycles(&core, &in, 3U, &t, &out);
    CHECK("H-IM-48", 0U == t.collisions,
          "spike while not moving is not a collision (D-18)");

    /* spike after the latch is set: no second request */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 0.0, 20U);
    run_cycles(&core, &in, 10U, &t, &out);
    in.stop_latched = true;
    in.a_fwd_mg     = 3000;
    run_cycles(&core, &in, 3U, &t, &out);
    CHECK("H-IM-49", 0U == t.collisions,
          "no collision request while already latched");

    /* classes */
    tally_reset(&t);
    imu_core_init(&core);
    make_in(&in, 0.0, 0U);
    run_cycles(&core, &in, 5U, &t, &out);
    CHECK("H-IM-50",
          (uint8_t) APP_MOTION_CLASS_STATIONARY == out.state.motion_class,
          "STATIONARY");
    make_in(&in, 0.0, 20U); /* 0 -> 20 cm/s step: accelerating */
    for (i = 0U; i < 3U; i++)
    {
        imu_core_step(&core, &in, &out);
        tally_add(&t, &out);
    }
    CHECK("H-IM-51",
          ((uint8_t) APP_MOTION_CLASS_ACCELERATING == out.state.motion_class) &&
              (1U == t.class_changes),
          "speed step -> ACCELERATING after the 30 ms hold, one change event");
    run_cycles(&core, &in, 40U, &t, &out);
    CHECK("H-IM-52",
          (uint8_t) APP_MOTION_CLASS_CRUISING == out.state.motion_class,
          "steady -> CRUISING");
    in.turning = true;
    run_cycles(&core, &in, 2U, &t, &out);
    CHECK("H-IM-53",
          (uint8_t) APP_MOTION_CLASS_CRUISING == out.state.motion_class,
          "2 cycles: not yet (hold)");
    run_cycles(&core, &in, 1U, &t, &out);
    CHECK("H-IM-54",
          (uint8_t) APP_MOTION_CLASS_TURNING == out.state.motion_class,
          "TURNING after 3 cycles");
    in.turning = false;
    run_cycles(&core, &in, 5U, &t, &out);
    make_in(&in, 0.0, 0U);
    run_cycles(&core, &in, 5U, &t, &out);
    CHECK("H-IM-55",
          (uint8_t) APP_MOTION_CLASS_STATIONARY == out.state.motion_class,
          "stop -> STATIONARY");

    /* packed word round trip of a live state */
    {
        uint32_t        w = 0U;
        app_imu_state_t v;
        (void) app_imu_pack(&out.state, &w);
        app_imu_unpack(w, &v);
        CHECK("H-IM-56",
              (v.motion_class == out.state.motion_class) &&
                  (v.quality == out.state.quality) &&
                  (v.episode_seq == out.state.episode_seq) &&
                  (v.pitch_half_deg == out.state.pitch_half_deg),
              "ImuState word round trip");
    }
}

int
main (void)
{
    test_maths();
    test_pitch();
    test_hump_normal();
    test_hump_terminals();
    test_collision_and_class();
    printf("imu host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
/* NOLINTEND(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */
