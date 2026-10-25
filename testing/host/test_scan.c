/** @file test_scan.c
 *  @brief Host tests for subsystem 5 scan_core (H-SC-*): servo mapping,
 *         geometry, forward-watch confirmation and NO_ECHO counting, coarse
 *         and fine plans, profile building for synthetic obstacles (centred,
 *         offset left/right, wide, none, too close), side-range confirmation,
 *         ObstacleProfile word round trip.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_cal.h"
#include "host_port.h"
#include "scan_core.h"

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

#define T_PI 3.14159265358979323846

/** Synthetic world: a box obstacle spanning lateral [y_lo, y_hi] cm at
 *  distance x_face cm ahead; walls beyond at wall_cm. Returns the range the
 *  sensor would report at an angle index (ray/box intersection, no beam). */
#define BEAM_HALF_DEG                                                          \
    7.0 /* HC-SR04: the box is seen when any ray within the beam hits it */

static uint16_t
world_range (uint32_t idx, double x_face, double y_lo, double y_hi,
             double wall_cm)
{
    double const centre = ((double) idx - 18.0) * 5.0;
    double       best   = wall_cm;
    int          k      = 0;
    for (k = -2; k <= 2; k++)
    {
        double const th =
            (centre + (double) k * BEAM_HALF_DEG / 2.0) * T_PI / 180.0;
        double const c = cos(th);
        if (c > 1e-6)
        {
            double const r = x_face / c; /* hit the face plane */
            double const y = r * sin(th);
            if ((y >= y_lo) && (y <= y_hi) && (r < best))
            {
                best = r;
            }
        }
    }
    return (best >= 511.0) ? 511U : (uint16_t) lround(best);
}

/** Run coarse plan, then fine plan, filling samples from the world. */
static void
scan_world (sc_samples_t * p_s, double x_face, double y_lo, double y_hi,
            double wall_cm, uint32_t * p_fine_n)
{
    uint8_t  plan[SC_ANGLE_COUNT];
    uint32_t n = 0U;
    uint32_t i = 0U;

    scan_core_samples_init(p_s);
    n = scan_core_coarse_plan(plan, SC_ANGLE_COUNT);
    for (i = 0U; i < n; i++)
    {
        scan_core_samples_set(
            p_s, plan[i], world_range(plan[i], x_face, y_lo, y_hi, wall_cm));
    }
    n = scan_core_fine_plan(p_s, plan, SC_ANGLE_COUNT);
    for (i = 0U; i < n; i++)
    {
        scan_core_samples_set(
            p_s, plan[i], world_range(plan[i], x_face, y_lo, y_hi, wall_cm));
    }
    *p_fine_n = n;
}

static void
test_servo_geometry (void)
{
    uint32_t i     = 0U;
    int32_t  worst = 0;

    CHECK("H-SC-01", 1500U == scan_core_servo_us(18U),
          "ahead = mid pulse (1500 us)");
    CHECK("H-SC-02",
          (APP_CAL_SERVO_MIN_US == scan_core_servo_us(0U)) &&
              (APP_CAL_SERVO_MAX_US == scan_core_servo_us(36U)),
          "idx 0 / 36 = pulse limits");
    CHECK("H-SC-03", APP_CAL_SERVO_MAX_US == scan_core_servo_us(99U),
          "index clamps at 36");
    CHECK("H-SC-04", 150U == scan_core_settle_ms(18U, 24U),
          "30 deg move settles in T_SERVO_SETTLE (150 ms)");
    CHECK("H-SC-05",
          (25U == scan_core_settle_ms(18U, 19U)) &&
              (APP_CAL_T_SERVO_MIN_SETTLE_MS == scan_core_settle_ms(18U, 18U)),
          "5 deg step = 25 ms; no move uses the 20 ms floor");
    CHECK("H-SC-06", 900U == scan_core_settle_ms(0U, 36U),
          "180 deg sweep = 900 ms (A11)");

    for (i = 0U; i <= 36U; i++)
    {
        double const  th = ((double) i - 18.0) * 5.0 * T_PI / 180.0;
        int32_t const es = (int32_t) lround(sin(th) * 32768.0);
        int32_t const ec = (int32_t) lround(cos(th) * 32768.0);
        int32_t       ds = scan_core_sin_q15(i) - es;
        int32_t       dc = scan_core_cos_q15(i) - ec;
        ds               = (ds < 0) ? -ds : ds;
        dc               = (dc < 0) ? -dc : dc;
        worst            = (ds > worst) ? ds : worst;
        worst            = (dc > worst) ? dc : worst;
    }
    CHECK("H-SC-07", worst <= 2,
          "sin/cos table within 2 LSB (Q15) over -90..+90");
    CHECK("H-SC-08", 0 == scan_core_lateral_cm(40U, 18U), "ahead: y = 0");
    CHECK("H-SC-09", 20 == scan_core_lateral_cm(40U, 24U),
          "+30 deg, 40 cm: y = +20 (LEFT positive)");
    CHECK("H-SC-10", -20 == scan_core_lateral_cm(40U, 12U),
          "-30 deg, 40 cm: y = -20");
    CHECK("H-SC-11", 63 == scan_core_lateral_cm(500U, 36U),
          "y clamps to the int7 field");
}

static void
test_watch (void)
{
    sc_watch_t w;
    bool       fault  = false;
    uint32_t   i      = 0U;
    uint32_t   raised = 0U;

    scan_core_watch_init(&w);
    raised += scan_core_watch_step(&w, 100U, &fault) ? 1U : 0U;
    raised += scan_core_watch_step(&w, 24U, &fault) ? 1U : 0U;
    CHECK("H-SC-12", 0U == raised, "one reading inside D_STOP is not enough");
    raised += scan_core_watch_step(&w, 23U, &fault) ? 1U : 0U;
    CHECK("H-SC-13", 1U == raised,
          "second consecutive reading at/inside D_STOP -> EVT_OBSTACLE");
    raised += scan_core_watch_step(&w, 20U, &fault) ? 1U : 0U;
    raised += scan_core_watch_step(&w, 15U, &fault) ? 1U : 0U;
    CHECK("H-SC-14", 1U == raised, "reported once while the obstacle stays");
    raised += scan_core_watch_step(&w, 27U, &fault)
                  ? 1U
                  : 0U; /* inside hysteresis band */
    raised += scan_core_watch_step(&w, 24U, &fault) ? 1U : 0U;
    raised += scan_core_watch_step(&w, 24U, &fault) ? 1U : 0U;
    CHECK("H-SC-15", 1U == raised, "27 cm (within D_STOP + 5) does not re-arm");
    raised += scan_core_watch_step(&w, 40U, &fault) ? 1U : 0U;
    raised += scan_core_watch_step(&w, 24U, &fault) ? 1U : 0U;
    raised += scan_core_watch_step(&w, 24U, &fault) ? 1U : 0U;
    CHECK("H-SC-16", 2U == raised,
          "clear beyond the hysteresis re-arms; next approach reports again");

    scan_core_watch_init(&w);
    (void) scan_core_watch_step(&w, 24U, &fault);
    (void) scan_core_watch_step(
        &w, 511U, &fault); /* a missed echo between two inside readings */
    raised = scan_core_watch_step(&w, 24U, &fault) ? 1U : 0U;
    CHECK("H-SC-17", 0U == raised,
          "a no-echo reading breaks the confirmation pair");

    scan_core_watch_init(&w);
    for (i = 0U; i < 4U; i++)
    {
        (void) scan_core_watch_step(&w, 511U, &fault);
    }
    CHECK("H-SC-18", !fault, "4 echo timeouts: no fault yet");
    (void) scan_core_watch_step(&w, 511U, &fault);
    CHECK("H-SC-19", fault, "5th consecutive timeout -> NO_ECHO fault");
    (void) scan_core_watch_step(&w, 100U, &fault);
    CHECK("H-SC-20", !fault, "a valid echo clears NO_ECHO");
}

static void
test_profiles (void)
{
    sc_samples_t           s;
    app_obstacle_profile_t p;
    uint8_t                plan[SC_ANGLE_COUNT];
    uint32_t               n      = 0U;
    uint32_t               fine_n = 0U;

    n = scan_core_coarse_plan(plan, SC_ANGLE_COUNT);
    CHECK("H-SC-21",
          (9U == n) && (6U == plan[0]) && (18U == plan[4]) && (30U == plan[8]),
          "coarse plan: 9 points, -60..+60 deg in 15 deg steps");

    /* nothing ahead */
    scan_world(&s, 300.0, -10.0, 10.0, 511.0, &fine_n);
    scan_core_build_profile(&s, 300U, &p);
    CHECK("H-SC-22",
          (0U == fine_n) && ((uint8_t) APP_AVOID_CONTINUE == p.action) &&
              (0U == p.width_cm) && (300U == p.closest_cm) &&
              (300U == p.range_cm),
          "no sample inside R_PROFILE: no fine scan, CONTINUE, width 0");

    /* 20 cm wide box centred on the line, face at 22 cm (just after braking) */
    scan_world(&s, 22.0, -10.0, 10.0, 511.0, &fine_n);
    scan_core_build_profile(&s, 22U, &p);
    CHECK("H-SC-23", (fine_n >= 7U) && (fine_n <= 19U),
          "fine plan covers the coarse span plus one step each side");
    CHECK("H-SC-24",
          (p.width_cm >= 15U) && (p.width_cm <= 25U) && (p.y_min_cm <= -7) &&
              (p.y_max_cm >= 7),
          "centred 20 cm box seen through a 14 deg beam: width 20 +-5 after "
          "correction");
    CHECK("H-SC-25",
          (22U == p.closest_cm) && (p.scan_angle_idx >= 15U) &&
              (p.scan_angle_idx <= 21U),
          "closest point about straight ahead");
    CHECK("H-SC-26",
          ((uint8_t) APP_AVOID_TURN_LEFT == p.action) ||
              ((uint8_t) APP_AVOID_TURN_RIGHT == p.action),
          "centred box with free sides: a turn is recommended");

    /* box offset to the left: y in [0, 20] -> go RIGHT (smaller offset) */
    scan_world(&s, 25.0, 0.0, 20.0, 511.0, &fine_n);
    scan_core_build_profile(&s, 25U, &p);
    CHECK("H-SC-27",
          ((uint8_t) APP_AVOID_TURN_RIGHT == p.action) && (p.y_max_cm > 10) &&
              (p.y_min_cm >= -3),
          "left-offset box -> TURN_RIGHT");

    /* box offset to the right: y in [-20, 0] -> go LEFT */
    scan_world(&s, 25.0, -20.0, 0.0, 511.0, &fine_n);
    scan_core_build_profile(&s, 25U, &p);
    CHECK("H-SC-28",
          ((uint8_t) APP_AVOID_TURN_LEFT == p.action) && (p.y_min_cm < -10) &&
              (p.y_max_cm <= 3),
          "right-offset box -> TURN_LEFT");

    /* wall on the left side too close: only RIGHT valid */
    scan_core_samples_init(&s);
    {
        uint8_t  cp[SC_ANGLE_COUNT];
        uint32_t i = 0U;
        n          = scan_core_coarse_plan(cp, SC_ANGLE_COUNT);
        for (i = 0U; i < n; i++)
        {
            uint16_t r = world_range(cp[i], 22.0, -10.0, 10.0, 511.0);
            if (cp[i] > 24U)
            {
                r = 40U; /* something 40 cm away on the left: below R_SIDE_FREE
                          */
            }
            scan_core_samples_set(&s, cp[i], r);
        }
        n = scan_core_fine_plan(&s, cp, SC_ANGLE_COUNT);
        for (i = 0U; i < n; i++)
        {
            uint16_t r = world_range(cp[i], 22.0, -10.0, 10.0, 511.0);
            if (cp[i] > 24U)
            {
                r = 40U;
            }
            scan_core_samples_set(&s, cp[i], r);
        }
    }
    scan_core_build_profile(&s, 22U, &p);
    CHECK("H-SC-29", (uint8_t) APP_AVOID_TURN_RIGHT == p.action,
          "left side not free -> TURN_RIGHT");

    /* wide box: no valid side -> STOP (controller decides reverse/retry) */
    scan_world(&s, 22.0, -35.0, 35.0, 511.0, &fine_n);
    scan_core_build_profile(&s, 22U, &p);
    CHECK("H-SC-30",
          ((uint8_t) APP_AVOID_STOP == p.action) && (p.width_cm >= 40U),
          "70 cm box: offsets exceed Y_PATH_MAX -> STOP");

    /* too close for the pivot: REVERSE */
    scan_world(&s, 12.0, -10.0, 10.0, 511.0, &fine_n);
    scan_core_build_profile(&s, 12U, &p);
    CHECK("H-SC-31", (uint8_t) APP_AVOID_REVERSE == p.action,
          "forward range < R_swept + M_clear -> REVERSE");

    /* width field saturates at 63 */
    scan_world(&s, 40.0, -60.0, 60.0, 511.0, &fine_n);
    scan_core_build_profile(&s, 40U, &p);
    CHECK("H-SC-32", p.width_cm <= 63U, "width clamps to 6 bits");

    /* packed round trip */
    {
        uint32_t               w0 = 0U;
        uint32_t               w1 = 0U;
        app_obstacle_profile_t v;
        p.seq        = 9U;
        p.scan_state = (uint8_t) SC_STATE_FINE;
        (void) app_obstacle_pack(&p, &w0, &w1);
        app_obstacle_unpack(w0, w1, &v);
        CHECK("H-SC-33",
              (v.y_min_cm == p.y_min_cm) && (v.y_max_cm == p.y_max_cm) &&
                  (v.width_cm == p.width_cm) &&
                  (v.closest_cm == p.closest_cm) && (v.action == p.action) &&
                  (9U == v.seq) && ((uint8_t) SC_STATE_FINE == v.scan_state) &&
                  (v.scan_angle_idx == p.scan_angle_idx),
              "ObstacleProfile two-word round trip");
    }
}

static void
test_side (void)
{
    uint8_t c = 0U;
    CHECK("H-SC-34", !scan_core_side_step(&c, 8U),
          "one unsafe side reading: not yet");
    CHECK("H-SC-35", scan_core_side_step(&c, 9U),
          "two consecutive < D_SIDE_UNSAFE -> EVT_SIDE_UNSAFE");
    (void) scan_core_side_step(&c, 30U);
    CHECK("H-SC-36", !scan_core_side_step(&c, 8U),
          "a safe reading resets the pair");
    (void) scan_core_side_step(&c, 511U);
    CHECK("H-SC-37", 0U == c,
          "no echo on the side counts as safe (unknown), count reset");
}

int
main (void)
{
    test_servo_geometry();
    test_watch();
    test_profiles();
    test_side();
    printf("scan host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
/* NOLINTEND(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */
