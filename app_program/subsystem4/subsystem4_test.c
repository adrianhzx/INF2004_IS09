/** @file subsystem4_test.c
 *  @brief On-target tests for subsystem 4 (make TEST=1). IDs S4-U01..S4-U06.
 *         Logged in testing/subsystem4/TEST_LOG.md. Needs the LSM303DLHC on
 *         GROVE 3 and the car level and still on the bench.
 */

#include "subsystem4.h"

#if APP_TEST

#include "../shared/app_cal.h"
#include "../shared/app_imu.h"
#include "../shared/app_safety.h"
#include "../test_report.h"
#include "drv_i2c.h"
#include "drv_lsm303.h"

#define S4_T_G_MIN_MG 800U /* |a| at rest within 1 g +-20 % */
#define S4_T_G_MAX_MG 1200U
#define S4_T_LEVEL_HALF_DEG 20 /* |pitch| < 10 deg on a level bench */
#define S4_T_MAX_CYCLE_US 600U /* 7-byte burst at 400 kHz ~250 us + core */

EXPORT void
subsystem4_test (void)
{
    uint8_t         ctrl1 = 0U;
    uint8_t         ira   = 0U;
    int16_t         x     = 0;
    int16_t         y     = 0;
    int16_t         z     = 0;
    uint32_t        c0    = 0U;
    uint32_t        c1    = 0U;
    uint32_t        mag   = 0U;
    app_imu_state_t s0;
    app_imu_state_t s1;

    TEST_INIT();

    /* S4-U01: sensor present and configured (no WHO_AM_I on this part). */
    TEST_CHECK("S4-U01", drv_lsm303_identify(&ctrl1, &ira),
               "LSM303DLHC answers: CTRL_REG1_A 0x57, IRA_REG_M 'H'");
    tm_printf((UB *) "[TEST] S4-U01 INFO  ctrl1=0x%02x ira=0x%02x aborts=%u "
                     "timeouts=%u\n",
              ctrl1, ira, (unsigned) drv_i2c_aborts(),
              (unsigned) drv_i2c_timeouts());

    /* S4-U02: ImuTask cadence and cost. */
    c0 = s4_imu_cycle_count();
    (void) tk_dly_tsk(1000);
    c1 = s4_imu_cycle_count();
    TEST_CHECK("S4-U02a", ((c1 - c0) >= 95U) && ((c1 - c0) <= 105U),
               "ImuTask 100 +-5 cycles/s");
    tm_printf((UB *) "[TEST] S4-U02b INFO  longest IMU cycle = %u us, read "
                     "errors = %u\n",
              (unsigned) s4_imu_max_cycle_us(),
              (unsigned) s4_imu_read_errors());
    TEST_CHECK("S4-U02b", s4_imu_max_cycle_us() < S4_T_MAX_CYCLE_US,
               "IMU cycle < 600 us");

    /* S4-U03: gravity magnitude at rest. */
    s4_imu_last_sample(&x, &y, &z);
    mag = (uint32_t) ((int32_t) x * x) + (uint32_t) ((int32_t) y * y) +
          (uint32_t) ((int32_t) z * z);
    tm_printf((UB *) "[TEST] S4-U03 INFO  x=%d y=%d z=%d mg\n", x, y, z);
    TEST_CHECK("S4-U03",
               (mag >= (S4_T_G_MIN_MG * S4_T_G_MIN_MG)) &&
                   (mag <= (S4_T_G_MAX_MG * S4_T_G_MAX_MG)),
               "|a| = 1 g +-20 % at rest");

    /* S4-U04: ImuState freshness and level pitch (HV-12 axis mapping). */
    app_imu_read(&s0);
    TEST_CHECK("S4-U04a",
               s4_imu_sensor_ok() &&
                   (0U == (app_fault_get() & APP_FAULT_IMU_FAULT)),
               "sensor healthy, IMU_FAULT clear");
    TEST_CHECK("S4-U04b",
               (s0.pitch_half_deg > -S4_T_LEVEL_HALF_DEG) &&
                   (s0.pitch_half_deg < S4_T_LEVEL_HALF_DEG),
               "pitch within +-10 deg on a level bench (else fix "
               "APP_CAL_IMU_AXIS_*/SIGN)");
    TEST_CHECK("S4-U04c",
               !s0.hump_active && (0U == s0.max_height_mm) &&
                   ((uint8_t) APP_MOTION_CLASS_STATIONARY == s0.motion_class),
               "no episode, no max, STATIONARY at rest");

    /* S4-U05: no collision request while handled at rest (D-18). */
    tm_printf((UB *) "[TEST] S4-U05 INFO  tap the car once now (2 s window)\n");
    (void) tk_dly_tsk(2000);
    app_imu_read(&s1);
    TEST_CHECK("S4-U05",
               !app_stop_latched() &&
                   (APP_STOP_CAUSE_NONE == app_stop_cause_get()),
               "a tap at rest does not latch a stop");

    /* S4-U06: word keeps updating (seq field is the episode seq, so use
     * pitch/class liveness via cycles). */
    c0 = s4_imu_cycle_count();
    (void) tk_dly_tsk(15);
    TEST_CHECK("S4-U06", s4_imu_cycle_count() != c0,
               "ImuState refreshed within 15 ms");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
