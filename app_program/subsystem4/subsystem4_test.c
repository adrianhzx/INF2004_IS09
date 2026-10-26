/** @file subsystem4_test.c
 *  @brief On-target tests for subsystem 4 (make TEST=1). IDs S4-U01..S4-U07.
 *         Logged in testing/subsystem4/TEST_LOG.md. Needs the LSM303DLHC on
 *         GROVE 3 and the car level and still on the bench.
 */

#include "subsystem4.h"

#if APP_TEST

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_imu.h"
#include "../shared/app_safety.h"
#include "../test_report.h"
#include "drv_i2c.h"
#include "drv_lsm303.h"

#define S4_T_G_MIN_MG 800U /* |a| at rest within 1 g +-20 % */
#define S4_T_G_MAX_MG 1200U
#define S4_T_LEVEL_HALF_DEG 20 /* |pitch| < 10 deg on a level bench */
#define S4_T_MAX_CYCLE_US 600U /* 7-byte burst at 400 kHz ~250 us + core */
/* S4-U07 live IMU loop: 1 = print live readings forever after S4-U06 (the
   rest of the test run never runs); set 0 before committing. */
#define S4_T_LIVE_LOOP 0
#define S4_T_LIVE_GAP_MS 250U /* 4 lines per second */
#define S4_T_SCAN_FIRST 0x08U
#define S4_T_SCAN_LAST 0x77U

#if S4_T_LIVE_LOOP
static char const * const s4t_class_name[8] = {
    "STATIONARY", "ACCELERATING", "TURNING",  "CLIMBING",
    "DESCENDING", "IMPACT",       "CRUISING", "?"};
static char const * const s4t_quality_name[4] = {"NORMAL", "REJECTED",
                                                 "DEGRADED", "INVALID"};

/** Integer square root (for |a| in mg), no floating point. */
static uint32_t
s4t_isqrt (uint32_t v)
{
    uint32_t r   = 0U;
    uint32_t bit = 1UL << 30U;

    while (bit > v)
    {
        bit >>= 2U;
    }
    while (0U != bit)
    {
        if (v >= (r + bit))
        {
            v -= r + bit;
            r = (r >> 1U) + bit;
        }
        else
        {
            r >>= 1U;
        }
        bit >>= 2U;
    }
    return r;
}

/** S4-U07 wiring check: bus line levels, an I2C address scan and the
 *  ImuTask state. Idle I2C lines must read SDA=1 SCL=1; the GY-511
 *  (LSM303DLHC) answers at 0x19 (accel) and 0x1E (magnetometer). */
static void
s4t_diag (void)
{
    uint32_t a     = 0U;
    uint32_t found = 0U;
    uint8_t  b     = 0U;
    T_RTSK   rt;

    tm_printf((UB *) "[TEST] S4-U07 DIAG  bus idle: SDA(GP%u)=%u SCL(GP%u)=%u "
                     "(both must be 1; 0 = sensor unpowered, wires swapped "
                     "or shorted)\n",
              (unsigned) APP_PIN_IMU_SDA,
              (unsigned) app_hw_gpio_read(APP_PIN_IMU_SDA),
              (unsigned) APP_PIN_IMU_SCL,
              (unsigned) app_hw_gpio_read(APP_PIN_IMU_SCL));
    tm_printf((UB *) "[TEST] S4-U07 DIAG  I2C scan 0x08-0x77:");
    for (a = S4_T_SCAN_FIRST; a <= S4_T_SCAN_LAST; a++)
    {
        if (drv_i2c_read_regs((uint8_t) a, 0U, &b, 1U))
        {
            tm_printf((UB *) " 0x%02x", (unsigned) a);
            found++;
        }
    }
    tm_printf((UB *) "  -> %u device(s); expect 0x19 and 0x1e. aborts=%u "
                     "timeouts=%u\n",
              (unsigned) found, (unsigned) drv_i2c_aborts(),
              (unsigned) drv_i2c_timeouts());
    if (E_OK == tk_ref_tsk(tid_imu, &rt))
    {
        tm_printf((UB *) "[TEST] S4-U07 DIAG  ImuTask tid=%d state=0x%x "
                         "(1 run, 2 ready, 4 waiting, 8 suspended) "
                         "wupcnt=%u cycles=%u sensor_ok=%u\n",
                  (int) tid_imu, (unsigned) rt.tskstat, (unsigned) rt.wupcnt,
                  (unsigned) s4_imu_cycle_count(),
                  (unsigned) s4_imu_sensor_ok());
    }
    else
    {
        tm_printf((UB *) "[TEST] S4-U07 DIAG  ImuTask not found (tid=%d)\n",
                  (int) tid_imu);
    }
}

/** S4-U07: print the live IMU readings forever. Tilt the car for pitch,
 *  roll it over a hump for CLIMBING/DESCENDING, knock it for IMPACT. */
static void
s4t_live_loop (void)
{
    uint32_t        i   = 0U;
    uint32_t        c0  = 0U;
    uint32_t        mag = 0U;
    int16_t         x   = 0;
    int16_t         y   = 0;
    int16_t         z   = 0;
    int32_t         ph  = 0;
    app_imu_state_t st;

    tm_printf((UB *) "[TEST] S4-U07 INFO  live IMU LOOP (never ends; unplug "
                     "to stop). Tilt nose up/down, knock the car, roll it "
                     "over a hump\n");
    s4t_diag();
    c0 = s4_imu_cycle_count();
    for (i = 0U;; i++)
    {
        (void) tk_dly_tsk(S4_T_LIVE_GAP_MS);
        s4_imu_last_sample(&x, &y, &z);
        app_imu_read(&st);
        mag = s4t_isqrt((uint32_t) ((int32_t) x * x) +
                        (uint32_t) ((int32_t) y * y) +
                        (uint32_t) ((int32_t) z * z));
        ph  = (int32_t) st.pitch_half_deg;
        tm_printf((UB *) "[TEST] S4-U07 #%04u  x=%5d y=%5d z=%5d mg  |a|=%4u "
                         "mg  pitch=%c%2d.%c deg  %-12s",
                  (unsigned) i, x, y, z, (unsigned) mag, (ph < 0) ? '-' : '+',
                  (int) (((ph < 0) ? -ph : ph) / 2),
                  (0 != (ph % 2)) ? '5' : '0',
                  s4t_class_name[st.motion_class & 7U]);
        tm_printf(
            (UB *) "  hump=%s h=%umm max=%umm q=%s  %u cyc/s err=%u%s%s"
                   "  [sda=%u scl=%u to=%u ab=%u]\n",
            st.hump_active ? "ON " : "off", (unsigned) st.height_mm,
            (unsigned) st.max_height_mm, s4t_quality_name[st.quality & 3U],
            (unsigned) (((s4_imu_cycle_count() - c0) * 1000U) /
                        S4_T_LIVE_GAP_MS),
            (unsigned) s4_imu_read_errors(),
            (0U != (app_fault_get() & APP_FAULT_IMU_FAULT)) ? "  IMU_FAULT"
                                                            : "",
            (APP_STOP_CAUSE_COLLISION == app_stop_cause_get())
                ? "  COLLISION-STOP"
                : "",
            (unsigned) app_hw_gpio_read(APP_PIN_IMU_SDA),
            (unsigned) app_hw_gpio_read(APP_PIN_IMU_SCL),
            (unsigned) drv_i2c_timeouts(), (unsigned) drv_i2c_aborts());
        c0 = s4_imu_cycle_count();
    }
}
#endif

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

#if S4_T_LIVE_LOOP
    s4t_live_loop(); /* S4-U07: never returns */
#endif

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
