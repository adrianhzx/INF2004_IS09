/** @file task_imu.c
 *  @brief ImuTask (priority 5, 10 ms, CYC_10MS slot IMU). Owner: Buddy 4.
 *
 *  Each cycle: read the accelerometer (bounded polled I2C), read
 *  MotionStatus (fresh / moving / turning / distance delta), read the latch
 *  byte, run imu_core, publish the ImuState word with one aligned store,
 *  raise EVT_HUMP_START / EVT_HUMP_END on FLG_CTRL with telemetry events,
 *  and on a collision call app_safe_stop_request(COLLISION) FIRST and only
 *  then set IMU_FAULT through the helper (Rev B section 3). A failing sensor
 *  sets IMU_FAULT after APP_CAL_IMU_FAULT_ERRORS consecutive errors, keeps
 *  publishing quality INVALID, and is re-initialised every
 *  APP_CAL_IMU_RETRY_MS; the car is never stopped for a sensor fault.
 */

#include "subsystem4.h"

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_imu.h"
#include "../shared/app_ipc.h"
#include "../shared/app_motion.h"
#include "../shared/app_port.h"
#include "../shared/app_safety.h"
#include "drv_lsm303.h"
#include "imu_core.h"

#define S4_CYCLE_MS 10U
#define S4_RETRY_CYCLES (APP_CAL_IMU_RETRY_MS / S4_CYCLE_MS)
#define S4_STALE_CYCLES                                                        \
    ((APP_CAL_T_STALE_MOTION_MS + S4_CYCLE_MS - 1U) / S4_CYCLE_MS)
#define S4_SEQ_SHIFT 8U
#define S4_QUALITY_SHIFT 12U

static imu_core_t      g_s4_core;
static lsm303_sample_t g_s4_sample;
static bool            g_s4_sensor_ok = false;
static uint8_t         g_s4_errors    = 0U;
static uint16_t        g_s4_retry     = 0U;
static bool            g_s4_fault = false; /* IMU_FAULT currently set by us */
static bool            g_s4_coll_fault =
    false; /* set by a collision, cleared on latch release */
static uint8_t           g_s4_last_tag    = 0U;
static int16_t           g_s4_last_dist   = 0;
static bool              g_s4_motion_seen = false;
static uint8_t           g_s4_stale       = 0U;
static bool              g_s4_stale_fault = false;
static volatile uint32_t g_s4_cycles      = 0U;
static volatile uint32_t g_s4_max_us      = 0U;
static volatile uint32_t g_s4_read_errors = 0U;

static void
sensor_bring_up (void)
{
    g_s4_sensor_ok = drv_lsm303_init();
    g_s4_errors    = 0U;
    g_s4_retry     = 0U;
}

static void
read_sensor (imu_in_t * p_in)
{

    if (!g_s4_sensor_ok)
    {
        g_s4_retry++;
        if (g_s4_retry >= S4_RETRY_CYCLES)
        {
            sensor_bring_up();
        }
    }
    else
    {
        if (drv_lsm303_read(&g_s4_sample))
        {
            g_s4_errors = 0U;
        }
        else
        {
            g_s4_read_errors++;
            g_s4_errors++;
            if (g_s4_errors >= APP_CAL_IMU_FAULT_ERRORS)
            {
                g_s4_sensor_ok = false; /* re-init path takes over */
                g_s4_retry     = 0U;
            }
        }
    }
    /* a non-fresh sample (ODR phase) or a single bus error reuses the previous
       values; the sample is invalid only once the sensor is declared faulty */
    imu_core_map_axes(g_s4_sample.x_mg, g_s4_sample.y_mg, g_s4_sample.z_mg,
                      p_in);
    p_in->sample_valid = g_s4_sensor_ok;
}

static void
read_motion (imu_in_t * p_in)
{
    app_motion_status_t ms;
    bool                changed = false;

    app_motion_read(&ms);
    changed = (!g_s4_motion_seen) || (ms.cycle_tag != g_s4_last_tag);
    if (changed)
    {
        g_s4_stale = 0U;
    }
    else if (g_s4_stale < S4_STALE_CYCLES)
    {
        g_s4_stale++;
    }
    else
    {
        /* saturated */
    }
    p_in->motion_fresh = g_s4_motion_seen && (g_s4_stale < S4_STALE_CYCLES);
    p_in->moving       = (ms.speed_l_cm_s > 0U) || (ms.speed_r_cm_s > 0U);
    p_in->turning      = ((uint8_t) APP_MOTION_TURNING == ms.motion_state);
    p_in->speed_cm_s =
        (uint8_t) (((uint32_t) ms.speed_l_cm_s + (uint32_t) ms.speed_r_cm_s) /
                   2U);
    p_in->dist_delta_mm = g_s4_motion_seen
                              ? (int16_t) (ms.distance_mm - g_s4_last_dist)
                              : (int16_t) 0;
    g_s4_last_dist      = ms.distance_mm;
    g_s4_last_tag       = ms.cycle_tag;
    g_s4_motion_seen    = true;
    p_in->stop_latched  = app_stop_latched();

    /* diagnostic only: STALE_MOTION fault mirrors what this task observes */
    if (!p_in->motion_fresh && !g_s4_stale_fault && g_s4_motion_seen)
    {
        g_s4_stale_fault = true;
        app_fault_set(APP_FAULT_STALE_MOTION);
    }
    else if (p_in->motion_fresh && g_s4_stale_fault)
    {
        g_s4_stale_fault = false;
        app_fault_clr(APP_FAULT_STALE_MOTION);
    }
    else
    {
        /* unchanged */
    }
}

static void
publish (imu_in_t const * p_in, imu_out_t const * p_out)
{
    uint32_t word = 0U;
    UINT     evt  = 0U;

    (void) app_imu_pack(&p_out->state, &word);
    g_imu_state = word; /* one aligned store (rule 3) */

    if (p_out->collision)
    {
        app_safe_stop_request(
            APP_STOP_CAUSE_COLLISION);      /* first: braking never waits */
        app_fault_set(APP_FAULT_IMU_FAULT); /* then the mask (Rev B) */
        g_s4_coll_fault = true;
        g_s4_fault      = true;
    }
    if (p_out->evt_hump_start)
    {
        evt |= (UINT) APP_EVT_HUMP_START;
        app_ipc_telem_post(APP_TEV_HUMP_START, APP_SRC_IMU,
                           (uint16_t) p_out->state.episode_seq);
    }
    if (p_out->evt_hump_end)
    {
        evt |= (UINT) APP_EVT_HUMP_END;
        app_ipc_telem_post(
            APP_TEV_HUMP_END, APP_SRC_IMU,
            (uint16_t) (((uint16_t) p_out->end_quality << S4_QUALITY_SHIFT) |
                        ((uint16_t) p_out->end_seq << S4_SEQ_SHIFT) |
                        p_out->end_height_mm));
    }
    if (0U != evt)
    {
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), evt);
    }
    if (p_out->class_changed)
    {
        app_ipc_telem_post(APP_TEV_MOTION_CLASS, APP_SRC_IMU,
                           (uint16_t) p_out->state.motion_class);
    }

    /* IMU_FAULT: sensor unhealthy, or a collision until the latch is released
     */
    if (g_s4_coll_fault && !p_in->stop_latched)
    {
        g_s4_coll_fault = false;
    }
    if ((!g_s4_sensor_ok || g_s4_coll_fault) && !g_s4_fault)
    {
        g_s4_fault = true;
        app_fault_set(APP_FAULT_IMU_FAULT);
    }
    else if (g_s4_sensor_ok && !g_s4_coll_fault && g_s4_fault)
    {
        g_s4_fault = false;
        app_fault_clr(APP_FAULT_IMU_FAULT);
    }
    else
    {
        /* unchanged */
    }
}

void
s4_imu_cycle (void)
{
    uint32_t const t0 = app_hw_time_us();
    imu_in_t       in;
    imu_out_t      out;

    read_sensor(&in);
    read_motion(&in);
    imu_core_step(&g_s4_core, &in, &out);
    publish(&in, &out);

    g_s4_cycles++;
    {
        uint32_t const dt = (uint32_t) (app_hw_time_us() - t0);
        if (dt > g_s4_max_us)
        {
            g_s4_max_us = dt;
        }
    }
}

void
s4_imu_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    imu_core_init(&g_s4_core);
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR); /* woken by CYC_10MS slot IMU */
        (void) tk_can_wup(TSK_SELF);
        s4_imu_cycle();
    }
}

bool
s4_imu_sensor_init (void)
{
    sensor_bring_up();
    return g_s4_sensor_ok;
}

bool
s4_imu_sensor_ok (void)
{
    return g_s4_sensor_ok;
}

uint32_t
s4_imu_cycle_count (void)
{
    return g_s4_cycles;
}

uint32_t
s4_imu_max_cycle_us (void)
{
    return g_s4_max_us;
}

uint32_t
s4_imu_read_errors (void)
{
    return g_s4_read_errors;
}

void
s4_imu_last_sample (int16_t * p_x_mg, int16_t * p_y_mg, int16_t * p_z_mg)
{
    *p_x_mg = g_s4_sample.x_mg;
    *p_y_mg = g_s4_sample.y_mg;
    *p_z_mg = g_s4_sample.z_mg;
}
