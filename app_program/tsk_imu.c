/**
 * @file    tsk_imu.c
 * @brief   TSK_TERRAIN_IMU (DR Section 3 / 6.3, 10 ms): IS09 hump detector,
 *          collision detection and motion class (IS09 port, 19 Nov).
 *
 * Every 10 ms, under MTX_I2C_BUS: read the accelerometer (milli-g, IS09
 * settings, hal_imu.c), then run IS09 imu_core (is09/subsystem4, Buddy 4)
 * through is09_imu.c with the motion inputs IS09 takes from MotionStatus:
 * moving, turning, mean speed, signed distance since the last step, stop
 * latch. imu_core does everything the base imu_logic.c did before: pitch,
 * hump episodes (start, end, peak height by distance x sin(pitch), quality
 * NORMAL / REJECTED / DEGRADED / INVALID), the collision spike test and the
 * motion class. This task turns its events into the base signals:
 *   hump start  -> EVF_HUMP_START (TSK_NAV_PLANNER limits the speed)
 *   hump end    -> EVF_HUMP_END, run maximum (FR22, NORMAL episodes only)
 *   collision   -> EVF_COLLISION (TSK_SAFETY_MON: E-stop while driving)
 *   class       -> g_status.motion_event (telemetry)
 * The IS09 car has no INT1 wire, so the spike test is the only collision
 * source.
 *
 * Hump height (19 Nov): the base hump profiler (hump2.c) runs on the
 * same samples beside imu_core and only measures: HUMP2_START / HUMP2_PEAK /
 * HUMP2_END events with the base sum height and the tilt height, and the
 * "h2" telemetry object. It does not change the speed or the hump count.
 *
 * I2C faults (base behaviour, kept): a single failed read reuses the last
 * sample; 3 in a row -> bus recovery (9 SCL clocks + re-init) and the sample
 * is INVALID for imu_core (IS09 APP_CAL_IMU_FAULT_ERRORS); 3 failed
 * recoveries (or no IMU at start-up) -> FAULT_IMU_DEAD, the robot keeps
 * driving without hump / collision detection, re-tried every 2 s.
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "is09_glue.h"
#include "hump2.h"
#include "app_params.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"

#define CYCLE_MS         (10u)
#define MAX_RECOVERIES   (3u)
#define FAULT_ERRORS     (3u)        /* IS09 APP_CAL_IMU_FAULT_ERRORS */
#define RETRY_CYCLES     (2000u / CYCLE_MS)
#define MOVING_MM_S      (10.0f)     /* IS09: a wheel speed of 1 cm/s or more */

LOCAL bool imu_read(int16_t *p_x, int16_t *p_y, int16_t *p_z)
{
    bool b_ok;

    if (tk_loc_mtx(g_mtx_i2c, 5) < E_OK)
    {
        return false;
    }
    b_ok = hal_imu_accel_mg(p_x, p_y, p_z);
    (void)tk_unl_mtx(g_mtx_i2c);
    return b_ok;
}

LOCAL bool imu_recover(void)
{
    bool b_ok;

    if (tk_loc_mtx(g_mtx_i2c, 50) < E_OK)
    {
        return false;
    }
    hal_i2c_recover();
    b_ok = hal_imu_init();
    (void)tk_unl_mtx(g_mtx_i2c);
    return b_ok;
}

LOCAL float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* IS09 motion class -> the base telemetry name set */
LOCAL motion_event_t mev_of(uint8_t motion_class)
{
    switch (motion_class)
    {
        case IS09_CLASS_ACCELERATING: return MEV_ACCELERATING;
        case IS09_CLASS_TURNING:      return MEV_TURNING;
        case IS09_CLASS_CLIMBING:     return MEV_CLIMBING;
        case IS09_CLASS_DESCENDING:   return MEV_DESCENDING;
        case IS09_CLASS_IMPACT:       return MEV_IMPACT;
        case IS09_CLASS_CRUISING:     return MEV_CRUISING;
        default:                      return MEV_STATIONARY;
    }
}

LOCAL void hump_end_event(const is09_imu_out_t *p_out)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "IS09 method: ");
    jb_raw(&jb, is09_imu_quality_name(p_out->end_quality));
    jb_raw(&jb, ", height ");
    jb_uint(&jb, p_out->end_height_mm);
    jb_raw(&jb, " mm, run max ");
    jb_fixed(&jb, g_status.hump_max_mm, 0u);
    jb_raw(&jb, " mm");
    /* only a counted hump is an event (19 Nov): a rejected episode, e.g. the
     * jolt when the car stops, is only logged, so one hump shows once */
    if (p_out->end_quality == IS09_HUMP_Q_NORMAL)
    {
        (void)post_event("HUMP_END", text);
    }
    log_line(text);
}

/* the base hump profiler's events (measurement only) */
LOCAL void hump2_publish(const hump2_ev_t *p_ev)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    /* one event per whole hump, at its end (19 Nov); the card shows the live state */
    if (p_ev->b_end && p_ev->b_valid)
    {
        jb_raw(&jb, "sum ");
        jb_fixed(&jb, p_ev->height_mm, 1u);
        jb_raw(&jb, " mm, tilt ");
        jb_fixed(&jb, p_ev->tilt_mm, 1u);
        jb_raw(&jb, " mm, len ");
        jb_fixed(&jb, p_ev->len_mm, 0u);
        jb_raw(&jb, " mm (max ");
        jb_fixed(&jb, g_hump2.max_h_mm, 1u);
        jb_char(&jb, '/');
        jb_fixed(&jb, g_hump2.max_tilt_mm, 1u);
        jb_char(&jb, ')');
        (void)post_event("HUMP2_END", text);
        log_line(text);
    }
    else
    {
        /* nothing this cycle */
    }
}

LOCAL bool is_driving(sys_state_t st)
{
    return (st == SYS_LINE_FOLLOWING) || (st == SYS_OBSTACLE_AVOIDANCE) || (st == SYS_MANEUVER);
}

LOCAL void hump2_params(void)
{
    hump2_par_t *p = &g_hump2_par;

    p->start_deg     = g_params.hump2_start_deg;
    p->level_deg     = g_params.hump2_level_deg;
    p->min_height_mm = g_params.hump2_min_mm;
    p->min_climb_mm  = g_params.hump2_min_climb_mm;
    p->start_blank_ms = (uint32_t)g_params.hump2_start_blank_ms;
    p->max_len_mm    = HUMP2_MAX_LEN_MM;
    p->alpha         = g_params.hump2_alpha;
    p->wheelbase_mm  = g_params.hump_wheelbase_mm;
    p->start_samples = 3u;                    /* base HUMP_START_SAMPLES */
    p->end_samples   = 5u;                    /* base HUMP_END_SAMPLES */
    p->b_accel_comp  = (g_params.imu_accel_comp > 0.5f);
}

LOCAL void publish(const is09_imu_out_t *p_out)
{
    if (p_out->b_collision)
    {
        flag_set(EVF_COLLISION);              /* first: braking never waits */
        (void)tk_wup_tsk(g_tsk_safety);
        (void)post_event("COLLISION", "IS09 imu_core: forward acceleration spike");
    }
    if (p_out->b_hump_start)
    {
        flag_set(EVF_HUMP_START);             /* the speed limit; no event (see hump_end_event) */
        log_line("IS09 method: hump start");
    }
    if (p_out->b_hump_end)
    {
        if (p_out->end_quality == IS09_HUMP_Q_NORMAL)
        {
            g_status.humps++;
            g_status.hump_peak_mm = (float)p_out->end_height_mm;
            if ((float)p_out->end_height_mm > g_status.hump_max_mm)
            {
                g_status.hump_max_mm = (float)p_out->end_height_mm;   /* FR22 */
            }
        }
        flag_set(EVF_HUMP_END);
        hump_end_event(p_out);
    }
    if (p_out->b_class_changed)
    {
        (void)post_event("MOTION_CLASS", is09_imu_class_name(p_out->motion_class));
    }
    g_status.motion_event = (uint32_t)mev_of(p_out->motion_class);
    g_status.pitch_deg    = p_out->pitch_deg;
    g_status.hump_mm      = p_out->b_hump_active ? (float)p_out->height_mm : 0.0f;
    if (p_out->b_hump_active)
    {
        g_status.hump_peak_mm = (float)p_out->height_mm;
    }
}

void tsk_imu(INT stacd, void *exinf)
{
    odometry_t     odo;
    bool           b_odo_seen = false;
    float          dist_prev = 0.0f;
    float          dist_frac = 0.0f;
    int16_t        x = 0;
    int16_t        y = 0;
    int16_t        z = 1000;
    uint32_t       fails = 0u;
    uint32_t       recoveries = 0u;
    bool           b_dead = ((g_status.faults & FAULT_IMU_DEAD) != 0u);
    uint32_t       retry_n = 0u;
    uint32_t       odo_um_prev = 0u;
    bool           b_um_seen = false;
    sys_state_t    st_prev = SYS_IDLE;

    (void)stacd;
    (void)exinf;
    is09_imu_init();
    hump2_params();
    hump2_init(&g_hump2);

    for (;;)
    {
        is09_imu_in_t  in;
        is09_imu_out_t out;
        sys_state_t    st;
        float          dist;
        float          spd;

        (void)tk_slp_tsk(TMO_FEVR);          /* released every 10 ms by the cyclic tick */
        g_hb[HB_IMU]++;
        st = sys_state();
        odometry_get(&odo);

        /* the sensor */
        if (b_dead)
        {
            retry_n++;
            if (retry_n >= RETRY_CYCLES)       /* every 2 s */
            {
                retry_n = 0u;
                if (imu_recover())
                {
                    b_dead     = false;
                    recoveries = 0u;
                    fails      = 0u;
                    fault_clear(FAULT_IMU_DEAD | FAULT_IMU_I2C);
                    (void)post_event("IMU_OK", hal_imu_chip_name());
                    log_line("[imu] GY-511 answering: hump and collision detection on");
                }
            }
        }
        else if (imu_read(&x, &y, &z))
        {
            fails = 0u;
        }
        else
        {
            fails++;
            fault_set(FAULT_IMU_I2C);
            if (fails >= FAULT_ERRORS)
            {
                (void)post_event("FAULT_IMU_I2C", "I2C0 timeout: bus recovery");
                if (imu_recover())
                {
                    recoveries = 0u;
                    fails      = 0u;
                }
                else
                {
                    recoveries++;
                    if (recoveries >= MAX_RECOVERIES)
                    {
                        b_dead = true;
                        fault_set(FAULT_IMU_DEAD);
                        (void)post_event("FAULT_IMU", "3 failed recoveries: driving without the IMU");
                        log_line("[imu] GY-511 lost: driving without hump/collision detection");
                    }
                }
            }
        }

        /* the motion inputs IS09 imu_core takes from MotionStatus */
        dist = 0.5f * (odo.dist_r_mm + odo.dist_l_mm);        /* signed, forward + */
        if (!b_odo_seen)
        {
            b_odo_seen = true;
            dist_prev  = dist;
        }
        dist_frac += dist - dist_prev;                          /* keep the sub-mm part */
        dist_prev  = dist;
        if (dist_frac > 1000.0f)
        {
            dist_frac = 1000.0f;                                /* IS09 clamps a step to 1 m anyway */
        }
        if (dist_frac < -1000.0f)
        {
            dist_frac = -1000.0f;
        }
        in.dist_delta_mm = (int16_t)dist_frac;
        dist_frac       -= (float)in.dist_delta_mm;

        spd = 0.5f * (absf(g_status.spd_l) + absf(g_status.spd_r));
        in.x_mg           = x;
        in.y_mg           = y;
        in.z_mg           = z;
        in.b_sample_valid = !b_dead && (fails < FAULT_ERRORS);
        in.b_motion_fresh = true;            /* odometry: TSK_MOTION_CTRL, watched by the safety monitor */
        in.b_moving       = (absf(g_status.spd_l) >= MOVING_MM_S) || (absf(g_status.spd_r) >= MOVING_MM_S);
        in.b_turning      = (st == SYS_MANEUVER) || (absf(odo.turn_dps) > TURNING_DPS);
        in.speed_cm_s     = (uint8_t)((spd >= 2550.0f) ? 255u : (uint32_t)(spd / 10.0f));
        in.b_stop_latched = (st == SYS_ESTOP);
        in.a_comp_mg      = (g_params.imu_accel_comp > 0.5f) ? (int16_t)(odo.a_fwd_g * 1000.0f) : 0;

        is09_imu_set_thresholds(g_params.hump_on_deg, g_params.hump_off_deg);
        is09_imu_step(&in, &out);
        publish(&out);
        g_status.turn_rate_dps = odo.turn_dps;

        /* base hump profiler on the same sample (measurement only) */
        {
            hump2_in_t h2;
            hump2_ev_t ev;

            if (is_driving(st) && !is_driving(st_prev))
            {
                hump2_new_run(&g_hump2);   /* a new run: clear the last / max heights */
            }
            hump2_params();
            is09_imu_map_axes(x, y, z, &h2.fwd_mg, &h2.lat_mg, &h2.up_mg);
            h2.b_sample_valid = in.b_sample_valid;
            h2.a_fwd_g        = odo.a_fwd_g;
            h2.ds_mm          = (float)(odo.odo_um - odo_um_prev) / 1000.0f;   /* base: monotonic distance */
            h2.b_learn_offset = (st == SYS_IDLE) && (absf(odo.v_mm_s) < 5.0f);
            h2.b_driving      = is_driving(st);
            if (!b_um_seen)
            {
                b_um_seen = true;
                h2.ds_mm  = 0.0f;                 /* first cycle: no previous reading */
            }
            odo_um_prev = odo.odo_um;
            if (hump2_step(&g_hump2, &g_hump2_par, &h2, &ev))
            {
                hump2_publish(&ev);
            }
            st_prev = st;
        }
    }
}
