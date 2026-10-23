/** @file task_scan.c
 *  @brief ObstacleScanTask (priority 6) and EchoIsr (GP17, both edges via
 *         the shared dispatcher). Owner: Buddy 5.
 *
 *  Modes: IDLE, WATCH (forward ranging every 100 ms tick, EVT_OBSTACLE after
 *  two consecutive readings at/inside D_STOP), SIDE_L / SIDE_R (side ranging
 *  every tick, EVT_SIDE_UNSAFE after two readings inside D_SIDE_UNSAFE), and
 *  the synchronous PROFILE scan (coarse 15 deg sweep -60..+60, fine 5 deg
 *  window around the obstacle, profile under MTX_OBSTACLE, EVT_SCAN_DONE or
 *  EVT_SCAN_FAIL). Every wait is bounded; ABORT interrupts a scan between
 *  samples. With either hardware gate unsigned the task answers every
 *  request honestly (scan_state DISABLED, range sentinel, SCAN_FAIL) and
 *  never drives TRIG or the servo.
 */

#include "subsystem5.h"

#include "../shared/app_cal.h"
#include "../shared/app_gpio_irq.h"
#include "../shared/app_hw.h"
#include "../shared/app_ipc.h"
#include "../shared/app_port.h"
#include "../shared/app_safety.h"
#include "drv_servo.h"
#include "scan_core.h"

#define S5_TRIG_US 10U
#define S5_ECHO_WAIT_MS (APP_CAL_T_ECHO_TIMEOUT_MS + 5U)
#define S5_MTX_WAIT_MS 10
#define S5_SEQ_MASK 0x0FU
#define S5_ACTION_SHIFT 8U
#define S5_US_PER_MS 1000U
#define S5_REQ_BITS                                                            \
    (APP_SCAN_REQ_WATCH_ON | APP_SCAN_REQ_WATCH_OFF | APP_SCAN_REQ_PROFILE |   \
     APP_SCAN_REQ_SIDE_L | APP_SCAN_REQ_SIDE_R | APP_SCAN_REQ_IDLE |           \
     APP_SCAN_TICK | APP_SCAN_ABORT)

typedef enum
{
    S5_MODE_IDLE   = 0,
    S5_MODE_WATCH  = 1,
    S5_MODE_SIDE_L = 2,
    S5_MODE_SIDE_R = 3
} s5_mode_t;

static uint8_t           g_s5_mode      = (uint8_t) S5_MODE_IDLE;
static uint32_t          g_s5_servo_idx = SC_ANGLE_AHEAD;
static sc_watch_t        g_s5_watch;
static sc_samples_t      g_s5_samples;
static uint8_t           g_s5_side_count = 0U;
static uint8_t           g_s5_seq        = 0U;
static bool              g_s5_abort      = false;
static bool              g_s5_no_echo    = false;
static bool              g_s5_scan_fault = false;
static bool              g_s5_attached   = false;
static volatile uint32_t g_s5_echo_rise  = 0U; /* ISR-private */
static volatile uint32_t g_s5_watch_n    = 0U;
static volatile uint32_t g_s5_done_n     = 0U;
static volatile uint32_t g_s5_fail_n     = 0U;
static volatile uint32_t g_s5_last_ms    = 0U;

/* ---- EchoIsr (dispatcher callback, task-independent context) ------------- */
static void
echo_isr (uint32_t pin, uint32_t events, uint32_t stamp_us)
{
    (void) pin;
    if (0U != (events & APP_GPIO_IRQ_EDGE_RISE))
    {
        if (APP_ECHO_ARMED == g_echo_status)
        {
            g_s5_echo_rise = stamp_us;
            g_echo_status  = APP_ECHO_RISING;
        }
    }
    if (0U != (events & APP_GPIO_IRQ_EDGE_FALL))
    {
        if (APP_ECHO_RISING == g_echo_status)
        {
            uint32_t const w = (uint32_t) (stamp_us - g_s5_echo_rise);
            if (w > APP_ECHO_WIDTH_MAX_US)
            {
                g_echo_status = APP_ECHO_OVERRUN;
            }
            else
            {
                g_echo_width_us =
                    (uint16_t) w; /* halfword store before the status byte */
                g_echo_status = APP_ECHO_DONE;
            }
            (void) tk_set_flg(flg_scan,
                              (UINT) APP_SCAN_ECHO_DONE); /* ISR-legal */
        }
    }
}

/* ---- helpers ------------------------------------------------------------- */

bool
s5_scan_enabled (void)
{
    return (0 != APP_CAL_ECHO_SHIFTER_VERIFIED) && (0 != APP_CAL_SERVO_ARMED) &&
           g_s5_attached;
}

void
s5_echo_irq_enable (bool on)
{
    if (!g_s5_attached)
    {
        return;
    }
    if (on)
    {
        app_gpio_irq_enable(APP_PIN_US_ECHO);
    }
    else
    {
        app_gpio_irq_disable(APP_PIN_US_ECHO);
    }
}

/** Bounded wait for ABORT only (other request bits stay pending). */
static void
wait_abort_ms (uint32_t ms)
{
    UINT ptn = 0U;
    if (E_OK == tk_wai_flg(flg_scan, (UINT) APP_SCAN_ABORT,
                           TWF_ORW | TWF_BITCLR, &ptn, (TMO) ms))
    {
        g_s5_abort = true;
    }
}

static void
servo_move (uint32_t idx)
{
    uint32_t const settle = scan_core_settle_ms(g_s5_servo_idx, idx);
    drv_servo_set_us(scan_core_servo_us(idx));
    g_s5_servo_idx = idx;
    wait_abort_ms(settle);
}

/** One ranging: TRIG pulse, wait for EchoIsr or the 30 ms timeout. */
static uint16_t
range_once (void)
{
    UINT ptn = 0U;
    ER   er  = E_OK;

    if (!s5_scan_enabled())
    {
        return (uint16_t) APP_RANGE_CM_SENTINEL;
    }
    g_echo_status = APP_ECHO_ARMED; /* task is the only writer of this value */
    app_hw_gpio_write(APP_PIN_US_TRIG, true);
    app_hw_delay_us(S5_TRIG_US);
    app_hw_gpio_write(APP_PIN_US_TRIG, false);

    er = tk_wai_flg(flg_scan, (UINT) (APP_SCAN_ECHO_DONE | APP_SCAN_ABORT),
                    TWF_ORW | TWF_BITCLR, &ptn, (TMO) S5_ECHO_WAIT_MS);
    if ((E_OK == er) && (0U != (ptn & APP_SCAN_ABORT)))
    {
        g_s5_abort = true;
    }
    if ((E_OK == er) && (0U != (ptn & APP_SCAN_ECHO_DONE)) &&
        (APP_ECHO_DONE == g_echo_status))
    {
        g_echo_status = APP_ECHO_IDLE;
        return app_obstacle_us_to_cm(g_echo_width_us);
    }
    g_echo_status = APP_ECHO_IDLE; /* timeout, overrun or sequence error */
    return (uint16_t) APP_RANGE_CM_SENTINEL;
}

static void
publish_profile (app_obstacle_profile_t * p_view, uint8_t scan_state,
                 bool new_seq)
{
    uint32_t w0 = 0U;
    uint32_t w1 = 0U;

    if (new_seq)
    {
        g_s5_seq = (uint8_t) ((g_s5_seq + 1U) & S5_SEQ_MASK);
    }
    p_view->seq        = g_s5_seq;
    p_view->scan_state = scan_state;
    (void) app_obstacle_pack(p_view, &w0, &w1);
    if (E_OK == tk_loc_mtx(mtx_obstacle, (TMO) S5_MTX_WAIT_MS))
    {
        g_obstacle_profile[0] = w0;
        g_obstacle_profile[1] = w1;
        (void) tk_unl_mtx(mtx_obstacle);
    }
}

bool
s5_profile_copy (app_obstacle_profile_t * p_view, uint32_t * p_w0,
                 uint32_t * p_w1)
{
    uint32_t w0 = 0U;
    uint32_t w1 = 0U;

    if (E_OK != tk_loc_mtx(mtx_obstacle, (TMO) S5_MTX_WAIT_MS))
    {
        return false;
    }
    w0 = g_obstacle_profile[0];
    w1 = g_obstacle_profile[1];
    (void) tk_unl_mtx(mtx_obstacle);
    app_obstacle_unpack(w0, w1, p_view);
    *p_w0 = w0;
    *p_w1 = w1;
    return true;
}

/** Keep the forward range / closest fields fresh without a full scan. */
static void
publish_range (uint16_t range_cm, uint8_t scan_state)
{
    app_obstacle_profile_t v;
    uint32_t               w0 = 0U;
    uint32_t               w1 = 0U;

    if (!s5_profile_copy(&v, &w0, &w1))
    {
        return;
    }
    v.range_cm = range_cm;
    publish_profile(&v, scan_state, false);
}

static void
set_no_echo (bool fault)
{
    if (fault && !g_s5_no_echo)
    {
        g_s5_no_echo = true;
        app_fault_set(APP_FAULT_NO_ECHO);
    }
    else if (!fault && g_s5_no_echo)
    {
        g_s5_no_echo = false;
        app_fault_clr(APP_FAULT_NO_ECHO);
    }
    else
    {
        /* unchanged */
    }
}

/* ---- modes --------------------------------------------------------------- */

static void
watch_tick (void)
{
    bool           no_echo = false;
    uint16_t const r       = range_once();
    bool const     raise   = scan_core_watch_step(&g_s5_watch, r, &no_echo);

    g_s5_watch_n++;
    set_no_echo(s5_scan_enabled() && no_echo);
    publish_range(r, s5_scan_enabled() ? (uint8_t) SC_STATE_WATCH
                                       : (uint8_t) SC_STATE_DISABLED);
    if (raise)
    {
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_OBSTACLE);
        app_ipc_telem_post(APP_TEV_OBSTACLE, APP_SRC_SCAN, r);
    }
}

static void
side_tick (void)
{
    uint16_t const r = range_once();

    publish_range(r, s5_scan_enabled() ? (uint8_t) SC_STATE_SIDE
                                       : (uint8_t) SC_STATE_DISABLED);
    if (scan_core_side_step(&g_s5_side_count, r))
    {
        g_s5_side_count = 0U;
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_SIDE_UNSAFE);
        app_ipc_telem_post(APP_TEV_OBSTACLE, APP_SRC_SCAN, r);
    }
}

static bool
scan_plan (uint8_t const * p_idx, uint32_t n, uint32_t t0)
{
    uint32_t i = 0U;
    for (i = 0U; i < n; i++)
    {
        servo_move(p_idx[i]);
        if (g_s5_abort)
        {
            return false;
        }
        scan_core_samples_set(&g_s5_samples, p_idx[i], range_once());
        if (g_s5_abort || (((uint32_t) (app_hw_time_us() - t0)) >
                           (APP_CAL_T_SCAN_TIMEOUT_MS * S5_US_PER_MS)))
        {
            return false;
        }
    }
    return true;
}

static void
profile_scan (void)
{
    uint32_t const         t0 = app_hw_time_us();
    uint8_t                plan[SC_ANGLE_COUNT];
    uint32_t               n  = 0U;
    bool                   ok = s5_scan_enabled();
    app_obstacle_profile_t v;

    g_s5_abort = false;
    scan_core_samples_init(&g_s5_samples);
    if (ok)
    {
        n  = scan_core_coarse_plan(plan, SC_ANGLE_COUNT);
        ok = scan_plan(plan, n, t0);
    }
    if (ok)
    {
        n  = scan_core_fine_plan(&g_s5_samples, plan, SC_ANGLE_COUNT);
        ok = scan_plan(plan, n, t0);
    }
    if (ok)
    {
        servo_move(SC_ANGLE_AHEAD);
    }
    g_s5_last_ms = ((uint32_t) (app_hw_time_us() - t0)) / S5_US_PER_MS;

    scan_core_build_profile(&g_s5_samples,
                            g_s5_samples.range_cm[SC_ANGLE_AHEAD], &v);
    if (ok)
    {
        g_s5_done_n++;
        if (g_s5_scan_fault)
        {
            g_s5_scan_fault = false;
            app_fault_clr(APP_FAULT_SCAN_TIMEOUT);
        }
        publish_profile(&v, (uint8_t) SC_STATE_FINE, true);
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_SCAN_DONE);
        app_ipc_telem_post(
            APP_TEV_SCAN_DONE, APP_SRC_SCAN,
            (uint16_t) (((uint16_t) v.action << S5_ACTION_SHIFT) | v.width_cm));
    }
    else
    {
        g_s5_fail_n++;
        v.action = (uint8_t)
            APP_AVOID_STOP; /* an unfinished scan never recommends a move */
        if (s5_scan_enabled() && !g_s5_abort && !g_s5_scan_fault)
        {
            g_s5_scan_fault = true;
            app_fault_set(APP_FAULT_SCAN_TIMEOUT); /* controller decides the
                                                      stop (Rev B) */
        }
        publish_profile(&v,
                        s5_scan_enabled() ? (uint8_t) SC_STATE_FAIL
                                          : (uint8_t) SC_STATE_DISABLED,
                        true);
        if (!g_s5_abort)
        {
            (void) tk_set_flg((ID) app_ipc_flg_ctrl(),
                              (UINT) APP_EVT_SCAN_FAIL);
        }
    }
}

/* ---- task ---------------------------------------------------------------- */

bool
s5_scan_init_hw (void)
{
    drv_servo_init(scan_core_servo_us(SC_ANGLE_AHEAD));
    app_hw_gpio_output(
        APP_PIN_US_TRIG,
        false); /* TRIG idles low; never pulsed while unverified */
    app_hw_gpio_input(APP_PIN_US_ECHO, APP_GPIO_PULL_NONE);
    g_s5_attached =
        app_gpio_irq_attach(APP_PIN_US_ECHO, APP_GPIO_IRQ_EDGE_BOTH, echo_isr);
    if (g_s5_attached)
    {
        app_gpio_irq_disable(APP_PIN_US_ECHO);
        if (s5_scan_enabled())
        {
            app_gpio_irq_enable(APP_PIN_US_ECHO);
        }
    }
    scan_core_watch_init(&g_s5_watch);
    return g_s5_attached;
}

void
s5_scan_task (INT stacd, void * p_exinf)
{
    UINT ptn = 0U;
    ER   er  = E_OK;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        er = tk_wai_flg(flg_scan, (UINT) S5_REQ_BITS, TWF_ORW | TWF_BITCLR,
                        &ptn, TMO_FEVR);
        if (er < E_OK)
        {
            continue;
        }
        if (0U != (ptn & (APP_SCAN_ABORT | APP_SCAN_REQ_IDLE)))
        {
            g_s5_mode = (uint8_t) S5_MODE_IDLE;
            continue; /* the controller re-requests what it needs */
        }
        if (0U != (ptn & APP_SCAN_REQ_WATCH_OFF))
        {
            if ((uint8_t) S5_MODE_WATCH == g_s5_mode)
            {
                g_s5_mode = (uint8_t) S5_MODE_IDLE;
            }
        }
        if (0U != (ptn & APP_SCAN_REQ_WATCH_ON))
        {
            g_s5_mode = (uint8_t) S5_MODE_WATCH;
            scan_core_watch_init(&g_s5_watch);
            servo_move(SC_ANGLE_AHEAD);
        }
        if (0U != (ptn & APP_SCAN_REQ_SIDE_L))
        {
            g_s5_mode       = (uint8_t) S5_MODE_SIDE_L;
            g_s5_side_count = 0U;
            servo_move(SC_ANGLE_LEFT);
        }
        if (0U != (ptn & APP_SCAN_REQ_SIDE_R))
        {
            g_s5_mode       = (uint8_t) S5_MODE_SIDE_R;
            g_s5_side_count = 0U;
            servo_move(SC_ANGLE_RIGHT);
        }
        if (0U != (ptn & APP_SCAN_REQ_PROFILE))
        {
            g_s5_mode = (uint8_t) S5_MODE_IDLE; /* forward watch stops while the
                                                   car is braked and scanning */
            profile_scan();
            continue;
        }
        if (0U != (ptn & APP_SCAN_TICK))
        {
            if ((uint8_t) S5_MODE_WATCH == g_s5_mode)
            {
                watch_tick();
            }
            else if (((uint8_t) S5_MODE_SIDE_L == g_s5_mode) ||
                     ((uint8_t) S5_MODE_SIDE_R == g_s5_mode))
            {
                side_tick();
            }
            else
            {
                /* idle: no ranging, TRIG stays low */
            }
        }
    }
}

/* ---- diagnostics -----------------------------------------------------------
 */

uint32_t
s5_watch_count (void)
{
    return g_s5_watch_n;
}

uint32_t
s5_scan_done_count (void)
{
    return g_s5_done_n;
}

uint32_t
s5_scan_fail_count (void)
{
    return g_s5_fail_n;
}

uint32_t
s5_last_scan_ms (void)
{
    return g_s5_last_ms;
}

uint8_t
s5_mode (void)
{
    return g_s5_mode;
}
