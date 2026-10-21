/** @file vehicle_controller.c
 *  @brief VehicleControllerTask implementation (Phase 1 scope, see header).
 */

#include "vehicle_controller.h"

#include "app_cal.h"
#include "app_gpio_irq.h"
#include "app_hw.h"
#include "app_ipc.h"
#include "app_port.h"
#include "app_safety.h"
#include "app_telemetry.h"

#define VC_TICK_MS 20U
#define VC_MS_TO_TICKS(ms) (((ms) + VC_TICK_MS - 1U) / VC_TICK_MS)

volatile uint8_t g_ctrl_heartbeat = 0U;

static int32_t  g_vc_tid       = 0;
static uint32_t g_vc_boot_ok   = 0U;
static bool     g_vc_boot_done = false;

/* Controller-private mission data (single writer of g_mission_status). */
static app_mission_status_t g_vc_ms;
static uint8_t              g_vc_motion_seq = 0U;

/* Button debounce state (sampled every tick, ISR only wakes us). */
static uint32_t g_vc_start_held_ticks = 0U;
static bool     g_vc_start_armed_edge = false; /* one START per press */
static uint32_t g_vc_s7_hold_ticks    = 0U;

static bool
button_pressed (uint32_t pin)
{
    bool const level = app_hw_gpio_read(pin);
#if APP_CAL_BUTTON_ACTIVE_LOW
    return !level;
#else
    return level;
#endif
}

static void
publish_mission (void)
{
    g_vc_ms.seq      = (uint8_t) ((g_vc_ms.seq + 1U) & 0x0FU);
    g_mission_status = app_mission_pack(&g_vc_ms); /* one aligned store */
}

static void
set_state (app_sys_state_t new_state)
{
    if (g_vc_ms.sys_state != (uint8_t) new_state)
    {
        g_vc_ms.sys_state = (uint8_t) new_state;
        publish_mission();
        app_ipc_telem_post(APP_TEV_SYS_STATE, APP_SRC_CONTROLLER,
                           (uint16_t) new_state);
    }
}

/* ---- ISRs (task-independent, bounded) ---------------------------------- */

static void
button_isr (uint32_t pin, uint32_t events, uint32_t stamp_us)
{
    (void) stamp_us;
    if (APP_PIN_STOP_BTN == pin)
    {
        /* STOP acts on the pressing edge directly from the ISR: the brake
           path must not wait for the 20 ms tick (FR24, NFR1). */
#if APP_CAL_BUTTON_ACTIVE_LOW
        bool const press = (0U != (events & APP_GPIO_IRQ_EDGE_FALL));
#else
        bool const press = (0U != (events & APP_GPIO_IRQ_EDGE_RISE));
#endif
        if (press)
        {
            app_safe_stop_request(APP_STOP_CAUSE_OPERATOR_STOP);
        }
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_STOP_BTN);
    }
    else
    {
        (void) tk_set_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_START_BTN);
    }
}

/* ---- recovery guard ----------------------------------------------------- */

static bool
initiating_condition_cleared (app_stop_cause_t cause)
{
    uint16_t const faults = app_fault_get();
    switch (cause)
    {
        case APP_STOP_CAUSE_OPERATOR_STOP:
            return !button_pressed(APP_PIN_STOP_BTN);
        case APP_STOP_CAUSE_COLLISION:
            return 0U == (faults & APP_FAULT_IMU_FAULT);
        case APP_STOP_CAUSE_STALL:
            return 0U ==
                   (faults & (APP_FAULT_MOTOR_STALL | APP_FAULT_ENCODER_FAULT));
        case APP_STOP_CAUSE_STALE_LINE:
            return 0U == (faults & APP_FAULT_STALE_LINE);
        case APP_STOP_CAUSE_STALE_MOTION:
            return 0U == (faults & APP_FAULT_STALE_MOTION);
        case APP_STOP_CAUSE_CTRL_HEARTBEAT:
            return 0U == (faults & APP_FAULT_CTRL_HEARTBEAT);
        case APP_STOP_CAUSE_SCAN_TIMEOUT:
            return 0U == (faults & APP_FAULT_SCAN_TIMEOUT);
        case APP_STOP_CAUSE_NO_CLEARANCE: /* operator must move the car */
        case APP_STOP_CAUSE_LINE_SEARCH_TIMEOUT:
            return true;
        case APP_STOP_CAUSE_CRITICAL_RUNTIME: /* needs a reboot */
        case APP_STOP_CAUSE_REMOTE_STOP:
        case APP_STOP_CAUSE_NONE:
        case APP_STOP_CAUSE_COUNT:
        default:
            return false;
    }
}

static void
s7_tick (void)
{
    app_motion_status_t motion;
    bool const          coherent   = app_motion_read(&motion);
    bool const          speed_zero = coherent && (0U == motion.speed_l_cm_s) &&
                            (0U == motion.speed_r_cm_s) &&
                            ((APP_MOTION_STOP_LATCHED == motion.motion_state) ||
                             (APP_MOTION_BRAKED == motion.motion_state));

    if (button_pressed(APP_PIN_START_BTN))
    {
        g_vc_s7_hold_ticks++;
    }
    else
    {
        g_vc_s7_hold_ticks = 0U;
    }

    if (speed_zero && initiating_condition_cleared(app_stop_cause_get()) &&
        (g_vc_s7_hold_ticks >= VC_MS_TO_TICKS(APP_CAL_T_HOLD_MS)))
    {
        if (app_safety_controller_clear())
        {
            g_vc_s7_hold_ticks    = 0U;
            g_vc_start_armed_edge = false; /* the held START is consumed */
            g_vc_ms.pending_cmd   = APP_NAV_NONE;
            g_vc_ms.retry_count   = 0U;
            set_state(APP_SYS_S0_BOOT_BRAKED);
        }
    }
}

/* ---- START handling ------------------------------------------------------ */

static void
try_start_run (void)
{
#if APP_CAL_MOTION_ARMED
    if (vehicle_controller_send_motion(APP_MC_FOLLOW, APP_CAL_V_NOM_CM_S, 0,
                                       false) >= 0)
    {
        set_state(APP_SYS_S2_LINE_FOLLOWING); /* Phase 7 fills S2 behaviour */
    }
#else
    /* Fail closed: motion calibration not signed off (app_cal.h, HV-3/HV-4). */
    app_ipc_telem_post(APP_TEV_CMD_REJECTED, APP_SRC_CONTROLLER, 0x0001U);
#endif
}

static void
tick (void)
{
    g_ctrl_heartbeat++;

    /* Belt and braces: the latch can only be set by app_safe_stop_request,
       which also raises EVT_SAFE_STOP, but never let S7 be missed. */
    if (app_stop_latched() &&
        (APP_SYS_S7_SAFE_STOP != (app_sys_state_t) g_vc_ms.sys_state))
    {
        g_vc_s7_hold_ticks = 0U;
        set_state(APP_SYS_S7_SAFE_STOP);
    }

    /* START debounce: pressed for >= T_START_DEBOUNCE, acted on once per press.
     */
    if (button_pressed(APP_PIN_START_BTN))
    {
        if (g_vc_start_held_ticks < 0xFFFFU)
        {
            g_vc_start_held_ticks++;
        }
    }
    else
    {
        g_vc_start_held_ticks = 0U;
        g_vc_start_armed_edge = true;
    }

    switch ((app_sys_state_t) g_vc_ms.sys_state)
    {
        case APP_SYS_S0_BOOT_BRAKED:
            if (g_vc_boot_done && !app_stop_latched())
            {
                app_motion_status_t motion;
                (void) app_motion_read(&motion);
                if (APP_MOTION_BRAKED == motion.motion_state)
                {
                    set_state(APP_SYS_S1_IDLE);
                }
            }
            break;
        case APP_SYS_S1_IDLE:
            if (g_vc_start_armed_edge &&
                (g_vc_start_held_ticks >=
                 VC_MS_TO_TICKS(APP_CAL_T_START_DEBOUNCE_MS)))
            {
                g_vc_start_armed_edge = false;
                try_start_run();
            }
            break;
        case APP_SYS_S7_SAFE_STOP:
            s7_tick();
            break;
        case APP_SYS_S2_LINE_FOLLOWING:
        case APP_SYS_S3_EXECUTE_NAV:
        case APP_SYS_S4_OBSTACLE_BYPASS:
        case APP_SYS_S5_LINE_SEARCH:
        case APP_SYS_S6_RUN_COMPLETE:
        default:
            /* NOT IMPLEMENTED (Phase 7). Unreachable while APP_CAL_MOTION_ARMED
               is 0: try_start_run() never leaves S1. */
            break;
    }
}

static void
remote_cmd (void)
{
    uint8_t const          byte = g_remote_cmd; /* one byte load */
    app_remote_cmd_t const cmd  = app_remote_cmd_type(byte);

    if (APP_REMOTE_CMD_STOP == cmd)
    {
        /* D-01 default: validated remote STOP on the controller event path. */
        app_safe_stop_request(APP_STOP_CAUSE_OPERATOR_STOP);
    }
    else if ((APP_REMOTE_CMD_START == cmd) &&
             (APP_SYS_S1_IDLE == (app_sys_state_t) g_vc_ms.sys_state))
    {
        try_start_run();
    }
    else
    {
        app_ipc_telem_post(
            APP_TEV_CMD_REJECTED, APP_SRC_CONTROLLER,
            (uint16_t) (0x0100U | (uint16_t) APP_CMD_START_STATE));
    }
}

static void
vehicle_controller_task (INT stacd, void * p_exinf)
{
    UINT ptn;
    ER   er;

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        er = tk_wai_flg((ID) app_ipc_flg_ctrl(), (UINT) APP_EVT_CTRL_ALL,
                        TWF_ORW | TWF_BITCLR, &ptn, TMO_FEVR);
        if (er < E_OK)
        {
            continue;
        }
        if (0U != (ptn & APP_EVT_SAFE_STOP))
        {
            g_vc_s7_hold_ticks = 0U;
            set_state(APP_SYS_S7_SAFE_STOP);
        }
        if (0U != (ptn & APP_EVT_REMOTE_CMD))
        {
            remote_cmd();
        }
        if (0U != (ptn & APP_EVT_TICK))
        {
            tick();
        }
        /* EVT_LINE_*, EVT_JUNCTION, EVT_BARCODE, EVT_MOTION_*, EVT_HUMP_*,
           EVT_OBSTACLE, EVT_SCAN_*: consumed in Phase 7. Bits are cleared by
           TWF_BITCLR so nothing is left pending from a previous run. */
    }
}

/* ---- public ------------------------------------------------------------- */

void
vehicle_controller_set_boot_check (uint32_t hw_ok_bits)
{
    g_vc_boot_ok = hw_ok_bits;
    g_vc_boot_done =
        (APP_HW_OK_TIMER_RUNS == (hw_ok_bits & APP_HW_OK_TIMER_RUNS)) &&
        (APP_HW_OK_PWM_RELEASED == (hw_ok_bits & APP_HW_OK_PWM_RELEASED));
    if (!g_vc_boot_done)
    {
        /* Fail closed: a dead timer or PWM block means no safe motion. */
        app_safe_stop_request(APP_STOP_CAUSE_CRITICAL_RUNTIME);
    }
    app_ipc_telem_post(APP_TEV_BOOT, APP_SRC_CONTROLLER, (uint16_t) hw_ok_bits);
}

int32_t
vehicle_controller_init (void)
{
    g_vc_ms.sys_state   = (uint8_t) APP_SYS_S0_BOOT_BRAKED;
    g_vc_ms.pending_cmd = (uint8_t) APP_NAV_NONE;
    g_vc_ms.retry_count = 0U;
    g_vc_ms.seq         = 0U;
    g_mission_status    = app_mission_pack(&g_vc_ms);

    app_hw_gpio_input(APP_PIN_START_BTN, APP_CAL_BUTTON_ACTIVE_LOW
                                             ? APP_GPIO_PULL_UP
                                             : APP_GPIO_PULL_DOWN);
    app_hw_gpio_input(APP_PIN_STOP_BTN, APP_CAL_BUTTON_ACTIVE_LOW
                                            ? APP_GPIO_PULL_UP
                                            : APP_GPIO_PULL_DOWN);
    if (!app_gpio_irq_attach(APP_PIN_STOP_BTN, APP_GPIO_IRQ_EDGE_BOTH,
                             button_isr))
    {
        return E_OBJ;
    }
    if (!app_gpio_irq_attach(APP_PIN_START_BTN, APP_GPIO_IRQ_EDGE_BOTH,
                             button_isr))
    {
        return E_OBJ;
    }
    return E_OK;
}

int32_t
vehicle_controller_start (void)
{
    T_CTSK ctsk;
    ID     tid;
    ER     er;

    ctsk.exinf   = NULL;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = (FP) vehicle_controller_task;
    ctsk.itskpri = APP_PRI_CONTROLLER;
    ctsk.stksz   = (SZ) APP_STACK_CONTROLLER;
    tid          = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        return (int32_t) tid;
    }
    g_vc_tid = (int32_t) tid;
    app_safety_bind(g_vc_tid, app_ipc_flg_ctrl(), APP_EVT_SAFE_STOP);
    er = tk_sta_tsk(tid, 0);
    return (int32_t) er;
}

app_sys_state_t
vehicle_controller_state (void)
{
    app_mission_status_t view;
    app_mission_unpack(g_mission_status, &view);
    return (app_sys_state_t) view.sys_state;
}

int32_t
vehicle_controller_send_motion (app_motion_cmd_type_t type, uint8_t speed_cm_s,
                                int16_t param, bool until_line)
{
    app_motion_cmd_t cmd;
    uint32_t         record;
    int32_t const    mbf = app_ipc_mbf_motion();
    ER               er;

    if ((0 == mbf) || app_stop_latched())
    {
        return -1;
    }
    g_vc_motion_seq = (uint8_t) ((g_vc_motion_seq + 1U) & 0x0FU);
    cmd.type        = (uint8_t) type;
    cmd.speed_cm_s  = speed_cm_s;
    cmd.param       = param;
    cmd.until_line  = until_line;
    cmd.seq         = g_vc_motion_seq;
    (void) app_motion_cmd_pack(&cmd, &record);
    er = tk_snd_mbf((ID) mbf, &record, (INT) APP_MC_RECORD_BYTES, TMO_POL);
    if (er < E_OK)
    {
        app_ipc_telem_post(APP_TEV_MOTION_REJECT, APP_SRC_CONTROLLER,
                           (uint16_t) ((uint16_t) g_vc_motion_seq << 8));
        return -1;
    }
    return (int32_t) g_vc_motion_seq;
}
