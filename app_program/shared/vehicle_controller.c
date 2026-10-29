/** @file vehicle_controller.c
 *  @brief VehicleControllerTask: kernel shell around mission_core (see
 *         header). Owns the button IRQs, the controller heartbeat and the
 *         only writer of g_mission_status.
 */
#include "vehicle_controller.h"

#include "../subsystem5/subsystem5.h"
#include "app_barcode.h"
#include "app_cal.h"
#include "app_gpio_irq.h"
#include "app_hw.h"
#include "app_ipc.h"
#include "app_line.h"
#include "app_obstacle.h"
#include "app_port.h"
#include "app_safety.h"
#include "app_telemetry.h"
#include "mission_core.h"

#define VC_REJECT_SEQ_SHIFT 8U

volatile uint8_t g_ctrl_heartbeat = 0U;

static int32_t  g_vc_tid       = 0;
static uint32_t g_vc_boot_ok   = 0U;
static bool     g_vc_boot_done = false;
static uint32_t g_vc_steps     = 0U;

static mission_core_t g_vc_core; /* controller-private, single writer */
static uint8_t        g_vc_remote_gen_seen = 0U;

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

/* ---- input gathering ----------------------------------------------------- */

static void
gather (uint32_t events, mission_in_t * p_in)
{
    uint32_t w0 = 0U;
    uint32_t w1 = 0U;

    p_in->events    = events;
    p_in->boot_done = g_vc_boot_done;
#if defined(APP_TEST) && APP_TEST
    /* TEST image: the self-tests drive the car themselves (S2-U06,
       INT-13..INT-20). A START (e.g. the remote START that S1-U08 injects)
       must never begin a mission run here, so missions see "not armed" and
       refuse it exactly as in a disarmed build (INT-04). The demo image is
       unchanged. */
    p_in->motion_armed = false;
#else
    p_in->motion_armed = (0 != APP_CAL_MOTION_ARMED);
#endif
    p_in->stop_latched  = app_stop_latched();
    p_in->stop_cause    = (uint8_t) app_stop_cause_get();
    p_in->faults        = app_fault_get();
    p_in->start_pressed = button_pressed(APP_PIN_START_BTN);
    p_in->stop_pressed  = button_pressed(APP_PIN_STOP_BTN);
    p_in->remote_start  = false;
    p_in->remote_stop   = false;
    if (0U != (events & APP_EVT_REMOTE_CMD))
    {
        uint8_t const          gen  = g_remote_cmd_gen;
        uint8_t const          byte = g_remote_cmd; /* one byte load */
        app_remote_cmd_t const cmd  = app_remote_cmd_type(byte);
        if (gen != g_vc_remote_gen_seen)
        {
            g_vc_remote_gen_seen = gen;
            p_in->remote_start   = (APP_REMOTE_CMD_START == cmd);
            p_in->remote_stop    = (APP_REMOTE_CMD_STOP == cmd);
        }
    }
    p_in->motion_valid = app_motion_read(&p_in->motion);
    app_line_read(&p_in->line);
    p_in->barcode_byte  = g_barcode_byte;
    p_in->profile_valid = s5_profile_copy(&p_in->profile, &w0, &w1);
}

/* ---- output dispatch ----------------------------------------------------- */

static void
act (mission_out_t const * p_out)
{
    uint32_t i = 0U;

    if ((uint8_t) APP_STOP_CAUSE_NONE != p_out->stop_request)
    {
        app_safe_stop_request((app_stop_cause_t) p_out->stop_request);
    }
    if (p_out->send_motion)
    {
        (void) vehicle_controller_send_motion(
            (app_motion_cmd_type_t) p_out->mc_type, p_out->mc_speed_cm_s,
            p_out->mc_param, p_out->mc_until_line, p_out->mc_seq);
    }
    if ((0U != p_out->scan_req) && (0 != app_ipc_flg_scan()))
    {
        (void) tk_set_flg((ID) app_ipc_flg_scan(), (UINT) p_out->scan_req);
    }
    if (p_out->clear_latch)
    {
        (void) app_safety_controller_clear(); /* refused unless we are the
                                                 bound controller: then the
                                                 latch stays and the next
                                                 step re-enters S7 */
    }
    if (p_out->mission_changed)
    {
        g_mission_status = mission_core_status(&g_vc_core); /* one store */
    }
    for (i = 0U; i < p_out->n_tev; i++)
    {
        app_ipc_telem_post((app_telem_event_t) p_out->tev[i].type,
                           APP_SRC_CONTROLLER, p_out->tev[i].payload);
    }
}

static void
vehicle_controller_task (INT stacd, void * p_exinf)
{
    static mission_in_t  in; /* large: keep off the task stack */
    static mission_out_t out;
    UINT                 ptn;
    ER                   er;

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
        if (0U != (ptn & APP_EVT_TICK))
        {
            g_ctrl_heartbeat++;
        }
        gather((uint32_t) ptn, &in);
        mission_core_step(&g_vc_core, &in, &out);
        act(&out);
        g_vc_steps++;
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
    mission_core_init(&g_vc_core);
    g_mission_status = mission_core_status(&g_vc_core);

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

uint32_t
vehicle_controller_step_count (void)
{
    return g_vc_steps;
}

int32_t
vehicle_controller_send_motion (app_motion_cmd_type_t type, uint8_t speed_cm_s,
                                int16_t param, bool until_line, uint8_t seq)
{
    app_motion_cmd_t cmd;
    uint32_t         record;
    int32_t const    mbf = app_ipc_mbf_motion();
    ER               er;

    if ((0 == mbf) || app_stop_latched())
    {
        return -1;
    }
    cmd.type       = (uint8_t) type;
    cmd.speed_cm_s = speed_cm_s;
    cmd.param      = param;
    cmd.until_line = until_line;
    cmd.seq        = seq;
    (void) app_motion_cmd_pack(&cmd, &record);
    er = tk_snd_mbf((ID) mbf, &record, (INT) APP_MC_RECORD_BYTES, TMO_POL);
    if (er < E_OK)
    {
        app_ipc_telem_post(APP_TEV_MOTION_REJECT, APP_SRC_CONTROLLER,
                           (uint16_t) ((uint16_t) seq << VC_REJECT_SEQ_SHIFT));
        return -1;
    }
    return 0;
}
