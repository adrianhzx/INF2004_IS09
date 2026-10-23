/** @file app_main.c
 *  @brief START HERE. usermain() is the program's entry point.
 *
 *  How micro T-Kernel starts the code:
 *    reset -> kernel initialises itself -> creates the "initial task"
 *          -> initial task calls usermain()
 *  usermain() runs in that initial task at priority 1 (the highest). On a
 *  single core nothing else runs until it blocks; on the dual-core build the
 *  OTHER core may start a new task at once. So usermain() works in phases:
 *    0. shared hardware bring-up and self-check (app_hw_init)
 *    1. shared kernel objects (FLG_CTRL, CYC_10MS), every subsystem's
 *       kernel objects (subsystemN_init), controller objects
 *    2. every subsystem starts its tasks (subsystemN_start), then the
 *       controller, then the 10 ms cyclic handler is started LAST so no
 *       task is woken before it exists.
 *  If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *  Safety: motion is braked by default. Nothing moves until the controller
 *  is in S1, the calibration is signed off (APP_CAL_MOTION_ARMED) and the
 *  operator presses START (Rev B section 5).
 */

#include "app.h"

#include "shared/app_cal.h"
#include "shared/app_gpio_irq.h"
#include "shared/app_hw.h"
#include "shared/app_ipc.h"
#include "shared/app_safety.h"
#include "shared/vehicle_controller.h"

EXPORT ID
app_start_task_stack (FP entry, PRI pri, SZ stksz, const char * name)
{
    T_CTSK ctsk;
    ID     tid;
    ER     er;

    ctsk.exinf   = NULL;
    ctsk.task    = entry;
    ctsk.itskpri = pri;
    ctsk.stksz   = stksz;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    ctsk.assprc = 0;
#endif
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, name);
        return tid;
    }
    er = tk_sta_tsk(tid, 0);
    CHECK(er, name);
    return (er < E_OK) ? (ID) er : tid;
}

EXPORT ID
app_start_task (FP entry, PRI pri, UW prc, const char * name)
{
    (void) prc; /* Rev B: single-core schedule; no affinity requested */
    return app_start_task_stack(entry, pri, (SZ) TASK_STACK_SIZE, name);
}

#if APP_TEST && defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
IMPORT UW tm_usb_state (void); /* lib/libtm usb_console.c: 3 = host has DTR */
#define APP_TEST_CONSOLE_WAIT_MS 15000

/* TEST image only: wait (bounded) until a serial terminal is open, so no
   [TEST] line is printed before anyone is listening. */
static void
app_test_wait_console (void)
{
    INT waited = 0;

    while ((tm_usb_state() < 3U) && (waited < APP_TEST_CONSOLE_WAIT_MS))
    {
        (void) tk_dly_tsk(100);
        waited += 100;
    }
    (void) tk_dly_tsk(300); /* let the terminal settle */
}
#endif

EXPORT INT
usermain (void)
{
    T_RVER   ver;
    uint32_t hw_ok;

#if APP_TEST && defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC
    app_test_wait_console();
#endif
    tk_ref_ver(&ver);
    tm_printf((UB *) "\n=== IS09 autonomous car, micro T-Kernel on Pico W"
                     " (spec 0x%04x, product 0x%04x) ===\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf(
        (UB *) "dual-core (SMP) build - NOT the validated demo profile\n");
#else
    tm_printf((UB *) "single-core build (demo profile)\n");
#endif

    /* Phase 0: shared hardware bring-up and the safety foundation. */
    hw_ok = app_hw_init();
    tm_printf((UB *) "hw self-check: 0x%x (timer %d, pwm %d, io %d)\n", hw_ok,
              (hw_ok & APP_HW_OK_TIMER_RUNS) ? 1 : 0,
              (hw_ok & APP_HW_OK_PWM_RELEASED) ? 1 : 0,
              (hw_ok & APP_HW_OK_IO_RELEASED) ? 1 : 0);
    app_safety_init();
    CHECK(app_ipc_init(), "app_ipc_init");
    CHECK(app_gpio_irq_init(), "app_gpio_irq_init");

    /* Phase 1: every subsystem creates its kernel objects. Motion (2) first
       so MBF_MOTION_CMD exists before the controller; telemetry (1) so the
       event buffer exists before anyone posts. */
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(subsystem5_init(), "subsystem5_init");
    CHECK(vehicle_controller_init(), "vehicle_controller_init");
    vehicle_controller_set_boot_check(
        hw_ok); /* latches CRITICAL_RUNTIME on failure */

    /* Phase 2: start tasks, controller, then the cyclic handler last. */
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(subsystem5_start(), "subsystem5_start");
    CHECK(subsystem1_start(), "subsystem1_start");
    CHECK(vehicle_controller_start(), "vehicle_controller_start");
    CHECK(app_ipc_start(), "app_ipc_start");

    tm_printf(
        (UB *) "all subsystems started; motion armed=%d, echo verified=%d\n",
        APP_CAL_MOTION_ARMED, APP_CAL_ECHO_SHIFTER_VERIFIED);

#if APP_TEST
    /* Test build (make TEST=1): run every self-test in turn, here in the
       initial task, while the system is running. */
    tm_printf((UB *) "[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    app_shared_test(); /* last: SH-U08 leaves the system latched in S7 */
    tm_printf((UB *) "[TEST] === test run end ===\n");
#endif

    tk_slp_tsk(TMO_FEVR); /* returning would shut the system down */
    return 0;
}
