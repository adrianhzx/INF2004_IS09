/** @file app.h
 *  @brief Definitions shared by the whole team. Changed only by team
 *         agreement (TEAM.md). Anything that belongs to ONE subsystem goes in
 *         that subsystem's own header (app_program/subsystemN/subsystemN.h).
 *
 *  IS09 Rev B: the shared data layer lives in app_program/shared/
 *  (app_data.h is the single authority for packed fields, app_safety.h the
 *  STOP_LATCH contract, app_ipc.h the kernel-object catalogue).
 *
 *  Pins: free for projects are GP0..GP22 and GP26..GP28. GP0/GP1 carry the
 *  LEFT ENCODER (Rev B) and are free only because the demo profile builds
 *  with CRASH_REPORT=0 (BOARD_UART0_FREE=1). Never build CRASH_REPORT=1 or
 *  CONSOLE=uart with the encoder wired. GP23/24/25/29 belong to the radio.
 *  Every pin is claimed in TEAM.md section 2 and app_program/shared/app_hw.h.
 */

#ifndef APP_H
#define APP_H

/* Order matters: tkernel.h defines TARGET_DIR used by bsp/libbsp.h. */
#include <tk/tkernel.h>  /* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h> /* tm_printf() console output */
#include <bsp/libbsp.h>  /* board helpers (kernel BSP) */

/* Default stack for tasks started through app_start_task(); the Rev B tasks
   use the per-task sizes in shared/app_ipc.h through app_start_task_stack(). */
#define TASK_STACK_SIZE 2048

/** Create and start a task. prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0
 *  for "any core" (ignored on SMP=0). Defined in app_main.c. */
IMPORT ID app_start_task (FP entry, PRI pri, UW prc, const char * name);

/** Same, with an explicit stack size in bytes. */
IMPORT ID app_start_task_stack (FP entry, PRI pri, SZ stksz, const char * name);

/* Every subsystem provides these. usermain() calls ALL the _init() functions
   first and only then ALL the _start() functions, so every kernel object
   exists before any task can use it. */
IMPORT ER subsystem1_init (void); /* Buddy 1: Wi-Fi/MQTT telemetry */
IMPORT ER subsystem2_init (void); /* Buddy 2: motion + encoders */
IMPORT ER subsystem3_init (void); /* Buddy 3: line sensing + barcode */
IMPORT ER subsystem4_init (void); /* Buddy 4: IMU / hump / collision */
IMPORT ER subsystem5_init (void); /* Buddy 5: ultrasonic + servo scan */

IMPORT ER subsystem1_start (void);
IMPORT ER subsystem2_start (void);
IMPORT ER subsystem3_start (void);
IMPORT ER subsystem4_start (void);
IMPORT ER subsystem5_start (void);

#if APP_TEST
IMPORT void subsystem1_test (void);
IMPORT void subsystem2_test (void);
IMPORT void subsystem3_test (void);
IMPORT void subsystem4_test (void);
IMPORT void subsystem5_test (void);
IMPORT void app_shared_test (void); /* shared foundation (app_program/shared) */
#endif

/** Print a readable message if a kernel call failed (negative error code). */
#define CHECK(er, what)                                                        \
    do                                                                         \
    {                                                                          \
        INT check_er_ = (INT) (er); /* evaluate once */                        \
        if (check_er_ < E_OK)                                                  \
        {                                                                      \
            tm_printf((UB *) "ERROR %s: %d\n", (what), check_er_);             \
        }                                                                      \
    } while (0)

#endif /* APP_H */
