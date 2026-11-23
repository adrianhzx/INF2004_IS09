/**
 * @file    app_main.c
 * @brief   Robo-Wheeler on micro T-Kernel 3.0 (template:
 *          sirfonzie/mtk3smp-rp2040). usermain() = DR INITIALISING state.
 *
 * Order: kernel objects -> drivers (motors braked first) -> IMU self-test ->
 * interrupts and the 10 ms tick -> the seven DR tasks. The CYW43 radio
 * service and the USB console are started by the template before usermain().
 * TSK_SAFETY_MON then moves INITIALISING -> IDLE, or -> ERROR if a
 * calibration value is missing (listed on the USB console).
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "os_glue.h"
#include "app_params.h"
#include "hal.h"
#include "is09_glue.h"
#include "nav_map.h"
#include "robot_config.h"
#include "usb_console_compat.h"

EXPORT INT usermain(void)
{
    INT i;

    /* give a USB host up to 2 s to enumerate so the banner is not lost;
       the robot runs the same without a laptop */
    for (i = 0; (i < 20) && (tm_usb_state() < 2u); i++)
    {
        (void)tk_dly_tsk(100);
    }
    tm_printf((UB *)"\n=== IS09 car software (IS09 barcode, IMU, ultrasonic), micro T-Kernel 3.0 ===\n");

    g_status.state = (uint32_t)SYS_INIT;
    params_init();

    /* IS09 barcode table as the default map; MAP: over MQTT replaces it */
    {
        char err[32];

        if (nav_map_parse(BARCODE_DEFAULT_MAP, err, sizeof(err)) > 0)
        {
            tm_printf((UB *)"[init] barcode map %s\n", BARCODE_DEFAULT_MAP);
        }
    }

    if (!os_create_objects())
    {
        tm_printf((UB *)"[init] kernel objects failed: stopped\n");
        (void)tk_slp_tsk(TMO_FEVR);
        return 0;
    }

    hal_init();                              /* motors braked before anything else */
    params_apply();

    if (!hal_imu_init())
    {
        /* degraded mode: the robot still drives; humps and collisions are
           not detected until the IMU answers (TSK_TERRAIN_IMU retries) */
        tm_printf((UB *)"[init] IMU (GY-511 on I2C0 GP4/GP5: LSM303DLHC at 0x19 or LSM303D at 0x1E/0x1D)"
                        " not answering: running WITHOUT hump/collision detection, retrying every 2 s\n");
        fault_set(FAULT_IMU_DEAD);
    }
    else
    {
        tm_printf((UB *)"[init] IMU: %s (IS09 imu_core: hump, collision, motion class)\n", hal_imu_chip_name());
    }

    if (!is09_us_enabled())
    {
        tm_printf((UB *)"[init] ULTRASONIC OFF: APP_CAL_ECHO_SHIFTER_VERIFIED is 0 in is09/shared/app_cal.h"
                        " (IS09 HV-5: measure ECHO <= 3.3 V at GP17 first). No obstacle detection.\n");
    }

    if (!os_start_interrupts() || !os_start_tasks())
    {
        hal_motor_kill();
        tm_printf((UB *)"[init] start-up failed: motors off\n");
    }
    else
    {
        tm_printf((UB *)"[init] 7 tasks running. BTN1 = start / end run, BTN2 = stop\n");
    }

    /* the initial task must not return (the kernel would shut down) */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
