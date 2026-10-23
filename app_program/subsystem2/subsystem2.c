/** @file subsystem2.c
 *  @brief Subsystem 2: creates MBF_MOTION_CMD, brings the motor driver up in
 *         BRAKE, attaches the encoder ISRs and starts MotionControlTask.
 */

#include "subsystem2.h"

#include "../shared/app_hw.h"
#include "drv_encoder.h"
#include "drv_motor.h"

EXPORT ID mbf_motion_cmd = 0;
EXPORT ID tid_motion     = 0;

EXPORT ER
subsystem2_init (void)
{
    T_CMBF cmbf;

    cmbf.exinf     = NULL;
    cmbf.mbfatr    = TA_TFIFO;
    cmbf.bufsz     = (SZ) APP_MBF_MOTION_BYTES;
    cmbf.maxmsz    = (INT) APP_MC_RECORD_BYTES;
    cmbf.bufptr    = NULL;
    mbf_motion_cmd = tk_cre_mbf(&cmbf);
    if (mbf_motion_cmd < E_OK)
    {
        return mbf_motion_cmd;
    }
    app_ipc_register_motion_mbf((int32_t) mbf_motion_cmd);

    /* Motors: PWM slices configured with both compares at 0 = BRAKE before
       the pins are routed to PWM, so GP8-GP11 never see a driving level. */
    drv_motor_init();

    if (!drv_encoder_init())
    {
        return E_OBJ; /* a GP0-GP3 callback was already taken: wiring plan
                         broken */
    }
    return E_OK;
}

EXPORT ER
subsystem2_start (void)
{
    tid_motion =
        app_start_task_stack((FP) s2_motion_task, APP_PRI_MOTION,
                             (SZ) APP_STACK_MOTION, "MotionControlTask");
    if (tid_motion < E_OK)
    {
        return tid_motion;
    }
    app_ipc_register_cyclic_task(APP_CYC_SLOT_MOTION, (int32_t) tid_motion);
    return E_OK;
}
