/** @file subsystem2.c
 *  @brief Subsystem 2 placeholder: creates its kernel objects only.
 *         STATUS: NOT IMPLEMENTED (Phase 2). Motor-safe: no pin written.
 */

#include "subsystem2.h"

EXPORT ID mbf_motion_cmd = 0;

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
    /* PLACEHOLDER: GP8-GP11 are left in their reset state (inputs, motor
       driver inputs low = BRAKE per the Robo Pico truth table). No PWM is
       configured until Phase 2. */
    return E_OK;
}

EXPORT ER
subsystem2_start (void)
{
    /* PLACEHOLDER: no task yet. The real task is created in Phase 2. */
    return E_OK;
}
