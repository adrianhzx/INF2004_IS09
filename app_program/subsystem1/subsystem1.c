/** @file subsystem1.c
 *  @brief Subsystem 1 placeholder: creates its kernel objects only.
 *         STATUS: NOT IMPLEMENTED (Phase 6). Motor-safe: no pin written.
 */

#include "subsystem1.h"

EXPORT ID mbf_telem_evt = 0;

EXPORT ER
subsystem1_init (void)
{
    T_CMBF cmbf;

    cmbf.exinf    = NULL;
    cmbf.mbfatr   = TA_TFIFO;
    cmbf.bufsz    = (SZ) APP_MBF_TELEM_BYTES;
    cmbf.maxmsz   = (INT) APP_TE_RECORD_BYTES;
    cmbf.bufptr   = NULL;
    mbf_telem_evt = tk_cre_mbf(&cmbf);
    if (mbf_telem_evt < E_OK)
    {
        return mbf_telem_evt;
    }
    app_ipc_register_telem_mbf((int32_t) mbf_telem_evt);
    return E_OK;
}

EXPORT ER
subsystem1_start (void)
{
    /* PLACEHOLDER: no task yet. The real task is created in Phase 6. */
    return E_OK;
}
