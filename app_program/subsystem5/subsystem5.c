/** @file subsystem5.c
 *  @brief Subsystem 5 placeholder: creates its kernel objects only.
 *         STATUS: NOT IMPLEMENTED (Phase 5). Motor-safe: no pin written.
 */

#include "subsystem5.h"

EXPORT ID flg_scan     = 0;
EXPORT ID mtx_obstacle = 0;

EXPORT ER
subsystem5_init (void)
{
    T_CFLG cflg;
    T_CMTX cmtx;

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    flg_scan     = tk_cre_flg(&cflg);
    if (flg_scan < E_OK)
    {
        return flg_scan;
    }
    app_ipc_register_scan_flag((int32_t) flg_scan);

    cmtx.exinf   = NULL;
    cmtx.mtxatr  = TA_INHERIT;
    cmtx.ceilpri = 0;
    mtx_obstacle = tk_cre_mtx(&cmtx);
    if (mtx_obstacle < E_OK)
    {
        return mtx_obstacle;
    }
    /* PLACEHOLDER: GP15 servo, GP16 TRIG, GP17 ECHO untouched until Phase 5
       and until APP_CAL_ECHO_SHIFTER_VERIFIED (HV-5). */
    return E_OK;
}

EXPORT ER
subsystem5_start (void)
{
    /* PLACEHOLDER: no task yet. The real task is created in Phase 5. */
    return E_OK;
}
