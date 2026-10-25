/** @file subsystem5.c
 *  @brief Subsystem 5: FLG_SCAN, MTX_OBSTACLE (TA_INHERIT), servo / TRIG /
 *         ECHO bring-up behind the HV-5 and HV-7 gates, ObstacleScanTask.
 */

#include "subsystem5.h"

EXPORT ID flg_scan     = 0;
EXPORT ID mtx_obstacle = 0;
EXPORT ID tid_scan     = 0;

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
    if (!s5_scan_init_hw())
    {
        return E_OBJ; /* GP17 already owned by another callback */
    }
    return E_OK;
}

EXPORT ER
subsystem5_start (void)
{
    tid_scan = app_start_task_stack((FP) s5_scan_task, APP_PRI_SCAN,
                                    (SZ) APP_STACK_SCAN, "ObstacleScanTask");
    if (tid_scan < E_OK)
    {
        return tid_scan;
    }
    return E_OK;
}
