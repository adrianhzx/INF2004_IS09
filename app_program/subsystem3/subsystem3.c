/** @file subsystem3.c
 *  @brief Subsystem 3 placeholder: creates its kernel objects only.
 *         STATUS: NOT IMPLEMENTED (Phase 3). Motor-safe: no pin written.
 */

#include "subsystem3.h"

EXPORT ID flg_barcode = 0;

EXPORT ER
subsystem3_init (void)
{
    T_CFLG cflg;

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    flg_barcode  = tk_cre_flg(&cflg);
    return (flg_barcode < E_OK) ? flg_barcode : E_OK;
}

EXPORT ER
subsystem3_start (void)
{
    /* PLACEHOLDER: no task yet. The real task is created in Phase 3. */
    return E_OK;
}
