/** @file subsystem3.c
 *  @brief Subsystem 3: creates FLG_BARCODE, brings up the ADC and the GP7
 *         edge capture (disarmed), starts LineSensingTask and
 *         BarcodeDecodeTask.
 */

#include "subsystem3.h"

#include "drv_line_adc.h"

EXPORT ID flg_barcode = 0;
EXPORT ID tid_line    = 0;
EXPORT ID tid_barcode = 0;

EXPORT ER
subsystem3_init (void)
{
    T_CFLG cflg;

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    flg_barcode  = tk_cre_flg(&cflg);
    if (flg_barcode < E_OK)
    {
        return flg_barcode;
    }
    if (!drv_line_adc_init())
    {
        return E_OBJ; /* ADC block never became READY: wiring/clock fault */
    }
    if (!s3_barcode_init_isr())
    {
        return E_OBJ; /* GP7 already owned by another callback */
    }
    return E_OK;
}

EXPORT ER
subsystem3_start (void)
{
    tid_line = app_start_task_stack((FP) s3_line_task, APP_PRI_LINE,
                                    (SZ) APP_STACK_LINE, "LineSensingTask");
    if (tid_line < E_OK)
    {
        return tid_line;
    }
    app_ipc_register_cyclic_task(APP_CYC_SLOT_LINE, (int32_t) tid_line);

    tid_barcode =
        app_start_task_stack((FP) s3_barcode_task, APP_PRI_BARCODE,
                             (SZ) APP_STACK_BARCODE, "BarcodeDecodeTask");
    if (tid_barcode < E_OK)
    {
        return tid_barcode;
    }
    return E_OK;
}
