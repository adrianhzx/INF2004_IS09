/** @file subsystem3.h
 *  @brief Subsystem 3 public interface: line sensing (LineSensingTask,
 *         priority 4, 10 ms) and barcode (BarcodeDecodeTask, priority 7,
 *         BarcodeEdgeIsr on GP7). Owner: Buddy 3 Xu Zhiqiang 2501297.
 *
 *  Owned objects: FLG_BARCODE, the ADC block (GP26/GP27), the GP7 dispatcher
 *  callback. Published: g_line_state (shared/app_line.h), g_barcode_byte and
 *  g_edge_ring (shared/app_barcode.h). Other subsystems include this as
 *  "../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../shared/app_ipc.h"

IMPORT ID
    flg_barcode;    /* IF-BC: controller sets ARM/DISARM, ISR sets EDGE_AVAIL */
IMPORT ID tid_line; /* LineSensingTask (CYC_10MS slot LINE) */
IMPORT ID tid_barcode; /* BarcodeDecodeTask */

/* Task bodies and diagnostics (tests, calibration bench). */
IMPORT void     s3_line_task (INT stacd, void * p_exinf);
IMPORT void     s3_line_cycle (void);
IMPORT void     s3_line_set_references (uint16_t white_l, uint16_t black_l,
                                        uint16_t white_c, uint16_t black_c);
IMPORT uint32_t s3_line_cycle_count (void);
IMPORT uint32_t s3_line_max_cycle_us (void);
IMPORT void     s3_line_raw (uint16_t * p_left, uint16_t * p_centre,
                             uint8_t * p_n_left, uint8_t * p_n_centre);

IMPORT void s3_barcode_task (INT stacd, void * p_exinf);
IMPORT bool s3_barcode_init_isr (void);
IMPORT void s3_barcode_arm (
    bool arm); /* task-context helper; the controller uses FLG_BARCODE */
IMPORT uint32_t s3_barcode_edge_count (void);
IMPORT uint32_t s3_barcode_result_count (void);
IMPORT uint8_t  s3_barcode_state (void);
#if defined(APP_TEST) && APP_TEST
/* TEST build only (S3-U08): last capture at the quiet zone, widths in 8 us
   units. Returns false while the task is writing it (try again). */
IMPORT bool s3_barcode_dbg_last (uint16_t * p_width, uint8_t * p_count,
                                 uint32_t * p_seq);
IMPORT void s3_barcode_dbg_stats (uint32_t * p_decoded, uint32_t * p_invalid,
                                  uint32_t * p_stray);
#endif

#endif /* SUBSYSTEM3_H */
