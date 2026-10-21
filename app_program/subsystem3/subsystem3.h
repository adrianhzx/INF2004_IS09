/** @file subsystem3.h
 *  @brief Subsystem 3 public interface: line sensing and barcode
 * (LineSensingTask 4, BarcodeDecodeTask 7). Owner: Buddy 3 Xu Zhiqiang 2501297.
 *
 *  STATUS: PLACEHOLDER - NOT IMPLEMENTED (delivered in Phase 3).
 *  This placeholder only creates the kernel objects this subsystem owns
 *  (FLG_BARCODE) so that the shared foundation links and the
 *  integration contract is fixed. It drives no pin and starts no task.
 *  Other subsystems include it as "../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../shared/app_ipc.h"

IMPORT ID
    flg_barcode; /* IF-BC: controller arms/disarms, ISR signals EDGE_AVAIL */

#endif /* SUBSYSTEM3_H */
