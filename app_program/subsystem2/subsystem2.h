/** @file subsystem2.h
 *  @brief Subsystem 2 public interface: motion and encoders (MotionControlTask,
 * priority 2). Owner: Buddy 2 S Reuben Kanna 2500695.
 *
 *  STATUS: PLACEHOLDER - NOT IMPLEMENTED (delivered in Phase 2).
 *  This placeholder only creates the kernel objects this subsystem owns
 *  (MBF_MOTION_CMD (4 x 4-byte records, 32 B with headers, D-07)) so that the
 * shared foundation links and the integration contract is fixed. It drives no
 * pin and starts no task. Other subsystems include it as
 * "../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "../shared/app_ipc.h"

IMPORT ID
    mbf_motion_cmd; /* IF-MC: VehicleControllerTask sends packed commands */

#endif /* SUBSYSTEM2_H */
