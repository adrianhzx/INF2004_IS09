/** @file subsystem5.h
 *  @brief Subsystem 5 public interface: ultrasonic scan and obstacle profile
 * (ObstacleScanTask, priority 6). Owner: Buddy 5 Ho Zhi Xian Adrian 2500280.
 *
 *  STATUS: PLACEHOLDER - NOT IMPLEMENTED (delivered in Phase 5).
 *  This placeholder only creates the kernel objects this subsystem owns
 *  (FLG_SCAN, MTX_OBSTACLE (TA_INHERIT)) so that the shared foundation links
 * and the integration contract is fixed. It drives no pin and starts no task.
 *  Other subsystems include it as "../subsystem5/subsystem5.h".
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../shared/app_ipc.h"

IMPORT ID flg_scan;     /* IF-SC: controller requests, EchoIsr ECHO_DONE */
IMPORT ID mtx_obstacle; /* IF-OP: ObstacleProfile copy mutex (TA_INHERIT) */

#endif /* SUBSYSTEM5_H */
