/** @file subsystem4.h
 *  @brief Subsystem 4 public interface: IMU, hump and collision (ImuTask,
 * priority 5). Owner: Buddy 4 Wong Wei Qi 2500430.
 *
 *  STATUS: PLACEHOLDER - NOT IMPLEMENTED (delivered in Phase 4).
 *  This placeholder only creates the kernel objects this subsystem owns
 *  ((none: ImuState is a lock-free word in shared/app_blackboard.c)) so that
 * the shared foundation links and the integration contract is fixed. It drives
 * no pin and starts no task. Other subsystems include it as
 * "../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../shared/app_ipc.h"

/* no kernel objects published yet */

#endif /* SUBSYSTEM4_H */
