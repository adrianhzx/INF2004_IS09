/** @file subsystem1.h
 *  @brief Subsystem 1 public interface: Wi-Fi / MQTT telemetry
 * (TelemetryCommsTask, priority 8). Owner: Buddy 1 Xavier Lam Jun Jie 2500887.
 *
 *  STATUS: PLACEHOLDER - NOT IMPLEMENTED (delivered in Phase 6).
 *  This placeholder only creates the kernel objects this subsystem owns
 *  (MBF_TELEM_EVT (32 x 4-byte records, 256 B with headers, D-07)) so that the
 * shared foundation links and the integration contract is fixed. It drives no
 * pin and starts no task. Other subsystems include it as
 * "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../shared/app_ipc.h"

IMPORT ID mbf_telem_evt; /* IF-TE: app_ipc_telem_post() writes here */

#endif /* SUBSYSTEM1_H */
