/** @file s1_bridge.h
 *  @brief Subsystem 1 side of the MQTT bridge (TEAM.md section 3 pattern,
 *         decision D-10). Application tasks never call cyw43, lwIP or MQTT.
 *         TelemetryCommsTask fills MBF_TELEM_OUT and empties MBF_TELEM_IN;
 *         code running INSIDE the kernel Wi-Fi task (mqtt_utk.c) polls them
 *         with TMO_POL and does the socket I/O.
 *
 *  These functions are safe from any task: they only store message-buffer
 *  IDs or read volatile status words. They are provided by
 *  lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c once the bridge change is
 *  agreed with the team and the instructor. Until then s1_bridge.c supplies
 *  weak defaults: the bridge reports "not connected", so TelemetryCommsTask
 *  stays offline (snapshots skipped, events held, drops counted) and the
 *  rest of the firmware is unaffected.
 *
 *  Keep these prototypes identical to the ones in mqtt_utk.h (plain C types
 *  on purpose: the lwIP side cannot include kernel or application headers).
 */

#ifndef S1_BRIDGE_H
#define S1_BRIDGE_H

#include <stdint.h>

/** Hand the bridge its message buffers and the topic to subscribe for
 *  commands. out_mbf = 0 detaches it (used by the on-target tests). */
void mqtt_utk_bridge_attach (int32_t out_mbf, int32_t in_mbf,
                             char const * p_cmd_topic);

/** 1 while an MQTT session is accepted by the broker, else 0. */
uint32_t mqtt_utk_bridge_connected (void);

/** Current reconnect delay in seconds (0 when connected or not backing off). */
uint32_t mqtt_utk_bridge_backoff_s (void);

/** Publish requests the bridge could not hand to MQTT (saturating). */
uint32_t mqtt_utk_bridge_pub_fail (void);

/** Sessions closed because publishing stalled for 5 s (silent link loss). */
uint32_t mqtt_utk_bridge_stalls (void);

#endif /* S1_BRIDGE_H */
