/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h) and, once
 * connected, a periodic heartbeat publish on MQTT_TOPIC. Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
void mqtt_utk_start(void);

/*
 * Application bridge (TEAM.md section 3; IS09 decision D-10). Safe to call
 * from any task: these only store message-buffer IDs or read status words.
 * Prototypes must stay identical to app_program/subsystem1/s1_bridge.h.
 */
#include <stdint.h>
void mqtt_utk_bridge_attach(int32_t out_mbf, int32_t in_mbf,
			    const char *cmd_topic);
uint32_t mqtt_utk_bridge_connected(void);
uint32_t mqtt_utk_bridge_backoff_s(void);
uint32_t mqtt_utk_bridge_pub_fail(void);

#endif /* MQTT_UTK_H */
