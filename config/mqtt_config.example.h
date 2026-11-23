/*
 * mqtt_config.h - robot: MQTT broker settings (local file)
 *
 * Copy this file to config/mqtt_config.h and edit it. mqtt_config.h is
 * git-ignored, like wifi_credentials.h: never commit a populated copy.
 *
 * The broker is Mosquitto on the laptop, so MQTT_BROKER_IP is the LAPTOP's
 * IPv4 address on the hotspot (ipconfig -> "Wireless LAN adapter Wi-Fi" ->
 * IPv4 Address), not the hotspot's gateway address.
 */
#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define MQTT_CONFIG_SET        (0)    /* set to 1 after editing */

#if !MQTT_CONFIG_SET
#error "config/mqtt_config.h: set MQTT_BROKER_IP, then set MQTT_CONFIG_SET to 1."
#endif

#define MQTT_BROKER_IP         "LAPTOP_IP"   /* the laptop's IPv4 address */
#define MQTT_BROKER_PORT       (1883u)
#define MQTT_CLIENT_ID         "is09-robot"

#endif /* MQTT_CONFIG_H */
