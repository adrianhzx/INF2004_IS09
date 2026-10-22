/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>
#include <stdint.h>
#include <string.h>

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60

/* Reconnect back-off: 1, 2, 4, 8, then 10 s (Week 6 NFR18 cap of 10 s). */
#define MQTT_UTK_RETRY_FIRST_MS 1000
#define MQTT_UTK_RETRY_MAX_MS   10000

/*
 * Application bridge (TEAM.md section 3, IS09 decision D-10).
 *
 * TelemetryCommsTask (app_program/subsystem1) never calls lwIP or MQTT. It
 * queues publish requests in a message buffer and reads inbound command
 * payloads from another; this code, running inside cyw43_utk_task like the
 * rest of lwIP, polls both with TMO_POL (never blocks).
 *   out message: [topic_len u8][topic bytes, no NUL][payload]
 *   in message : the raw payload of one PUBLISH on the command topic,
 *                truncated to MQTT_UTK_BRIDGE_CMD_MAX bytes
 * While no application has attached, the original demo heartbeat runs.
 *
 * tk/tkernel.h must stay out of this lwIP-namespace file (see cyw43_utk.c),
 * so the two calls are declared here with their micro T-Kernel 3.0 types
 * (ID, INT and TMO are 32-bit signed on this port; TMO_POL is 0).
 */
extern int32_t tk_snd_mbf(int32_t mbfid, const void *msg, int32_t msgsz,
			  int32_t tmout);
extern int32_t tk_rcv_mbf(int32_t mbfid, void *msg, int32_t tmout);

#define MQTT_UTK_TMO_POL		0
#define MQTT_UTK_BRIDGE_POLL_MS		20
#define MQTT_UTK_BRIDGE_PER_POLL	4
#define MQTT_UTK_BRIDGE_MSG_MAX		64	/* >= the app's 43-byte request */
#define MQTT_UTK_BRIDGE_TOPIC_MAX	48
#define MQTT_UTK_BRIDGE_CMD_MAX		8

static volatile int32_t		br_out;
static volatile int32_t		br_in;
static const char * volatile	br_cmd_topic;
static volatile uint32_t	br_connected;
static volatile uint32_t	br_backoff_s;
static volatile uint32_t	br_pub_fail;
static u8_t			br_subscribed;
static u8_t			br_rx_is_cmd;
static uint8_t			br_rx[MQTT_UTK_BRIDGE_CMD_MAX];
static u16_t			br_rx_len;
static uint32_t			retry_ms = MQTT_UTK_RETRY_FIRST_MS;

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * heartbeat this demo publishes; anything longer is truncated and flagged
 * rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

static void heartbeat_timeout(void *arg);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;

	br_rx_is_cmd = (br_in > 0 && br_cmd_topic != NULL &&
			strcmp(topic, br_cmd_topic) == 0)? 1: 0;
	br_rx_len = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	if(br_rx_is_cmd) {
		/* Command payloads are binary: queue them for TelemetryCommsTask,
		 * which validates length, version, CRC, type and replay. A longer
		 * payload arrives truncated, so its length check fails there. */
		for(i = 0; i < len && br_rx_len < MQTT_UTK_BRIDGE_CMD_MAX; i++) {
			br_rx[br_rx_len++] = data[i];
		}
		if((flags & MQTT_DATA_FLAG_LAST) != 0 && br_rx_len > 0) {
			(void)tk_snd_mbf(br_in, br_rx, (int32_t)br_rx_len,
					 MQTT_UTK_TMO_POL);
		}
		return;
	}

	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
			/* The console is a terminal -- keep raw bytes from a
			 * payload this demo did not publish from scrambling it. */
			UB c = data[i];
			rx_data[rx_len++] = (c >= 0x20 && c < 0x7f)? c: '.';
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_data[rx_len] = '\0';
		tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
			  rx_truncated? " (truncated)": "");
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

static void heartbeat_timeout(void *arg)
{
	static unsigned int	count = 0;
	UB			payload[48];
	int			len;

	(void)arg;

	if(br_out == 0 && mqtt_client_is_connected(client)) {
		len = tm_sprintf(payload, (const UB*)"hb %u", count++);
		tm_printf((UB*)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
		(void)mqtt_publish(client, MQTT_TOPIC, payload, (u16_t)len,
				   0 /* qos */, 0 /* retain */,
				   publish_done_cb, NULL);
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
	sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void bridge_subscribe_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: bridge subscribed to %s\n",
			  (const char*)br_cmd_topic);
	} else {
		br_subscribed = 0;	/* try again on the next poll */
		tm_printf((UB*)"mqtt: bridge subscribe failed, err=%d\n",
			  (int)result);
	}
}

/* Runs every MQTT_UTK_BRIDGE_POLL_MS on cyw43_utk_task (sys_check_timeouts).
 * Bounded: at most MQTT_UTK_BRIDGE_PER_POLL publishes per call. */
static void bridge_poll(void *arg)
{
	uint8_t		msg[MQTT_UTK_BRIDGE_MSG_MAX];
	char		topic[MQTT_UTK_BRIDGE_TOPIC_MAX + 1];
	int32_t		n;
	uint32_t	k;
	uint32_t	tlen;

	(void)arg;

	if(br_out > 0 && client != NULL && mqtt_client_is_connected(client)) {
		if(!br_subscribed && br_cmd_topic != NULL) {
			br_subscribed = 1;
			if(mqtt_subscribe(client, (const char*)br_cmd_topic,
					  0 /* qos */, bridge_subscribe_cb,
					  NULL) != ERR_OK) {
				br_subscribed = 0;
			}
		}
		for(k = 0; k < MQTT_UTK_BRIDGE_PER_POLL; k++) {
			n = tk_rcv_mbf(br_out, msg, MQTT_UTK_TMO_POL);
			if(n < 2) {
				break;	/* empty (E_TMOUT) or malformed */
			}
			tlen = msg[0];
			if(tlen == 0 || tlen > MQTT_UTK_BRIDGE_TOPIC_MAX ||
			   (int32_t)(1 + tlen) > n) {
				br_pub_fail++;
				continue;
			}
			memcpy(topic, &msg[1], tlen);
			topic[tlen] = '\0';
			if(mqtt_publish(client, topic, &msg[1 + tlen],
					(u16_t)(n - 1 - (int32_t)tlen),
					0 /* qos */, 0 /* retain */, NULL,
					NULL) != ERR_OK) {
				br_pub_fail++;	/* request consumed: counted */
			}
		}
	}
	sys_timeout(MQTT_UTK_BRIDGE_POLL_MS, bridge_poll, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		retry_ms = MQTT_UTK_RETRY_FIRST_MS;
		br_backoff_s = 0;
		br_subscribed = 0;	/* bridge_poll subscribes again */
		br_connected = 1;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe to the same topic this client publishes to, so every
		 * heartbeat comes back from the broker and is printed by
		 * incoming_data_cb(). A matching RX for each TX is end-to-end
		 * proof of the round trip without needing mosquitto_sub on the
		 * broker host. Callbacks must be registered before subscribing,
		 * or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating chain, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(heartbeat_timeout, NULL);
		sys_untimeout(bridge_poll, NULL);
		if(br_out == 0) {
			err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
					     subscribe_done_cb, NULL);
			if(err != ERR_OK) {
				tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
					  (int)err);
			}
			sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
		}
		sys_timeout(MQTT_UTK_BRIDGE_POLL_MS, bridge_poll, NULL);
	} else {
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		br_connected = 0;
		br_backoff_s = retry_ms / 1000;
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, (int)(retry_ms / 1000));
		/* Hold off the 2 ms poll loop's mqtt_utk_start() until the
		 * back-off expires; otherwise a dropped session reconnects
		 * immediately and the back-off never applies. */
		connecting = 1;
		sys_untimeout(retry_timeout, NULL);
		sys_timeout(retry_ms, retry_timeout, NULL);
		retry_ms = (retry_ms * 2 > MQTT_UTK_RETRY_MAX_MS)?
			   MQTT_UTK_RETRY_MAX_MS: retry_ms * 2;
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}

/* ---- application bridge API (safe from any task: plain stores/loads) ---- */

void mqtt_utk_bridge_attach(int32_t out_mbf, int32_t in_mbf,
			    const char *cmd_topic)
{
	br_cmd_topic = cmd_topic;
	br_in = in_mbf;
	br_subscribed = 0;
	br_out = out_mbf;	/* last: the poll keys on it */
}

uint32_t mqtt_utk_bridge_connected(void)
{
	return br_connected;
}

uint32_t mqtt_utk_bridge_backoff_s(void)
{
	return br_connected? 0: br_backoff_s;
}

uint32_t mqtt_utk_bridge_pub_fail(void)
{
	return br_pub_fail;
}
