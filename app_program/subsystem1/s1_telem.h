/** @file s1_telem.h
 *  @brief Subsystem 1 (Buddy 1): pure telemetry logic with no kernel calls,
 *         so every rule here runs in the host tests (testing/subsystem1/host).
 *         TelemetryCommsTask (subsystem1.c) only adds the kernel plumbing.
 *
 *  Contents:
 *   - topics and the publish-request format handed to the MQTT bridge
 *   - connection state derived from Wi-Fi link + MQTT session (Statechart 4)
 *   - fixed-rate cadence (200 ms snapshot, 1 s heartbeat) without drift
 *   - inbound command filter (length, version, CRC, type, replay, START
 *     guard after every (re)connect: D-02, R7)
 *   - snapshot assembly from the published blackboard words (FR20)
 */

#ifndef S1_TELEM_H
#define S1_TELEM_H

#include <stdbool.h>
#include <stdint.h>

#include "../shared/app_data.h"
#include "../shared/app_imu.h"
#include "../shared/app_line.h"
#include "../shared/app_motion.h"
#include "../shared/app_obstacle.h"
#include "../shared/app_telemetry.h"

/* ---- Topics (D-13; must match tools/telem_decode.py, client "is09-car") -- */
#define S1_TOPIC_BASE "is09/is09-car/"
#define S1_CMD_TOPIC S1_TOPIC_BASE "cmd"

typedef enum
{
    S1_TOPIC_SNAP  = 0,
    S1_TOPIC_EVT   = 1,
    S1_TOPIC_HB    = 2,
    S1_TOPIC_OBS   = 3,
    S1_TOPIC_COUNT = 4
} s1_topic_t;

/* ---- Timing (Week 6 Rev B: 20 ms loop, 200 ms snapshot, 1 s heartbeat) --- */
#define S1_LOOP_MS 20U /* requested; 20-30 ms actual on the 10 ms tick */
#define S1_SNAP_PERIOD_MS 200U
#define S1_HB_PERIOD_MS 1000U
#define S1_EVT_PER_CYCLE 4U /* bounded work per loop */
#define S1_CMD_PER_CYCLE 2U

/* ---- Publish request (MBF_TELEM_OUT message) ----------------------------
 * [topic_len u8][topic, no NUL][payload]. The bridge in mqtt_utk.c parses
 * exactly this; it never needs to know our topics or frame types.          */
#define S1_TOPIC_MAX_BYTES 24U
#define S1_PAYLOAD_MAX_BYTES APP_SNAPSHOT_BYTES /* largest frame: 18 B */
#define S1_PUB_MAX_BYTES (1U + S1_TOPIC_MAX_BYTES + S1_PAYLOAD_MAX_BYTES)
#define S1_OUT_DEPTH 8U
#define S1_CMD_MAX_BYTES 8U /* longer inbound payloads arrive truncated */
#define S1_IN_DEPTH 4U

char const * s1_topic_name (s1_topic_t topic);

/** Build a publish request. Returns its length, or 0 if it does not fit. */
uint32_t s1_pub_pack (s1_topic_t topic, uint8_t const * p_payload, uint32_t len,
                      uint8_t * p_out);

/** Split a publish request (used by tests and documents the bridge format).
 *  p_topic receives a NUL-terminated copy (S1_TOPIC_MAX_BYTES + 1 bytes). */
bool s1_pub_unpack (uint8_t const * p_msg, uint32_t len, char * p_topic,
                    uint8_t const ** pp_payload, uint32_t * p_payload_len);

/* ---- Connection state (Statechart 4 observed from the kernel task) ----- */
app_conn_state_t s1_conn_derive (bool wifi_up, bool mqtt_up, uint8_t backoff_s);

/** Fault bits that should be SET for this state (diagnostic only: they never
 *  feed a stop decision, D-08 / NFR17). */
uint16_t s1_conn_faults (app_conn_state_t state);

/* ---- Fixed-rate cadence ---------------------------------------------------
 */
typedef struct
{
    uint32_t next_ms;
    uint32_t period_ms;
} s1_cadence_t;

void s1_cadence_start (s1_cadence_t * p_cad, uint32_t now_ms,
                       uint32_t period_ms);

/** True once per period. The deadline advances by exactly one period, so
 *  the mean rate does not drift with loop jitter; after a stall of more
 *  than one period it resynchronises instead of bursting. */
bool s1_cadence_due (s1_cadence_t * p_cad, uint32_t now_ms);

/* ---- Inbound command filter ------------------------------------------------
 */
typedef struct
{
    uint8_t  last_seq;        /* 0xFF = none accepted yet */
    bool     online;          /* false until the first ONLINE entry */
    uint32_t online_since_ms; /* start of the current START guard */
} s1_cmd_ctx_t;

void s1_cmd_init (s1_cmd_ctx_t * p_ctx);

/** Arm the START guard: call on every entry to ONLINE (a broker re-delivers
 *  a retained START on each new subscription, D-02). */
void s1_cmd_on_online (s1_cmd_ctx_t * p_ctx, uint32_t now_ms);
void s1_cmd_on_offline (s1_cmd_ctx_t * p_ctx);

/** Validate one inbound frame. On APP_CMD_OK, the command and sequence are set
 * and the replay memory advances; a rejected frame never changes it. The
 * S1-only rule for START is the controller's (vehicle_controller.c, D-01). */
app_cmd_verdict_t s1_cmd_filter (s1_cmd_ctx_t * p_ctx, uint8_t const * p_frame,
                                 uint32_t len, uint32_t now_ms,
                                 app_remote_cmd_t * p_cmd, uint8_t * p_seq);

/* ---- Snapshot assembly ------------------------------------------------------
 */
typedef struct
{
    uint32_t            mission_word; /* g_mission_status */
    app_motion_status_t motion;       /* coherent pair (app_motion_read) */
    uint16_t            enc_l;
    uint16_t            enc_r;
    uint32_t            line_word;    /* g_line_state */
    uint8_t             barcode_byte; /* g_barcode_byte */
    uint32_t            imu_word;     /* g_imu_state */
    uint32_t            obst_w0;      /* ObstacleProfile under MTX_OBSTACLE */
    uint32_t            obst_w1;
    uint8_t             stop_cause; /* app_stop_cause_get() */
    uint16_t            fault_mask; /* g_fault_mask */
} s1_inputs_t;

void s1_snapshot_build (s1_inputs_t const * p_in, uint8_t seq,
                        app_snapshot_t * p_snap);

#endif /* S1_TELEM_H */
