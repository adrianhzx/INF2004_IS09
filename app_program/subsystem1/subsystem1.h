/** @file subsystem1.h
 *  @brief Subsystem 1 public interface: Wi-Fi / MQTT telemetry and remote
 *         commands (TelemetryCommsTask, priority 8, 20 ms loop).
 *         Owner: Buddy 1 Xavier Lam Jun Jie 2500887.
 *
 *  STATUS: Phase 6 implemented (host-tested, target-built). On-target and
 *  bench results: testing/subsystem1/TEST_LOG.md.
 *
 *  Objects owned (TEAM.md section 5/6):
 *   MBF_TELEM_EVT  32 x 4 B event records (256 B with headers, D-07).
 *                  Writers: any task/ISR via app_ipc_telem_post() (never
 *                  waits, drops counted). Reader: TelemetryCommsTask.
 *   MBF_TELEM_OUT  8 publish requests (s1_telem.h format). Writer:
 *                  TelemetryCommsTask. Reader: MQTT bridge in the kernel
 *                  Wi-Fi task (TMO_POL).
 *   MBF_TELEM_IN   4 inbound command payloads (<= 8 B). Writer: MQTT bridge
 *                  (TMO_POL). Reader: TelemetryCommsTask.
 *  Published words (single writer: TelemetryCommsTask): g_comms_word,
 *  g_remote_cmd (+ g_remote_cmd_gen), fault bits WIFI_LOSS / BROKER_LOSS
 *  (diagnostic only, never a stop trigger).
 *  Signals: FLG_CTRL APP_EVT_REMOTE_CMD after a validated command.
 *  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../shared/app_ipc.h"
#include "s1_telem.h"

IMPORT ID mbf_telem_evt; /* IF-TE: app_ipc_telem_post() writes here */
IMPORT ID mbf_telem_out; /* IF-NB: bridge reads publish requests */
IMPORT ID mbf_telem_in;  /* IF-NB: bridge writes inbound command payloads */

/** Counters for the console, tests and the Week 13 evidence. Written only by
 *  TelemetryCommsTask; each field is one aligned word. */
typedef struct
{
    uint32_t loops;                /* 20 ms loop iterations */
    uint32_t max_work_us;          /* longest loop body (excludes the sleep) */
    uint32_t sent[S1_TOPIC_COUNT]; /* requests queued per topic */
    uint32_t sent_bytes;           /* payload bytes queued (frames only) */
    uint32_t out_full;             /* requests not queued: OUT buffer full */
    uint32_t cmd_ok;               /* validated commands */
    uint32_t cmd_rejected;         /* rejected inbound frames */
    uint32_t last_reject;          /* app_cmd_verdict_t of the last reject */
    uint32_t conn_state;           /* app_conn_state_t */
    uint32_t online_entries;       /* number of ONLINE entries */
    uint32_t max_gap_us;           /* longest time between loop starts */
} s1_stats_t;

void subsystem1_stats (s1_stats_t * p_stats);

#if APP_TEST
/** Test-only link override. FORCE_* detaches the real bridge so the test
 *  can read MBF_TELEM_OUT and write MBF_TELEM_IN itself; REAL reattaches. */
typedef enum
{
    S1_LINK_REAL          = 0,
    S1_LINK_FORCE_OFFLINE = 1,
    S1_LINK_FORCE_ONLINE  = 2
} s1_link_mode_t;

void subsystem1_test_link (s1_link_mode_t mode);
#endif

#endif /* SUBSYSTEM1_H */
