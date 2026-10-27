/** @file subsystem1.c
 *  @brief TelemetryCommsTask (Buddy 1): priority 8, 20 ms loop.
 *
 *  Every cycle, in this order, with bounded work and no blocking call:
 *   1. observe the link (Wi-Fi from cyw43_utk_get_status, MQTT from the
 *      bridge) and derive conn_state; on a change post CONN_STATE, arm the
 *      START guard on ONLINE entry, report the drop count;
 *   2. mirror WIFI_LOSS / BROKER_LOSS into fault_mask (diagnostic only);
 *   3. publish g_comms_word (one aligned halfword store);
 *   4. validate up to S1_CMD_PER_CYCLE inbound command frames; on accept
 *      write g_remote_cmd and raise APP_EVT_REMOTE_CMD for the controller;
 *   5. only while ONLINE: 200 ms snapshot, 1 s heartbeat, obstacle profile
 *      on change, and up to S1_EVT_PER_CYCLE queued events.
 *  While not ONLINE nothing is published: snapshots are skipped and events
 *  stay in MBF_TELEM_EVT, where producers' non-blocking posts are dropped
 *  and counted once it is full (Week 6 Statechart 4, NFR17).
 *
 *  The task never calls cyw43, lwIP or MQTT (TEAM.md section 3, D-10).
 */

#include "subsystem1.h"

#include "../shared/app_barcode.h"
#include "../shared/app_hw.h"
#include "../shared/app_safety.h"
#include "../subsystem5/subsystem5.h"
#include "s1_bridge.h"

#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
#include "cyw43_utk.h"
#define S1_HAVE_WIFI 1
#define S1_CYW43_LINK_UP                                                       \
    3 /* CYW43_LINK_UP in cyw43.h (header not includable                       \
         from application code) */
#else
#define S1_HAVE_WIFI 0
#endif

/* mqtt_utk.c cannot include the kernel headers, so it declares tk_snd_mbf /
   tk_rcv_mbf itself with int32_t arguments and TMO_POL as 0 (D-10). Pin the
   ABI those declarations assume; a port with other widths fails to build. */
APP_STATIC_ASSERT(sizeof(ID) == 4U, bridge_abi_id_32bit);
APP_STATIC_ASSERT(sizeof(INT) == 4U, bridge_abi_int_32bit);
APP_STATIC_ASSERT(sizeof(TMO) == 4U, bridge_abi_tmo_32bit);
APP_STATIC_ASSERT(TMO_POL == 0, bridge_abi_tmo_pol_zero);

EXPORT ID mbf_telem_evt = 0;
EXPORT ID mbf_telem_out = 0;
EXPORT ID mbf_telem_in  = 0;

static s1_stats_t   g_s1_stats;
static s1_cmd_ctx_t g_s1_cmd;
static s1_cadence_t g_s1_snap_cad;
static s1_cadence_t g_s1_hb_cad;
#if !APP_TEST
static s1_cadence_t g_s1_status_cad;
static bool         g_s1_status_started = false;
#endif
static app_conn_state_t g_s1_conn           = APP_CONN_WIFI_CONNECTING;
static bool             g_s1_conn_valid     = false;
static uint16_t         g_s1_faults_have    = 0U;
static uint8_t          g_s1_snap_seq       = 0U;
static uint8_t          g_s1_hb_seq         = 0U;
static uint8_t          g_s1_obs_seq        = 0U;
static uint8_t          g_s1_obs_last       = APP_PACK_BYTE_MASK;
static bool             g_s1_report_pending = false;
static uint32_t         g_s1_last_loop_us   = 0U;
static bool             g_s1_have_loop      = false;

#if APP_TEST
static volatile uint8_t g_s1_link_mode = (uint8_t) S1_LINK_REAL;
#if APP_TEST
/* S1-U06 capture: generation is odd while the three values are being
   written (single writer: this task; single reader: the test task). */
static volatile uint32_t g_s1_dbg_gen       = 0U;
static volatile uint32_t g_s1_dbg_line_word = 0U;
static volatile uint32_t g_s1_dbg_imu_word  = 0U;
static volatile uint8_t  g_s1_dbg_snap_seq  = 0U;
static volatile bool     g_s1_dbg_valid     = false;
#endif
#endif

static char const * const g_s1_conn_names[] = {
    "WIFI_CONNECTING", "MQTT_CONNECTING", "ONLINE", "BACKOFF"};

/* ---- kernel objects ------------------------------------------------------ */

static ID
s1_create_mbf (uint32_t bytes, uint32_t max_msg)
{
    T_CMBF cmbf;

    cmbf.exinf  = NULL;
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ) bytes;
    cmbf.maxmsz = (INT) max_msg;
    cmbf.bufptr = NULL;
    return tk_cre_mbf(&cmbf);
}

EXPORT ER
subsystem1_init (void)
{
    mbf_telem_evt =
        s1_create_mbf(APP_MBF_TELEM_BYTES, (uint32_t) APP_TE_RECORD_BYTES);
    if (mbf_telem_evt < E_OK)
    {
        return mbf_telem_evt;
    }
    app_ipc_register_telem_mbf((int32_t) mbf_telem_evt);

    mbf_telem_out = s1_create_mbf(APP_MBF_BYTES(S1_OUT_DEPTH, S1_PUB_MAX_BYTES),
                                  S1_PUB_MAX_BYTES);
    if (mbf_telem_out < E_OK)
    {
        return mbf_telem_out;
    }
    mbf_telem_in = s1_create_mbf(APP_MBF_BYTES(S1_IN_DEPTH, S1_CMD_MAX_BYTES),
                                 S1_CMD_MAX_BYTES);
    if (mbf_telem_in < E_OK)
    {
        return mbf_telem_in;
    }
    s1_cmd_init(&g_s1_cmd);
    return E_OK;
}

/* ---- link observation ------------------------------------------------------
 */

static bool
s1_wifi_up (void)
{
#if S1_HAVE_WIFI
    T_CYW43_UTK_STATUS st;
    cyw43_utk_get_status(&st);
    return (0U != st.ready) && (S1_CYW43_LINK_UP == st.link_status);
#else
    return false;
#endif
}

static app_conn_state_t
s1_link_state (uint8_t * p_backoff_s)
{
    uint32_t const backoff = mqtt_utk_bridge_backoff_s();

    *p_backoff_s =
        (uint8_t) ((backoff > APP_BACKOFF_S_MAX) ? APP_BACKOFF_S_MAX : backoff);
#if APP_TEST
    if ((uint8_t) S1_LINK_FORCE_ONLINE == g_s1_link_mode)
    {
        *p_backoff_s = 0U;
        return APP_CONN_ONLINE;
    }
    if ((uint8_t) S1_LINK_FORCE_OFFLINE == g_s1_link_mode)
    {
        *p_backoff_s = 0U;
        return APP_CONN_WIFI_CONNECTING;
    }
#endif
    return s1_conn_derive(s1_wifi_up(), 0U != mqtt_utk_bridge_connected(),
                          *p_backoff_s);
}

static void
s1_on_conn_change (app_conn_state_t state, uint32_t now_ms)
{
    tm_printf((UB *) "telem: %s\n", (UB *) g_s1_conn_names[state & 3U]);
    if (APP_CONN_ONLINE == state)
    {
        g_s1_stats.online_entries++;
        s1_cmd_on_online(&g_s1_cmd, now_ms);
        s1_cadence_start(&g_s1_snap_cad, now_ms, S1_SNAP_PERIOD_MS);
        s1_cadence_start(&g_s1_hb_cad, now_ms, S1_HB_PERIOD_MS);
        /* Week 6 6.4: the drop count is reported after reconnection. It is
           NOT posted into MBF_TELEM_EVT: after an outage that buffer is
           usually full, so the report itself would be dropped (found by
           S1-U07, 22 Oct). It goes straight to MBF_TELEM_OUT instead. */
        g_s1_report_pending = true;
    }
    else
    {
        app_ipc_telem_post(APP_TEV_CONN_STATE, APP_SRC_COMMS, (uint16_t) state);
        s1_cmd_on_offline(&g_s1_cmd);
    }
}

static void
s1_update_faults (app_conn_state_t state)
{
    uint16_t const want  = s1_conn_faults(state);
    uint16_t const set   = (uint16_t) (want & (uint16_t) ~g_s1_faults_have);
    uint16_t const clear = (uint16_t) (g_s1_faults_have & (uint16_t) ~want);

    if (0U != set)
    {
        app_fault_set(set);
    }
    if (0U != clear)
    {
        app_fault_clr(clear);
    }
    g_s1_faults_have = want;
}

/* ---- outbound
 * ---------------------------------------------------------------- */

/** True if MBF_TELEM_OUT can take `count` more requests of msg_len bytes. */
static bool
s1_out_room_n (uint32_t count, uint32_t msg_len)
{
    T_RMBF rmbf;

    if ((mbf_telem_out <= 0) || (tk_ref_mbf(mbf_telem_out, &rmbf) < E_OK))
    {
        return false;
    }
    return (uint32_t) rmbf.frbufsz >= APP_MBF_BYTES(count, msg_len);
}

static bool
s1_out_room (uint32_t msg_len)
{
    return s1_out_room_n(1U, msg_len);
}

static bool
s1_out_send (s1_topic_t topic, uint8_t const * p_payload, uint32_t len)
{
    uint8_t        msg[S1_PUB_MAX_BYTES];
    uint32_t const n  = s1_pub_pack(topic, p_payload, len, msg);
    ER             er = E_OBJ;

    if ((0U != n) && (mbf_telem_out > 0))
    {
        er = tk_snd_mbf(mbf_telem_out, msg, (INT) n, TMO_POL);
    }
    if (er < E_OK)
    {
        g_s1_stats.out_full++;
        return false;
    }
    g_s1_stats.sent[topic]++;
    g_s1_stats.sent_bytes += len;
    return true;
}

static bool
s1_read_obstacle (uint32_t * p_w0, uint32_t * p_w1)
{
    /* MTX_OBSTACLE has priority inheritance; poll so this low-priority task
       never waits on the scan task (skip one cycle instead). */
    if ((mtx_obstacle <= 0) || (tk_loc_mtx(mtx_obstacle, TMO_POL) < E_OK))
    {
        return false;
    }
    *p_w0 = g_obstacle_profile[0];
    *p_w1 = g_obstacle_profile[1];
    (void) tk_unl_mtx(mtx_obstacle);
    return true;
}

/** Queue one event record straight to MBF_TELEM_OUT (bypassing
 *  MBF_TELEM_EVT, which may be full). */
static bool
s1_send_event_direct (app_telem_event_t type, uint16_t payload)
{
    uint8_t        frame[APP_EVENT_FRAME_BYTES];
    uint8_t const  stamp = (uint8_t) ((app_hw_time_us() / 1000U) & 0xFFU);
    uint32_t const record =
        app_ipc_telem_pack(type, APP_SRC_COMMS, payload, stamp);

    app_event_frame_encode(record, frame);
    return s1_out_send(S1_TOPIC_EVT, frame, APP_EVENT_FRAME_BYTES);
}

/** ONLINE entry reports: CONN_STATE then DROPPED, retried every loop until
 *  both are queued. */
static void
s1_send_online_reports (void)
{
    if (!g_s1_report_pending)
    {
        return;
    }
    if (!s1_out_room_n(2U, 1U + S1_TOPIC_MAX_BYTES + APP_EVENT_FRAME_BYTES))
    {
        return; /* both or neither: try again next loop */
    }
    (void) s1_send_event_direct(APP_TEV_CONN_STATE, (uint16_t) APP_CONN_ONLINE);
    (void) s1_send_event_direct(APP_TEV_DROPPED,
                                (uint16_t) app_ipc_telem_dropped());
    g_s1_report_pending = false;
}

static void
s1_publish_snapshot (void)
{
    s1_inputs_t    in;
    app_snapshot_t snap;
    uint8_t        frame[APP_SNAPSHOT_BYTES];

    in.mission_word = g_mission_status;
    (void) app_motion_read(&in.motion);
    in.enc_l        = g_enc_ticks_l;
    in.enc_r        = g_enc_ticks_r;
    in.line_word    = g_line_state;
    in.barcode_byte = g_barcode_byte;
    in.imu_word     = g_imu_state;
    in.obst_w0      = 0U;
    in.obst_w1      = 0U;
    if (!s1_read_obstacle(&in.obst_w0, &in.obst_w1))
    {
        in.obst_w0 = (uint32_t) APP_RANGE_CM_SENTINEL << APP_OP0_RANGE_SHIFT;
    }
    in.stop_cause = (uint8_t) app_stop_cause_get();
    in.fault_mask = g_fault_mask;

    s1_snapshot_build(&in, g_s1_snap_seq, &snap);
    (void) app_snapshot_encode(&snap, frame);
#if APP_TEST
    g_s1_dbg_gen++; /* odd: writing */
    g_s1_dbg_line_word = in.line_word;
    g_s1_dbg_imu_word  = in.imu_word;
    g_s1_dbg_snap_seq  = g_s1_snap_seq;
    g_s1_dbg_valid     = true;
    g_s1_dbg_gen++; /* even: complete */
#endif
    if (s1_out_send(S1_TOPIC_SNAP, frame, APP_SNAPSHOT_BYTES))
    {
        g_s1_snap_seq++;
    }
}

static void
s1_publish_heartbeat (app_conn_state_t state, uint8_t backoff_s)
{
    app_heartbeat_t hb;
    uint8_t         frame[APP_HB_FRAME_BYTES];
    uint32_t const  up_s = app_ipc_ms() / 1000U;

    hb.seq        = g_s1_hb_seq;
    hb.uptime_s   = (uint16_t) ((up_s > 0xFFFFU) ? 0xFFFFU : up_s);
    hb.conn_state = (uint8_t) state;
    hb.backoff_s  = backoff_s;
    hb.dropped    = app_ipc_telem_dropped();
    app_heartbeat_encode(&hb, frame);
    if (s1_out_send(S1_TOPIC_HB, frame, APP_HB_FRAME_BYTES))
    {
        g_s1_hb_seq++;
    }
}

static void
s1_publish_obstacle_on_change (void)
{
    uint32_t               w0 = 0U;
    uint32_t               w1 = 0U;
    app_obstacle_profile_t view;
    uint8_t                frame[APP_OBS_FRAME_BYTES];

    if (!s1_read_obstacle(&w0, &w1))
    {
        return;
    }
    app_obstacle_unpack(w0, w1, &view);
    if (view.seq == g_s1_obs_last)
    {
        return;
    }
    app_obs_frame_encode(g_s1_obs_seq, w0, w1, frame);
    if (s1_out_send(S1_TOPIC_OBS, frame, APP_OBS_FRAME_BYTES))
    {
        g_s1_obs_last = view.seq;
        g_s1_obs_seq++;
    }
}

static void
s1_drain_events (void)
{
    uint32_t       i      = 0U;
    uint32_t       record = 0U;
    uint8_t        frame[APP_EVENT_FRAME_BYTES];
    uint32_t const req_len = 1U + S1_TOPIC_MAX_BYTES + APP_EVENT_FRAME_BYTES;

    for (i = 0U; i < S1_EVT_PER_CYCLE; i++)
    {
        /* Leave an event in MBF_TELEM_EVT unless it can be queued, so it is
           never taken out and then lost between the two buffers. */
        if (!s1_out_room(req_len))
        {
            break;
        }
        if (tk_rcv_mbf(mbf_telem_evt, &record, TMO_POL) < E_OK)
        {
            break;
        }
        app_event_frame_encode(record, frame);
        (void) s1_out_send(S1_TOPIC_EVT, frame, APP_EVENT_FRAME_BYTES);
    }
}

/* ---- inbound
 * ------------------------------------------------------------------ */

static void
s1_handle_commands (uint32_t now_ms)
{
    uint32_t i = 0U;
    uint8_t  buf[S1_CMD_MAX_BYTES];

    for (i = 0U; i < S1_CMD_PER_CYCLE; i++)
    {
        app_remote_cmd_t  cmd = APP_REMOTE_CMD_INVALID0;
        uint8_t           seq = 0U;
        app_cmd_verdict_t verdict;
        INT const         n = (mbf_telem_in > 0)
                                  ? tk_rcv_mbf(mbf_telem_in, buf, TMO_POL)
                                  : (INT) E_OBJ;
        if (n < 0)
        {
            break;
        }
        verdict =
            s1_cmd_filter(&g_s1_cmd, buf, (uint32_t) n, now_ms, &cmd, &seq);
        if (APP_CMD_OK == verdict)
        {
            g_s1_stats.cmd_ok++;
            g_remote_cmd = app_remote_cmd_pack(cmd, seq); /* one byte store */
            g_remote_cmd_gen++;
            app_ipc_telem_post(
                APP_TEV_REMOTE_CMD, APP_SRC_COMMS,
                (uint16_t) (((uint32_t) cmd << 8U) | (uint32_t) seq));
            /* The controller (priority 3) runs before this task resumes, so
               g_remote_cmd is consumed before it can be overwritten. */
            (void) tk_set_flg((ID) app_ipc_flg_ctrl(),
                              (UINT) APP_EVT_REMOTE_CMD);
        }
        else
        {
            g_s1_stats.cmd_rejected++;
            g_s1_stats.last_reject = (uint32_t) verdict;
            app_ipc_telem_post(APP_TEV_CMD_REJECTED, APP_SRC_COMMS,
                               (uint16_t) verdict);
        }
    }
}

/* ---- console status
 * ---------------------------------------------------------- */

#if !APP_TEST
/** One line every 10 s, so a console opened at any time shows whether the
 *  task is alive, what it thinks the link is, and whether the bridge is
 *  failing (bench B1: the car fell silent and nothing said why). Not in the
 *  TEST image, to keep its [TEST] log and timing unchanged. */
static void
s1_status_line (app_conn_state_t state, uint32_t now_ms)
{
    if (!g_s1_status_started)
    {
        s1_cadence_start(&g_s1_status_cad, now_ms + S1_STATUS_PERIOD_MS,
                         S1_STATUS_PERIOD_MS);
        g_s1_status_started = true;
    }
    if (!s1_cadence_due(&g_s1_status_cad, now_ms))
    {
        return;
    }
    tm_printf(
        (UB *) "telem: %s up %u s | sent snap %u hb %u evt %u obs %u | "
               "out_full %u pub_fail %u stalls %u | dropped %u | "
               "cmd ok %u rej %u\n",
        (UB *) g_s1_conn_names[state & 3U], (unsigned) (now_ms / 1000U),
        (unsigned) g_s1_stats.sent[S1_TOPIC_SNAP],
        (unsigned) g_s1_stats.sent[S1_TOPIC_HB],
        (unsigned) g_s1_stats.sent[S1_TOPIC_EVT],
        (unsigned) g_s1_stats.sent[S1_TOPIC_OBS],
        (unsigned) g_s1_stats.out_full, (unsigned) mqtt_utk_bridge_pub_fail(),
        (unsigned) mqtt_utk_bridge_stalls(), (unsigned) app_ipc_telem_dropped(),
        (unsigned) g_s1_stats.cmd_ok, (unsigned) g_s1_stats.cmd_rejected);
}
#endif

/* ---- task
 * ---------------------------------------------------------------------- */

static void
telemetry_comms_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        uint32_t const   t0      = app_hw_time_us();
        uint32_t const   now_ms  = app_ipc_ms();
        uint8_t          backoff = 0U;
        app_conn_state_t state   = s1_link_state(&backoff);
        uint32_t         work_us = 0U;

        /* Longest gap between loop starts: how long higher-priority work
           (including the kernel Wi-Fi task at priority 4) kept this task
           off the CPU. A15 evidence. */
        if (g_s1_have_loop)
        {
            uint32_t const gap = (uint32_t) (t0 - g_s1_last_loop_us);
            if (gap > g_s1_stats.max_gap_us)
            {
                g_s1_stats.max_gap_us = gap;
            }
        }
        g_s1_last_loop_us = t0;
        g_s1_have_loop    = true;

        if ((!g_s1_conn_valid) || (state != g_s1_conn))
        {
            g_s1_conn       = state;
            g_s1_conn_valid = true;
            s1_on_conn_change(state, now_ms);
        }
        g_s1_stats.conn_state = (uint32_t) state;
        s1_update_faults(state);
        g_comms_word = app_comms_pack(state, backoff, app_ipc_telem_dropped());

        s1_handle_commands(now_ms);
#if !APP_TEST
        s1_status_line(state, now_ms);
#endif

        if (APP_CONN_ONLINE == state)
        {
            s1_send_online_reports();
            if (s1_cadence_due(&g_s1_snap_cad, now_ms))
            {
                s1_publish_snapshot();
            }
            if (s1_cadence_due(&g_s1_hb_cad, now_ms))
            {
                s1_publish_heartbeat(state, backoff);
            }
            s1_publish_obstacle_on_change();
            s1_drain_events();
        }

        work_us = (uint32_t) (app_hw_time_us() - t0);
        if (work_us > g_s1_stats.max_work_us)
        {
            g_s1_stats.max_work_us = work_us;
        }
        g_s1_stats.loops++;
        (void) tk_dly_tsk((TMO) S1_LOOP_MS);
    }
}

EXPORT ER
subsystem1_start (void)
{
    ID const tid =
        app_start_task_stack((FP) telemetry_comms_task, (PRI) APP_PRI_TELEMETRY,
                             (SZ) APP_STACK_TELEMETRY, "TelemetryCommsTask");

    if (tid < E_OK)
    {
        return (ER) tid;
    }
    mqtt_utk_bridge_attach((int32_t) mbf_telem_out, (int32_t) mbf_telem_in,
                           S1_CMD_TOPIC);
    return E_OK;
}

EXPORT void
subsystem1_stats (s1_stats_t * p_stats)
{
    *p_stats = g_s1_stats;
}

#if APP_TEST
EXPORT void
subsystem1_test_link (s1_link_mode_t mode)
{
    if (S1_LINK_REAL == mode)
    {
        g_s1_link_mode = (uint8_t) mode;
        mqtt_utk_bridge_attach((int32_t) mbf_telem_out, (int32_t) mbf_telem_in,
                               S1_CMD_TOPIC);
    }
    else
    {
        mqtt_utk_bridge_attach(0, 0, NULL);
        g_s1_link_mode = (uint8_t) mode;
    }
}

EXPORT bool
subsystem1_test_snap_inputs (uint8_t * p_seq, uint32_t * p_line_word,
                             uint32_t * p_imu_word)
{
    uint32_t const g0 = g_s1_dbg_gen;

    if ((0U != (g0 & 1U)) || !g_s1_dbg_valid)
    {
        return false;
    }
    *p_seq       = g_s1_dbg_snap_seq;
    *p_line_word = g_s1_dbg_line_word;
    *p_imu_word  = g_s1_dbg_imu_word;
    return g0 == g_s1_dbg_gen;
}
#endif
