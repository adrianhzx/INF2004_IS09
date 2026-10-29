/** @file subsystem1_test.c
 *  @brief On-target tests for subsystem 1 (TelemetryCommsTask). Built only
 *         with: make TEST=1. No broker, Wi-Fi or other subsystem is needed:
 *         the test detaches the real bridge and plays its part itself
 *         (reads MBF_TELEM_OUT, writes MBF_TELEM_IN).
 *         Test IDs S1-U01 .. S1-U10 (testing/subsystem1/TEST_LOG.md).
 *
 *  Never sends a STOP end to end: it would latch S7 before the remaining
 *  suites run. STOP is covered by host test H-S1-CMD and bench test B5.
 */

#include "subsystem1.h"

#if APP_TEST

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_safety.h"
#include "../test_report.h"

#if defined(TM_WIFI_CYW43) && TM_WIFI_CYW43
#include "cyw43_utk.h"
#define S1T_HAVE_WIFI 1
#else
#define S1T_HAVE_WIFI 0
#endif

#define S1T_RADIO_WAIT_MS 10000U

#define S1T_MARK_BASE 0xA500U /* payload marker for S1-U09 events */
#define S1T_MARK_COUNT 10U
#define S1T_U06_MIN_CHECKED 8U /* of the 10-12 snapshots in 2.1 s */

typedef struct
{
    uint32_t n[S1_TOPIC_COUNT];
    uint32_t snap_seq_gaps;
    int32_t  last_snap_seq; /* -1 = none yet */
    uint32_t snap_decode_fail;
    uint32_t marks;         /* S1-U09 marker events seen */
    uint32_t dropped_evt;   /* last APP_TEV_DROPPED payload + 1 (0 = none) */
    uint32_t snap_checked;  /* S1-U06: snapshots compared with their inputs */
    uint32_t snap_mismatch; /* S1-U06: fields differing from those inputs */
    app_snapshot_t  last_snap;
    app_heartbeat_t last_hb;
    bool            have_hb;
} s1t_seen_t;

/** S1-U06: compare a decoded snapshot with the line and IMU blackboard words
 *  the task captured when it built that same seq. The live LineSensingTask
 *  and ImuTask rewrite both words every 10 ms, so the comparison must be
 *  against what the task actually read, never against a value written by
 *  the test (that version of the test failed on 26-27 Oct for that reason).
 *  A snapshot whose seq does not match the capture (the task is already on
 *  the next one) is simply not counted. */
static void
s1t_check_snap_inputs (app_snapshot_t const * p_snap, s1t_seen_t * p_seen)
{
    uint8_t          seq       = 0U;
    uint32_t         line_word = 0U;
    uint32_t         imu_word  = 0U;
    app_line_state_t line;
    app_imu_state_t  imu;

    if (!subsystem1_test_snap_inputs(&seq, &line_word, &imu_word) ||
        (seq != p_snap->seq))
    {
        return;
    }
    app_line_unpack(line_word, &line);
    app_imu_unpack(imu_word, &imu);
    p_seen->snap_checked++;
    if ((p_snap->line_error != line.line_error) ||
        (p_snap->on_line != line.on_line) ||
        (p_snap->hump_max_mm != imu.max_height_mm) ||
        (p_snap->hump_active != imu.hump_active) ||
        (p_snap->motion_class != imu.motion_class))
    {
        p_seen->snap_mismatch++;
    }
}

static bool
s1t_streq (char const * p_a, char const * p_b)
{
    uint32_t i = 0U;

    while (('\0' != p_a[i]) && (p_a[i] == p_b[i]))
    {
        i++;
    }
    return p_a[i] == p_b[i];
}

static void
s1t_classify (uint8_t const * p_msg, uint32_t len, s1t_seen_t * p_seen)
{
    char            topic[S1_TOPIC_MAX_BYTES + 1U];
    uint8_t const * p_pl   = NULL;
    uint32_t        pl_len = 0U;
    uint32_t        t      = 0U;

    if (!s1_pub_unpack(p_msg, len, topic, &p_pl, &pl_len))
    {
        return;
    }
    for (t = 0U; t < (uint32_t) S1_TOPIC_COUNT; t++)
    {
        if (s1t_streq(topic, s1_topic_name((s1_topic_t) t)))
        {
            p_seen->n[t]++;
            break;
        }
    }
    if ((uint32_t) S1_TOPIC_SNAP == t)
    {
        app_snapshot_t snap;
        if ((APP_SNAPSHOT_BYTES != pl_len) || !app_snapshot_decode(p_pl, &snap))
        {
            p_seen->snap_decode_fail++;
            return;
        }
        if ((p_seen->last_snap_seq >= 0) &&
            (snap.seq != (uint8_t) (p_seen->last_snap_seq + 1)))
        {
            p_seen->snap_seq_gaps++;
        }
        p_seen->last_snap_seq = (int32_t) snap.seq;
        p_seen->last_snap     = snap;
        s1t_check_snap_inputs(&snap, p_seen);
    }
    else if ((uint32_t) S1_TOPIC_HB == t)
    {
        p_seen->have_hb = app_heartbeat_decode(p_pl, &p_seen->last_hb);
    }
    else if (((uint32_t) S1_TOPIC_EVT == t) &&
             (APP_EVENT_FRAME_BYTES == pl_len))
    {
        uint8_t        type    = 0U;
        uint8_t        src     = 0U;
        uint16_t       payload = 0U;
        uint8_t        stamp   = 0U;
        uint32_t const rec     = app_unpack_le32(p_pl);
        app_ipc_telem_unpack(rec, &type, &src, &payload, &stamp);
        if (((uint8_t) APP_TEV_RECOVERY == type) &&
            ((payload & 0xFF00U) == S1T_MARK_BASE))
        {
            p_seen->marks++;
        }
        if ((uint8_t) APP_TEV_DROPPED == type)
        {
            p_seen->dropped_evt = (uint32_t) payload + 1U;
        }
    }
    else
    {
        /* obstacle frames: counted only */
    }
}

/** Act as the bridge for ms milliseconds: empty MBF_TELEM_OUT every 10 ms. */
static void
s1t_drain (uint32_t ms, s1t_seen_t * p_seen)
{
    uint8_t        msg[S1_PUB_MAX_BYTES];
    uint32_t const t0 = app_ipc_ms();

    do
    {
        INT n = tk_rcv_mbf(mbf_telem_out, msg, TMO_POL);
        while (n >= 0)
        {
            s1t_classify(msg, (uint32_t) n, p_seen);
            n = tk_rcv_mbf(mbf_telem_out, msg, TMO_POL);
        }
        (void) tk_dly_tsk(10);
    } while ((uint32_t) (app_ipc_ms() - t0) < ms);
}

static void
s1t_seen_init (s1t_seen_t * p_seen)
{
    uint32_t t = 0U;

    for (t = 0U; t < (uint32_t) S1_TOPIC_COUNT; t++)
    {
        p_seen->n[t] = 0U;
    }
    p_seen->snap_seq_gaps    = 0U;
    p_seen->last_snap_seq    = -1;
    p_seen->snap_decode_fail = 0U;
    p_seen->snap_checked     = 0U;
    p_seen->snap_mismatch    = 0U;
    p_seen->marks            = 0U;
    p_seen->dropped_evt      = 0U;
    p_seen->have_hb          = false;
}

static void
s1t_inject (app_remote_cmd_t cmd, uint8_t seq, bool corrupt, uint32_t len)
{
    uint8_t frame[S1_CMD_MAX_BYTES] = {0U};

    app_cmd_frame_encode(cmd, seq, frame);
    if (corrupt)
    {
        frame[APP_CMD_OFF_CRC] = (uint8_t) (frame[APP_CMD_OFF_CRC] ^ 0x5AU);
    }
    (void) tk_snd_mbf(mbf_telem_in, frame, (INT) len, TMO_POL);
    (void) tk_dly_tsk(60); /* > two 20 ms loops */
}

static app_conn_state_t
s1t_conn (void)
{
    return (app_conn_state_t) app_pack_get(g_comms_word, APP_CW_STATE_SHIFT,
                                           APP_CW_STATE_BITS);
}

/** Wait (bounded) for the kernel Wi-Fi task to finish bringing the radio up,
 *  so S1-U02/U03 measure this task and not the one-off radio start-up. */
static void
s1t_wait_radio (void)
{
#if S1T_HAVE_WIFI
    T_CYW43_UTK_STATUS st;
    uint32_t const     t0 = app_ipc_ms();

    do
    {
        cyw43_utk_get_status(&st);
        if (0U != st.init_complete)
        {
            break;
        }
        (void) tk_dly_tsk(50);
    } while ((uint32_t) (app_ipc_ms() - t0) < S1T_RADIO_WAIT_MS);
    tm_printf((UB *) "[TEST] S1 INFO  radio init %s after %u ms\n",
              (0U != st.init_complete) ? "complete" : "NOT complete",
              (unsigned) (app_ipc_ms() - t0));
#endif
}

EXPORT void
subsystem1_test (void)
{
    static s1t_seen_t seen; /* large: keep off the initial task stack */
    s1_stats_t        st0;
    s1_stats_t        st1;
    T_RMBF            rmbf;
    uint32_t          i      = 0U;
    uint32_t          t_on   = 0U;
    uint32_t          max_us = 0U;
    uint8_t           drop0  = 0U;
    uint8_t           gen0   = 0U;
    uint32_t          cyc0   = 0U;
    TEST_INIT();

    /* S1-U01: the three message buffers exist with the documented limits. */
    TEST_CHECK("S1-U01",
               (tk_ref_mbf(mbf_telem_evt, &rmbf) == E_OK) &&
                   (rmbf.maxmsz == (INT) APP_TE_RECORD_BYTES) &&
                   (tk_ref_mbf(mbf_telem_out, &rmbf) == E_OK) &&
                   (rmbf.maxmsz == (INT) S1_PUB_MAX_BYTES) &&
                   (tk_ref_mbf(mbf_telem_in, &rmbf) == E_OK) &&
                   (rmbf.maxmsz == (INT) S1_CMD_MAX_BYTES),
               "MBF_TELEM_EVT/OUT/IN created (4 / 43 / 8 B messages)");

    /* S1-U02: the task runs its 20 ms loop (about 50 per second). */
    s1t_wait_radio();
    subsystem1_stats(&st0);
    tm_printf((UB *) "[TEST] S1-U02 INFO  longest gap between loops since "
                     "boot = %u us\n",
              (unsigned) st0.max_gap_us);
    (void) tk_dly_tsk(1000);
    subsystem1_stats(&st1);
    tm_printf((UB *) "[TEST] S1-U02 INFO  longest gap during the test "
                     "window = %u us%s\n",
              (unsigned) st1.max_gap_us,
              (st1.max_gap_us > st0.max_gap_us) ? "" : " (no new maximum)");
    /* tk_dly_tsk(20) on a 10 ms tick sleeps 20-30 ms (the kernel adds one
       tick so it never wakes early), so the loop runs 33-50 times a second.
       Publishing rates do not depend on it: they use time deadlines. */
    tm_printf((UB *) "[TEST] S1-U02 INFO  %u loops in 1 s\n",
              (unsigned) (st1.loops - st0.loops));
    TEST_CHECK("S1-U02",
               ((st1.loops - st0.loops) >= 33U) &&
                   ((st1.loops - st0.loops) <= 51U),
               "TelemetryCommsTask loop runs 33-51 times per second");

    /* S1-U03: offline = WIFI_CONNECTING, WIFI_LOSS set, nothing published. */
    subsystem1_test_link(S1_LINK_FORCE_OFFLINE);
    (void) tk_dly_tsk(100);
    subsystem1_stats(&st0);
    (void) tk_dly_tsk(1000);
    subsystem1_stats(&st1);
    tm_printf(
        (UB *) "[TEST] S1-U03 INFO  conn %u, faults 0x%04x, snap %u->%u, "
               "evt %u->%u, loops %u->%u\n",
        (unsigned) s1t_conn(), (unsigned) g_fault_mask,
        (unsigned) st0.sent[S1_TOPIC_SNAP], (unsigned) st1.sent[S1_TOPIC_SNAP],
        (unsigned) st0.sent[S1_TOPIC_EVT], (unsigned) st1.sent[S1_TOPIC_EVT],
        (unsigned) st0.loops, (unsigned) st1.loops);
    TEST_CHECK("S1-U03",
               (APP_CONN_WIFI_CONNECTING == s1t_conn()) &&
                   (0U != (g_fault_mask & APP_FAULT_WIFI_LOSS)) &&
                   (st1.sent[S1_TOPIC_SNAP] == st0.sent[S1_TOPIC_SNAP]) &&
                   (st1.sent[S1_TOPIC_EVT] == st0.sent[S1_TOPIC_EVT]),
               "offline: WIFI_CONNECTING, WIFI_LOSS set, no publish for 1 s");

    /* S1-U04: producers never wait; overflow is dropped and counted. */
    drop0 = app_ipc_telem_dropped();
    for (i = 0U; i < 40U; i++)
    {
        uint32_t const t0 = app_hw_time_us();
        app_ipc_telem_post(APP_TEV_NONE, APP_SRC_SHARED, (uint16_t) i);
        {
            uint32_t const dt = (uint32_t) (app_hw_time_us() - t0);
            max_us            = (dt > max_us) ? dt : max_us;
        }
    }
    tm_printf((UB *) "[TEST] S1-U04 INFO  longest post = %u us\n",
              (unsigned) max_us);
    TEST_CHECK("S1-U04", (max_us <= 50U) && (app_ipc_telem_dropped() > drop0),
               "40 posts while offline: each <= 50 us, drops counted");

    /* S1-U05..U07: online. The blackboard -> wire path (S1-U06) is proved by
       comparing every snapshot with the live words the task read for it
       (s1t_check_snap_inputs); the test writes nothing into another task's
       blackboard. */
    s1t_seen_init(&seen);
    subsystem1_test_link(S1_LINK_FORCE_ONLINE);
    t_on = app_ipc_ms();
    s1t_drain(2100U, &seen);

    TEST_CHECK("S1-U05",
               (APP_CONN_ONLINE == s1t_conn()) &&
                   (0U == (g_fault_mask &
                           (APP_FAULT_WIFI_LOSS | APP_FAULT_BROKER_LOSS))) &&
                   (seen.n[S1_TOPIC_SNAP] >= 10U) &&
                   (seen.n[S1_TOPIC_SNAP] <= 12U) &&
                   (0U == seen.snap_seq_gaps) && (0U == seen.snap_decode_fail),
               "online: 10-12 snapshots in 2.1 s (200 ms), seq unbroken, all "
               "decode");
    tm_printf((UB *) "[TEST] S1-U05 INFO  snap %u, hb %u, evt %u, obs %u\n",
              (unsigned) seen.n[S1_TOPIC_SNAP], (unsigned) seen.n[S1_TOPIC_HB],
              (unsigned) seen.n[S1_TOPIC_EVT], (unsigned) seen.n[S1_TOPIC_OBS]);

    tm_printf((UB *) "[TEST] S1-U06 INFO  %u snapshots compared with their "
                     "blackboard inputs, %u mismatched; last: err=%d on=%u "
                     "hump_max=%u active=%u class=%u\n",
              (unsigned) seen.snap_checked, (unsigned) seen.snap_mismatch,
              (int) seen.last_snap.line_error,
              (unsigned) seen.last_snap.on_line,
              (unsigned) seen.last_snap.hump_max_mm,
              (unsigned) seen.last_snap.hump_active,
              (unsigned) seen.last_snap.motion_class);
    TEST_CHECK("S1-U06",
               (seen.snap_checked >= S1T_U06_MIN_CHECKED) &&
                   (0U == seen.snap_mismatch),
               "snapshot carries the live blackboard values (line, IMU "
               "fields) it was built from");

    tm_printf((UB *) "[TEST] S1-U07 INFO  hb %u (decoded %u, conn %u), evt %u, "
                     "drop report %s\n",
              (unsigned) seen.n[S1_TOPIC_HB], (unsigned) seen.have_hb,
              (unsigned) seen.last_hb.conn_state,
              (unsigned) seen.n[S1_TOPIC_EVT],
              (0U != seen.dropped_evt) ? "seen" : "MISSING");
    TEST_CHECK("S1-U07",
               seen.have_hb && (seen.n[S1_TOPIC_HB] >= 2U) &&
                   (seen.n[S1_TOPIC_HB] <= 4U) &&
                   ((uint8_t) APP_CONN_ONLINE == seen.last_hb.conn_state) &&
                   (seen.n[S1_TOPIC_EVT] >= 32U) && (0U != seen.dropped_evt),
               "heartbeat 1 s (2-4 in 2.1 s), backlog drained, drop count "
               "reported");

    /* S1-U08: inbound command validation (D-02, R7). Its last step delivers
       a VALID remote START; in an armed build the controller would act on it
       and drive the car during the test run (29 Oct calibration run), so the
       check runs only in disarmed builds. */
#if APP_CAL_MOTION_ARMED
    tm_printf((UB *) "[TEST] S1-U08 NOT RUN (motion armed: a valid START "
                     "would start a real run)\n");
    (void) gen0;
    (void) st0;
    (void) st1;
    (void) t_on;
    (void) &s1t_inject;
#else
    gen0 = g_remote_cmd_gen;
    subsystem1_stats(&st0);
    s1t_inject(APP_REMOTE_CMD_START, 10U, true, APP_CMD_FRAME_BYTES);
    subsystem1_stats(&st1);
    {
        bool const crc_ok = ((uint32_t) APP_CMD_BAD_CRC == st1.last_reject) &&
                            (g_remote_cmd_gen == gen0);
        s1t_inject(APP_REMOTE_CMD_START, 10U, false, 4U);
        subsystem1_stats(&st1);
        {
            bool const len_ok =
                ((uint32_t) APP_CMD_BAD_LENGTH == st1.last_reject);
            s1t_inject(APP_REMOTE_CMD_START, 10U, false, APP_CMD_FRAME_BYTES);
            subsystem1_stats(&st1);
            {
                bool const guard_ok =
                    ((uint32_t) APP_CMD_START_GUARD == st1.last_reject) &&
                    (g_remote_cmd_gen == gen0);
                while ((uint32_t) (app_ipc_ms() - t_on) <
                       (APP_CAL_T_REMOTE_START_GUARD_MS + 200U))
                {
                    s1t_drain(100U, &seen);
                }
                s1t_inject(APP_REMOTE_CMD_START, 10U, false,
                           APP_CMD_FRAME_BYTES);
                {
                    bool const accept_ok =
                        (g_remote_cmd_gen == (uint8_t) (gen0 + 1U)) &&
                        (APP_REMOTE_CMD_START ==
                         app_remote_cmd_type(g_remote_cmd)) &&
                        (10U == app_remote_cmd_seq(g_remote_cmd)) &&
                        !app_stop_latched();
                    s1t_inject(APP_REMOTE_CMD_START, 10U, false,
                               APP_CMD_FRAME_BYTES);
                    subsystem1_stats(&st1);
                    TEST_CHECK(
                        "S1-U08",
                        crc_ok && len_ok && guard_ok && accept_ok &&
                            ((uint32_t) APP_CMD_REPLAY == st1.last_reject) &&
                            (g_remote_cmd_gen == (uint8_t) (gen0 + 1U)) &&
                            ((st1.cmd_rejected - st0.cmd_rejected) == 4U),
                        "bad CRC, bad length, START in guard, replay "
                        "rejected; valid START accepted once");
                }
            }
        }
    }
#endif /* S1-U08 */
    s1t_drain(200U, &seen);

    /* S1-U09: OUT full (bridge stalled 500 ms) -> task keeps running, events
       stay queued in MBF_TELEM_EVT and none is lost when it resumes. */
    s1t_seen_init(&seen);
    subsystem1_stats(&st0);
    for (i = 0U; i < S1T_MARK_COUNT; i++)
    {
        app_ipc_telem_post(APP_TEV_RECOVERY, APP_SRC_SHARED,
                           (uint16_t) (S1T_MARK_BASE | i));
    }
    (void) tk_dly_tsk(500); /* nobody drains OUT */
    subsystem1_stats(&st1);
    {
        bool const kept_running = (st1.loops - st0.loops) >= 14U;
        bool const saw_full     = st1.out_full > st0.out_full;
        s1t_drain(1000U, &seen);
        TEST_CHECK("S1-U09",
                   kept_running && saw_full && (S1T_MARK_COUNT == seen.marks),
                   "bridge stalled 500 ms: loop runs, OUT full counted, all "
                   "10 events delivered after");
    }

    /* S1-U10: timing isolation (NFR17 proxy). While telemetry is busy the
       10 ms cyclic service still runs 100 times per second, and the loop
       body stays within its 2 ms budget. */
    cyc0 = app_ipc_cyclic_count();
    for (i = 0U; i < 20U; i++)
    {
        app_ipc_telem_post(APP_TEV_NONE, APP_SRC_SHARED, (uint16_t) i);
    }
    s1t_drain(1000U, &seen);
    subsystem1_stats(&st1);
    {
        uint32_t const cyc = app_ipc_cyclic_count() - cyc0;
        tm_printf((UB *) "[TEST] S1-U10 INFO  cyclic %u per s, longest "
                         "telemetry loop %u us\n",
                  (unsigned) cyc, (unsigned) st1.max_work_us);
        TEST_CHECK("S1-U10",
                   (cyc >= 98U) && (cyc <= 102U) && (st1.max_work_us <= 2000U),
                   "cyclic 100 +-2 per s under telemetry load; loop body <= "
                   "2 ms");
    }

    subsystem1_test_link(S1_LINK_REAL);
    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
