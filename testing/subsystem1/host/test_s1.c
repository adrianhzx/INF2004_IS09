/** @file test_s1.c
 *  @brief Host unit tests for subsystem 1 pure logic (s1_telem.c).
 *         make -C testing/subsystem1/host && ./testing/subsystem1/host/test_s1
 *  Test IDs: H-S1-PUB, H-S1-CON, H-S1-CAD, H-S1-CMD, H-S1-SNAP
 *  (testing/subsystem1/TEST_LOG.md).
 */

#include <stdio.h>
#include <string.h>

#include "../../../app_program/shared/app_barcode.h"
#include "../../../app_program/shared/app_cal.h"
#include "../../../app_program/subsystem1/s1_telem.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond)                                                        \
    do                                                                         \
    {                                                                          \
        if (cond)                                                              \
        {                                                                      \
            g_pass++;                                                          \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            g_fail++;                                                          \
            printf("FAIL %s (%s:%d)\n", (id), __FILE__, __LINE__);             \
        }                                                                      \
    } while (0)

/* ---- publish requests ------------------------------------------------------
 */
static void
test_pub (void)
{
    uint8_t         msg[S1_PUB_MAX_BYTES];
    uint8_t         pl[S1_PAYLOAD_MAX_BYTES + 1U];
    char            topic[S1_TOPIC_MAX_BYTES + 1U];
    uint8_t const * p_out = NULL;
    uint32_t        plen  = 0U;
    uint32_t        n     = 0U;
    uint32_t        t     = 0U;
    uint32_t        i     = 0U;

    for (i = 0U; i < sizeof(pl); i++)
    {
        pl[i] = (uint8_t) (0xC0U + i);
    }
    for (t = 0U; t < (uint32_t) S1_TOPIC_COUNT; t++)
    {
        char const * const name = s1_topic_name((s1_topic_t) t);
        n = s1_pub_pack((s1_topic_t) t, pl, S1_PAYLOAD_MAX_BYTES, msg);
        CHECK("H-S1-PUB-01 pack fits", (n > 0U) && (n <= S1_PUB_MAX_BYTES));
        CHECK("H-S1-PUB-02 unpack",
              s1_pub_unpack(msg, n, topic, &p_out, &plen));
        CHECK("H-S1-PUB-03 topic", 0 == strcmp(topic, name));
        CHECK("H-S1-PUB-04 payload",
              (plen == S1_PAYLOAD_MAX_BYTES) && (0 == memcmp(p_out, pl, plen)));
        CHECK("H-S1-PUB-05 topic base",
              0 == strncmp(name, "is09/is09-car/", 14U));
    }
    CHECK("H-S1-PUB-06 oversize payload refused",
          0U == s1_pub_pack(S1_TOPIC_SNAP, pl, S1_PAYLOAD_MAX_BYTES + 1U, msg));
    CHECK("H-S1-PUB-07 bad topic refused",
          0U == s1_pub_pack(S1_TOPIC_COUNT, pl, 4U, msg));
    msg[0] = 0U;
    CHECK("H-S1-PUB-08 zero topic length rejected",
          !s1_pub_unpack(msg, 10U, topic, &p_out, &plen));
    msg[0] = 30U;
    CHECK("H-S1-PUB-09 topic longer than message rejected",
          !s1_pub_unpack(msg, 10U, topic, &p_out, &plen));
    CHECK("H-S1-PUB-10 command topic",
          0 == strcmp(S1_CMD_TOPIC, "is09/is09-car/cmd"));
}

/* ---- connection state
 * -------------------------------------------------------- */
static void
test_conn (void)
{
    CHECK("H-S1-CON-01",
          APP_CONN_WIFI_CONNECTING == s1_conn_derive(false, false, 0U));
    CHECK("H-S1-CON-02",
          APP_CONN_WIFI_CONNECTING == s1_conn_derive(false, true, 4U));
    CHECK("H-S1-CON-03",
          APP_CONN_MQTT_CONNECTING == s1_conn_derive(true, false, 0U));
    CHECK("H-S1-CON-04", APP_CONN_BACKOFF == s1_conn_derive(true, false, 2U));
    CHECK("H-S1-CON-05", APP_CONN_ONLINE == s1_conn_derive(true, true, 0U));
    CHECK("H-S1-CON-06",
          APP_FAULT_WIFI_LOSS == s1_conn_faults(APP_CONN_WIFI_CONNECTING));
    CHECK("H-S1-CON-07",
          APP_FAULT_BROKER_LOSS == s1_conn_faults(APP_CONN_MQTT_CONNECTING));
    CHECK("H-S1-CON-08",
          APP_FAULT_BROKER_LOSS == s1_conn_faults(APP_CONN_BACKOFF));
    CHECK("H-S1-CON-09", 0U == s1_conn_faults(APP_CONN_ONLINE));
    /* NFR17 / D-08: every bit this subsystem sets is diagnostic only. */
    CHECK("H-S1-CON-10", 0U == ((APP_FAULT_WIFI_LOSS | APP_FAULT_BROKER_LOSS) &
                                (uint16_t) ~APP_FAULT_DIAGNOSTIC_MASK));
}

/* ---- cadence
 * ------------------------------------------------------------------ */
static uint32_t g_rng = 12345U;
static uint32_t
rnd (uint32_t mod)
{
    g_rng = (g_rng * 1103515245U) + 12345U;
    return (g_rng >> 16U) % mod;
}

static void
run_cadence (uint32_t start_ms, uint32_t secs, uint32_t * p_count,
             uint32_t * p_min_gap, uint32_t * p_max_gap)
{
    s1_cadence_t   cad;
    uint32_t       now  = start_ms;
    uint32_t       last = 0U;
    bool           have = false;
    uint32_t const end  = start_ms + (secs * 1000U);

    *p_count   = 0U;
    *p_min_gap = 0xFFFFFFFFU;
    *p_max_gap = 0U;
    s1_cadence_start(&cad, now, S1_SNAP_PERIOD_MS);
    while ((int32_t) (end - now) > 0)
    {
        if (s1_cadence_due(&cad, now))
        {
            if (have)
            {
                uint32_t const gap = now - last;
                *p_min_gap         = (gap < *p_min_gap) ? gap : *p_min_gap;
                *p_max_gap         = (gap > *p_max_gap) ? gap : *p_max_gap;
            }
            last = now;
            have = true;
            (*p_count)++;
        }
        now += 20U + rnd(11U); /* real loop: 20-30 ms on the 10 ms tick */
    }
}

static void
test_cadence (void)
{
    s1_cadence_t cad;
    uint32_t     count = 0U;
    uint32_t     mn    = 0U;
    uint32_t     mx    = 0U;
    uint32_t     due   = 0U;
    uint32_t     t     = 0U;

    run_cadence(5000U, 60U, &count, &mn, &mx);
    printf("H-S1-CAD: 60 s -> %u snapshots, gaps %u..%u ms\n", (unsigned) count,
           (unsigned) mn, (unsigned) mx);
    CHECK("H-S1-CAD-01 mean 5 Hz over 60 s (300 +-1)",
          (count >= 299U) && (count <= 301U));
    CHECK("H-S1-CAD-02 gap jitter within one loop (170..230 ms)",
          (mn >= 170U) && (mx <= 230U));

    /* across the 32-bit millisecond wrap */
    run_cadence(0xFFFF0000U, 120U, &count, &mn, &mx);
    CHECK("H-S1-CAD-03 no drift across ms wrap (600 +-1)",
          (count >= 599U) && (count <= 601U));

    /* a 2 s stall produces one due, not a burst of ten */
    s1_cadence_start(&cad, 0U, S1_SNAP_PERIOD_MS);
    (void) s1_cadence_due(&cad, 0U);
    for (t = 2000U; t < 2100U; t += 20U)
    {
        due += s1_cadence_due(&cad, t) ? 1U : 0U;
    }
    CHECK("H-S1-CAD-04 stall resynchronises without burst", 1U == due);
}

/* ---- commands
 * --------------------------------------------------------------------- */
static app_cmd_verdict_t
send (s1_cmd_ctx_t * p_ctx, app_remote_cmd_t cmd, uint8_t seq, uint32_t now,
      int corrupt, uint32_t len)
{
    uint8_t          f[S1_CMD_MAX_BYTES] = {0U};
    app_remote_cmd_t got                 = APP_REMOTE_CMD_INVALID0;
    uint8_t          gseq                = 0U;

    app_cmd_frame_encode(cmd, seq, f);
    if (1 == corrupt)
    {
        f[APP_CMD_OFF_CRC] ^= 0x01U;
    }
    if (2 == corrupt)
    {
        f[APP_FR_OFF_VER]  = 2U;
        f[APP_CMD_OFF_CRC] = app_crc8(f, APP_CMD_CRC_LEN);
    }
    return s1_cmd_filter(p_ctx, f, len, now, &got, &gseq);
}

static void
test_cmd (void)
{
    s1_cmd_ctx_t   c;
    uint32_t const g  = APP_CAL_T_REMOTE_START_GUARD_MS;
    uint32_t const on = 10000U;

    s1_cmd_init(&c);
    CHECK("H-S1-CMD-01 START before any ONLINE rejected",
          APP_CMD_START_GUARD ==
              send(&c, APP_REMOTE_CMD_START, 1U, 500U, 0, 3U));
    s1_cmd_on_online(&c, on);
    CHECK("H-S1-CMD-02 START inside guard rejected",
          APP_CMD_START_GUARD ==
              send(&c, APP_REMOTE_CMD_START, 7U, on + g - 1U, 0, 3U));
    CHECK("H-S1-CMD-03 guard rejection does not consume seq",
          APP_CMD_OK == send(&c, APP_REMOTE_CMD_START, 7U, on + g, 0, 3U));
    CHECK("H-S1-CMD-04 replay of same seq rejected",
          APP_CMD_REPLAY ==
              send(&c, APP_REMOTE_CMD_START, 7U, on + g + 50U, 0, 3U));
    CHECK("H-S1-CMD-05 STOP accepted",
          APP_CMD_OK == send(&c, APP_REMOTE_CMD_STOP, 8U, on + g + 60U, 0, 3U));
    CHECK("H-S1-CMD-06 bad CRC",
          APP_CMD_BAD_CRC ==
              send(&c, APP_REMOTE_CMD_STOP, 9U, on + g + 70U, 1, 3U));
    CHECK("H-S1-CMD-07 bad version",
          APP_CMD_BAD_VERSION ==
              send(&c, APP_REMOTE_CMD_STOP, 9U, on + g + 70U, 2, 3U));
    CHECK("H-S1-CMD-08 short frame",
          APP_CMD_BAD_LENGTH ==
              send(&c, APP_REMOTE_CMD_STOP, 9U, on + g + 70U, 0, 2U));
    CHECK("H-S1-CMD-09 truncated long payload (8 B)",
          APP_CMD_BAD_LENGTH ==
              send(&c, APP_REMOTE_CMD_STOP, 9U, on + g + 70U, 0, 8U));
    CHECK("H-S1-CMD-10 type 0 and 3 invalid",
          (APP_CMD_BAD_TYPE ==
           send(&c, APP_REMOTE_CMD_INVALID0, 9U, on + g + 70U, 0, 3U)) &&
              (APP_CMD_BAD_TYPE ==
               send(&c, APP_REMOTE_CMD_INVALID3, 9U, on + g + 70U, 0, 3U)));

    /* reconnect: guard re-armed, replay memory kept */
    s1_cmd_on_offline(&c);
    CHECK("H-S1-CMD-11 START while offline rejected",
          APP_CMD_START_GUARD ==
              send(&c, APP_REMOTE_CMD_START, 20U, on + 20000U, 0, 3U));
    s1_cmd_on_online(&c, on + 30000U);
    CHECK("H-S1-CMD-12 retained START right after reconnect rejected",
          APP_CMD_START_GUARD ==
              send(&c, APP_REMOTE_CMD_START, 21U, on + 30010U, 0, 3U));
    CHECK("H-S1-CMD-13 STOP inside reconnect guard still accepted",
          APP_CMD_OK == send(&c, APP_REMOTE_CMD_STOP, 22U, on + 30020U, 0, 3U));
    CHECK("H-S1-CMD-14 replay across reconnect rejected",
          APP_CMD_REPLAY ==
              send(&c, APP_REMOTE_CMD_STOP, 22U, on + 30030U, 0, 3U));
    CHECK("H-S1-CMD-15 null frame",
          APP_CMD_BAD_LENGTH == s1_cmd_filter(&c, NULL, 3U, on, NULL, NULL));
}

/* ---- snapshot
 * ---------------------------------------------------------------------- */
static void
test_snapshot (void)
{
    s1_inputs_t            in;
    app_snapshot_t         snap;
    app_snapshot_t         back;
    uint8_t                frame[APP_SNAPSHOT_BYTES];
    app_mission_status_t   ms  = {(uint8_t) APP_SYS_S7_SAFE_STOP, 2U, 1U, 9U};
    app_line_state_t       ls  = {-100, 40, true, 1U, 3U};
    app_imu_state_t        is  = {-20, 1U, 4U, 50U, 127U, 5U, false};
    app_obstacle_profile_t op  = {511U, 18U, 30U, -20, 25, 140U, 3U, 2U, 7U};
    bool                   sat = false;

    memset(&in, 0, sizeof(in));
    in.mission_word        = app_mission_pack(&ms);
    in.motion.speed_l_cm_s = 127U;
    in.motion.speed_r_cm_s = 3U;
    in.motion.distance_mm  = -32000;
    in.motion.motion_state = 4U;
    in.enc_l               = 65535U;
    in.enc_r               = 1U;
    (void) app_line_pack(&ls, &in.line_word);
    in.barcode_byte = app_barcode_pack((app_barcode_result_t) 4, 6U);
    (void) app_imu_pack(&is, &in.imu_word);
    (void) app_obstacle_pack(&op, &in.obst_w0, &in.obst_w1);
    in.stop_cause = (uint8_t) APP_STOP_CAUSE_OPERATOR_STOP;
    in.fault_mask = (uint16_t) (APP_FAULT_WIFI_LOSS | APP_FAULT_NO_ECHO);

    s1_snapshot_build(&in, 200U, &snap);
    sat = app_snapshot_encode(&snap, frame);
    CHECK("H-S1-SNAP-01 decodes", app_snapshot_decode(frame, &back));
    CHECK("H-S1-SNAP-02 no saturation for in-range fields", !sat);
    CHECK("H-S1-SNAP-03 header and safety fields",
          (back.seq == 200U) && (back.sys_state == 7U) &&
              (back.stop_cause == 1U) &&
              (back.fault_mask == (APP_FAULT_WIFI_LOSS | APP_FAULT_NO_ECHO)));
    CHECK("H-S1-SNAP-04 motion fields",
          (back.motion_state == 4U) && (back.speed_l == 127U) &&
              (back.speed_r == 3U) && (back.enc_l == 65535U) &&
              (back.enc_r == 1U) && (back.distance_mm == -32000));
    CHECK("H-S1-SNAP-05 line and barcode",
          (back.line_error == -100) && back.on_line && (back.barcode == 4U));
    CHECK("H-S1-SNAP-06 IMU fields", (back.hump_max_mm == 127U) &&
                                         !back.hump_active &&
                                         (back.motion_class == 5U));
    CHECK("H-S1-SNAP-07 obstacle fields (511 = no echo)",
          (back.range_cm == 511U) && (back.action == 3U));
    CHECK("H-S1-SNAP-08 frame is 18 bytes", 18U == APP_SNAPSHOT_BYTES);
}

int
main (void)
{
    test_pub();
    test_conn();
    test_cadence();
    test_cmd();
    test_snapshot();
    printf("subsystem1 host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
