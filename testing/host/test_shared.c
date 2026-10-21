/** @file test_shared.c
 *  @brief Host unit tests for the shared data layer (Phase 1).
 *         Test IDs: H-PK (pack), H-BB (blackboards), H-ER (edge ring),
 *         H-TF (telemetry frames), H-CMD (command frame), H-SF (safety).
 *         Also emits vectors.txt consumed by tools/telem_decode.py --selftest.
 */

#include <stdio.h>
#include <string.h>

#include "app_barcode.h"
#include "app_data.h"
#include "app_imu.h"
#include "app_ipc.h"
#include "app_line.h"
#include "app_motion.h"
#include "app_obstacle.h"
#include "app_pack.h"
#include "app_safety.h"
#include "app_telemetry.h"
#include "host_port.h"

/* Test vectors are literal expected values by nature; the magic-number and
   function-size checks are suspended for this file only (reviewed). */
/* NOLINTBEGIN(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, desc)                                                  \
    do                                                                         \
    {                                                                          \
        if (cond)                                                              \
        {                                                                      \
            g_pass++;                                                          \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            g_fail++;                                                          \
            printf("FAIL %s %s (%s:%d)\n", id, desc, __FILE__, __LINE__);      \
        }                                                                      \
    } while (0)

static FILE * gp_vec = NULL;

static void
hex_line (char const * tag, uint8_t const * p, uint32_t n, char const * fields)
{
    uint32_t i = 0U;
    if (NULL == gp_vec)
    {
        return;
    }
    fprintf(gp_vec, "%s ", tag);
    for (i = 0U; i < n; i++)
    {
        fprintf(gp_vec, "%02x", p[i]);
    }
    fprintf(gp_vec, " %s\n", fields);
}

/* ---- H-PK: generic pack helpers ------------------------------------------ */
static void
test_pack (void)
{
    bool    sat = false;
    int32_t v   = 0;

    CHECK("H-PK-01", 0xFFU == app_pack_mask(8U), "mask 8");
    CHECK("H-PK-02", 0xFFFFFFFFU == app_pack_mask(32U), "mask 32");
    CHECK("H-PK-03", 0x1U == app_pack_mask(1U), "mask 1");

    /* signed round trips at the edges for 7, 8, 16 bits */
    sat = false;
    v   = app_pack_u_to_s(app_pack_s_to_u(-128, 8U), 8U);
    CHECK("H-PK-04", -128 == v, "int8 min");
    v = app_pack_u_to_s(app_pack_s_to_u(127, 8U), 8U);
    CHECK("H-PK-05", 127 == v, "int8 max");
    v = app_pack_u_to_s(app_pack_s_to_u(-1, 8U), 8U);
    CHECK("H-PK-06", -1 == v, "int8 -1");
    v = app_pack_u_to_s(app_pack_s_to_u(0, 8U), 8U);
    CHECK("H-PK-07", 0 == v, "int8 0");
    v = app_pack_u_to_s(app_pack_s_to_u(-32768, 16U), 16U);
    CHECK("H-PK-08", -32768 == v, "int16 min");
    v = app_pack_u_to_s(app_pack_s_to_u(32767, 16U), 16U);
    CHECK("H-PK-09", 32767 == v, "int16 max");
    v = app_pack_u_to_s(app_pack_s_to_u(-64, 7U), 7U);
    CHECK("H-PK-10", -64 == v, "int7 min");
    v = app_pack_u_to_s(app_pack_s_to_u(63, 7U), 7U);
    CHECK("H-PK-11", 63 == v, "int7 max");
    v = app_pack_u_to_s(0x80000000U, 32U);
    CHECK("H-PK-12", (-2147483647 - 1) == v, "int32 min via 32-bit field");

    /* saturation flags */
    sat = false;
    v   = app_pack_sat_s(200, 8U, &sat);
    CHECK("H-PK-13", (127 == v) && sat, "int8 +sat");
    sat = false;
    v   = app_pack_sat_s(-200, 8U, &sat);
    CHECK("H-PK-14", (-128 == v) && sat, "int8 -sat");
    sat = false;
    CHECK("H-PK-15", (127U == app_pack_sat_u(300U, 7U, &sat)) && sat, "u7 sat");
    sat = false;
    CHECK("H-PK-16", (127U == app_pack_sat_u(127U, 7U, &sat)) && !sat,
          "u7 edge no sat");

    /* example 2 from the master prompt: steering error -100 */
    {
        uint32_t w = 0U;
        sat        = false;
        w          = app_pack_set_s(w, 8U, 8U, -100, &sat);
        CHECK("H-PK-17", (0x9C00U == w) && !sat,
              "-100 encoded as 0x9C in bits 8-15");
        CHECK("H-PK-18", -100 == app_pack_get_s(w, 8U, 8U),
              "-100 decoded with sign extension");
        w = app_pack_set_s(w, 8U, 8U, 100, &sat);
        CHECK("H-PK-19", 100 == app_pack_get_s(w, 8U, 8U), "+100 round trip");
    }
    /* field insert must not disturb neighbours */
    {
        uint32_t w = 0xFFFFFFFFU;
        w          = app_pack_set(w, 4U, 4U, 0x0U);
        CHECK("H-PK-20", 0xFFFFFF0FU == w,
              "clearing a nibble leaves neighbours");
        w = app_pack_set(w, 4U, 4U, 0xFFU); /* over-wide value is masked */
        CHECK("H-PK-21", 0xFFFFFFFFU == w,
              "over-wide value is masked to the field");
    }
    /* little-endian helpers */
    {
        uint8_t b[4];
        app_pack_le32(b, 0x11223344U);
        CHECK("H-PK-22", (0x44U == b[0]) && (0x11U == b[3]), "le32 byte order");
        CHECK("H-PK-23", 0x11223344U == app_unpack_le32(b), "le32 round trip");
        app_pack_le16(b, 0xABCDU);
        CHECK("H-PK-24", (0xCDU == b[0]) && (0xABCDU == app_unpack_le16(b)),
              "le16");
    }
    CHECK("H-PK-25", 255U == app_pack_inc_sat8(255U),
          "sat8 increment stays at 255");
}

/* ---- H-BB: blackboards --------------------------------------------------- */
static void
test_blackboards (void)
{
    bool sat = false;

    /* LineState edges */
    {
        app_line_state_t in;
        app_line_state_t out;
        uint32_t         w = 0U;
        in.line_error      = APP_LINE_ERROR_MIN;
        in.steer           = APP_LINE_ERROR_MAX;
        in.on_line         = true;
        in.event           = APP_LINE_EVT_LOST;
        in.seq             = 15U;
        sat                = app_line_pack(&in, &w);
        app_line_unpack(w, &out);
        CHECK("H-BB-01",
              !sat && (in.line_error == out.line_error) &&
                  (in.steer == out.steer) && out.on_line &&
                  (in.event == out.event) && (15U == out.seq),
              "LineState edge round trip");
        CHECK("H-BB-02", 0U == (w >> APP_LS_USED_BITS),
              "LineState upper 9 bits clear");
        in.event = 5U; /* invalid enum -> saturates to 3 and flags */
        sat      = app_line_pack(&in, &w);
        CHECK("H-BB-03", sat, "invalid LineState event flagged");
    }
    /* MotionStatus incl. negative distance, cycle tag coherence */
    {
        app_motion_status_t in;
        app_motion_status_t out;
        uint32_t            a = 0U;
        uint32_t            b = 0U;
        memset(&in, 0, sizeof in);
        in.speed_l_cm_s  = 127U;
        in.speed_r_cm_s  = 1U;
        in.distance_mm   = -1234;
        in.cycle_tag     = 2U;
        in.motion_state  = APP_MOTION_TURNING;
        in.done_seq      = 15U;
        in.active_seq    = 3U;
        in.dir_r_reverse = true;
        in.done_result   = APP_MOTION_RESULT_ABORTED;
        sat              = app_motion_pack(&in, &a, &b);
        app_motion_unpack(a, b, &out);
        CHECK("H-BB-04",
              !sat && (-1234 == out.distance_mm) &&
                  (127U == out.speed_l_cm_s) && (1U == out.speed_r_cm_s) &&
                  (2U == out.cycle_tag) &&
                  (APP_MOTION_TURNING == out.motion_state) &&
                  (15U == out.done_seq) && (3U == out.active_seq) &&
                  out.dir_r_reverse && !out.dir_l_reverse &&
                  (APP_MOTION_RESULT_ABORTED == out.done_result),
              "MotionStatus round trip");
        CHECK("H-BB-05", 0U == (b >> APP_MSB_USED_BITS),
              "motion word B upper bits clear");
        in.speed_l_cm_s = 200U;
        sat             = app_motion_pack(&in, &a, &b);
        CHECK("H-BB-06",
              sat && (127U == app_pack_get(a, APP_MSA_SPEED_L_SHIFT, 7U)),
              "speed > 127 saturates and flags");
        /* coherent read: tags equal */
        g_motion_status_a = a;
        g_motion_status_b = b;
        CHECK("H-BB-07", app_motion_read(&out),
              "coherent read with equal tags");
        g_motion_status_b = app_pack_set(b, APP_MSB_TAG_SHIFT, 2U, 1U);
        CHECK("H-BB-08", !app_motion_read(&out),
              "mixed-cycle pair detected (D-04)");
    }
    /* Motion command */
    {
        app_motion_cmd_t in;
        app_motion_cmd_t out;
        uint32_t         r = 0U;
        in.type            = APP_MC_TURN;
        in.speed_cm_s      = 15U;
        in.param           = -900; /* -90.0 deg */
        in.until_line      = true;
        in.seq             = 9U;
        sat                = app_motion_cmd_pack(&in, &r);
        app_motion_cmd_unpack(r, &out);
        CHECK("H-BB-09",
              !sat && (APP_MC_TURN == out.type) && (15U == out.speed_cm_s) &&
                  (-900 == out.param) && out.until_line && (9U == out.seq),
              "MotionCmd TURN -90.0 until_line round trip");
        CHECK("H-BB-10", 0U == (r >> APP_MC_USED_BITS),
              "MotionCmd 30 meaningful bits");
        in.param = -32768;
        in.type  = APP_MC_MOVE;
        (void) app_motion_cmd_pack(&in, &r);
        app_motion_cmd_unpack(r, &out);
        CHECK("H-BB-11", -32768 == out.param, "MOVE reverse distance min");
    }
    /* ImuState exactly 32 bits, saturation (example 3) */
    {
        app_imu_state_t in;
        app_imu_state_t out;
        uint32_t        w = 0U;
        in.pitch_half_deg = -127;
        in.quality        = APP_HUMP_Q_DEGRADED;
        in.episode_seq    = 15U;
        in.height_mm      = 127U;
        in.max_height_mm  = 100U;
        in.motion_class   = APP_MOTION_CLASS_IMPACT;
        in.hump_active    = true;
        sat               = app_imu_pack(&in, &w);
        app_imu_unpack(w, &out);
        CHECK("H-BB-12",
              !sat && (-127 == out.pitch_half_deg) &&
                  (APP_HUMP_Q_DEGRADED == out.quality) &&
                  (15U == out.episode_seq) && (127U == out.height_mm) &&
                  (100U == out.max_height_mm) &&
                  (APP_MOTION_CLASS_IMPACT == out.motion_class) &&
                  out.hump_active,
              "ImuState round trip");
        in.height_mm =
            200U; /* over-range height must saturate and flag, not wrap */
        sat = app_imu_pack(&in, &w);
        app_imu_unpack(w, &out);
        CHECK("H-BB-13", sat && (127U == out.height_mm),
              "hump height 200 -> 127 flagged");
        CHECK("H-BB-14", 127U == app_imu_height_sat(128U, &sat),
              "height helper saturates at 127");
    }
    /* ObstacleProfile two words */
    {
        app_obstacle_profile_t in;
        app_obstacle_profile_t out;
        uint32_t               w0 = 0U;
        uint32_t               w1 = 0U;
        in.range_cm               = 511U;
        in.scan_angle_idx         = 36U;
        in.width_cm               = 63U;
        in.y_min_cm               = -64;
        in.y_max_cm               = 63;
        in.closest_cm             = 25U;
        in.action                 = APP_AVOID_REVERSE;
        in.scan_state             = 7U;
        in.seq                    = 15U;
        sat                       = app_obstacle_pack(&in, &w0, &w1);
        app_obstacle_unpack(w0, w1, &out);
        CHECK("H-BB-15",
              !sat && (511U == out.range_cm) && (36U == out.scan_angle_idx) &&
                  (63U == out.width_cm) && (-64 == out.y_min_cm) &&
                  (63 == out.y_max_cm) && (25U == out.closest_cm) &&
                  (APP_AVOID_REVERSE == out.action) && (7U == out.scan_state) &&
                  (15U == out.seq),
              "ObstacleProfile round trip");
        CHECK("H-BB-16", (0U == (w0 >> 28U)) && (0U == (w1 >> 26U)),
              "ObstacleProfile reserved bits clear");
        in.range_cm = 600U;
        sat         = app_obstacle_pack(&in, &w0, &w1);
        CHECK("H-BB-17", sat, "range > 511 flagged");
        CHECK("H-BB-18", 511U == app_obstacle_us_to_cm(30000U),
              "30 ms echo -> sentinel");
        CHECK("H-BB-19", 10U == app_obstacle_us_to_cm(580U), "580 us -> 10 cm");
    }
    /* MissionStatus, comms word, remote cmd, barcode byte */
    {
        app_mission_status_t ms;
        app_mission_status_t mo;
        uint32_t             w  = 0U;
        uint16_t             cw = 0U;
        uint8_t              rc = 0U;
        uint8_t              bc = 0U;
        ms.sys_state            = APP_SYS_S7_SAFE_STOP;
        ms.pending_cmd          = APP_NAV_UTURN;
        ms.retry_count          = 3U;
        ms.seq                  = 15U;
        w                       = app_mission_pack(&ms);
        app_mission_unpack(w, &mo);
        CHECK("H-BB-20",
              (7U == mo.sys_state) && (4U == mo.pending_cmd) &&
                  (3U == mo.retry_count) && (15U == mo.seq) &&
                  (0U == (w >> APP_MS_USED_BITS)),
              "MissionStatus round trip, 12 used bits");
        cw = app_comms_pack(APP_CONN_BACKOFF, 10U, 255U);
        CHECK("H-BB-21",
              (3U == app_pack_get(cw, 0U, 2U)) &&
                  (10U == app_pack_get(cw, 2U, 4U)) &&
                  (255U == app_pack_get(cw, 6U, 8U)) &&
                  (0U == (cw >> APP_CW_USED_BITS)),
              "comms_word round trip");
        cw = app_comms_pack(APP_CONN_ONLINE, 12U, 0U);
        CHECK("H-BB-22", 10U == app_pack_get(cw, 2U, 4U),
              "backoff > 10 s saturates to 10");
        rc = app_remote_cmd_pack(APP_REMOTE_CMD_STOP, 63U);
        CHECK("H-BB-23",
              (APP_REMOTE_CMD_STOP == app_remote_cmd_type(rc)) &&
                  (63U == app_remote_cmd_seq(rc)),
              "RemoteCmd byte");
        bc = app_barcode_pack(APP_BC_INV_OVERFLOW, 15U);
        CHECK("H-BB-24",
              (APP_BC_INV_OVERFLOW == app_barcode_result(bc)) &&
                  (15U == app_barcode_seq(bc)),
              "barcode byte");
    }
}

/* ---- H-ER: edge ring ---------------------------------------------------- */
static void
test_edge_ring (void)
{
    app_edge_ring_t ring;
    uint16_t        e   = 0U;
    uint32_t        i   = 0U;
    bool            sat = false;

    memset(&ring, 0, sizeof ring);
    CHECK("H-ER-01", !app_edge_ring_pop(&ring, &e), "empty ring pops nothing");
    for (i = 0U; i < (APP_EDGE_RING_ENTRIES - 1U); i++)
    {
        CHECK("H-ER-02",
              app_edge_ring_push(&ring,
                                 app_edge_pack((i & 1U) != 0U, i * 8U, &sat)),
              "push until full-1");
    }
    CHECK("H-ER-03", !app_edge_ring_push(&ring, 0U) && (1U == ring.overflow),
          "64th push overflows and flags");
    CHECK("H-ER-04", (APP_EDGE_RING_ENTRIES - 1U) == app_edge_ring_count(&ring),
          "count 63");
    for (i = 0U; i < (APP_EDGE_RING_ENTRIES - 1U); i++)
    {
        CHECK("H-ER-05",
              app_edge_ring_pop(&ring, &e) &&
                  (app_edge_level(e) == ((i & 1U) != 0U)) &&
                  (app_edge_delta_us(e) == i * 8U),
              "pop in order with level and delta");
    }
    CHECK("H-ER-06", !app_edge_ring_pop(&ring, &e), "drained");
    /* wrap-around: head/tail cross the end */
    for (i = 0U; i < 200U; i++)
    {
        (void) app_edge_ring_push(&ring, (uint16_t) i);
        (void) app_edge_ring_pop(&ring, &e);
        CHECK("H-ER-07", e == (uint16_t) i, "wrap-around FIFO order");
    }
    sat = false;
    e   = app_edge_pack(true, 300000U, &sat); /* 300 ms > 262 ms */
    CHECK("H-ER-08", sat && (0x7FFFU == (e & 0x7FFFU)) && app_edge_level(e),
          "delta saturates at 0x7FFF");
    sat = false;
    e   = app_edge_pack(false, 150000U, &sat); /* T_QUIET fits */
    CHECK("H-ER-09", !sat && (150000U == app_edge_delta_us(e)),
          "150 ms fits without saturation");
}

/* ---- H-TF: telemetry frames --------------------------------------------- */
static void
test_frames (void)
{
    app_snapshot_t in;
    app_snapshot_t out;
    uint8_t        buf[APP_SNAPSHOT_BYTES];
    bool           sat = false;
    char           fields[256];

    memset(&in, 0, sizeof in);
    in.seq          = 200U;
    in.sys_state    = APP_SYS_S4_OBSTACLE_BYPASS;
    in.motion_state = APP_MOTION_MOVING;
    in.stop_cause   = APP_STOP_CAUSE_CRITICAL_RUNTIME;
    in.fault_mask   = 0x1FFFU;
    in.speed_l      = 127U;
    in.speed_r      = 64U;
    in.enc_l        = 0xFFFFU;
    in.enc_r        = 1U;
    in.distance_mm  = -32768;
    in.line_error   = -100;
    in.on_line      = true;
    in.barcode      = APP_BC_INV_OVERFLOW;
    in.hump_max_mm  = 127U;
    in.hump_active  = true;
    in.motion_class = APP_MOTION_CLASS_CRUISING;
    in.range_cm     = 511U;
    in.action       = APP_AVOID_REVERSE;
    sat             = app_snapshot_encode(&in, buf);
    CHECK("H-TF-01", !sat, "snapshot encode without saturation");
    CHECK("H-TF-02", (APP_FRAME_VERSION == buf[0]) && (200U == buf[1]),
          "snapshot ver/seq bytes");
    CHECK("H-TF-03", app_snapshot_decode(buf, &out), "snapshot decode ok");
    CHECK("H-TF-04",
          (in.sys_state == out.sys_state) &&
              (in.motion_state == out.motion_state) &&
              (in.stop_cause == out.stop_cause) &&
              (in.fault_mask == out.fault_mask) &&
              (in.speed_l == out.speed_l) && (in.speed_r == out.speed_r) &&
              (in.enc_l == out.enc_l) && (in.enc_r == out.enc_r) &&
              (in.distance_mm == out.distance_mm) &&
              (in.line_error == out.line_error) &&
              (in.on_line == out.on_line) && (in.barcode == out.barcode) &&
              (in.hump_max_mm == out.hump_max_mm) &&
              (in.hump_active == out.hump_active) &&
              (in.motion_class == out.motion_class) &&
              (in.range_cm == out.range_cm) && (in.action == out.action) &&
              (in.seq == out.seq),
          "snapshot full field round trip");
    CHECK("H-TF-05", 0U == (buf[17] & 0xF8U),
          "snapshot bits 139-143 reserved zero");
    (void) snprintf(fields, sizeof fields,
                    "seq=%u sys=%u mot=%u cause=%u faults=%u sl=%u sr=%u el=%u "
                    "er=%u dist=%d "
                    "lerr=%d onl=%u bc=%u hmax=%u hact=%u cls=%u rng=%u act=%u",
                    in.seq, in.sys_state, in.motion_state, in.stop_cause,
                    in.fault_mask, in.speed_l, in.speed_r, in.enc_l, in.enc_r,
                    in.distance_mm, in.line_error, in.on_line ? 1U : 0U,
                    in.barcode, in.hump_max_mm, in.hump_active ? 1U : 0U,
                    in.motion_class, in.range_cm, in.action);
    hex_line("SNAP", buf, APP_SNAPSHOT_BYTES, fields);

    /* second vector: all zero except version, plus positive values */
    memset(&in, 0, sizeof in);
    in.seq         = 1U;
    in.sys_state   = APP_SYS_S2_LINE_FOLLOWING;
    in.distance_mm = 12345;
    in.line_error  = 7;
    in.speed_l     = 20U;
    in.speed_r     = 21U;
    in.range_cm    = 100U;
    (void) app_snapshot_encode(&in, buf);
    (void) snprintf(fields, sizeof fields,
                    "seq=%u sys=%u mot=%u cause=%u faults=%u sl=%u sr=%u el=%u "
                    "er=%u dist=%d "
                    "lerr=%d onl=%u bc=%u hmax=%u hact=%u cls=%u rng=%u act=%u",
                    in.seq, in.sys_state, in.motion_state, in.stop_cause,
                    in.fault_mask, in.speed_l, in.speed_r, in.enc_l, in.enc_r,
                    in.distance_mm, in.line_error, 0U, in.barcode,
                    in.hump_max_mm, 0U, in.motion_class, in.range_cm,
                    in.action);
    hex_line("SNAP", buf, APP_SNAPSHOT_BYTES, fields);
    buf[0] = 2U;
    CHECK("H-TF-06", !app_snapshot_decode(buf, &out), "wrong version rejected");

    /* saturation reporting */
    in.range_cm = 1000U;
    sat         = app_snapshot_encode(&in, buf);
    CHECK("H-TF-07", sat, "range 1000 saturates and is reported");

    /* heartbeat */
    {
        app_heartbeat_t hb;
        app_heartbeat_t ho;
        uint8_t         h[APP_HB_FRAME_BYTES];
        hb.seq        = 9U;
        hb.uptime_s   = 65535U;
        hb.conn_state = APP_CONN_BACKOFF;
        hb.backoff_s  = 10U;
        hb.dropped    = 255U;
        app_heartbeat_encode(&hb, h);
        CHECK("H-TF-08",
              app_heartbeat_decode(h, &ho) && (9U == ho.seq) &&
                  (65535U == ho.uptime_s) && (3U == ho.conn_state) &&
                  (10U == ho.backoff_s) && (255U == ho.dropped),
              "heartbeat round trip");
        (void) snprintf(fields, sizeof fields,
                        "seq=%u up=%u conn=%u bo=%u drop=%u", hb.seq,
                        hb.uptime_s, hb.conn_state, hb.backoff_s, hb.dropped);
        hex_line("HB", h, APP_HB_FRAME_BYTES, fields);
    }
    /* obstacle frame */
    {
        uint8_t  o[APP_OBS_FRAME_BYTES];
        uint8_t  seq = 0U;
        uint32_t w0  = 0U;
        uint32_t w1  = 0U;
        app_obs_frame_encode(33U, 0xDEADBEEFU, 0x01234567U, o);
        CHECK("H-TF-09",
              app_obs_frame_decode(o, &seq, &w0, &w1) && (33U == seq) &&
                  (0xDEADBEEFU == w0) && (0x01234567U == w1),
              "obstacle frame round trip");
        hex_line("OBS", o, APP_OBS_FRAME_BYTES,
                 "seq=33 w0=3735928559 w1=19088743");
    }
    /* event record */
    {
        uint32_t r =
            app_ipc_telem_pack(APP_TEV_HUMP_END, APP_SRC_IMU, 0xBEEFU, 0xA5U);
        uint8_t  t  = 0U;
        uint8_t  s  = 0U;
        uint16_t p  = 0U;
        uint8_t  st = 0U;
        uint8_t  e[4];
        app_ipc_telem_unpack(r, &t, &s, &p, &st);
        CHECK("H-TF-10",
              (APP_TEV_HUMP_END == t) && (APP_SRC_IMU == s) && (0xBEEFU == p) &&
                  (0xA5U == st),
              "event record round trip");
        app_event_frame_encode(r, e);
        hex_line("EVT", e, 4U, "type=10 src=3 payload=48879 stamp=165");
    }
}

/* ---- H-CMD: command frame -------------------------------------------------
 */
static void
test_cmd (void)
{
    uint8_t           f[3];
    uint8_t           last        = 0xFFU;
    app_remote_cmd_t  cmd         = APP_REMOTE_CMD_INVALID0;
    uint8_t           seq         = 0U;
    app_cmd_verdict_t v           = APP_CMD_OK;
    uint8_t const     crc_check[] = {0x31U, 0x32U, 0x33U, 0x34U, 0x35U,
                                     0x36U, 0x37U, 0x38U, 0x39U};

    /* CRC-8 poly 0x07 of "123456789" is 0xF4 (standard check value) */
    CHECK("H-CMD-01", 0xF4U == app_crc8(crc_check, 9U),
          "CRC-8/0x07 check value 0xF4");

    app_cmd_frame_encode(APP_REMOTE_CMD_START, 5U, f);
    hex_line("CMD", f, 3U, "cmd=1 seq=5");
    v = app_cmd_frame_decode(f, 3U, &last, &cmd, &seq);
    CHECK("H-CMD-02",
          (APP_CMD_OK == v) && (APP_REMOTE_CMD_START == cmd) && (5U == seq) &&
              (5U == last),
          "valid START accepted");
    v = app_cmd_frame_decode(f, 3U, &last, &cmd, &seq);
    CHECK("H-CMD-03", APP_CMD_REPLAY == v, "same seq again is a replay (D-02)");
    v = app_cmd_frame_decode(f, 2U, &last, &cmd, &seq);
    CHECK("H-CMD-04", APP_CMD_BAD_LENGTH == v, "truncated frame rejected");
    f[2] ^= 0x01U;
    v = app_cmd_frame_decode(f, 3U, &last, &cmd, &seq);
    CHECK("H-CMD-05", APP_CMD_BAD_CRC == v, "corrupted CRC rejected");
    app_cmd_frame_encode(APP_REMOTE_CMD_STOP, 6U, f);
    hex_line("CMD", f, 3U, "cmd=2 seq=6");
    f[0] = 9U;
    f[2] = app_crc8(f, 2U);
    v    = app_cmd_frame_decode(f, 3U, &last, &cmd, &seq);
    CHECK("H-CMD-06", APP_CMD_BAD_VERSION == v,
          "wrong version rejected even with good CRC");
    f[0] = APP_FRAME_VERSION;
    f[1] = app_remote_cmd_pack(APP_REMOTE_CMD_INVALID3, 7U);
    f[2] = app_crc8(f, 2U);
    v    = app_cmd_frame_decode(f, 3U, &last, &cmd, &seq);
    CHECK("H-CMD-07", APP_CMD_BAD_TYPE == v, "reserved command code rejected");
    f[1] = app_remote_cmd_pack(APP_REMOTE_CMD_INVALID0, 7U);
    f[2] = app_crc8(f, 2U);
    v    = app_cmd_frame_decode(f, 3U, &last, &cmd, &seq);
    CHECK("H-CMD-08", APP_CMD_BAD_TYPE == v,
          "zero frame never decodes as START");
    /* all-zero payload (what a cleared retained message might look like) */
    f[0] = 0U;
    f[1] = 0U;
    f[2] = 0U;
    v    = app_cmd_frame_decode(f, 3U, &last, &cmd, &seq);
    CHECK("H-CMD-09", APP_CMD_OK != v, "all-zero frame rejected");
    /* every single-bit corruption of a valid START must be rejected */
    {
        uint32_t bit      = 0U;
        uint32_t accepted = 0U;
        app_cmd_frame_encode(APP_REMOTE_CMD_START, 11U, f);
        for (bit = 0U; bit < 24U; bit++)
        {
            uint8_t g[3];
            uint8_t l2 = 0xFFU;
            memcpy(g, f, 3U);
            g[bit / 8U] ^= (uint8_t) (1U << (bit % 8U));
            if (APP_CMD_OK == app_cmd_frame_decode(g, 3U, &l2, &cmd, &seq))
            {
                accepted++;
            }
        }
        CHECK("H-CMD-10", 0U == accepted,
              "no single-bit-flipped frame is accepted");
    }
}

/* ---- H-SF: safety -----------------------------------------------------------
 */
static volatile uint16_t g_race_word = 0U;

static void
isr_sets_bit_b (void)
{
    app_shared_bits_set(&g_race_word, 0x0002U);
}

static void
isr_requests_stop (void)
{
    app_safe_stop_request(APP_STOP_CAUSE_COLLISION);
}

static void
test_safety (void)
{
    uint32_t i    = 0U;
    uint32_t lost = 0U;

    app_safety_init();
    app_safety_bind(42, 7, APP_EVT_SAFE_STOP);
    host_current_tid   = 1;
    host_flg_set_count = 0U;

    CHECK("H-SF-01",
          !app_stop_latched() && (APP_STOP_CAUSE_NONE == app_stop_cause_get()),
          "boot: latch clear, cause NONE");

    /* helper: pended ISR writer runs at the end of the task's critical
       section; no bit lost over 10 000 iterations */
    for (i = 0U; i < 10000U; i++)
    {
        host_irq_hook = isr_sets_bit_b;
        app_shared_bits_set(&g_race_word, 0x0001U);
        if (0x0003U != (g_race_word & 0x0003U))
        {
            lost++;
        }
        app_shared_bits_clr(&g_race_word, 0x0003U);
    }
    CHECK("H-SF-02", 0U == lost,
          "no lost bit with ISR writer pended inside the helper");
    CHECK("H-SF-03", 1U == host_irq_max_depth, "helper never nests the mask");

    /* first cause wins, latch set before flag, flag raised once */
    host_flg_set_count = 0U;
    host_last_flg_ptn  = 0U;
    app_safe_stop_request(APP_STOP_CAUSE_STALL);
    CHECK("H-SF-04",
          app_stop_latched() && (APP_STOP_CAUSE_STALL == app_stop_cause_get()),
          "first request latches and records STALL");
    CHECK("H-SF-05",
          (1U == host_flg_set_count) &&
              (APP_EVT_SAFE_STOP == host_last_flg_ptn),
          "EVT_SAFE_STOP raised");
    app_safe_stop_request(APP_STOP_CAUSE_COLLISION);
    CHECK("H-SF-06", APP_STOP_CAUSE_STALL == app_stop_cause_get(),
          "second cause does not overwrite");

    /* race: ISR stop request pended while a task request is inside the RMW */
    app_safety_init();
    app_safety_bind(42, 7, APP_EVT_SAFE_STOP);
    host_irq_hook = isr_requests_stop;
    app_safe_stop_request(APP_STOP_CAUSE_OPERATOR_STOP);
    CHECK("H-SF-07",
          APP_STOP_CAUSE_OPERATOR_STOP == app_stop_cause_get() &&
              app_stop_latched(),
          "task request wins the race it entered first; latch set");

    /* controller-only clear */
    host_current_tid = 1;
    CHECK("H-SF-08", !app_safety_controller_clear() && app_stop_latched(),
          "foreign task cannot clear");
    host_current_tid = 42;
    CHECK("H-SF-09",
          app_safety_controller_clear() && !app_stop_latched() &&
              (APP_STOP_CAUSE_NONE == app_stop_cause_get()),
          "controller clears latch and cause");
    CHECK("H-SF-10", !app_safety_controller_clear(),
          "clearing twice reports nothing to clear");

    /* invalid cause never silently no-ops; REMOTE_STOP maps to OPERATOR_STOP
     * (D-01) */
    app_safe_stop_request((app_stop_cause_t) 99);
    CHECK("H-SF-11", APP_STOP_CAUSE_CRITICAL_RUNTIME == app_stop_cause_get(),
          "invalid cause -> CRITICAL_RUNTIME");
    host_current_tid = 42;
    (void) app_safety_controller_clear();
    app_safe_stop_request(APP_STOP_CAUSE_REMOTE_STOP);
    CHECK("H-SF-12", APP_STOP_CAUSE_OPERATOR_STOP == app_stop_cause_get(),
          "REMOTE_STOP mapped (D-01 default)");

    /* fault mask: set/clr through helper, diagnostic mask never includes stop
     * causes */
    (void) app_safety_controller_clear();
    app_fault_set(APP_FAULT_WIFI_LOSS | APP_FAULT_MOTOR_STALL);
    CHECK("H-SF-13",
          (APP_FAULT_WIFI_LOSS | APP_FAULT_MOTOR_STALL) == app_fault_get(),
          "fault bits set");
    CHECK("H-SF-14", !app_stop_latched(),
          "setting fault bits never latches a stop");
    app_fault_clr(APP_FAULT_WIFI_LOSS);
    CHECK("H-SF-15", APP_FAULT_MOTOR_STALL == app_fault_get(),
          "fault bit cleared individually");
    CHECK("H-SF-16",
          0U == (APP_FAULT_DIAGNOSTIC_MASK &
                 (APP_FAULT_MOTOR_STALL | APP_FAULT_IMU_FAULT |
                  APP_FAULT_STALE_LINE | APP_FAULT_CTRL_HEARTBEAT)),
          "safety faults are not in the diagnostic-only mask");

    /* telemetry post never blocks and counts drops when the buffer is full */
    host_mbf_full = 1;
    for (i = 0U; i < 300U; i++)
    {
        app_ipc_telem_post(APP_TEV_NONE, APP_SRC_SHARED, 0U);
    }
    CHECK("H-SF-17", 255U == app_ipc_telem_dropped(),
          "dropped counter saturates at 255");
    host_mbf_full = 0;
}

int
main (void)
{
    gp_vec = fopen("vectors.txt", "w");
    test_pack();
    test_blackboards();
    test_edge_ring();
    test_frames();
    test_cmd();
    test_safety();
    if (NULL != gp_vec)
    {
        (void) fclose(gp_vec);
    }
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
/* NOLINTEND(readability-magic-numbers,readability-function-size,cppcoreguidelines-avoid-magic-numbers)
 */
