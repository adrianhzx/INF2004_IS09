/** @file s1_telem.c
 *  @brief Pure telemetry logic for TelemetryCommsTask (see s1_telem.h).
 *         No kernel calls: compiled unchanged into the host tests.
 */

#include "s1_telem.h"

#include <stddef.h>

#include "../shared/app_barcode.h"
#include "../shared/app_cal.h"

/* Shortest valid publish request: the length byte plus one topic byte. */
#define S1_PUB_MIN_BYTES 2U

static char const * const g_s1_topics[S1_TOPIC_COUNT] = {
    S1_TOPIC_BASE "snap", S1_TOPIC_BASE "evt", S1_TOPIC_BASE "hb",
    S1_TOPIC_BASE "obs"};

/* ---- topics and publish requests ---------------------------------------- */

char const *
s1_topic_name (s1_topic_t topic)
{
    if ((uint32_t) topic >= (uint32_t) S1_TOPIC_COUNT)
    {
        return NULL;
    }
    return g_s1_topics[topic];
}

static uint32_t
s1_strlen (char const * p_str)
{
    uint32_t n = 0U;

    while ((n <= S1_TOPIC_MAX_BYTES) && ('\0' != p_str[n]))
    {
        n++;
    }
    return n;
}

uint32_t
s1_pub_pack (s1_topic_t topic, uint8_t const * p_payload, uint32_t len,
             uint8_t * p_out)
{
    char const * const p_topic = s1_topic_name(topic);
    uint32_t           tlen    = 0U;
    uint32_t           i       = 0U;

    if ((NULL == p_topic) || (NULL == p_out) ||
        ((NULL == p_payload) && (0U != len)) || (len > S1_PAYLOAD_MAX_BYTES))
    {
        return 0U;
    }
    tlen = s1_strlen(p_topic);
    if ((0U == tlen) || (tlen > S1_TOPIC_MAX_BYTES))
    {
        return 0U;
    }
    p_out[0] = (uint8_t) tlen;
    for (i = 0U; i < tlen; i++)
    {
        p_out[1U + i] = (uint8_t) p_topic[i];
    }
    for (i = 0U; i < len; i++)
    {
        p_out[1U + tlen + i] = p_payload[i];
    }
    return 1U + tlen + len;
}

bool
s1_pub_unpack (uint8_t const * p_msg, uint32_t len, char * p_topic,
               uint8_t const ** pp_payload, uint32_t * p_payload_len)
{
    uint32_t tlen = 0U;
    uint32_t i    = 0U;

    if ((NULL == p_msg) || (len < S1_PUB_MIN_BYTES))
    {
        return false;
    }
    tlen = p_msg[0];
    if ((0U == tlen) || (tlen > S1_TOPIC_MAX_BYTES) || ((1U + tlen) > len))
    {
        return false;
    }
    for (i = 0U; i < tlen; i++)
    {
        p_topic[i] = (char) p_msg[1U + i];
    }
    p_topic[tlen]  = '\0';
    *pp_payload    = &p_msg[1U + tlen];
    *p_payload_len = len - 1U - tlen;
    return true;
}

/* ---- connection state -----------------------------------------------------
 */

app_conn_state_t
s1_conn_derive (bool wifi_up, bool mqtt_up, uint8_t backoff_s)
{
    if (!wifi_up)
    {
        return APP_CONN_WIFI_CONNECTING;
    }
    if (mqtt_up)
    {
        return APP_CONN_ONLINE;
    }
    return (0U != backoff_s) ? APP_CONN_BACKOFF : APP_CONN_MQTT_CONNECTING;
}

uint16_t
s1_conn_faults (app_conn_state_t state)
{
    uint16_t bits = 0U;

    switch (state)
    {
        case APP_CONN_WIFI_CONNECTING:
            bits = (uint16_t) APP_FAULT_WIFI_LOSS;
            break;
        case APP_CONN_MQTT_CONNECTING:
        case APP_CONN_BACKOFF:
            bits = (uint16_t) APP_FAULT_BROKER_LOSS;
            break;
        case APP_CONN_ONLINE:
        default:
            bits = 0U;
            break;
    }
    return bits;
}

/* ---- cadence ----------------------------------------------------------------
 */

void
s1_cadence_start (s1_cadence_t * p_cad, uint32_t now_ms, uint32_t period_ms)
{
    p_cad->period_ms = period_ms;
    p_cad->next_ms   = now_ms; /* first one is due immediately */
}

bool
s1_cadence_due (s1_cadence_t * p_cad, uint32_t now_ms)
{
    /* Signed difference handles the 32-bit millisecond wrap. */
    int32_t const late = (int32_t) (now_ms - p_cad->next_ms);

    if (late < 0)
    {
        return false;
    }
    if ((uint32_t) late >= p_cad->period_ms)
    {
        /* Stalled for a whole period or more: resynchronise, no burst. */
        p_cad->next_ms = now_ms + p_cad->period_ms;
    }
    else
    {
        p_cad->next_ms = p_cad->next_ms + p_cad->period_ms;
    }
    return true;
}

/* ---- inbound commands
 * -------------------------------------------------------- */

void
s1_cmd_init (s1_cmd_ctx_t * p_ctx)
{
    p_ctx->last_seq        = APP_PACK_BYTE_MASK;
    p_ctx->online          = false;
    p_ctx->online_since_ms = 0U;
}

void
s1_cmd_on_online (s1_cmd_ctx_t * p_ctx, uint32_t now_ms)
{
    p_ctx->online          = true;
    p_ctx->online_since_ms = now_ms;
}

void
s1_cmd_on_offline (s1_cmd_ctx_t * p_ctx)
{
    p_ctx->online = false;
}

app_cmd_verdict_t
s1_cmd_filter (s1_cmd_ctx_t * p_ctx, uint8_t const * p_frame, uint32_t len,
               uint32_t now_ms, app_remote_cmd_t * p_cmd, uint8_t * p_seq)
{
    uint8_t           last    = p_ctx->last_seq; /* commit only on accept */
    app_remote_cmd_t  cmd     = APP_REMOTE_CMD_INVALID0;
    uint8_t           seq     = 0U;
    app_cmd_verdict_t verdict = APP_CMD_BAD_LENGTH;

    if ((NULL == p_frame) || (0U == len))
    {
        return APP_CMD_BAD_LENGTH;
    }
    verdict = app_cmd_frame_decode(p_frame, len, &last, &cmd, &seq);
    if (APP_CMD_OK != verdict)
    {
        return verdict;
    }
    if (APP_REMOTE_CMD_START == cmd)
    {
        uint32_t const since = now_ms - p_ctx->online_since_ms;
        if ((!p_ctx->online) || (since < APP_CAL_T_REMOTE_START_GUARD_MS))
        {
            return APP_CMD_START_GUARD; /* replay memory unchanged */
        }
    }
    p_ctx->last_seq = last;
    *p_cmd          = cmd;
    *p_seq          = seq;
    return APP_CMD_OK;
}

/* ---- snapshot
 * ----------------------------------------------------------------- */

void
s1_snapshot_build (s1_inputs_t const * p_in, uint8_t seq,
                   app_snapshot_t * p_snap)
{
    app_mission_status_t   ms;
    app_line_state_t       line;
    app_imu_state_t        imu;
    app_obstacle_profile_t obst;

    app_mission_unpack(p_in->mission_word, &ms);
    app_line_unpack(p_in->line_word, &line);
    app_imu_unpack(p_in->imu_word, &imu);
    app_obstacle_unpack(p_in->obst_w0, p_in->obst_w1, &obst);

    p_snap->seq          = seq;
    p_snap->sys_state    = ms.sys_state;
    p_snap->motion_state = p_in->motion.motion_state;
    p_snap->stop_cause   = p_in->stop_cause;
    p_snap->fault_mask   = p_in->fault_mask;
    p_snap->speed_l      = p_in->motion.speed_l_cm_s;
    p_snap->speed_r      = p_in->motion.speed_r_cm_s;
    p_snap->enc_l        = p_in->enc_l;
    p_snap->enc_r        = p_in->enc_r;
    p_snap->distance_mm  = p_in->motion.distance_mm;
    p_snap->line_error   = line.line_error;
    p_snap->on_line      = line.on_line;
    p_snap->barcode      = (uint8_t) app_barcode_result(p_in->barcode_byte);
    p_snap->hump_max_mm  = imu.max_height_mm;
    p_snap->hump_active  = imu.hump_active;
    p_snap->motion_class = imu.motion_class;
    p_snap->range_cm     = obst.range_cm;
    p_snap->action       = obst.action;
}
