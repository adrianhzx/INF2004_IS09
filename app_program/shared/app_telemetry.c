/** @file app_telemetry.c
 *  @brief Wire frame codecs. Pure functions: no kernel calls, host-testable.
 */

#include "app_telemetry.h"

#define APP_CRC8_POLY 0x07U
#define APP_CRC8_INIT 0x00U
#define APP_CRC8_MSB 0x80U
#define APP_BYTE_MASK 0xFFU
#define APP_BITS_PER_BYTE 8U

/* ---- bit stream -------------------------------------------------------- */

void
app_bits_put (uint8_t * p_buf, uint32_t bit_off, uint32_t bits, uint32_t value)
{
    uint32_t i = 0U;
    for (i = 0U; i < bits; i++)
    {
        uint32_t const pos  = bit_off + i;
        uint32_t const byte = pos / APP_BITS_PER_BYTE;
        uint32_t const bit  = pos % APP_BITS_PER_BYTE;
        uint32_t const cur  = (uint32_t) p_buf[byte]; /* promote unsigned */
        if (0U != ((value >> i) & 1U))
        {
            p_buf[byte] = (uint8_t) (cur | (1U << bit));
        }
        else
        {
            p_buf[byte] = (uint8_t) (cur & ~(1U << bit) & APP_BYTE_MASK);
        }
    }
}

uint32_t
app_bits_get (uint8_t const * p_buf, uint32_t bit_off, uint32_t bits)
{
    uint32_t value = 0U;
    uint32_t i     = 0U;
    for (i = 0U; i < bits; i++)
    {
        uint32_t const pos  = bit_off + i;
        uint32_t const byte = pos / APP_BITS_PER_BYTE;
        uint32_t const bit  = pos % APP_BITS_PER_BYTE;
        uint32_t const cur  = (uint32_t) p_buf[byte];
        if (0U != ((cur >> bit) & 1U))
        {
            value |= (1UL << i);
        }
    }
    return value;
}

/* ---- snapshot ---------------------------------------------------------- */

bool
app_snapshot_encode (app_snapshot_t const * p_snap, uint8_t * p_out)
{
    bool     sat = false;
    uint32_t i   = 0U;

    for (i = 0U; i < APP_SNAPSHOT_BYTES; i++)
    {
        p_out[i] = 0U;
    }
    app_bits_put(p_out, APP_SNAP_OFF_VER, APP_SNAP_W_BYTE, APP_FRAME_VERSION);
    app_bits_put(p_out, APP_SNAP_OFF_SEQ, APP_SNAP_W_BYTE, p_snap->seq);
    app_bits_put(p_out, APP_SNAP_OFF_SYS_STATE, APP_SNAP_W_STATE,
                 app_pack_sat_u(p_snap->sys_state, APP_SNAP_W_STATE, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_MOTION_ST, APP_SNAP_W_STATE,
                 app_pack_sat_u(p_snap->motion_state, APP_SNAP_W_STATE, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_STOP_CAUSE, APP_SNAP_W_CAUSE,
                 app_pack_sat_u(p_snap->stop_cause, APP_SNAP_W_CAUSE, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_FAULTS, APP_SNAP_W_FAULTS,
                 p_snap->fault_mask);
    app_bits_put(p_out, APP_SNAP_OFF_SPEED_L, APP_SNAP_W_SPEED,
                 app_pack_sat_u(p_snap->speed_l, APP_SNAP_W_SPEED, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_SPEED_R, APP_SNAP_W_SPEED,
                 app_pack_sat_u(p_snap->speed_r, APP_SNAP_W_SPEED, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_ENC_L, APP_SNAP_W_ENC, p_snap->enc_l);
    app_bits_put(p_out, APP_SNAP_OFF_ENC_R, APP_SNAP_W_ENC, p_snap->enc_r);
    app_bits_put(p_out, APP_SNAP_OFF_DISTANCE, APP_SNAP_W_DIST,
                 app_pack_s_to_u(p_snap->distance_mm, APP_SNAP_W_DIST));
    app_bits_put(p_out, APP_SNAP_OFF_LINE_ERR, APP_SNAP_W_LERR,
                 app_pack_s_to_u(p_snap->line_error, APP_SNAP_W_LERR));
    app_bits_put(p_out, APP_SNAP_OFF_ON_LINE, APP_SNAP_W_FLAG,
                 p_snap->on_line ? 1U : 0U);
    app_bits_put(p_out, APP_SNAP_OFF_BARCODE, APP_SNAP_W_BC,
                 app_pack_sat_u(p_snap->barcode, APP_SNAP_W_BC, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_HUMP_MAX, APP_SNAP_W_HUMP,
                 app_pack_sat_u(p_snap->hump_max_mm, APP_SNAP_W_HUMP, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_HUMP_ACT, APP_SNAP_W_FLAG,
                 p_snap->hump_active ? 1U : 0U);
    app_bits_put(p_out, APP_SNAP_OFF_MOTION_CL, APP_SNAP_W_CLASS,
                 app_pack_sat_u(p_snap->motion_class, APP_SNAP_W_CLASS, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_RANGE, APP_SNAP_W_RANGE,
                 app_pack_sat_u(p_snap->range_cm, APP_SNAP_W_RANGE, &sat));
    app_bits_put(p_out, APP_SNAP_OFF_ACTION, APP_SNAP_W_ACTION,
                 app_pack_sat_u(p_snap->action, APP_SNAP_W_ACTION, &sat));
    return sat;
}

bool
app_snapshot_decode (uint8_t const * p_in, app_snapshot_t * p_snap)
{
    if (APP_FRAME_VERSION !=
        app_bits_get(p_in, APP_SNAP_OFF_VER, APP_SNAP_W_BYTE))
    {
        return false;
    }
    p_snap->seq =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_SEQ, APP_SNAP_W_BYTE);
    p_snap->sys_state =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_SYS_STATE, APP_SNAP_W_STATE);
    p_snap->motion_state =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_MOTION_ST, APP_SNAP_W_STATE);
    p_snap->stop_cause =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_STOP_CAUSE, APP_SNAP_W_CAUSE);
    p_snap->fault_mask =
        (uint16_t) app_bits_get(p_in, APP_SNAP_OFF_FAULTS, APP_SNAP_W_FAULTS);
    p_snap->speed_l =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_SPEED_L, APP_SNAP_W_SPEED);
    p_snap->speed_r =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_SPEED_R, APP_SNAP_W_SPEED);
    p_snap->enc_l =
        (uint16_t) app_bits_get(p_in, APP_SNAP_OFF_ENC_L, APP_SNAP_W_ENC);
    p_snap->enc_r =
        (uint16_t) app_bits_get(p_in, APP_SNAP_OFF_ENC_R, APP_SNAP_W_ENC);
    p_snap->distance_mm = (int16_t) app_pack_u_to_s(
        app_bits_get(p_in, APP_SNAP_OFF_DISTANCE, APP_SNAP_W_DIST),
        APP_SNAP_W_DIST);
    p_snap->line_error = (int8_t) app_pack_u_to_s(
        app_bits_get(p_in, APP_SNAP_OFF_LINE_ERR, APP_SNAP_W_LERR),
        APP_SNAP_W_LERR);
    p_snap->on_line =
        (0U != app_bits_get(p_in, APP_SNAP_OFF_ON_LINE, APP_SNAP_W_FLAG));
    p_snap->barcode =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_BARCODE, APP_SNAP_W_BC);
    p_snap->hump_max_mm =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_HUMP_MAX, APP_SNAP_W_HUMP);
    p_snap->hump_active =
        (0U != app_bits_get(p_in, APP_SNAP_OFF_HUMP_ACT, APP_SNAP_W_FLAG));
    p_snap->motion_class =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_MOTION_CL, APP_SNAP_W_CLASS);
    p_snap->range_cm =
        (uint16_t) app_bits_get(p_in, APP_SNAP_OFF_RANGE, APP_SNAP_W_RANGE);
    p_snap->action =
        (uint8_t) app_bits_get(p_in, APP_SNAP_OFF_ACTION, APP_SNAP_W_ACTION);
    return true;
}

/* ---- heartbeat --------------------------------------------------------- */

void
app_heartbeat_encode (app_heartbeat_t const * p_hb, uint8_t * p_out)
{
    uint32_t const backoff = (p_hb->backoff_s > APP_BACKOFF_S_MAX)
                                 ? APP_BACKOFF_S_MAX
                                 : p_hb->backoff_s;
    uint32_t const conn    = (uint32_t) p_hb->conn_state & APP_HB_CONN_MASK;

    p_out[APP_FR_OFF_VER] = APP_FRAME_VERSION;
    p_out[APP_FR_OFF_SEQ] = p_hb->seq;
    app_pack_le16(&p_out[APP_HB_OFF_UPTIME], p_hb->uptime_s);
    p_out[APP_HB_OFF_FLAGS] = (uint8_t) (conn | ((backoff & APP_HB_BACKOFF_MASK)
                                                 << APP_HB_BACKOFF_SHIFT));
    p_out[APP_HB_OFF_DROP]  = p_hb->dropped;
}

bool
app_heartbeat_decode (uint8_t const * p_in, app_heartbeat_t * p_hb)
{
    uint32_t const flags = (uint32_t) p_in[APP_HB_OFF_FLAGS];

    if (APP_FRAME_VERSION != p_in[APP_FR_OFF_VER])
    {
        return false;
    }
    p_hb->seq        = p_in[APP_FR_OFF_SEQ];
    p_hb->uptime_s   = app_unpack_le16(&p_in[APP_HB_OFF_UPTIME]);
    p_hb->conn_state = (uint8_t) (flags & APP_HB_CONN_MASK);
    p_hb->backoff_s =
        (uint8_t) ((flags >> APP_HB_BACKOFF_SHIFT) & APP_HB_BACKOFF_MASK);
    p_hb->dropped = p_in[APP_HB_OFF_DROP];
    return true;
}

/* ---- obstacle ---------------------------------------------------------- */

void
app_obs_frame_encode (uint8_t seq, uint32_t word0, uint32_t word1,
                      uint8_t * p_out)
{
    p_out[APP_FR_OFF_VER] = APP_FRAME_VERSION;
    p_out[APP_FR_OFF_SEQ] = seq;
    app_pack_le32(&p_out[APP_OBS_OFF_W0], word0);
    app_pack_le32(&p_out[APP_OBS_OFF_W1], word1);
}

bool
app_obs_frame_decode (uint8_t const * p_in, uint8_t * p_seq, uint32_t * p_w0,
                      uint32_t * p_w1)
{
    if (APP_FRAME_VERSION != p_in[APP_FR_OFF_VER])
    {
        return false;
    }
    *p_seq = p_in[APP_FR_OFF_SEQ];
    *p_w0  = app_unpack_le32(&p_in[APP_OBS_OFF_W0]);
    *p_w1  = app_unpack_le32(&p_in[APP_OBS_OFF_W1]);
    return true;
}

/* ---- command ----------------------------------------------------------- */

uint8_t
app_crc8 (uint8_t const * p_data, uint32_t len)
{
    uint32_t crc = APP_CRC8_INIT;
    uint32_t i   = 0U;
    uint32_t bit = 0U;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint32_t) p_data[i];
        for (bit = 0U; bit < APP_BITS_PER_BYTE; bit++)
        {
            if (0U != (crc & APP_CRC8_MSB))
            {
                crc = ((crc << 1U) ^ APP_CRC8_POLY) & APP_BYTE_MASK;
            }
            else
            {
                crc = (crc << 1U) & APP_BYTE_MASK;
            }
        }
    }
    return (uint8_t) crc;
}

void
app_cmd_frame_encode (app_remote_cmd_t cmd, uint8_t seq, uint8_t * p_out)
{
    p_out[APP_FR_OFF_VER]   = APP_FRAME_VERSION;
    p_out[APP_CMD_OFF_BODY] = app_remote_cmd_pack(cmd, seq);
    p_out[APP_CMD_OFF_CRC]  = app_crc8(p_out, APP_CMD_CRC_LEN);
}

app_cmd_verdict_t
app_cmd_frame_decode (uint8_t const * p_in, uint32_t len, uint8_t * p_last_seq,
                      app_remote_cmd_t * p_cmd, uint8_t * p_seq)
{
    app_remote_cmd_t cmd = APP_REMOTE_CMD_INVALID0;
    uint8_t          seq = 0U;

    if (APP_CMD_FRAME_BYTES != len)
    {
        return APP_CMD_BAD_LENGTH;
    }
    if (APP_FRAME_VERSION != p_in[APP_FR_OFF_VER])
    {
        return APP_CMD_BAD_VERSION;
    }
    if (app_crc8(p_in, APP_CMD_CRC_LEN) != p_in[APP_CMD_OFF_CRC])
    {
        return APP_CMD_BAD_CRC;
    }
    cmd = app_remote_cmd_type(p_in[APP_CMD_OFF_BODY]);
    seq = app_remote_cmd_seq(p_in[APP_CMD_OFF_BODY]);
    if ((APP_REMOTE_CMD_START != cmd) && (APP_REMOTE_CMD_STOP != cmd))
    {
        return APP_CMD_BAD_TYPE;
    }
    if ((NULL != p_last_seq) && (APP_BYTE_MASK != *p_last_seq) &&
        (seq == *p_last_seq))
    {
        return APP_CMD_REPLAY;
    }
    if (NULL != p_last_seq)
    {
        *p_last_seq = seq;
    }
    *p_cmd = cmd;
    *p_seq = seq;
    return APP_CMD_OK;
}
