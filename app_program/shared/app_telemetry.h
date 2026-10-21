/** @file app_telemetry.h
 *  @brief Wire frames (versioned little-endian binary), comms_word, RemoteCmd
 *         and MissionStatus codecs. Owner: Buddy 1 (frames, comms_word);
 *         controller owns MissionStatus. Layout authority: app_data.h.
 *
 *  Frames (all start with [version u8][seq u8]):
 *   SNAP 18 B  adopted 139-bit snapshot, bit-packed LSB-first (bit n of the
 *              stream is bit (n % 8) of byte (n / 8)). Offsets below.
 *   EVT   4 B  one MBF_TELEM_EVT record, little-endian uint32.
 *   HB    6 B  D-06: uptime_s u16, conn_state 2b | backoff_s 4b, dropped u8.
 *   OBS  10 B  D-06: ObstacleProfile word0, word1 (little-endian uint32).
 *   CMD   3 B  inbound: [ver][cmd << 6 | seq][crc8 over bytes 0-1].
 *  CRC-8: polynomial 0x07, init 0x00, MSB-first, no reflection, xorout 0x00
 *  (D-13). Topics: is09/<client>/snap, /evt, /hb, /obs, /cmd.
 *
 *  tools/telem_decode.py implements exactly these layouts; the test vectors
 *  in testing/host/vectors.txt are shared between C and Python.
 */

#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"
#include "app_pack.h"

/* ---- published words ------------------------------------------------- */
extern volatile uint32_t g_mission_status; /* writer: VehicleControllerTask */
extern volatile uint16_t g_comms_word;     /* writer: TelemetryCommsTask */
extern volatile uint8_t  g_remote_cmd; /* writer: TelemetryCommsTask (D-11) */
extern volatile uint8_t g_remote_cmd_gen; /* increments per validated command */

/* ---- MissionStatus --------------------------------------------------- */
typedef struct
{
    uint8_t sys_state;   /* app_sys_state_t */
    uint8_t pending_cmd; /* app_nav_cmd_t */
    uint8_t retry_count; /* 0..3 */
    uint8_t seq;         /* 0..15 */
} app_mission_status_t;

static inline uint32_t
app_mission_pack (app_mission_status_t const * p_view)
{
    uint32_t w = 0U;
    w          = app_pack_set(w, APP_MS_STATE_SHIFT, APP_MS_STATE_BITS,
                              p_view->sys_state);
    w          = app_pack_set(w, APP_MS_PENDING_SHIFT, APP_MS_PENDING_BITS,
                              p_view->pending_cmd);
    w          = app_pack_set(w, APP_MS_RETRY_SHIFT, APP_MS_RETRY_BITS,
                              p_view->retry_count);
    w = app_pack_set(w, APP_MS_SEQ_SHIFT, APP_MS_SEQ_BITS, p_view->seq);
    return w;
}

static inline void
app_mission_unpack (uint32_t word, app_mission_status_t * p_view)
{
    p_view->sys_state =
        (uint8_t) app_pack_get(word, APP_MS_STATE_SHIFT, APP_MS_STATE_BITS);
    p_view->pending_cmd =
        (uint8_t) app_pack_get(word, APP_MS_PENDING_SHIFT, APP_MS_PENDING_BITS);
    p_view->retry_count =
        (uint8_t) app_pack_get(word, APP_MS_RETRY_SHIFT, APP_MS_RETRY_BITS);
    p_view->seq =
        (uint8_t) app_pack_get(word, APP_MS_SEQ_SHIFT, APP_MS_SEQ_BITS);
}

/* ---- comms_word -------------------------------------------------------- */
static inline uint16_t
app_comms_pack (app_conn_state_t state, uint8_t backoff_s, uint8_t dropped)
{
    uint32_t w = 0U;
    w          = app_pack_set(w, APP_CW_STATE_SHIFT, APP_CW_STATE_BITS,
                              (uint32_t) state);
    /* valid range is 0..10 s (NFR18 cap), narrower than the 4-bit field */
    w = app_pack_set(w, APP_CW_BACKOFF_SHIFT, APP_CW_BACKOFF_BITS,
                     (backoff_s > APP_BACKOFF_S_MAX) ? APP_BACKOFF_S_MAX
                                                     : backoff_s);
    w = app_pack_set(w, APP_CW_DROPPED_SHIFT, APP_CW_DROPPED_BITS, dropped);
    return (uint16_t) w;
}

/* ---- RemoteCmd byte --------------------------------------------------- */
static inline uint8_t
app_remote_cmd_pack (app_remote_cmd_t cmd, uint8_t seq)
{
    return (uint8_t) ((((uint32_t) cmd & APP_RC_CMD_MASK) << APP_RC_CMD_SHIFT) |
                      ((uint32_t) seq & APP_RC_SEQ_MASK));
}
static inline app_remote_cmd_t
app_remote_cmd_type (uint8_t byte)
{
    return (app_remote_cmd_t) (((uint32_t) byte >> APP_RC_CMD_SHIFT) &
                               APP_RC_CMD_MASK);
}
static inline uint8_t
app_remote_cmd_seq (uint8_t byte)
{
    return (uint8_t) ((uint32_t) byte & APP_RC_SEQ_MASK);
}

/* ---- bit-stream helpers (LSB-first) ---------------------------------- */
void     app_bits_put (uint8_t * p_buf, uint32_t bit_off, uint32_t bits,
                       uint32_t value);
uint32_t app_bits_get (uint8_t const * p_buf, uint32_t bit_off, uint32_t bits);

/* ---- snapshot frame --------------------------------------------------- */
#define APP_SNAP_OFF_VER 0U
#define APP_SNAP_OFF_SEQ 8U
#define APP_SNAP_OFF_SYS_STATE 16U
#define APP_SNAP_OFF_MOTION_ST 19U
#define APP_SNAP_OFF_STOP_CAUSE 22U
#define APP_SNAP_OFF_FAULTS 26U
#define APP_SNAP_OFF_SPEED_L 42U
#define APP_SNAP_OFF_SPEED_R 49U
#define APP_SNAP_OFF_ENC_L 56U
#define APP_SNAP_OFF_ENC_R 72U
#define APP_SNAP_OFF_DISTANCE 88U
#define APP_SNAP_OFF_LINE_ERR 104U
#define APP_SNAP_OFF_ON_LINE 112U
#define APP_SNAP_OFF_BARCODE 113U
#define APP_SNAP_OFF_HUMP_MAX 116U
#define APP_SNAP_OFF_HUMP_ACT 123U
#define APP_SNAP_OFF_MOTION_CL 124U
#define APP_SNAP_OFF_RANGE 127U
#define APP_SNAP_OFF_ACTION 136U
#define APP_SNAP_OFF_END 139U

/* field widths (bits) in the same order */
#define APP_SNAP_W_BYTE 8U
#define APP_SNAP_W_STATE 3U
#define APP_SNAP_W_CAUSE 4U
#define APP_SNAP_W_FAULTS 16U
#define APP_SNAP_W_SPEED 7U
#define APP_SNAP_W_ENC 16U
#define APP_SNAP_W_DIST 16U
#define APP_SNAP_W_LERR 8U
#define APP_SNAP_W_FLAG 1U
#define APP_SNAP_W_BC 3U
#define APP_SNAP_W_HUMP 7U
#define APP_SNAP_W_CLASS 3U
#define APP_SNAP_W_RANGE 9U
#define APP_SNAP_W_ACTION 3U
#define APP_SNAP_RESERVED_BITS (APP_SNAPSHOT_BYTES * 8U - APP_SNAP_OFF_END)
#define APP_HB_CONN_MASK 0x03U
#define APP_HB_BACKOFF_SHIFT 2U
#define APP_HB_BACKOFF_MASK 0x0FU
#define APP_CMD_CRC_LEN 2U
/* byte offsets inside the small frames */
#define APP_FR_OFF_VER 0U
#define APP_FR_OFF_SEQ 1U
#define APP_HB_OFF_UPTIME 2U
#define APP_HB_OFF_FLAGS 4U
#define APP_HB_OFF_DROP 5U
#define APP_OBS_OFF_W0 2U
#define APP_OBS_OFF_W1 6U
#define APP_CMD_OFF_BODY 1U
#define APP_CMD_OFF_CRC 2U

typedef struct
{
    uint8_t  seq;
    uint8_t  sys_state;    /* 3 b */
    uint8_t  motion_state; /* 3 b */
    uint8_t  stop_cause;   /* 4 b */
    uint16_t fault_mask;   /* 16 b */
    uint8_t  speed_l;      /* 7 b */
    uint8_t  speed_r;      /* 7 b */
    uint16_t enc_l;        /* 16 b */
    uint16_t enc_r;        /* 16 b */
    int16_t  distance_mm;  /* 16 b signed */
    int8_t   line_error;   /* 8 b signed */
    bool     on_line;      /* 1 b */
    uint8_t  barcode;      /* 3 b */
    uint8_t  hump_max_mm;  /* 7 b */
    bool     hump_active;  /* 1 b */
    uint8_t  motion_class; /* 3 b */
    uint16_t range_cm;     /* 9 b */
    uint8_t  action;       /* 3 b */
} app_snapshot_t;

/** Encode into exactly APP_SNAPSHOT_BYTES. Returns true if any field had
 *  to be saturated (reported in the next event as a diagnostic). */
bool app_snapshot_encode (app_snapshot_t const * p_snap, uint8_t * p_out);
/** Decode; returns false on wrong version. */
bool app_snapshot_decode (uint8_t const * p_in, app_snapshot_t * p_snap);

/* ---- heartbeat frame (D-06) ------------------------------------------- */
typedef struct
{
    uint8_t  seq;
    uint16_t uptime_s;
    uint8_t  conn_state; /* 2 b */
    uint8_t  backoff_s;  /* 4 b */
    uint8_t  dropped;    /* 8 b */
} app_heartbeat_t;

void app_heartbeat_encode (app_heartbeat_t const * p_hb, uint8_t * p_out);
bool app_heartbeat_decode (uint8_t const * p_in, app_heartbeat_t * p_hb);

/* ---- obstacle frame (D-06) -------------------------------------------- */
void app_obs_frame_encode (uint8_t seq, uint32_t word0, uint32_t word1,
                           uint8_t * p_out);
bool app_obs_frame_decode (uint8_t const * p_in, uint8_t * p_seq,
                           uint32_t * p_w0, uint32_t * p_w1);

/* ---- event frame -------------------------------------------------------- */
static inline void
app_event_frame_encode (uint32_t record, uint8_t * p_out)
{
    app_pack_le32(p_out, record);
}

/* ---- command frame ------------------------------------------------------ */
uint8_t app_crc8 (uint8_t const * p_data, uint32_t len);

/** Build a 3-byte command frame (used by tests and the Python encoder). */
void app_cmd_frame_encode (app_remote_cmd_t cmd, uint8_t seq, uint8_t * p_out);

typedef enum
{
    APP_CMD_OK          = 0,
    APP_CMD_BAD_LENGTH  = 1,
    APP_CMD_BAD_VERSION = 2,
    APP_CMD_BAD_CRC     = 3,
    APP_CMD_BAD_TYPE    = 4,
    APP_CMD_REPLAY      = 5, /* D-02: same seq as the last accepted frame */
    APP_CMD_RETAINED    = 6, /* broker-retained frame rejected */
    APP_CMD_START_GUARD = 7, /* D-02: START inside the reconnect guard */
    APP_CMD_START_STATE = 8  /* START while not in S1 (controller decides) */
} app_cmd_verdict_t;

/** Validate length, version, CRC, type and replay against *p_last_seq
 *  (0xFF = none yet). On APP_CMD_OK, *p_cmd and *p_seq are set and
 *  *p_last_seq is updated. Retained and state guards are applied by the
 *  caller (they need broker/controller context). */
app_cmd_verdict_t app_cmd_frame_decode (uint8_t const * p_in, uint32_t len,
                                        uint8_t *          p_last_seq,
                                        app_remote_cmd_t * p_cmd,
                                        uint8_t *          p_seq);

#endif /* APP_TELEMETRY_H */
