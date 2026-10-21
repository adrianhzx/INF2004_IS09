/** @file app_data.h
 *  @brief Single authoritative layout of every packed application data field.
 *
 *  Source: Week 6 Design Review Rev B section 3.1 and IS09_ImmTri1 Version B
 *  "Preliminary data representation table".  Every position, width, unit,
 *  sentinel and range below is the adopted Tribunal-1 Version B value unless
 *  the comment says PROVISIONAL (fixed by assumptions A4/A6/A8/A10) or
 *  D-xx (see docs/DECISION_REGISTER.md).
 *
 *  Rules (adopted):
 *   1. This header is the only place a field position is defined.
 *   2. Fields crossing a task/ISR boundary or the network use the unsigned
 *      mask/shift helpers in app_pack.h.  C bit-fields are permitted only in
 *      task-private structs.
 *   3. One writer per word.  A word of <= 32 bits is published with one aligned
 *      store and read with one aligned load; anything larger keeps a mutex.
 *   4. Multi-writer words (fault_mask, safety_word) change only through
 *      app_shared_bits_set() / app_shared_bits_clr() (app_safety.h).
 *   5. STOP_LATCH is a dedicated byte, never packed.
 *   6. Pack helpers saturate and report over-range.
 *
 *  Owner: Buddy 2 (header owner, R5); each buddy owns the rows of their task.
 */

#ifndef APP_DATA_H
#define APP_DATA_H

#include <stdint.h>

/* The kernel port typedefs size_t itself (include/tk/syslib.h) and conflicts
   with <stddef.h>, so NULL is provided here for application code. */
#ifndef NULL
#define NULL ((void *) 0)
#endif

/* ------------------------------------------------------------------------ */
/* Compile-time guard usable in C99 (no _Static_assert).                     */
/* ------------------------------------------------------------------------ */
#define APP_STATIC_ASSERT(cond, name)                                          \
    typedef char app_static_assert_##name[(cond) ? 1 : -1]

/* ------------------------------------------------------------------------ */
/* STOP_LATCH - dedicated byte (rule 5). Logical 1 bit, physical 8 bits.     */
/* Writers: app_safe_stop_request() (ISR + tasks). Clear: controller only.  */
/* ------------------------------------------------------------------------ */
#define APP_STOP_LATCH_CLEAR 0U
#define APP_STOP_LATCH_SET 1U

/* ------------------------------------------------------------------------ */
/* safety_word (uint8_t): bits 0-3 stop_cause, bits 4-7 reserved (0).        */
/* Multi-writer: RMW only through the helper (rule 4).                      */
/* ------------------------------------------------------------------------ */
#define APP_SAFETY_CAUSE_SHIFT 0U
#define APP_SAFETY_CAUSE_BITS 4U
#define APP_SAFETY_CAUSE_MASK 0x0FU

/** Stop causes: NONE + 10 adopted causes (11 values, 4 bits). */
typedef enum
{
    APP_STOP_CAUSE_NONE = 0,
    APP_STOP_CAUSE_OPERATOR_STOP =
        1, /* STOP button, or validated remote STOP via D-01 */
    APP_STOP_CAUSE_COLLISION           = 2, /* ImuTask */
    APP_STOP_CAUSE_STALL               = 3, /* MotionControlTask */
    APP_STOP_CAUSE_STALE_LINE          = 4, /* MotionControlTask */
    APP_STOP_CAUSE_STALE_MOTION        = 5, /* D-14 */
    APP_STOP_CAUSE_CTRL_HEARTBEAT      = 6, /* MotionControlTask */
    APP_STOP_CAUSE_SCAN_TIMEOUT        = 7, /* VehicleControllerTask */
    APP_STOP_CAUSE_NO_CLEARANCE        = 8, /* VehicleControllerTask */
    APP_STOP_CAUSE_LINE_SEARCH_TIMEOUT = 9, /* VehicleControllerTask */
    APP_STOP_CAUSE_CRITICAL_RUNTIME =
        10, /* any task: boot self-check, kernel error */
    APP_STOP_CAUSE_REMOTE_STOP =
        11, /* NOT ADOPTED (D-01); only if APP_ADOPT_REMOTE_STOP_CAUSE */
    APP_STOP_CAUSE_COUNT = 12
} app_stop_cause_t;

APP_STATIC_ASSERT(APP_STOP_CAUSE_COUNT <= 16, stop_cause_fits_4_bits);

/* ------------------------------------------------------------------------ */
/* fault_mask (uint16_t): 13 named faults, bits 13-15 reserved.             */
/* Set/clear only via app_shared_bits_*(). Reading is one 16-bit load.      */
/* Setting a bit never latches a stop (app_safe_stop_request does).         */
/* ------------------------------------------------------------------------ */
#define APP_FAULT_MOTOR_STALL (1U << 0U)    /* Motion */
#define APP_FAULT_ENCODER_FAULT (1U << 1U)  /* Motion */
#define APP_FAULT_IMU_FAULT (1U << 2U)      /* IMU */
#define APP_FAULT_NO_ECHO (1U << 3U)        /* Scan */
#define APP_FAULT_WIFI_LOSS (1U << 4U)      /* Comms - diagnostic only */
#define APP_FAULT_BROKER_LOSS (1U << 5U)    /* Comms - diagnostic only */
#define APP_FAULT_IR_CONTRAST (1U << 6U)    /* Line */
#define APP_FAULT_STALE_LINE (1U << 7U)     /* Motion */
#define APP_FAULT_STALE_MOTION (1U << 8U)   /* IMU / Controller */
#define APP_FAULT_CTRL_HEARTBEAT (1U << 9U) /* Motion */
#define APP_FAULT_SCAN_TIMEOUT (1U << 10U)  /* Scan / Controller */
#define APP_FAULT_TELEM_BUF_FULL                                               \
    (1U << 11U) /* any producer - diagnostic only                              \
                 */
#define APP_FAULT_BARCODE_OVERFLOW (1U << 12U) /* Barcode - diagnostic only */
#define APP_FAULT_RESERVED_MASK 0xE000U

/** Faults that are diagnostic only and must NEVER be a stop trigger (D-08). */
#define APP_FAULT_DIAGNOSTIC_MASK                                              \
    (APP_FAULT_WIFI_LOSS | APP_FAULT_BROKER_LOSS | APP_FAULT_TELEM_BUF_FULL |  \
     APP_FAULT_BARCODE_OVERFLOW)

/* ------------------------------------------------------------------------ */
/* MissionStatus (uint32_t), writer: VehicleControllerTask.                 */
/*   bits 0-2  system_state S0..S7                                          */
/*   bits 3-5  pending_cmd                                                  */
/*   bits 6-7  retry_count 0..3                                             */
/*   bits 8-11 seq (4 b, staleness; Version B "a 4-bit seq in each word")   */
/*   bits 12-31 reserved (0)                                                */
/* ------------------------------------------------------------------------ */
#define APP_MS_STATE_SHIFT 0U
#define APP_MS_STATE_BITS 3U
#define APP_MS_PENDING_SHIFT 3U
#define APP_MS_PENDING_BITS 3U
#define APP_MS_RETRY_SHIFT 6U
#define APP_MS_RETRY_BITS 2U
#define APP_MS_SEQ_SHIFT 8U
#define APP_MS_SEQ_BITS 4U
#define APP_MS_USED_BITS 12U

typedef enum
{
    APP_SYS_S0_BOOT_BRAKED     = 0,
    APP_SYS_S1_IDLE            = 1,
    APP_SYS_S2_LINE_FOLLOWING  = 2,
    APP_SYS_S3_EXECUTE_NAV     = 3,
    APP_SYS_S4_OBSTACLE_BYPASS = 4,
    APP_SYS_S5_LINE_SEARCH     = 5,
    APP_SYS_S6_RUN_COMPLETE    = 6,
    APP_SYS_S7_SAFE_STOP       = 7
} app_sys_state_t;

/** pending_cmd / barcode navigation command codes (3 bits). */
typedef enum
{
    APP_NAV_NONE     = 0,
    APP_NAV_LEFT     = 1, /* Code 39 'A' */
    APP_NAV_RIGHT    = 2, /* Code 39 'B' */
    APP_NAV_STRAIGHT = 3, /* Code 39 'C' */
    APP_NAV_UTURN    = 4  /* Code 39 'D' */
} app_nav_cmd_t;

#define APP_RETRY_MAX 3U /* N_RETRY_MAX <= 3 (2 bits) */

/* ------------------------------------------------------------------------ */
/* Encoder ISR counters: two aligned uint16_t, ISR-only writer, wrap-safe   */
/* deltas in the consumer (uint16 subtraction). No shared word with tasks.  */
/* ------------------------------------------------------------------------ */
#define APP_ENC_TICKS_BITS 16U

/* Ticks per 10 ms (Motion-private, saturating uint8_t). PROVISIONAL (A4). */
#define APP_TICKS_PER_CYCLE_MAX 255U

/* ------------------------------------------------------------------------ */
/* MotionStatus word A (uint32_t), writer: MotionControlTask.               */
/*   bits 0-6   speed_l  0..127 cm/s (magnitude)                            */
/*   bits 7-13  speed_r  0..127 cm/s                                        */
/*   bits 14-29 distance_mm int16 two's complement (+-32767 mm)             */
/*   bits 30-31 cycle_tag (D-04, uses the 2 reserved bits)                  */
/* MotionStatus word B "motion_word" (uint32_t), same writer.               */
/*   bits 0-2   motion_state                                                */
/*   bits 3-4   cycle_tag (D-04, must equal word A's tag for a coherent pair)*/
/*   bits 5-8   done_seq   seq of the last COMPLETED command (EVT_MOTION_DONE)*/
/*   bits 9-12  active_seq seq of the command being executed               */
/*   bit  13    dir_l 1 = reverse   (FR8 signed reverse motion, see ledger) */
/*   bit  14    dir_r 1 = reverse                                           */
/*   bits 15-16 done_result 0 OK, 1 REJECTED, 2 ABORTED, 3 TIMEOUT          */
/*   bits 17-31 reserved                                                    */
/* ------------------------------------------------------------------------ */
#define APP_MSA_SPEED_L_SHIFT 0U
#define APP_MSA_SPEED_R_SHIFT 7U
#define APP_MSA_SPEED_BITS 7U
#define APP_MSA_DIST_SHIFT 14U
#define APP_MSA_DIST_BITS 16U
#define APP_MSA_TAG_SHIFT 30U
#define APP_MSA_TAG_BITS 2U

#define APP_MSB_STATE_SHIFT 0U
#define APP_MSB_STATE_BITS 3U
#define APP_MSB_TAG_SHIFT 3U
#define APP_MSB_TAG_BITS 2U
#define APP_MSB_DONE_SHIFT 5U
#define APP_MSB_DONE_BITS 4U
#define APP_MSB_ACTIVE_SHIFT 9U
#define APP_MSB_ACTIVE_BITS 4U
#define APP_MSB_DIR_L_SHIFT 13U
#define APP_MSB_DIR_R_SHIFT 14U
#define APP_MSB_RESULT_SHIFT 15U
#define APP_MSB_RESULT_BITS 2U
#define APP_MSB_USED_BITS 17U

#define APP_SPEED_CM_S_MAX 127U

typedef enum
{
    APP_MOTION_BRAKED       = 0,
    APP_MOTION_FOLLOWING    = 1,
    APP_MOTION_MOVING       = 2,
    APP_MOTION_TURNING      = 3,
    APP_MOTION_STOP_LATCHED = 4
} app_motion_state_t;

typedef enum
{
    APP_MOTION_RESULT_OK       = 0,
    APP_MOTION_RESULT_REJECTED = 1,
    APP_MOTION_RESULT_ABORTED  = 2,
    APP_MOTION_RESULT_TIMEOUT  = 3
} app_motion_result_t;

/* ------------------------------------------------------------------------ */
/* MBF_MOTION_CMD record (uint32_t), 30 meaningful bits.                    */
/*   bits 0-1   type   0 BRAKE 1 FOLLOW 2 MOVE 3 TURN                       */
/*   bits 2-8   speed  0..127 cm/s                                          */
/*   bits 9-24  param  int16: MOVE = signed mm, TURN = signed 0.1 deg       */
/*   bit  25    until_line                                                  */
/*   bits 26-29 seq 0..15                                                   */
/*   bits 30-31 reserved (0)                                                */
/* ------------------------------------------------------------------------ */
#define APP_MC_TYPE_SHIFT 0U
#define APP_MC_TYPE_BITS 2U
#define APP_MC_SPEED_SHIFT 2U
#define APP_MC_SPEED_BITS 7U
#define APP_MC_PARAM_SHIFT 9U
#define APP_MC_PARAM_BITS 16U
#define APP_MC_UNTIL_LINE_SHIFT 25U
#define APP_MC_SEQ_SHIFT 26U
#define APP_MC_SEQ_BITS 4U
#define APP_MC_USED_BITS 30U
#define APP_MC_RECORD_BYTES 4U
#define APP_MC_QUEUE_DEPTH 4U

typedef enum
{
    APP_MC_BRAKE  = 0,
    APP_MC_FOLLOW = 1,
    APP_MC_MOVE   = 2,
    APP_MC_TURN   = 3
} app_motion_cmd_type_t;

/* ------------------------------------------------------------------------ */
/* LineState (uint32_t), writer: LineSensingTask, 23 meaningful bits.       */
/*   bits 0-7   line_error int8  -100..+100 (sign: + = line to the right)   */
/*   bits 8-15  steer      int8  -100..+100                                 */
/*   bit  16    on_line                                                     */
/*   bits 17-18 event 0 NONE 1 JUNCTION 2 LOST 3 FOUND                      */
/*   bits 19-22 seq                                                         */
/*   bits 23-31 reserved                                                    */
/* ------------------------------------------------------------------------ */
#define APP_LS_ERROR_SHIFT 0U
#define APP_LS_STEER_SHIFT 8U
#define APP_LS_INT8_BITS 8U
#define APP_LS_ON_LINE_SHIFT 16U
#define APP_LS_EVENT_SHIFT 17U
#define APP_LS_EVENT_BITS 2U
#define APP_LS_SEQ_SHIFT 19U
#define APP_LS_SEQ_BITS 4U
#define APP_LS_USED_BITS 23U
#define APP_LINE_ERROR_MAX 100
#define APP_LINE_ERROR_MIN (-100)

typedef enum
{
    APP_LINE_EVT_NONE     = 0,
    APP_LINE_EVT_JUNCTION = 1,
    APP_LINE_EVT_LOST     = 2,
    APP_LINE_EVT_FOUND    = 3
} app_line_event_t;

/* Raw ADC samples are task-private (12 bits each, 0..4095). */
#define APP_ADC_BITS 12U
#define APP_ADC_MAX 4095U

/* ------------------------------------------------------------------------ */
/* Barcode EdgeRing entry (uint16_t): bit 15 level, bits 0-14 delta in 8 us */
/* units, saturating at 0x7FFF (262 ms). 64 entries = 128 B. ISR writes     */
/* head (uint8_t), task reads tail (uint8_t). PROVISIONAL unit (A6).        */
/* ------------------------------------------------------------------------ */
#define APP_EDGE_LEVEL_SHIFT 15U
#define APP_EDGE_DELTA_MASK 0x7FFFU
#define APP_EDGE_DELTA_UNIT_US 8U
#define APP_EDGE_RING_ENTRIES 64U
#define APP_EDGE_RING_BYTES (APP_EDGE_RING_ENTRIES * 2U)

/* barcode byte (uint8_t), writer: BarcodeDecodeTask.                       */
/*   bits 0-2 result, bits 3-6 seq, bit 7 reserved                          */
#define APP_BC_RESULT_SHIFT 0U
#define APP_BC_RESULT_BITS 3U
#define APP_BC_SEQ_SHIFT 3U
#define APP_BC_SEQ_BITS 4U

typedef enum
{
    APP_BC_NONE         = 0,
    APP_BC_LEFT         = 1,
    APP_BC_RIGHT        = 2,
    APP_BC_STRAIGHT     = 3,
    APP_BC_UTURN        = 4,
    APP_BC_INV_NO_MATCH = 5,
    APP_BC_INV_OVERFLOW = 6
} app_barcode_result_t;

/* ------------------------------------------------------------------------ */
/* ImuState (uint32_t), writer: ImuTask, exactly 32 bits.                   */
/*   bits 0-7   pitch int8 in 0.5 deg units (+-63.5 deg)                    */
/*   bits 8-9   quality 0 NORMAL 1 REJECTED 2 DEGRADED 3 INVALID            */
/*   bits 10-13 episode seq                                                 */
/*   bits 14-20 height_mm 0..127     PROVISIONAL (A8), saturating          */
/*   bits 21-27 max_height_mm 0..127 PROVISIONAL (A8), saturating          */
/*   bits 28-30 motion_class (6 classes)                                    */
/*   bit  31    hump_active                                                 */
/* ------------------------------------------------------------------------ */
#define APP_IS_PITCH_SHIFT 0U
#define APP_IS_PITCH_BITS 8U
#define APP_IS_QUALITY_SHIFT 8U
#define APP_IS_QUALITY_BITS 2U
#define APP_IS_SEQ_SHIFT 10U
#define APP_IS_SEQ_BITS 4U
#define APP_IS_HEIGHT_SHIFT 14U
#define APP_IS_HEIGHT_BITS 7U
#define APP_IS_MAXH_SHIFT 21U
#define APP_IS_MAXH_BITS 7U
#define APP_IS_CLASS_SHIFT 28U
#define APP_IS_CLASS_BITS 3U
#define APP_IS_ACTIVE_SHIFT 31U
#define APP_IS_USED_BITS 32U
#define APP_HUMP_HEIGHT_MAX 127U

typedef enum
{
    APP_HUMP_Q_NORMAL   = 0,
    APP_HUMP_Q_REJECTED = 1,
    APP_HUMP_Q_DEGRADED = 2,
    APP_HUMP_Q_INVALID  = 3
} app_hump_quality_t;

typedef enum
{
    APP_MOTION_CLASS_STATIONARY   = 0,
    APP_MOTION_CLASS_ACCELERATING = 1,
    APP_MOTION_CLASS_TURNING      = 2,
    APP_MOTION_CLASS_CLIMBING     = 3,
    APP_MOTION_CLASS_DESCENDING   = 4,
    APP_MOTION_CLASS_IMPACT       = 5,
    APP_MOTION_CLASS_CRUISING     = 6 /* steady straight motion; 7 reserved */
} app_motion_class_t;

/* ------------------------------------------------------------------------ */
/* Echo capture: uint16_t echo_width_us 0..30000, ISR-only writer, plus a   */
/* uint8_t capture status/generation (app_obstacle.h).                      */
/* ------------------------------------------------------------------------ */
#define APP_ECHO_WIDTH_MAX_US 30000U
#define APP_RANGE_CM_BITS 9U
#define APP_RANGE_CM_SENTINEL 511U /* saturated or no echo */

/* ObstacleProfile: two uint32_t words, writer ObstacleScanTask, mutex copy.*/
/* 50 meaningful bits (D-12).                                               */
/* word0: bits 0-8 range_cm, 9-14 scan_angle_idx (0..36), 15-20 width_cm   */
/*        (0..63), 21-27 y_min int7 cm (-64..63), 28-31 reserved            */
/* word1: bits 0-6 y_max int7 cm, 7-15 closest_cm (0..511), 16-18 action,  */
/*        19-21 scan_state, 22-25 profile seq, 26-31 reserved               */
#define APP_OP0_RANGE_SHIFT 0U
#define APP_OP0_ANGLE_SHIFT 9U
#define APP_OP0_ANGLE_BITS 6U
#define APP_OP0_WIDTH_SHIFT 15U
#define APP_OP0_WIDTH_BITS 6U
#define APP_OP0_YMIN_SHIFT 21U
#define APP_OP_Y_BITS 7U
#define APP_OP1_YMAX_SHIFT 0U
#define APP_OP1_CLOSEST_SHIFT 7U
#define APP_OP1_ACTION_SHIFT 16U
#define APP_OP1_ACTION_BITS 3U
#define APP_OP1_SCANST_SHIFT 19U
#define APP_OP1_SCANST_BITS 3U
#define APP_OP1_SEQ_SHIFT 22U
#define APP_OP1_SEQ_BITS 4U
#define APP_OBSTACLE_WORDS 2U
#define APP_SCAN_ANGLE_IDX_MAX 36U
#define APP_OBST_WIDTH_CM_MAX 63U

typedef enum
{
    APP_AVOID_STOP       = 0,
    APP_AVOID_CONTINUE   = 1,
    APP_AVOID_TURN_LEFT  = 2,
    APP_AVOID_TURN_RIGHT = 3,
    APP_AVOID_REVERSE    = 4 /* reverse and reattempt */
} app_avoid_action_t;

/* ------------------------------------------------------------------------ */
/* comms_word (uint16_t), writer: TelemetryCommsTask, 14 meaningful bits.   */
/*   bits 0-1  conn_state, bits 2-5 backoff_s 0..10, bits 6-13 dropped (sat)*/
/* ------------------------------------------------------------------------ */
#define APP_CW_STATE_SHIFT 0U
#define APP_CW_STATE_BITS 2U
#define APP_CW_BACKOFF_SHIFT 2U
#define APP_CW_BACKOFF_BITS 4U
#define APP_CW_DROPPED_SHIFT 6U
#define APP_CW_DROPPED_BITS 8U
#define APP_CW_USED_BITS 14U
#define APP_BACKOFF_S_MAX 10U

typedef enum
{
    APP_CONN_WIFI_CONNECTING = 0,
    APP_CONN_MQTT_CONNECTING = 1,
    APP_CONN_ONLINE          = 2,
    APP_CONN_BACKOFF         = 3
} app_conn_state_t;

/* ------------------------------------------------------------------------ */
/* MBF_TELEM_EVT record (uint32_t): bits 0-4 type, 5-7 source, 8-23 payload,*/
/* 24-31 ms timestamp (wraps at 256 ms, D-05). 32 records.                  */
/* ------------------------------------------------------------------------ */
#define APP_TE_TYPE_SHIFT 0U
#define APP_TE_TYPE_BITS 5U
#define APP_TE_SOURCE_SHIFT 5U
#define APP_TE_SOURCE_BITS 3U
#define APP_TE_PAYLOAD_SHIFT 8U
#define APP_TE_PAYLOAD_BITS 16U
#define APP_TE_STAMP_SHIFT 24U
#define APP_TE_STAMP_BITS 8U
#define APP_TE_RECORD_BYTES 4U
#define APP_TE_QUEUE_DEPTH 32U

/** Telemetry event sources (3 bits). */
typedef enum
{
    APP_SRC_MOTION     = 0,
    APP_SRC_CONTROLLER = 1,
    APP_SRC_LINE       = 2,
    APP_SRC_IMU        = 3,
    APP_SRC_SCAN       = 4,
    APP_SRC_BARCODE    = 5,
    APP_SRC_COMMS      = 6,
    APP_SRC_SHARED     = 7
} app_telem_source_t;

/** Telemetry event types (5 bits, <= 32). Owner: Buddy 1. */
typedef enum
{
    APP_TEV_NONE          = 0,
    APP_TEV_SYS_STATE     = 1, /* payload: new state */
    APP_TEV_SAFE_STOP     = 2, /* payload: stop_cause */
    APP_TEV_FAULT_SET     = 3, /* payload: fault bit mask set */
    APP_TEV_FAULT_CLR     = 4,
    APP_TEV_MOTION_DONE   = 5, /* payload: seq<<8 | result */
    APP_TEV_MOTION_REJECT = 6,
    APP_TEV_LINE_EVENT    = 7,  /* payload: line event */
    APP_TEV_BARCODE       = 8,  /* payload: result<<8 | seq */
    APP_TEV_HUMP_START    = 9,  /* payload: episode seq */
    APP_TEV_HUMP_END      = 10, /* payload: quality<<12 | height */
    APP_TEV_OBSTACLE      = 11, /* payload: range_cm */
    APP_TEV_SCAN_DONE     = 12, /* payload: action<<8 | width */
    APP_TEV_REMOTE_CMD    = 13, /* payload: cmd<<8 | seq */
    APP_TEV_CMD_REJECTED  = 14, /* payload: reason */
    APP_TEV_CONN_STATE    = 15, /* payload: conn_state */
    APP_TEV_DROPPED       = 16, /* payload: dropped count */
    APP_TEV_BOOT          = 17, /* payload: self-check bits */
    APP_TEV_RECOVERY      = 18, /* payload: 0 */
    APP_TEV_MOTION_CLASS  = 19, /* payload: class */
    APP_TEV_COUNT         = 20
} app_telem_event_t;

APP_STATIC_ASSERT(APP_TEV_COUNT <= 32, telem_event_fits_5_bits);

/* ------------------------------------------------------------------------ */
/* RemoteCmd (uint8_t, D-11): bits 0-5 seq, bits 6-7 cmd.                   */
/* ------------------------------------------------------------------------ */
#define APP_RC_SEQ_MASK 0x3FU
#define APP_RC_CMD_SHIFT 6U
#define APP_RC_CMD_MASK 0x03U

typedef enum
{
    APP_REMOTE_CMD_INVALID0 = 0,
    APP_REMOTE_CMD_START    = 1,
    APP_REMOTE_CMD_STOP     = 2,
    APP_REMOTE_CMD_INVALID3 = 3
} app_remote_cmd_t;

/* ------------------------------------------------------------------------ */
/* Telemetry frame sizes (bytes on the wire, little-endian). Layout is in   */
/* app_telemetry.h; sizes are here so the budget is visible in one place.   */
/* ------------------------------------------------------------------------ */
#define APP_FRAME_VERSION 1U
#define APP_SNAPSHOT_BYTES 18U /* adopted: 139 meaningful bits */
#define APP_SNAPSHOT_BITS 139U
#define APP_EVENT_FRAME_BYTES 4U
#define APP_CMD_FRAME_BYTES 3U
#define APP_HB_FRAME_BYTES 6U   /* D-06 */
#define APP_OBS_FRAME_BYTES 10U /* D-06 */

#endif /* APP_DATA_H */
