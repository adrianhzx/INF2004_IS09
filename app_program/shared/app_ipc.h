/** @file app_ipc.h
 *  @brief Kernel object catalogue: task priorities, event-flag bit maps,
 *         message-buffer sizing, the shared 10 ms cyclic handler and the
 *         non-blocking telemetry event post.
 *
 *  Object ownership (TEAM.md section 5): the receiver creates the object.
 *   FLG_CTRL         shared/vehicle_controller (created by app_ipc_init)
 *   FLG_SCAN         subsystem5
 *   FLG_BARCODE      subsystem3
 *   MBF_MOTION_CMD   subsystem2
 *   MBF_TELEM_EVT    subsystem1
 *   MTX_OBSTACLE     subsystem5
 *   CYC_10MS         shared (app_ipc_init), wakes Motion, Line, IMU; ticks
 *                    the controller every 20 ms.
 */

#ifndef APP_IPC_H
#define APP_IPC_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"

/* ---- Task priorities (Rev B section 3; lower = higher) ------------------ */
#define APP_PRI_MOTION 2
#define APP_PRI_CONTROLLER 3
#define APP_PRI_LINE 4
#define APP_PRI_IMU 5
#define APP_PRI_SCAN 6
#define APP_PRI_BARCODE 7
#define APP_PRI_TELEMETRY 8

/* ---- Stack sizes (bytes). Measured in Phase 8 (NFR16 <= 75 %). --------- */
#define APP_STACK_MOTION 2048U
#define APP_STACK_CONTROLLER 2048U
#define APP_STACK_LINE 1536U
#define APP_STACK_IMU 1536U
#define APP_STACK_SCAN 2048U
#define APP_STACK_BARCODE 1536U
#define APP_STACK_TELEMETRY 2048U

/* ---- Message buffer sizing (D-07): 4-byte kernel header per message,   */
/* message rounded up to 4 bytes.                                           */
#define APP_MBF_HEADER_BYTES 4U
#define APP_MBF_ROUND4(n) ((((n) + 3U) / 4U) * 4U)
#define APP_MBF_BYTES(records, rec_bytes)                                      \
    ((records) * (APP_MBF_HEADER_BYTES + APP_MBF_ROUND4(rec_bytes)))

#define APP_MBF_MOTION_BYTES                                                   \
    APP_MBF_BYTES(APP_MC_QUEUE_DEPTH, APP_MC_RECORD_BYTES) /* 32 */
#define APP_MBF_TELEM_BYTES                                                    \
    APP_MBF_BYTES(APP_TE_QUEUE_DEPTH, APP_TE_RECORD_BYTES) /* 256 */

/* ---- FLG_CTRL bits (consumer: VehicleControllerTask) ------------------- */
#define APP_EVT_TICK (1UL << 0U)      /* every 20 ms from CYC_10MS */
#define APP_EVT_SAFE_STOP (1UL << 1U) /* app_safe_stop_request */
#define APP_EVT_LINE_LOST (1UL << 2U) /* LineSensingTask */
#define APP_EVT_LINE_FOUND (1UL << 3U)
#define APP_EVT_JUNCTION (1UL << 4U)
#define APP_EVT_BARCODE (1UL << 5U)     /* BarcodeDecodeTask */
#define APP_EVT_MOTION_DONE (1UL << 6U) /* MotionControlTask */
#define APP_EVT_MOTION_REJECT (1UL << 7U)
#define APP_EVT_HUMP_START (1UL << 8U) /* ImuTask */
#define APP_EVT_HUMP_END (1UL << 9U)
#define APP_EVT_OBSTACLE (1UL << 10U) /* ObstacleScanTask */
#define APP_EVT_SCAN_DONE (1UL << 11U)
#define APP_EVT_SCAN_FAIL (1UL << 12U)
#define APP_EVT_REMOTE_CMD (1UL << 13U)  /* TelemetryCommsTask */
#define APP_EVT_START_BTN (1UL << 14U)   /* button ISR (debounced in task) */
#define APP_EVT_STOP_BTN (1UL << 15U)    /* button ISR, informational */
#define APP_EVT_SIDE_UNSAFE (1UL << 16U) /* ObstacleScanTask side ranging */
#define APP_EVT_CTRL_ALL 0x0001FFFFUL

/* ---- FLG_SCAN bits (consumer: ObstacleScanTask) ------------------------ */
#define APP_SCAN_REQ_WATCH_ON (1UL << 0U)
#define APP_SCAN_REQ_WATCH_OFF (1UL << 1U)
#define APP_SCAN_REQ_PROFILE (1UL << 2U)
#define APP_SCAN_REQ_SIDE_L (1UL << 3U)
#define APP_SCAN_REQ_SIDE_R (1UL << 4U)
#define APP_SCAN_REQ_IDLE (1UL << 5U)
#define APP_SCAN_ECHO_DONE (1UL << 6U) /* EchoIsr */
#define APP_SCAN_TICK (1UL << 7U)      /* 100 ms watch tick (from CYC_10MS) */
#define APP_SCAN_ABORT (1UL << 8U)

/* ---- FLG_BARCODE bits (consumer: BarcodeDecodeTask) -------------------- */
#define APP_BC_EDGE_AVAIL (1UL << 0U) /* BarcodeEdgeIsr */
#define APP_BC_ARM (1UL << 1U)
#define APP_BC_DISARM (1UL << 2U)

/* ---- Cyclic 10 ms service ------------------------------------------------ */
/** Slots for tasks woken by tk_wup_tsk every 10 ms, in wake order. */
typedef enum
{
    APP_CYC_SLOT_MOTION = 0,
    APP_CYC_SLOT_LINE   = 1,
    APP_CYC_SLOT_IMU    = 2,
    APP_CYC_SLOT_COUNT  = 3
} app_cyc_slot_t;

/** Create FLG_CTRL and CYC_10MS (stopped). Call from usermain before any
 *  subsystem init. */
int32_t app_ipc_init (void);

/** Start the cyclic handler once every task to be woken has registered. */
int32_t app_ipc_start (void);

/** Register the task ID that a slot wakes every 10 ms (0 = none). */
void app_ipc_register_cyclic_task (app_cyc_slot_t slot, int32_t tid);

/** Register FLG_SCAN so the cyclic handler can deliver APP_SCAN_TICK. */
void app_ipc_register_scan_flag (int32_t flg_id);

/** Register MBF_TELEM_EVT (created by subsystem1). */
void app_ipc_register_telem_mbf (int32_t mbf_id);

/** Register MBF_MOTION_CMD (created by subsystem2) / query it (0 = none). */
void    app_ipc_register_motion_mbf (int32_t mbf_id);
int32_t app_ipc_mbf_motion (void);

/** FLG_CTRL event flag ID (valid after app_ipc_init). */
int32_t app_ipc_flg_ctrl (void);

/** FLG_SCAN event flag ID (0 until subsystem5 registered it). */
int32_t app_ipc_flg_scan (void);

/** Milliseconds since the cyclic handler started (10 ms resolution, wraps). */
uint32_t app_ipc_ms (void);

/** Non-blocking telemetry event post. Never waits; counts drops. Legal from
 *  an ISR (tk_snd_mbf with TMO_POL). */
void app_ipc_telem_post (app_telem_event_t type, app_telem_source_t source,
                         uint16_t payload);

/** Pack/unpack of the 4-byte event record. */
uint32_t app_ipc_telem_pack (app_telem_event_t type, app_telem_source_t source,
                             uint16_t payload, uint8_t stamp_ms);
void     app_ipc_telem_unpack (uint32_t record, uint8_t * p_type,
                               uint8_t * p_source, uint16_t * p_payload,
                               uint8_t * p_stamp);

/** Dropped-event counter (saturating 8-bit), mirrored by TelemetryCommsTask. */
uint8_t app_ipc_telem_dropped (void);

/** Diagnostics for the cyclic spike (count of handler invocations). */
uint32_t app_ipc_cyclic_count (void);

#endif /* APP_IPC_H */
