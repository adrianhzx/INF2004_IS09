/** @file app_safety.h
 *  @brief STOP_LATCH, stop_cause, fault_mask and the SafeStop_Request contract
 *         (Rev B section 3 "Safety objects and ownership").
 *
 *  Contract (adopted, D-03 wording):
 *   1. app_safe_stop_request() stores APP_STOP_LATCH_SET to the dedicated
 *      byte g_stop_latch with a plain aligned byte store, before anything else.
 *   2. If stop_cause == NONE it stores the cause; this write-if-NONE is a
 *      read-modify-write and goes through the interrupt-off helper, so the
 *      first cause wins even against a concurrent ISR.
 *   3. It sets FLG_CTRL.EVT_SAFE_STOP (ISR-legal tk_set_flg, verified).
 *   4. MotionControlTask observes g_stop_latch at the start of its next 10 ms
 *      cycle (one byte load, no critical section) and brakes.
 *  Only VehicleControllerTask may clear the latch and cause, only through
 *  app_safety_controller_clear(), only under the S7 recovery guard it owns.
 *
 *  Setting a fault bit NEVER latches a stop. Bits in APP_FAULT_DIAGNOSTIC_MASK
 *  are never a reason to call app_safe_stop_request() (D-08).
 */

#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"

/** Dedicated STOP_LATCH byte. Readers load it directly (one byte load). */
extern volatile uint8_t g_stop_latch;

/** One-time init; registers the controller task that may clear the latch.
 *  ctrl_tid may be 0 until the controller is started (app_safety_set_ctrl). */
void app_safety_init (void);

/** Record the controller task ID (only this task may clear the latch) and the
 *  FLG_CTRL event flag ID used for EVT_SAFE_STOP. */
void app_safety_bind (int32_t ctrl_tid, int32_t flg_ctrl_id,
                      uint32_t evt_safe_stop_bit);

static inline bool
app_stop_latched (void)
{
    return APP_STOP_LATCH_SET == g_stop_latch;
}

/** ISR-safe. Implements steps 1-3 above. Never blocks. */
void app_safe_stop_request (app_stop_cause_t cause);

/** Current first-cause (NONE when recovered). */
app_stop_cause_t app_stop_cause_get (void);

/** Controller-only clear. Returns false (and does nothing) if the caller is
 *  not the bound controller task or if the latch was already clear. The
 *  caller must have verified the S7 guard (state S7, speed zero, initiating
 *  condition cleared, START held T_HOLD) before calling. */
bool app_safety_controller_clear (void);

/* ---- Multi-writer bit helpers (rule 4) --------------------------------- */
/** Atomic |= / &= ~ on a shared 16-bit word (interrupts off ~10 cycles on
 *  single core; SIO spinlock + PRIMASK on SMP builds). */
void app_shared_bits_set (volatile uint16_t * p_word, uint16_t bits);
void app_shared_bits_clr (volatile uint16_t * p_word, uint16_t bits);
/** 8-bit variants (safety_word, barcode byte if ever multi-writer). */
void app_shared_bits_set8 (volatile uint8_t * p_byte, uint8_t bits);
void app_shared_bits_clr8 (volatile uint8_t * p_byte, uint8_t bits);

/* ---- fault_mask ---------------------------------------------------------- */
/** Set fault bits (through the helper) and post a telemetry event. */
void     app_fault_set (uint16_t bits);
void     app_fault_clr (uint16_t bits);
uint16_t app_fault_get (void);

/** Raw access for tests and telemetry (read is a single 16-bit load). */
extern volatile uint16_t g_fault_mask;

/** Statistics for the A3 race spike (counts of helper calls). */
extern volatile uint32_t g_safety_helper_calls;

#endif /* APP_SAFETY_H */
