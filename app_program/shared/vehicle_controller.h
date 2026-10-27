/** @file vehicle_controller.h
 *  @brief VehicleControllerTask (priority 3): sole mission arbiter, S0-S7,
 *         local START/STOP, guarded S7 recovery, controller heartbeat.
 *         Shared integration module (human owner: TBC, D-09).
 *
 *  The task is a thin shell around the pure statechart in mission_core.c:
 *  it waits on FLG_CTRL, gathers the inputs (events, buttons, blackboards,
 *  the obstacle profile under its mutex), runs one mission_core_step() and
 *  acts on the outputs in a fixed order: SafeStop request, motion record,
 *  scan request, guarded latch clear, MissionStatus publish, telemetry.
 *  Phase 7: S2-S5 implemented (line following, barcode -> junction -> nav,
 *  obstacle bypass with six actions and bounded retry, bounded line search).
 *  S6 has no producer on a loop course (D-28). Entering S2 is refused while
 *  APP_CAL_MOTION_ARMED == 0.
 */

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "app_data.h"
#include "app_motion.h"

/** Controller heartbeat, incremented every 20 ms tick while the controller
 *  is alive. MotionControlTask stops if it stays constant for
 *  APP_CAL_T_CTRL_HEARTBEAT_MS while driving (single writer, byte). */
extern volatile uint8_t g_ctrl_heartbeat;

/** Boot self-check bits from app_hw_init (APP_HW_OK_*), stored for S0->S1. */
void vehicle_controller_set_boot_check (uint32_t hw_ok_bits);

int32_t vehicle_controller_init (void);  /* binds safety, button IRQs */
int32_t vehicle_controller_start (void); /* creates and starts the task */

/** Current system state (from the published MissionStatus word). */
app_sys_state_t vehicle_controller_state (void);

/** Send a motion command record (non-blocking). Returns 0 or -1 if the
 *  queue is full / absent / latched. Controller-only API; the sequence
 *  number is assigned by the mission core. */
int32_t vehicle_controller_send_motion (app_motion_cmd_type_t type,
                                        uint8_t speed_cm_s, int16_t param,
                                        bool until_line, uint8_t seq);

/** Diagnostics for the on-target tests. */
uint32_t vehicle_controller_step_count (void);

#if APP_TEST
void vehicle_controller_test (void);
#endif

#endif /* VEHICLE_CONTROLLER_H */
