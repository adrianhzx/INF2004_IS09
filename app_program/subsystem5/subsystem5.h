/** @file subsystem5.h
 *  @brief Subsystem 5 public interface: ultrasonic scan and obstacle profile
 *         (ObstacleScanTask, priority 6, with EchoIsr). Owner: Buddy 5
 *         Ho Zhi Xian Adrian 2500280.
 *
 *  Owns GP15 (servo PWM), GP16 (TRIG) and GP17 (ECHO, dispatcher callback).
 *  Requests arrive on FLG_SCAN (APP_SCAN_REQ_*), results leave as
 *  EVT_OBSTACLE / EVT_SCAN_DONE / EVT_SCAN_FAIL / EVT_SIDE_UNSAFE on FLG_CTRL
 *  and as the two ObstacleProfile words under MTX_OBSTACLE.
 *  Hardware gates: APP_CAL_ECHO_SHIFTER_VERIFIED (TRIG never driven while 0)
 *  and APP_CAL_SERVO_ARMED (servo never energised while 0).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../shared/app_ipc.h"
#include "../shared/app_obstacle.h"

IMPORT ID flg_scan;     /* IF-SC: controller requests, EchoIsr ECHO_DONE */
IMPORT ID mtx_obstacle; /* IF-OP: ObstacleProfile copy mutex (TA_INHERIT) */
IMPORT ID tid_scan;     /* ObstacleScanTask */

/* task_scan.c */
void s5_scan_task (INT stacd, void * p_exinf);
bool s5_scan_init_hw (void); /* servo, TRIG, ECHO attach (disabled) */
/** Coherent copy of the profile under MTX_OBSTACLE (readers: controller,
 *  telemetry). Returns false if the mutex could not be taken in time. */
bool s5_profile_copy (app_obstacle_profile_t * p_view, uint32_t * p_w0,
                      uint32_t * p_w1);
bool s5_scan_enabled (void); /* both hardware gates signed */
void s5_echo_irq_enable (bool on);
/* diagnostics */
uint32_t s5_watch_count (void);
uint32_t s5_scan_done_count (void);
uint32_t s5_scan_fail_count (void);
uint32_t s5_last_scan_ms (void);
uint8_t  s5_mode (void);

#endif /* SUBSYSTEM5_H */
