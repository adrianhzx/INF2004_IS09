/** @file subsystem4.h
 *  @brief Subsystem 4 public interface: IMU, hump and collision (ImuTask,
 *         priority 5, 10 ms). Owner: Buddy 4 Wong Wei Qi 2500430.
 *
 *  Owns I2C0 (GP4/GP5) through subsystem4/drv_i2c.c; the kernel I2C device
 *  driver stays OFF. Publishes g_imu_state (one word, app_blackboard.c),
 *  EVT_HUMP_START / EVT_HUMP_END, telemetry events HUMP_START / HUMP_END /
 *  MOTION_CLASS, and calls app_safe_stop_request(COLLISION). Other
 *  subsystems include it as "../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../shared/app_ipc.h"

IMPORT ID tid_imu; /* ImuTask */

/* task_imu.c */
void     s4_imu_task (INT stacd, void * p_exinf);
void     s4_imu_cycle (void);
bool     s4_imu_sensor_init (void);
bool     s4_imu_sensor_ok (void);
uint32_t s4_imu_cycle_count (void);
uint32_t s4_imu_max_cycle_us (void);
uint32_t s4_imu_read_errors (void);
void s4_imu_last_sample (int16_t * p_x_mg, int16_t * p_y_mg, int16_t * p_z_mg);

#endif /* SUBSYSTEM4_H */
