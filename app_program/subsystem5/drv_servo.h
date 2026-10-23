/** @file drv_servo.h
 *  @brief SG90 scan servo on GP15 = PWM slice 7 channel B at 50 Hz, 1 us per
 *         count (div 125, TOP 19999). While APP_CAL_SERVO_ARMED is 0 the
 *         slice runs but GP15 is never routed to it, so the servo is not
 *         energised (HV-7). Owner: Buddy 5.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdbool.h>
#include <stdint.h>

#define DRV_SERVO_TOP 19999U /* 20 ms period at 1 MHz */
#define DRV_SERVO_DIV_INT 125U

void drv_servo_init (uint32_t pulse_us);
/** Clamp to [APP_CAL_SERVO_MIN_US, APP_CAL_SERVO_MAX_US] and set the pulse. */
void     drv_servo_set_us (uint32_t pulse_us);
uint32_t drv_servo_get_us (void); /* compare value read back from the slice */
bool     drv_servo_pin_routed (void); /* GP15 funcsel == PWM */

#endif /* DRV_SERVO_H */
