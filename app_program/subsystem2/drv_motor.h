/** @file drv_motor.h
 *  @brief Robo Pico on-board motor driver (MX1508 class) on GP8-GP11.
 *         Only MotionControlTask calls this module (single writer, NFR23).
 *
 *  Truth table (Robo Pico datasheet table 3): A low / B low = BRAKE,
 *  A high / B low = forward, A low / B high = backward, A high / B high =
 *  coast. PWM frequency 20 kHz (datasheet item 10). Both pins of a motor are
 *  driven by the two channels of one PWM slice (slice 4 = GP8/GP9, slice 5
 *  = GP10/GP11); the inactive pin is held at compare 0 = constant low, so
 *  "brake" is simply both compares at 0 and the pins never change function
 *  at run time.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DRV_MOTOR_LEFT  = 0, /* M1: GP8 (A) / GP9 (B) */
    DRV_MOTOR_RIGHT = 1  /* M2: GP10 (A) / GP11 (B) */
} drv_motor_id_t;

/** Release the PWM block (done by app_hw_init), configure slices 4 and 5 at
 *  20 kHz with both compares at 0 (brake), route GP8-GP11 to PWM. The motors
 *  are braked when this returns. */
void drv_motor_init (void);

/** Set one motor: pct -100..100 (sign = direction, 0 = brake). Values are
 *  clamped. Forward = A pulsed, B low. */
void drv_motor_set (drv_motor_id_t motor, int8_t pct);

/** Brake both motors (both pins low) - one call, no computation. */
void drv_motor_brake_all (void);

/** Read back what the hardware compare registers hold (tests, telemetry). */
void drv_motor_get_compare (drv_motor_id_t motor, uint16_t * p_cc_a,
                            uint16_t * p_cc_b);

/** PWM wrap value used (TOP + 1 = full scale). */
uint32_t drv_motor_top (void);

#endif /* DRV_MOTOR_H */
