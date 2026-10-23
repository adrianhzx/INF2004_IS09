/** @file drv_servo.c
 *  @brief Scan servo PWM. See drv_servo.h.
 */

#include "drv_servo.h"

#include "../shared/app_cal.h"
#include "../shared/app_hw.h"
#include "../shared/app_port.h"

#define SERVO_SLICE 7U
#define SERVO_SLICE_STRIDE 0x14UL
#define SERVO_CC_OFF 0x0CUL
#define SERVO_CC_B_SHIFT 16U
#define SERVO_CC_MASK 0xFFFFUL
#define SERVO_FUNCSEL_MASK 0x1FUL

static uint32_t
clamp_us (uint32_t us)
{
    if (us < APP_CAL_SERVO_MIN_US)
    {
        return APP_CAL_SERVO_MIN_US;
    }
    if (us > APP_CAL_SERVO_MAX_US)
    {
        return APP_CAL_SERVO_MAX_US;
    }
    return us;
}

void
drv_servo_init (uint32_t pulse_us)
{
    app_hw_pwm_config_slice(SERVO_SLICE, DRV_SERVO_DIV_INT, 0U, DRV_SERVO_TOP);
    app_hw_pwm_set_level(APP_PIN_SCAN_SERVO, clamp_us(pulse_us));
#if APP_CAL_SERVO_ARMED
    app_hw_pwm_pin(APP_PIN_SCAN_SERVO); /* HV-7 signed: energise */
#endif
    app_hw_pwm_enable(SERVO_SLICE, true);
}

void
drv_servo_set_us (uint32_t pulse_us)
{
    app_hw_pwm_set_level(APP_PIN_SCAN_SERVO, clamp_us(pulse_us));
}

uint32_t
drv_servo_get_us (void)
{
    uint32_t const cc =
        in_w(PWM_BASE + (SERVO_SLICE * SERVO_SLICE_STRIDE) + SERVO_CC_OFF);
    return (cc >> SERVO_CC_B_SHIFT) & SERVO_CC_MASK;
}

bool
drv_servo_pin_routed (void)
{
    return (uint32_t) GPIO_CTRL_FUNCSEL_PWM ==
           (in_w(GPIO_CTRL(APP_PIN_SCAN_SERVO)) & SERVO_FUNCSEL_MASK);
}
