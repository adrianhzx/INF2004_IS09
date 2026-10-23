/** @file drv_motor.c
 *  @brief Robo Pico motor PWM driver implementation. See drv_motor.h.
 */

#include "drv_motor.h"

#include "../shared/app_hw.h"
#include "../shared/app_port.h"

/* 125 MHz system clock / (TOP + 1) = 20 kHz with divider 1.0 */
#define DRV_MOTOR_PWM_TOP 6249U
#define DRV_MOTOR_DIV_INT 1U
#define DRV_MOTOR_DIV_FRAC 0U
#define DRV_MOTOR_PCT_MAX 100
#define DRV_MOTOR_PCT_MIN (-100)
#define DRV_MOTOR_CC_MASK 0xFFFFU
#define DRV_MOTOR_CC_B_SHIFT 16U
#define DRV_MOTOR_PWM_CC_OFF 0x0CU
#define DRV_MOTOR_SLICE_STRIDE 0x14U

typedef struct
{
    uint32_t pin_a;
    uint32_t pin_b;
    uint32_t slice;
} drv_motor_pins_t;

static drv_motor_pins_t const g_motor_pins[2] = {
    {APP_PIN_MOTOR_L_A, APP_PIN_MOTOR_L_B, 4U},
    {APP_PIN_MOTOR_R_A, APP_PIN_MOTOR_R_B, 5U},
};

static bool g_motor_ready = false;

void
drv_motor_init (void)
{
    uint32_t i = 0U;

    for (i = 0U; i < 2U; i++)
    {
        app_hw_pwm_config_slice(g_motor_pins[i].slice, DRV_MOTOR_DIV_INT,
                                DRV_MOTOR_DIV_FRAC,
                                DRV_MOTOR_PWM_TOP); /* CC = 0 */
        app_hw_pwm_pin(
            g_motor_pins[i].pin_a); /* funcsel PWM: outputs low now */
        app_hw_pwm_pin(g_motor_pins[i].pin_b);
        app_hw_pwm_enable(g_motor_pins[i].slice, true);
    }
    g_motor_ready = true;
}

void
drv_motor_set (drv_motor_id_t motor, int8_t pct)
{
    drv_motor_pins_t const * p_m   = NULL;
    int32_t                  value = (int32_t) pct;
    uint32_t                 level = 0U;

    if (!g_motor_ready || ((uint32_t) motor > (uint32_t) DRV_MOTOR_RIGHT))
    {
        return;
    }
    p_m = &g_motor_pins[motor];
    if (value > DRV_MOTOR_PCT_MAX)
    {
        value = DRV_MOTOR_PCT_MAX;
    }
    if (value < DRV_MOTOR_PCT_MIN)
    {
        value = DRV_MOTOR_PCT_MIN;
    }
    if (0 == value)
    {
        app_hw_pwm_set_level(p_m->pin_a, 0U);
        app_hw_pwm_set_level(p_m->pin_b, 0U);
        return;
    }
    /* |value| 1..100 -> compare 62..6250 (TOP + 1 = 100 %) */
    level =
        ((uint32_t) ((value < 0) ? -value : value) * (DRV_MOTOR_PWM_TOP + 1U)) /
        (uint32_t) DRV_MOTOR_PCT_MAX;
    if (value > 0)
    {
        app_hw_pwm_set_level(p_m->pin_b, 0U); /* inactive side low first */
        app_hw_pwm_set_level(p_m->pin_a, level);
    }
    else
    {
        app_hw_pwm_set_level(p_m->pin_a, 0U);
        app_hw_pwm_set_level(p_m->pin_b, level);
    }
}

void
drv_motor_brake_all (void)
{
    uint32_t i = 0U;
    for (i = 0U; i < 2U; i++)
    {
        app_hw_pwm_set_level(g_motor_pins[i].pin_a, 0U);
        app_hw_pwm_set_level(g_motor_pins[i].pin_b, 0U);
    }
}

void
drv_motor_get_compare (drv_motor_id_t motor, uint16_t * p_cc_a,
                       uint16_t * p_cc_b)
{
    uint32_t cc = 0U;

    if ((uint32_t) motor > (uint32_t) DRV_MOTOR_RIGHT)
    {
        *p_cc_a = 0U;
        *p_cc_b = 0U;
        return;
    }
    cc = in_w(PWM_BASE + (g_motor_pins[motor].slice * DRV_MOTOR_SLICE_STRIDE) +
              DRV_MOTOR_PWM_CC_OFF);
    *p_cc_a = (uint16_t) (cc & DRV_MOTOR_CC_MASK);
    *p_cc_b = (uint16_t) ((cc >> DRV_MOTOR_CC_B_SHIFT) & DRV_MOTOR_CC_MASK);
}

uint32_t
drv_motor_top (void)
{
    return DRV_MOTOR_PWM_TOP;
}
