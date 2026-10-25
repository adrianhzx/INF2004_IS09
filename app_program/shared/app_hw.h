/** @file app_hw.h
 *  @brief Board pin map (Rev B section 2) and reg-level RP2040 helpers
 *         shared by all subsystem drivers: GPIO, pin interrupts, PWM slices,
 *         the 1 MHz free-running timer and peripheral reset release.
 *
 *  This module performs NO motion.  It only owns the one-time hardware
 *  bring-up that the kernel port does not do (TIMER and PWM reset release,
 *  watchdog tick) and provides helpers.  Every pin is still written by its
 *  single owning task.
 *
 *  Register offsets are from the RP2040 datasheet (sections 2.14 IO_BANK0,
 *  2.19 PADS, 4.5 PWM, 4.6 TIMER, 4.7 WATCHDOG) cross-checked against the
 *  port's include/sys/sysdepend/cpu/rp2040/sysdef.h where it defines them.
 */

#ifndef APP_HW_H
#define APP_HW_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Logical GPIO map, Rev B section 2 (single owner each) ------------- */
#define APP_PIN_ENC_L_A 0U /* Motion / EncoderIsr */
#define APP_PIN_ENC_L_B 1U
#define APP_PIN_ENC_R_A 2U
#define APP_PIN_ENC_R_B 3U
#define APP_PIN_IMU_SDA 4U /* ImuTask, I2C0 */
#define APP_PIN_IMU_SCL 5U
#define APP_PIN_SPARE_6 6U
#define APP_PIN_BARCODE_DOUT 7U /* BarcodeEdgeIsr, both edges */
#define APP_PIN_MOTOR_L_A 8U    /* M1A, MotionControlTask only */
#define APP_PIN_MOTOR_L_B 9U    /* M1B */
#define APP_PIN_MOTOR_R_A 10U   /* M2A */
#define APP_PIN_MOTOR_R_B 11U   /* M2B */
#define APP_PIN_SPARE_12 12U
#define APP_PIN_SPARE_13 13U
#define APP_PIN_SPARE_14 14U
#define APP_PIN_SCAN_SERVO 15U /* ObstacleScanTask, 50 Hz */
#define APP_PIN_US_TRIG 16U    /* ObstacleScanTask */
#define APP_PIN_US_ECHO 17U    /* EchoIsr; MUST be <= 3.3 V (HV-5) */
#define APP_PIN_STATUS_RGB 18U /* VehicleControllerTask (optional) */
#define APP_PIN_SPARE_19 19U
#define APP_PIN_START_BTN 20U   /* VehicleControllerTask */
#define APP_PIN_STOP_BTN 21U    /* STOP ISR -> SafeStop_Request */
#define APP_PIN_BUZZER 22U      /* VehicleControllerTask (optional) */
#define APP_PIN_LINE_LEFT 26U   /* ADC0, LineSensingTask */
#define APP_PIN_LINE_CENTRE 27U /* ADC1, LineSensingTask */
#define APP_PIN_VBAT_SENSE 28U  /* ADC2, conditional jumper, NO barcode AOUT */
/* GP23/24/25/29: Pico W radio, never touched. */

#define APP_GPIO_COUNT 30U

/* ---- Interrupt numbers (RP2040 datasheet 2.3.2, table 80) -------------- */
#define APP_INTNO_IO_IRQ_BANK0 13U
#define APP_INTNO_ADC_FIFO 22U
#define APP_INTNO_I2C0 23U

/* ---- GPIO --------------------------------------------------------------- */
typedef enum
{
    APP_GPIO_PULL_NONE = 0,
    APP_GPIO_PULL_UP   = 1,
    APP_GPIO_PULL_DOWN = 2
} app_gpio_pull_t;

/** Configure a pin as an SIO input with the given pull and Schmitt trigger. */
void app_hw_gpio_input (uint32_t pin, app_gpio_pull_t pull);

/** Configure a pin as an SIO output driving @p level. */
void app_hw_gpio_output (uint32_t pin, bool level);

void app_hw_gpio_write (uint32_t pin, bool level);
bool app_hw_gpio_read (uint32_t pin);

/** Per-pin interrupt event bits (IO_BANK0 INTR/INTE/INTS nibble). */
#define APP_GPIO_IRQ_LEVEL_LOW (1U << 0U)
#define APP_GPIO_IRQ_LEVEL_HIGH (1U << 1U)
#define APP_GPIO_IRQ_EDGE_FALL (1U << 2U)
#define APP_GPIO_IRQ_EDGE_RISE (1U << 3U)
#define APP_GPIO_IRQ_EDGE_BOTH (APP_GPIO_IRQ_EDGE_FALL | APP_GPIO_IRQ_EDGE_RISE)

/** Enable (mask != 0) or disable (mask == 0) the PROC0 interrupt events of a
 *  pin. Pending edge events are acknowledged first. */
void app_hw_gpio_irq_enable (uint32_t pin, uint32_t event_mask);

/** Force a pin interrupt event (PROC0_INTF) - used by the on-target self-test
 *  to exercise the dispatcher without hardware. */
void app_hw_gpio_irq_force (uint32_t pin, uint32_t event_mask, bool on);

/** Read and acknowledge the PROC0 interrupt status of all 30 pins.
 *  p_status[i] receives the 4-bit event nibble of pin i. Edge bits are
 *  cleared in INTR (write-1-to-clear); level bits clear by themselves. */
void app_hw_gpio_irq_take_all (uint8_t * p_status);

/* ---- PWM ---------------------------------------------------------------- */
/** PWM slice (0..7) and channel (0 = A, 1 = B) of a pin. */
static inline uint32_t
app_hw_pwm_slice (uint32_t pin)
{
    return (pin >> 1) & 7U;
}
static inline uint32_t
app_hw_pwm_channel (uint32_t pin)
{
    return pin & 1U;
}

/** Configure a slice: clock divider (8.4 fixed point: int 1..255, frac 0..15)
 *  and wrap (TOP). The slice is left disabled; call app_hw_pwm_enable. */
void app_hw_pwm_config_slice (uint32_t slice, uint32_t div_int,
                              uint32_t div_frac, uint32_t top);
void app_hw_pwm_enable (uint32_t slice, bool enable);
/** Set the compare value of one channel (0 .. top+1). */
void app_hw_pwm_set_level (uint32_t pin, uint32_t level);
/** Route a pin to its PWM channel (funcsel PWM). */
void app_hw_pwm_pin (uint32_t pin);

/* ---- Time ---------------------------------------------------------------- */
/** Free-running 1 MHz counter (TIMER.TIMERAWL), wraps every ~71.6 min.
 *  Readers must use unsigned differences only. */
uint32_t app_hw_time_us (void);

/** Busy-wait (bounded, for drivers only; never from an ISR with us > 20). */
void app_hw_delay_us (uint32_t us);

/* ---- Interrupt masking (single core) --------------------------------- */
/** Mask local interrupts; returns the previous state for app_hw_irq_restore.
 *  On SMP=1 builds this is the application spinlock variant. */
uint32_t app_hw_irq_save (void);
void     app_hw_irq_restore (uint32_t saved);

/* ---- Boot ---------------------------------------------------------------- */
/** Self-check result bits reported by app_hw_init(). */
#define APP_HW_OK_TIMER_RUNS (1U << 0U)
#define APP_HW_OK_PWM_RELEASED (1U << 1U)
#define APP_HW_OK_IO_RELEASED (1U << 2U)

/** Release one peripheral from reset (RESETS_RESET_* bit from sysdef.h) and
 *  wait, bounded, for RESET_DONE. Never re-asserts a reset. Returns true when
 *  the block is out of reset. For task-owned peripherals (I2C0, ADC). */
bool app_hw_release_reset (uint32_t reset_bit);

/** Route a pin to a peripheral function (GPIO_CTRL_FUNCSEL_*) with input
 *  enable, schmitt trigger, optional pull-up and slow slew; the peripheral
 *  then owns the pad (I2C open-drain lines, for example). */
void app_hw_gpio_function (uint32_t pin, uint32_t funcsel, bool pull_up);

/** Release TIMER/PWM resets, start the watchdog tick, and verify that the
 *  microsecond timer advances. Returns the APP_HW_OK_* bits that passed.
 *  Called once from usermain() before any subsystem init. */
uint32_t app_hw_init (void);

#endif /* APP_HW_H */
