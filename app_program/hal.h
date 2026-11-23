/**
 * @file    hal.h
 * @brief   Hardware abstraction (drivers), built on the Pico SDK,
 *          ported to the IS09 car's wiring (pins.h, 19 Nov).
 *
 * The drivers (hal_*.c) include Pico SDK headers; the micro T-Kernel task
 * code cannot (the two define the same type names), so this interface uses
 * plain C types only. Interrupt handlers are defined here and registered
 * with tk_def_int() by os_glue.c (NFR13: drivers separated from tasks).
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

/* RP2040 interrupt numbers used by the robot */
#define HAL_IRQ_TIMER3   (3u)      /* TIMER alarm 3: 1 kHz barcode AO + line-sensor sampling */
#define HAL_IRQ_GPIO     (13u)     /* IO_BANK0: encoders, barcode DO, ultrasonic Echo */

/* ---- set-up ---- */
void     hal_init(void);                         /* pins, PWM, ADC, I2C, PIO; motors braked */
void     hal_irq_enable_sources(void);           /* per-pin edge IRQs + the 1 kHz alarm */
void     hal_gpio_isr(uint32_t intno);           /* registered with tk_def_int() */
void     hal_timer_isr(uint32_t intno);          /* registered with tk_def_int() */

uint64_t hal_time_us(void);
uint32_t hal_time_ms(void);

/* ---- motors (Robo Pico driver): IS09 car, M1 = LEFT wheel, M2 = RIGHT ---- */
#define HAL_MOTOR_RIGHT  (1u)                    /* logical ids, not M1/M2 */
#define HAL_MOTOR_LEFT   (2u)
void     hal_motor_set(uint32_t motor, float duty);  /* -1..+1 (capped at MOTOR_MAX_DUTY), 0 = brake (Low/Low) */
void     hal_motor_kill(void);                   /* both motors Low/Low now (E-stop) */
bool     hal_motor_killed(void);                 /* latched until hal_motor_unkill() */
void     hal_motor_unkill(void);

/* ---- encoders: wheel 0 = right, 1 = left; forward counts up ---- */
#define HAL_WHEEL_RIGHT  (0u)
#define HAL_WHEEL_LEFT   (1u)
int32_t  hal_enc_count(uint32_t wheel);          /* 4x quadrature, direction flags applied */
void     hal_enc_set_reversed(bool b_right, bool b_left);
uint32_t hal_odometer(void);                     /* |transitions| of both wheels (monotonic) */
uint32_t hal_enc_errors(void);                   /* impossible A/B transitions seen */

/* ---- IR line sensors (IS09: analog, ADC0 LEFT / ADC1 CENTRE) ----
 * The timer interrupt samples them at 1 kHz, alternately. tsk_line feeds
 * the means to is09_line_update() every 10 ms; the levels below are its
 * black / white decisions (true = black, LINE_DO_BLACK_HIGH = 1). */
void     hal_line_adc(uint32_t *p_left, uint32_t *p_centre);   /* mean since the last call */
bool     hal_ir_left_level(void);                /* LEFT sensor sees the line */
bool     hal_ir_right_level(void);               /* CENTRE sensor (right of LEFT) sees the line */

/* ---- barcode sensor ----
 * BARCODE_AO = 1: analog AO on GP28 (ADC2), sampled every 1 ms tick, as the
 * base reader expects. BARCODE_AO = 0: digital DO on GP7, every edge goes
 * from the GPIO interrupt to is09_bc_isr_edge(). */
uint32_t hal_ir_adc(void);                       /* last barcode AO sample */
void     hal_ir_adc_range(uint32_t *p_min, uint32_t *p_max);   /* barcode AO min/max since the last call (diagnostics) */
bool     hal_barcode_level(void);                /* live DO pin level (GP7) */
void     hal_barcode_config(uint32_t thr_black, uint32_t thr_white, bool b_black_high,
                            bool b_enable);
typedef struct
{
    uint32_t odo;                                /* hal_odometer() at the edge */
    uint32_t t_ms;
    bool     b_black;
} hal_bc_edge_t;
bool     hal_barcode_pop(hal_bc_edge_t *p_edge); /* oldest edge, false if none */
uint32_t hal_barcode_overruns(void);
/* BARCODE_DETECT = 1: the interrupt only records every 1 kHz sample with the
   odometer, and bc_detect.c (TSK_LINE_BARCODE) finds the bars. Turning raw
   mode on switches the threshold edges above off, and the reverse. */
void     hal_barcode_raw(bool b_enable);
bool     hal_barcode_pop_sample(int32_t *p_enc_r, int32_t *p_enc_l, uint32_t *p_adc);   /* oldest sample: wheel counts (direction flags applied) + ADC; false if none */

uint32_t hal_vbat_mv(void);                      /* 0 when VBAT sensing is off */

/* ---- HC-SR04 (Echo timed by the GPIO interrupt) ---- */
void     hal_us_trigger(void);                   /* 10 us pulse; >= 60 ms between pings */
float    hal_us_read_cm(void);                   /* after >= 30 ms: cm, or -1 = no echo */

/* ---- servo, buzzer, NeoPixels, buttons ---- */
void     hal_servo_deg(float physical_deg);      /* 0..180 physical, 90 = straight ahead */
void     hal_servo_range(uint32_t min_us, uint32_t max_us);   /* pulse at 0 and 180 deg */
void     hal_buzzer(uint32_t freq_hz);           /* 0 = off */
void     hal_neopixel(uint8_t r, uint8_t g, uint8_t b);   /* both on-board pixels */
bool     hal_btn1_down(void);
bool     hal_btn2_down(void);

/* ---- IMU (LSM303DLHC accelerometer on I2C0, IS09 settings: +-2 g HR) ---- */
typedef enum
{
    HAL_IMU_NONE = 0,
    HAL_IMU_LSM303DLHC,                          /* accel 0x19 (+ mag 0x1E), the IS09 part */
    HAL_IMU_LSM303D                              /* accel + mag on 0x1E / 0x1D */
} hal_imu_chip_t;

bool     hal_imu_init(void);                     /* finds the chip; +-2 g high resolution, 100 Hz (IS09 drv_lsm303) */
bool     hal_imu_accel_mg(int16_t *p_x, int16_t *p_y, int16_t *p_z);  /* false = I2C timeout (2 ms) */
bool     hal_imu_int1_src(uint8_t *p_src);       /* IS09 car: no INT1 wire, always 0 */
bool     hal_imu_int1_level(void);               /* IS09 car: always false */
bool     hal_imu_int1_used(void);                /* IS09 car: always false */
hal_imu_chip_t hal_imu_chip(void);
const char    *hal_imu_chip_name(void);          /* "LSM303DLHC at 0x19", "LSM303D at 0x1E", "none" */
void     hal_i2c_recover(void);                  /* 9 SCL clocks, then re-initialise I2C0 */

#endif /* HAL_H */
