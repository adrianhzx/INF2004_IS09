/**
 * @file    pins.h
 * @brief   Pin map of the IS09 car (Rev B section 2, app_hw.h in the IS09
 *          repo), used by this software.
 *
 * Plain numbers only, so both the Pico SDK side (hal_*.c) and the
 * micro T-Kernel side (tsk_*.c) can include it.
 *
 * IS09 vs base wiring (19 Nov port):
 *   - motors: M1 (GP8/9) is the LEFT wheel, M2 (GP10/11) the RIGHT wheel
 *     (base had them the other way round);
 *   - encoders: left GP0/1, right GP2/3 (base: right GP0/1, left GP16/17);
 *   - IMU on I2C0 GP4/GP5, no INT1 wire (base: I2C1 GP2/3, INT1 GP6);
 *   - line sensors are ANALOG on ADC0/ADC1 (base: digital DO on GP4/5);
 *   - barcode sensor is DIGITAL DO on GP7, timed by edge interrupts
 *     (base: analog AO on GP26);
 *   - HC-SR04 TRIG GP16, ECHO GP17; servo GP15 (base: GP7, GP27, GP12).
 */
#ifndef PINS_H
#define PINS_H

/* ---- Motors: Robo Pico on-board driver (A/B per motor) ---- */
#define PIN_ML_A             (8u)     /* M1A = LEFT wheel, PWM slice 4A */
#define PIN_ML_B             (9u)     /* M1B, PWM slice 4B */
#define PIN_MR_A             (10u)    /* M2A = RIGHT wheel, PWM slice 5A */
#define PIN_MR_B             (11u)    /* M2B, PWM slice 5B */
#define MOTOR_PWM_FREQ_HZ    (20000u)

/* ---- Wheel encoders (quadrature, 4x decoded) ---- */
#define PIN_ENC_L_A          (0u)
#define PIN_ENC_L_B          (1u)
#define PIN_ENC_R_A          (2u)
#define PIN_ENC_R_B          (3u)

/* ---- IMU: GY-511 / LSM303DLHC on I2C0 (no INT1 wire on the IS09 car) ---- */
#define PIN_I2C_SDA          (4u)
#define PIN_I2C_SCL          (5u)
#define IMU_I2C_HZ           (400000u)  /* APP_CAL_IMU_I2C_HZ */
#define IMU_ACCEL_ADDR       (0x19u)
#define IMU_MAG_ADDR         (0x1Eu)

/* ---- Barcode sensor ----
 * AO (19 Nov): GP28 = ADC2, read by the base analog reader (BARCODE_AO 1).
 * GP28 is also the Robo Pico's VBAT sense pin: the VBAT jumper must NOT be
 * soldered, and VBAT_ENABLED must stay 0. AO must stay <= 3.3 V.
 * DO: GP7, edges timed for the IS09 reader (BARCODE_AO 0). */
#define PIN_BARCODE_AO       (28u)
#define ADC_CH_BARCODE       (2u)
#define PIN_BARCODE_DO       (7u)

/* ---- Line sensors: analog, LEFT and CENTRE straddle the line ----
 * The IS09 "CENTRE" sensor sits to the right of the LEFT one, so it plays
 * the part of the base RIGHT line sensor. */
#define PIN_LINE_LEFT        (26u)    /* ADC0 */
#define PIN_LINE_CENTRE      (27u)    /* ADC1 */
#define ADC_CH_LINE_LEFT     (0u)
#define ADC_CH_LINE_CENTRE   (1u)

/* ---- HC-SR04 ultrasonic (ECHO MUST be <= 3.3 V at the pin: IS09 HV-5) ---- */
#define PIN_US_TRIG          (16u)
#define PIN_US_ECHO          (17u)

/* ---- SG90 pan servo ---- */
#define PIN_SERVO            (15u)    /* PWM slice 7B, 50 Hz */

/* ---- Robo Pico on-board parts ---- */
#define PIN_NEOPIXEL         (18u)
#define PIN_BTN1             (20u)    /* start / end run / acknowledge */
#define PIN_BTN2             (21u)    /* local stop (latched like an E-stop) */
#define PIN_BUZZER           (22u)    /* PWM slice 3A */
#define PIN_VBAT_SENSE       (28u)    /* only if the VBAT jumper is soldered: NOT on the IS09 car (barcode AO) */
#define ADC_CH_VBAT          (2u)     /* GP28 = ADC2, reads VBAT / 2 */

/* GP6, GP12-14 and GP19 are spare on the IS09 car. The template's liveness
   LED pin is GP19 (BOARD_LED_PIN in robot.mk). */

#endif /* PINS_H */
