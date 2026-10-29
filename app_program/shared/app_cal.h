/** @file app_cal.h
 *  @brief Calibration constants and arming switches.
 *
 *  EVERY number here that the design marks [TBC] is a placeholder with a
 *  deliberately conservative value.  A placeholder is NOT a measurement.
 *  Bench procedures that replace them are in docs/HARDWARE_VALIDATION.md
 *  (HV-n).  The ARMED switches must stay 0 until the named procedure has
 *  been performed and signed in docs/HARDWARE_VALIDATION.md.
 *
 *  Owner: integration owner (TBC, D-09); each buddy proposes their own values.
 */

#ifndef APP_CAL_H
#define APP_CAL_H

#include <stdint.h>

/* ---- Arming switches (0 = fail closed) --------------------------------- */

/** HV-3/HV-4 done: wheel geometry and encoder ticks measured, polarity
 *  verified on blocks. While 0, MotionControlTask rejects every drive
 *  command with APP_MOTION_RESULT_REJECTED and keeps the motors braked. */
#define APP_CAL_MOTION_ARMED 1

/** HV-5 done: ECHO measured <= 3.3 V after the divider/shifter, 5 V supply
 *  identified. While 0, ObstacleScanTask never drives TRIG. */
#define APP_CAL_ECHO_SHIFTER_VERIFIED 0

/** HV-7 done: servo pulse limits found at battery voltage. While 0,
 *  ObstacleScanTask configures PWM slice 7 but never routes GP15 to it, so
 *  the servo is not energised. */
#define APP_CAL_SERVO_ARMED 1

/** HV-6 done: button polarity confirmed. 1 = pressed reads LOW. */
#define APP_CAL_BUTTON_ACTIVE_LOW 1

/** D-01: 1 adopts stop cause REMOTE_STOP (needs design change approval). */
#define APP_ADOPT_REMOTE_STOP_CAUSE 0

/* ---- Timing (ms unless stated) ---------------------------------------- */
#define APP_CAL_T_HOLD_MS 1000U /* START held for recovery, S7->S0 [TBC] */
#define APP_CAL_T_START_DEBOUNCE_MS 40U /* [TBC] */
#define APP_CAL_T_STOP_DEBOUNCE_MS                                             \
    20U /* STOP acts on the first edge; debounce only re-arms [TBC] */
#define APP_CAL_T_CTRL_HEARTBEAT_MS                                            \
    100U /* Motion stops if no controller tick for this duration [TBC] */
#define APP_CAL_T_STALE_LINE_MS                                                \
    30U /* LineState older than this while FOLLOWING -> STALE_LINE (Version A) \
         */
#define APP_CAL_T_STALE_MOTION_MS                                              \
    30U /* MotionStatus older than this -> STALE_MOTION [TBC] */
#define APP_CAL_T_JUNC_MS                                                      \
    60U /* both sensors dark for T_JUNC [TBC]; 45 gave junctions on straights  \
         */
/* D-36 straddle bracket (sensors one line width apart, both on WHITE when
   the car is centred): both light means "straight ahead", not "lost". The
   line is lost only after both have been light for T_LOST (about 20 cm at
   V_NOM); until then FOLLOW drives straight with error 0. */
#define APP_CAL_T_LOST_MS                                                      \
    1500U /* both sensors light for T_LOST -> LOST [TBC] */
#define APP_CAL_T_QUIET_MS                                                     \
    150U /* barcode quiet zone (Rev B 150 ms proposed)                         \
          */
#define APP_CAL_T_GLITCH_US                                                    \
    300U /* barcode edge intervals shorter than this are merged [TBC] */
#define APP_CAL_T_REMOTE_START_GUARD_MS 3000U /* D-02 */
#define APP_CAL_T_SCAN_TIMEOUT_MS                                              \
    4000U /* profile scan must finish within [TBC] */
#define APP_CAL_T_LINE_SEARCH_MS 3000U /* bounded S5 search [TBC] */
#define APP_CAL_T_MOTION_TIMEOUT_MS                                            \
    8000U /* any MOVE/TURN must complete within [TBC]; FOLLOW is open-ended */
#define APP_CAL_T_ABORT_MS 300U     /* hump false trigger [TBC] */
#define APP_CAL_T_HUMP_MAX_MS 6000U /* episode time bound [TBC] */
#define APP_CAL_T_THETA_CONFIRM_MS                                             \
    50U /* pitch above theta_ON for 50 ms (Rev B) */
#define APP_CAL_T_US_TRIGGER_PERIOD_MS                                         \
    100U /* forward watch period (>= 60 ms, NFR5) */
#define APP_CAL_T_ECHO_TIMEOUT_MS 30U  /* NFR5 */
#define APP_CAL_T_SERVO_SETTLE_MS 150U /* per 30 deg, A11 [TBC] */
#define APP_CAL_T_BARCODE_PENDING_MS                                           \
    15000U /* pending command expires if no junction [TBC] */

/* ---- Speeds and geometry ----------------------------------------------- */
#define APP_CAL_V_NOM_CM_S 8U /* nominal line-following speed [TBC, HV-3] */
#define APP_CAL_V_REVERSE_CM_S 12U /* [TBC] */
#define APP_CAL_V_SEARCH_CM_S 10U  /* [TBC] */
#define APP_CAL_V_TURN_CM_S 10U    /* 29 Oct: 15 spun too fast [TBC] */
#define APP_CAL_WHEEL_DIA_MM 65U   /* 65 mm measured 27 Oct (HV-3) */
/* Polarity switches (HV-3/HV-4): 1 inverts that motor's drive direction or
   that encoder's count direction in software. Set so that FOLLOW/MOVE(+)
   turns both wheels in the car's forward direction AND both counts rise. */
#define APP_CAL_MOTOR_L_REVERSE 0   /* [TBC, HV-4] */
#define APP_CAL_MOTOR_R_REVERSE 0   /* [TBC, HV-4] */
#define APP_CAL_ENC_L_REVERSE 1     /* [TBC, HV-3] */
#define APP_CAL_ENC_R_REVERSE 0     /* [TBC, HV-3] */
#define APP_CAL_TRACK_WIDTH_MM 140U /* wheel-to-wheel [TBC, HV-3] */
#define APP_CAL_TICKS_PER_REV                                                  \
    2505U /* edges per wheel rev: 29 Oct, 3 x 200 mm MOVE [TBC, HV-3] */
#define APP_CAL_ENC_STALL_MS                                                   \
    300U /* no ticks while driving for this duration -> STALL [TBC] */
#define APP_CAL_ENC_STOP_MS                                                    \
    300U /* no edge for this duration -> measured speed 0 [TBC] */
#define APP_CAL_ENC_GLITCH_MAX                                                 \
    50U /* invalid quadrature transitions per run -> ENCODER_FAULT [TBC] */

/* ---- Motion control (Buddy 2). All gains Q8 fixed point (256 = 1.0). --- */
#define APP_CAL_PWM_MAX_PCT                                                    \
    50U /* |PWM| ceiling until HV-3/HV-4 are signed [TBC]; 80 was too fast */
#define APP_CAL_PWM_MIN_PCT                                                    \
    18U /* below this the motor does not turn [TBC, HV-4] */
#define APP_CAL_PWM_STALL_PCT                                                  \
    30U /* stall watch only above this demand [TBC]                            \
         */
#define APP_CAL_FF_Q8_PER_CM_S                                                 \
    640U /* feed-forward: PWM% x256 per cm/s (2.5 %/cm/s) [TBC] */
#define APP_CAL_SPEED_KP_Q8 512U          /* 2.0 %/cm/s [TBC] */
#define APP_CAL_SPEED_KI_Q8 64U           /* 0.25 %/(cm/s x cycle) [TBC] */
#define APP_CAL_SPEED_I_MAX_Q8 (30 * 256) /* integral clamp = 30 % PWM */
#define APP_CAL_STRAIGHT_KP_Q8                                                 \
    6U /* MOVE L-R correction per tick. 768 was 3 %/tick for 20 ticks/rev      \
          (0.3 %/mm); 6 keeps 0.3 %/mm at 2505 ticks/rev [TBC] */
/* Wheel speed from ticks counted over a window instead of one edge interval
   (29 Oct, see motion_core.h): 0 = edge interval only. */
#define APP_CAL_SPEED_WINDOW_CYCLES 5U    /* x 10 ms [TBC] */
#define APP_CAL_SPEED_WINDOW_MIN_TICKS 8U /* fewer -> edge interval [TBC] */
/* FOLLOW steering (29 Oct): steer moves the wheel speed SETPOINTS apart by
   this % of the set speed at steer +-100 (100 = outer 2x, inner stopped). */
#define APP_CAL_STEER_SPEED_PCT 150U /* [TBC, track] */
#define APP_CAL_STEER_GAIN_Q8                                                  \
    0U /* extra PWM per steer unit on top (was 128, then 48); the speed loops  \
          cancelled it, see D-38 */
#define APP_CAL_TURN_UNTIL_LINE_MIN_PCT                                        \
    50U /* until_line may finish after this % of the angle */
#define APP_CAL_TURN_UNTIL_LINE_MAX_PCT                                        \
    150U /* ... and aborts at this % if no line */

/* ---- Line sensing ------------------------------------------------------ */
#define APP_CAL_IR_MIN_CONTRAST 400U /* min (black - white) ADC span [TBC] */
#define APP_CAL_IR_DARK_IS_HIGH                                                \
    1 /* 1: black line reads a HIGHER ADC value [TBC, HV-10] */
/* Event thresholds (D-35): a sensor is DARK at or above DARK_PCT and LIGHT
   at or below LIGHT_PCT; in between it is "on an edge", which counts as on
   the line and is neither a junction nor a loss. With the two sensors one
   line width apart (18 mm, A7) a centred car reads both near 50. */
#define APP_CAL_LINE_DARK_PCT 45U  /* normalised >= this = dark [TBC] */
#define APP_CAL_LINE_LIGHT_PCT 12U /* normalised <= this = light [TBC] */
/* Tracking setpoint subtracted from (n_centre - n_left) [TBC, HV-10, A7]:
   0  = the line runs midway between the two sensors, both partly on it
        (needs a sensor pitch of about the 18 mm line width or less);
   50 = left-edge following: left sensor half on the line, centre sensor
        fully on it (wider pitch). The car is "centred" at error 0. */
#define APP_CAL_LINE_ERROR_SETPOINT 0
#define APP_CAL_LINE_KP_Q8 256U /* steer = Kp*error + Kd*d(error) [TBC] */
#define APP_CAL_LINE_KD_Q8                                                     \
    0U /* 64 kicked the steering each time a sensor hit the line (29 Oct) */
/* D-37 branch hold (line_core.h): an abrupt single DARK is held straight for
   up to HOLD_MS so the car goes straight through T / cross junctions. */
#define APP_CAL_LINE_BRANCH_HOLD_MS 0U /* 0 = off [TBC, track] */
#define APP_CAL_LINE_BRANCH_RISE_CYCLES                                        \
    3U /* LIGHT within this many 10 ms cycles before DARK = abrupt [TBC] */
#define APP_CAL_LINE_CAL_DECAY                                                 \
    1U /* running min/max relax by this many LSB per decay step */
#define APP_CAL_LINE_CAL_DECAY_CYCLES                                          \
    50U /* one decay step every this many 10 ms cycles (D-34: 2 LSB/s) [TBC]   \
         */
#define APP_CAL_LINE_INIT_WHITE                                                \
    1000U /* default references until calibrated [TBC] */
#define APP_CAL_LINE_INIT_BLACK 3000U

/* ---- Barcode (Code 39, measured 3:1, see ledger S6) --------------------- */
#define APP_CAL_BC_ELEMENTS 29U /* *, one character, * with two gaps */
#define APP_CAL_BC_N_MIN 20U    /* fewer at quiet = stray mark, silent */
#define APP_CAL_BC_RATIO_MIN_Q8                                                \
    384U /* wide/narrow must be >= 1.5 to classify */
#define APP_CAL_BC_RATIO_MAX_Q8 1536U /* ... and <= 6.0 */

/* ---- Obstacle ---------------------------------------------------------- */
#define APP_CAL_D_STOP_CM                                                      \
    25U /* two confirmations inside this -> EVT_OBSTACLE [TBC] */
#define APP_CAL_W_CAR_CM 16U       /* [TBC] */
#define APP_CAL_M_CLEAR_CM 8U      /* [TBC] */
#define APP_CAL_R_SWEPT_CM 12U     /* [TBC] */
#define APP_CAL_D_REVERSE_MM 150U  /* [TBC] */
#define APP_CAL_D_PASS_MAX_MM 600U /* [TBC] */
/* ---- Console diagnostics (demo image only) ----------------------------- */
#define APP_CAL_CONSOLE_CAR_MS                                                 \
    100U /* "car:" line on the USB console every this many ms; 0 = off. Works  \
            without Wi-Fi so the track can be debugged from the serial port */

/* ---- Mission controller (Phase 7, VehicleControllerTask) --------------- */
#define APP_CAL_D_JUNC_CLEAR_MM                                                \
    80U /* STRAIGHT or no command: MOVE this far across a junction [TBC] */
#define APP_CAL_D_JUNC_ADVANCE_MM                                              \
    30U /* sensor-to-axle offset driven before a junction pivot; 0 = none      \
           [TBC] */
#define APP_CAL_T_SPEED_ZERO_MS                                                \
    1000U /* BRAKE must report speed 0 within this [TBC] */
#define APP_CAL_T_SCAN_WAIT_MARGIN_MS                                          \
    500U /* added to T_SCAN_TIMEOUT before the controller gives up */
#define APP_CAL_N_RETRY_MAX 2U /* reverse-and-rescan attempts, 0..3 (Rev B) */
#define APP_CAL_D_REVERSE_MARGIN_MM                                            \
    50U /* corridor behind the car must be known clear for D_REVERSE + this */
#define APP_CAL_SIDE_FREE_COUNT                                                \
    3U /* consecutive side readings above R_SIDE_FREE = obstacle passed */
#define APP_CAL_D_PASS_MIN_MM                                                  \
    100U /* action 4: never declare the obstacle passed before this */
#define APP_CAL_D_PASS_EXTRA_MM                                                \
    200U /* action 4: car length plus margin driven after the side is free */
#define APP_CAL_D_RETURN_MARGIN_MM                                             \
    150U /* action 5: MOVE until_line bound = offset + this [TBC] */
#define APP_CAL_T_BYPASS_MAX_MS 30000U /* whole S4 bound [TBC] */
#define APP_CAL_SEARCH_ANGLE_DDEG                                              \
    600 /* S5 first sweep, 0.1 deg units (60 deg); second sweep is twice */
#define APP_CAL_D_SEARCH_CREEP_MM 150U /* S5 forward creep [TBC] */
#define APP_CAL_SERVO_MIN_US 1000U     /* 0 deg pulse [TBC, HV-7] */
#define APP_CAL_SERVO_MAX_US 2000U     /* 180 deg pulse [TBC, HV-7] */
#define APP_CAL_SERVO_LEFT_IS_MAX_PULSE                                        \
    1 /* 1: longer pulse turns the sensor LEFT [TBC, HV-7] */
#define APP_CAL_T_SERVO_MIN_SETTLE_MS 20U /* floor for a 5 deg step */
#define APP_CAL_R_PROFILE_CM                                                   \
    45U /* samples at or inside this belong to the obstacle [TBC] */
#define APP_CAL_R_SIDE_FREE_CM                                                 \
    60U /* samples beside the obstacle must exceed this for a valid side [TBC] \
         */
#define APP_CAL_Y_PATH_MAX_CM                                                  \
    40U /* largest lateral bypass offset the course allows [TBC] */
#define APP_CAL_US_BEAM_HALF_SIN_PERMILLE                                      \
    122U /* sin(7 deg): HC-SR04 half beam [TBC, A10] */
#define APP_CAL_D_SIDE_UNSAFE_CM                                               \
    10U /* side range below this twice -> EVT_SIDE_UNSAFE [TBC] */
#define APP_CAL_D_STOP_HYST_CM                                                 \
    5U /* forward watch re-arms above D_STOP + this */
#define APP_CAL_NO_ECHO_COUNT                                                  \
    5U /* consecutive echo timeouts -> NO_ECHO fault */

/* ---- IMU --------------------------------------------------------------- */
#define APP_CAL_THETA_ON_HALF_DEG 10 /* 5.0 deg [TBC, A8] */
#define APP_CAL_THETA_OFF_HALF_DEG 4 /* 2.0 deg [TBC, A8] */
#define APP_CAL_D_HUMP_MAX_MM 800U   /* [TBC] */
#define APP_CAL_K_CAL_PERMILLE                                                 \
    1000U                          /* height scale factor x/1000 [TBC, A8]     \
                                    */
#define APP_CAL_COLLISION_MG 2500U /* |a_x| spike threshold [TBC, A9] */
#define APP_CAL_COLLISION_ENABLE                                               \
    1 /* 0 while measuring hump entry shocks (A9, HV-13) */
#define APP_CAL_COLLISION_SAMPLES                                              \
    2U /* consecutive spike samples (20 ms) [TBC, A9] */
#define APP_CAL_IMPACT_HOLD_MS                                                 \
    500U                          /* motion class IMPACT hold after a spike    \
                                   */
#define APP_CAL_CLASS_HOLD_MS 30U /* other class changes need 3 cycles */
#define APP_CAL_ACCEL_CM_S 3U     /* |v - v_lp| >= this -> ACCELERATING [TBC] */
#define APP_CAL_HUMP_MAX_ACCEPT_DEGRADED                                       \
    0 /* 1: DEGRADED peaks may update hump_max (D-17) */
#define APP_CAL_IMU_LP_SHIFT                                                   \
    2U /* accel EMA alpha 1/4 at 100 Hz (~4 Hz corner) [TBC] */
#define APP_CAL_IMU_AXIS_FWD                                                   \
    0U /* sensor axis along the car: 0 X, 1 Y, 2 Z [TBC, HV-12] */
#define APP_CAL_IMU_AXIS_UP                                                    \
    2U /* sensor axis away from the floor [TBC, HV-12] */
#define APP_CAL_IMU_FWD_SIGN                                                   \
    1 /* +1 or -1: nose-up pitch must read positive [TBC, HV-12] */
#define APP_CAL_IMU_UP_SIGN                                                    \
    1 /* +1 or -1: level car reads about +1000 mg [TBC, HV-12] */
#define APP_CAL_IMU_I2C_HZ                                                     \
    400000U /* LSM303DLHC fast mode; 100000 if the Grove lead is noisy */
#define APP_CAL_IMU_FAULT_ERRORS 3U /* consecutive I2C errors -> IMU_FAULT */
#define APP_CAL_IMU_RETRY_MS                                                   \
    500U /* sensor re-initialisation period while faulted */

#endif /* APP_CAL_H */
