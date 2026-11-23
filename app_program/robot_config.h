/**
 * @file    robot_config.h
 * @brief   Calibration values and design constants.
 *
 * Calibration values come from the part tests and full tests. Copy them from
 * full_tests/include/robot_config.h once you have filled that in: the names
 * are the same. Every value still CAL_UNSET is listed on the USB console at
 * start-up and the robot stays in ERROR until it is set.
 *
 * Values marked [P] are also run-time parameters: they can be changed over
 * MQTT without recompiling ("SET:NAME=VALUE", see README_ROBOT.md, NFR14/15).
 * The value here is the start-up default.
 *
 * 21 Oct: values marked PLACEHOLDER are reasonable guesses so the robot can
 * run before calibration. Replace them with the measured values (or send the
 * measured values with laptop_tools/robot_params.py, which overrides these).
 *
 * 19 Nov, IS09 port: values marked [IS09] are taken from the IS09 repo's
 * app_program/shared/app_cal.h (the IS09 car's own measurements and
 * placeholders), so this software starts from what the IS09 car already
 * uses. The IS09 modules (is09/) keep reading app_cal.h directly.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#define CAL_UNSET                 (-1)

/* ---- Motors and encoders [IS09] ---- */
#define MOTOR_MAPPING_CONFIRMED   (1)           /* [IS09] HV-4: M1 = left, M2 = right, A high = forward */
#define MOTOR_L_REVERSE           (0)           /* [IS09] APP_CAL_MOTOR_L_REVERSE: 1 inverts the left motor */
#define MOTOR_R_REVERSE           (0)           /* [IS09] APP_CAL_MOTOR_R_REVERSE */
#define MOTOR_MAX_DUTY            (0.50f)       /* [IS09] APP_CAL_PWM_MAX_PCT 50: ceiling until HV-3/HV-4 are signed */
#define ENC_R_REVERSED            (0)           /* [IS09] APP_CAL_ENC_R_REVERSE */
#define ENC_L_REVERSED            (1)           /* [IS09] APP_CAL_ENC_L_REVERSE */
#define ENC_DIRECTION_CONFIRMED   (1)           /* [IS09] HV-3 */
#define ENC_A_EDGES_PER_REV       (1252.5f)     /* [IS09] APP_CAL_TICKS_PER_REV 2505 counts all A and B edges;
                                                   this value counts A edges only (x 2 = 2505) */
#define WHEEL_DIAMETER_MM         (65)          /* [IS09] APP_CAL_WHEEL_DIA_MM, measured 27 Oct */
#define TRACK_WIDTH_MM            (140)         /* [IS09] APP_CAL_TRACK_WIDTH_MM [TBC] */

/* ---- From full_tests: ft_speed_pid P1 ---- */
#define SPEED_FF_DEADBAND_PCT     (0)      /* [IS09] APP_CAL_FF_Q8_PER_CM_S 640 = 2.5 % per cm/s with no offset;
                                              the PI loop adds the rest (IS09 floor APP_CAL_PWM_MIN_PCT 18).
                                              Tune with ft_speed_pid P1 on the IS09 car */
#define SPEED_FF_MM_S_PER_PCT_X10 (40)     /* [IS09] 2.5 % per cm/s = 4.0 mm/s per % */
#define SPEED_KP                  (0.0020f)     /* [P] duty per mm/s of error */
#define SPEED_KI                  (0.0100f)     /* [P] duty per mm of error   */

/* ---- Line sensors [IS09]: analog LEFT / CENTRE, IS09 line_core ----
 * hal_ir_left_level() / hal_ir_right_level() return is09_line.c's decision,
 * true = black, so LINE_DO_BLACK_HIGH is 1 (line_core learns its own
 * white / black references). */
#define LINE_DO_BLACK_HIGH        (1)

/* ---- Barcode sensor AO on GP28 (19 Nov) ----
 * The base reader switches between these two levels (and reads at several
 * levels in between, BARCODE_LEVELS). Measure them: in IDLE, hold the
 * barcode sensor over white paper, then over a black bar, and read "ir" on
 * the dashboard (barcode card). SET:IR_WHITE_ADC= / SET:IR_BLACK_ADC= apply
 * at the next START. */
#define IR_WHITE_ADC              (200)         /* PLACEHOLDER: the original sensor read 160-240 on white */
#define IR_BLACK_ADC              (3900)        /* PLACEHOLDER: the original sensor read 3800-4095 on black */
#define LINE_DARK_PCT             (45)          /* [P] [IS09] APP_CAL_LINE_DARK_PCT: darkness 0..100 at or above
                                                   which a sensor sees the line */
#define LINE_HYS_PCT              (10)          /* [P] ... and it sees white again below LINE_DARK_PCT - this */
#define LINE_L_DARK_PCT           (15)          /* [P] (20 Nov) black level for the weak LEFT sensor only (CENTRE keeps
                                                   LINE_DARK_PCT). On the line LEFT peaked at 33-53 % (morning) and
                                                   only 24-40 % (16:47 run: raw 770-1530 against a black reference
                                                   of ~4000 set by lifting the car); on white it reads 0-2 %.
                                                   25 still missed the last left arc (peak 24). 0 = LINE_DARK_PCT */

/* ---- From part_tests: test_servo ---- */
#define SERVO_MIN_US              (1000)        /* [IS09] APP_CAL_SERVO_MIN_US: 0 deg = sensor 90 deg RIGHT */
#define SERVO_MAX_US              (2000)        /* [IS09] APP_CAL_SERVO_MAX_US: 180 deg = sensor 90 deg LEFT */
#define SERVO_CALIBRATED          (1)           /* [IS09] APP_CAL_SERVO_ARMED (HV-7) */
#define SERVO_LOW_ANGLE_IS_RIGHT  (1)           /* [IS09] APP_CAL_SERVO_LEFT_IS_MAX_PULSE 1 */
#define SERVO_TRIM_DEG            (0.0f)        /* [P] added to every pan angle, + = turn the sensor RIGHT.
                                                   IS09 has no trim. Tune with a flat board square ahead:
                                                   shortest reading at 0 */

/* ---- From part_tests: test_imu U5 ---- */
#define IMU_FORWARD_AXIS          (2)           /* [IS09] APP_CAL_IMU_AXIS_FWD 1 = Y (19 Nov lift test). Information
                                                   only: IS09 imu_core maps the axes with app_cal.h (AXIS_FWD /
                                                   AXIS_UP / signs) */
#define IMU_FORWARD_SIGN          (-1)          /* [IS09] APP_CAL_IMU_FWD_SIGN (information only, see above) */

/* ---- Geometry (ruler on the assembled car) ---- */
#define IR_TO_AXLE_MM             (30)          /* [P] [IS09] APP_CAL_D_JUNC_ADVANCE_MM: line sensors ahead of the axle [TBC] */
#define ROBOT_HALF_WIDTH_MM       (80)          /* [IS09] APP_CAL_W_CAR_CM 16 / 2 */

/* ---- Line following (DR 6.1) ---- */
#define LINE_BASE_MM_S            (80)          /* [P] forward speed while following (was 120: too fast for the
                                                   ~115-145 mm loops with uncalibrated wheel speed control) */
#define LINE_STEER                (0.40f)       /* [P] steering when one sensor sees black (was 0.30) */
#define LINE_STEER_RAMP           (3.00f)       /* [P] extra steering per second it stays black (was 1.5) */
#define LINE_HOLD_STEER           (0.30f)       /* [P] steering kept after the sensor clears (was 0.12) */
#define LINE_HOLD_MM              (60)          /* [P] ... for this distance (was 40) */
#define LINE_LIGHT_PCT            (12)          /* [IS09] APP_CAL_LINE_LIGHT_PCT: darkness at or below this = white
                                                   (line_meas.c: a "mark" is anything darker) */
#define LINE_JUNC_OTHER_PCT       (15)          /* [P] one sensor black and the other at least this dark (%) = a
                                                   junction (22:54, 19 Nov): the straddled line never darkens both,
                                                   a junction bar or branch does. 0 = off (both black only) */
#define LINE_JUNC_OTHER_MM        (40)          /* [P] ... if the other sensor's dark patch began within this many mm
                                                   (longer = it is on a barcode or the line's edge) */
#define BARCODE_STEER_MAX         (0.25f)       /* [P] while passing a barcode, steering is limited to this
                                                   (0 = no limit; full steering is 1.0) (19 Nov) */
#define LINE_JUNC_CLEAR_MM        (80)          /* [P] [IS09] APP_CAL_D_JUNC_CLEAR_MM: after both sensors see black
                                                   (junction) with no command, straight on this far whatever the
                                                   sensors see (19 Nov: the far side of the bar steered the car off) */
#define PICKUP_ADC                (3000)        /* [P] picked up (20 Nov): LEFT, CENTRE and barcode sensors all at or
                                                   above this raw value (in the air: 3300-4095; on the track the three
                                                   never reached 2000 together) ends the run. 0 = off (bench tests
                                                   with the wheels in the air) */
#define PICKUP_MM                 (30)          /* [P] ... for this much wheel travel (an 18 mm bar is shorter) */
#define PICKUP_MS                 (500)         /* [P] ... or this long, if the wheels are not turning */
#define LINE_CURVE_MM             (25)          /* black for longer than this = a curve */
#define LINE_CURVE_SLOW           (0.60f)       /* [P] base speed x (1 - this x |steer|): slow down in tight curves (0 = off; was 0.4) */
#define LINE_MIN_WHEEL_MM_S       (20)          /* [P] in a curve the inner wheel is braked (0) instead of being asked
                                                   for less than this: a slow target still gets the start-up duty and
                                                   creeps too fast, so the car runs wide (0 = off) */
#define LINE_BIAS_KI              (0.0f)        /* [P] curve memory, OFF by default: try 1.5-3 only after the speed
                                                   PI is calibrated (with a sluggish wheel loop it makes the car weave) */
#define LINE_BIAS_DECAY_MM        (400)         /* [P] curve memory fades over this much white */
#define LINE_LOST_MM              (250)         /* [P] no black this far after a curve = line lost */
#define LINE_SEARCH_DEG           (45)          /* first sweep of the search, doubled each pass */
#define LINE_SEARCH_MAX_DEG       (90)          /* sweeps 45, 90 deg each way, then EVF_LINE_LOST */
#define LINE_SEARCH_MM_S          (80)          /* wheel speed while searching */

/* ---- Barcodes (DR 6.1, FR5-FR8) ----
 * BARCODE_AO 1 (19 Nov): the base analog reader on the sensor's AO (GP28):
 * 1 kHz samples, widths in wheel distance, read at BARCODE_LEVELS levels.
 * BARCODE_AO 0: the IS09 reader on DO (GP7): edge times into IS09
 * barcode_core (settings in is09/shared/app_cal.h). Rebuild to switch. */
#define BARCODE_AO                (1)
#define BARCODE_AT_JUNCTION       (1)           /* [P] 1 = store, act at the next cross-junction (demo brief), 0 = act at once */
#define BARCODE_TURN_DEG          (90)
#define BARCODE_MAX_LEN_MM        (400)         /* abandon a window longer than this */
#define BARCODE_MIN_ELEMENTS      (9)           /* fewer = a stray mark (e.g. a crossing line) */
#define BARCODE_HYS               (0.05f)       /* [P] switch levels = mid +/- this x (black - white).
                                                   Was 0.10: small prints (75 %, 50 %) blur the narrow
                                                   bars so they no longer reach mid + 10 %. Raise it only
                                                   if a plain white/black area gives false edges. */
#define BARCODE_DETECT            (1)           /* [P] 1 = bars found on 0.25 mm distance bins with edges at the
                                                   mid level (bc_detect.c: copes with a blurred sensor spot),
                                                   0 = the original 1 kHz two-level interrupt (BARCODE_HYS) */
#define BARCODE_LEVELS            (6)           /* [P] BARCODE_DETECT = 1 only: read every barcode this many ways at once
                                                   (1 = mid level only; 3 = 50/35/65 %; 5 = also 25/75 %; 6 = also the
                                                   local-contrast detector for pale / streaky bars next to black ones),
                                                   so faded / grey prints still decode (bc_multi.c). Applies at START */
#define BARCODE_ADAPT_SWING       (0.15f)       /* [P] BARCODE_LEVELS = 6: the local-contrast read makes an edge only where
                                                   the signal within +-4 mm changes by more than this x (black - white).
                                                   Lower = paler bars, but more false edges from ADC noise. Applies at START */
#define BARCODE_SENSOR_Y_MM       (35)          /* [P] PLACEHOLDER (original car): barcode sensor's distance to the LEFT of
                                                   the car's centre line, mm (negative = right; 0 = use the wheels' mean
                                                   distance). Bar widths follow the sensor's own path, so steering over a
                                                   barcode does not stretch bars. MEASURE IT ON THE IS09 CAR */
#define BARCODE_DET_HYS           (0.02f)       /* [P] BARCODE_DETECT = 1 only: band = mid +/- this x (black - white).
                                                   The averaging allows a narrow band; raise it (0.03-0.05) only if
                                                   plain white paper gives BC_DBG "stray mark" windows */
#define BARCODE_DEBUG             (1)           /* [P] 1 = publish every barcode window's bar/space widths (BC_DBG events) */
#define BARCODE_DEFAULT_MAP       "W=LEFT,X=RIGHT,Y=FWD,Z=UTURN,F=UTURN"   /* 20 Nov: F added (U-turn) */
                                                /* [IS09] barcode_core table (A = LEFT, B = RIGHT, C = STRAIGHT,
                                                   D = U-TURN), loaded at start-up so the car works without
                                                   MQTT. A MAP: command or the retained is09/map replaces it */

/* ---- Motion primitives ---- */
#define TURN_MM_S                 (90)          /* [P] wheel speed for pivot turns */
#define TURN_SNAP                 (1)           /* [P] 1 = finish junction turns on the line sensors (19 Nov: on;
                                                   the 90 deg encoder turn went too far on the IS09 car) */
#define TURN_SNAP_EARLY_DEG       (45.0f)       /* [P] the encoder part stops this short of 90, then the car pivots
                                                   slowly until the new line is between the sensors (was 20) */
#define TURN_SNAP_MAX_DEG         (90.0f)       /* [P] ... for at most this much more (was 45) */
#define TURN_SNAP_PCT             (25)          /* [P] darkness (%) that counts as "the sensor is on the line" here */
#define BYPASS_MM_S               (60)          /* [P] speed for bypass legs (28 Oct: was 100; the leg watch sweeps the
                                                   sensor, one 0 / 30 / 0 / 60 deg cycle takes ~0.65 s = 4 cm at 60 mm/s) */
#define SPEED_MAX_MM_S            (250)         /* NFR18 speed cap */
#define HUMP_SPEED_PCT            (60)          /* DR A2: speed limit on a hump */
#define ENC_LOSS_DUTY             (0.30f)       /* DR 5: duty above this ... */
#define ENC_LOSS_MS               (200)         /* ... with no ticks for this long = encoder loss */

/* ---- Obstacles (DR 6.2): IS09 scan module ----
 * The forward watch (two readings in a row at or inside APP_CAL_D_STOP_CM,
 * 25 cm), the coarse and fine scans and the profile (edges, width, side)
 * come from IS09 scan_core (is09/subsystem5); the bypass below is from the base code.
 * IS09 gate: while APP_CAL_ECHO_SHIFTER_VERIFIED is 0 in is09/shared/
 * app_cal.h (HV-5: ECHO measured <= 3.3 V), TRIG is never pulsed and the
 * car never sees an obstacle. */
#define OBSTACLE_STOP_CM          (25)          /* [P] information only: the trigger is IS09 APP_CAL_D_STOP_CM */
#define OBST_MAX_TURN_DPS         (15.0f)       /* [P] turning faster than this: the sensor looks off the track,
                                                   so readings above PROXIMITY_STOP_CM do not trigger (0 = off).
                                                   Was 60: at 80 mm/s the loops turn at only 20-30 deg/s */
/* "all clear" is now IS09's: nothing at or inside APP_CAL_R_PROFILE_CM (45 cm) */
#define PROXIMITY_STOP_CM         (8)           /* NFR21 */
#define OBST_MARGIN_MM            (60)          /* [P] extra side clearance in the bypass */
#define OBST_MEASURE              (1)           /* [P] (20 Nov) measure the box while driving: the diagonal leg goes
                                                   out until the sensor (pointing along the path) passes the box edge,
                                                   then far enough for the car to clear it; the side pass measures the
                                                   side gap and the depth. 0 = the fixed plan from the scan */
#define OBST_MEAS_FACE_MM         (350)         /* [P] before the diagonal, reverse until the box front is about this far
                                                   from the axle. 350: after the 25 cm stop it hardly reverses; closer
                                                   means less beam lag at the edge (19:44 run, 500: edge read ~7 cm out).
                                                   A wide box may block the diagonal: the bypass then backs out and
                                                   retries (raise this, e.g. 500, for wide boxes) */
#define OBST_WIDTH_CM             (0)           /* [P] (20 Nov) known obstacle width: the scan's width is replaced
                                                   by this, centred where the scan put it (the HC-SR04 beam is far
                                                   wider than the 15 cm demo box at 20-30 cm: it measured 47-66 cm).
                                                   0 = use the scan width */
#define OBST_DEPTH_MM             (150)         /* [P] assumed depth (ultrasonic cannot see the back) */
#define OBST_LEG_STOP_CM          (15)          /* [P] during bypass legs and the return the sensor sweeps 0 / 30 / 0 / 60 deg
                                                   towards the obstacle side: anything in the car's path (car width
                                                   + OBST_PATH_SIDE_MM) closer than this ahead, seen twice at the same
                                                   angle = this side is blocked: stop, back out along the legs, then
                                                   the other side or a rescan. Keep it above PROXIMITY_STOP_CM (8,
                                                   the E-stop); 0 = off */
#define OBST_PATH_SIDE_MM         (20)          /* leg watch: corridor = car half width + this on each side */
#define OBST_BACKOFF_MAX_MM       (250)         /* longest reverse to get the 45 deg standoff (see obstacle_standoff_mm) */
#define OBST_RESCAN_REVERSE_MM    (100)         /* reverse before a rescan */
#define OBST_RETURN_PAST_MM       (80)          /* return: drive this far (sideways) past the original path before
                                                   deciding the line was missed */
#define OBST_US_AHEAD_MM          (80)          /* PLACEHOLDER: ultrasonic sensor ahead of the wheel axle (measure) */
#define US_IDLE_PING              (1)           /* [P] bench mode (19 Nov): 1 = ping every 0.2 s in IDLE so the
                                                   dashboard shows the distance; no stop or E-stop in IDLE */
#define OBST_SIDE_MAX_MM          (1000)        /* parallel leg follows the obstacle's side at most this far */
#define OBST_SIDE_STEP_MM         (100)         /* look-before-turning: drive on this far while the side is still there */
#define OBST_TURN_LOOK_STEPS      (8)           /* ... at most this many times */
#define OBST_LOOK_PIVOT_DEG       (30)          /* look-before-turning: pivot this far towards the obstacle; with the
                                                   servo at 60 the sensor looks square at its side (90 deg) */
#define OBST_TIE_PIVOT_DEG        (30)          /* tie: pivot this far + servo 60 deg = a 90 deg look each way */
#define OBST_RETRY_WIDER_MM       (100)         /* a side tried again after it was blocked: this much more side margin */
#define REACQ_MAX_LATERAL_MM      (300)         /* DR: abort if the line is not found */
#define SERVO_SETTLE_MS           (50)          /* DR A6 (the IS09 scan adds scan_core_settle_ms per move) */

/* ---- Terrain / IMU (DR 6.3): IS09 imu_core ----
 * Pitch, hump episodes (start, end, peak height, quality), collision spikes
 * and the motion class come from IS09 imu_core (is09/subsystem4) every
 * 10 ms. Its thresholds and the sensor axes are in is09/shared/app_cal.h
 * (APP_CAL_THETA_ON/OFF_HALF_DEG, APP_CAL_COLLISION_MG, APP_CAL_IMU_AXIS_*,
 * ...), as in the IS09 repo. */
#define TURNING_DPS               (30.0f)       /* encoder turn rate above this = "turning" for imu_core */
#define HUMP_ON_DEG               (3.0f)        /* [P] hump starts: detector pitch above this for 50 ms. IS09 has
                                                   5.0; a 1 cm ramp under a ~10 cm wheelbase tilts the body only
                                                   ~5-6 deg, which the slow filter does not keep above 5.0 */
#define HUMP_OFF_DEG              (1.5f)        /* [P] level again below this (IS09: 2.0) */
#define IMU_ACCEL_COMP            (1)           /* [P] 1 = take the encoder acceleration off the forward axis, so a
                                                   start / stop does not look like a slope (base DR A8) */

/* Hump height, second detector (19 Nov): the base hump profiler
 * (hump2.c) runs beside IS09 imu_core and only measures. It reports two
 * heights per hump: the base sum of sin(pitch) x distance, and the tilt
 * height L x sin(pitch). Base values in [ ]; the defaults are lowered for
 * the 1 cm hump (the base code would reject it). */
#define HUMP2_START_DEG           (2.0f)        /* [P] climb starts: pitch at or above this for 3 samples (60 ms) [5.0].
                                                   20:25 run: the real climb was only 2-4.4 deg */
#define HUMP2_LEVEL_DEG           (1.0f)        /* [P] crest / level again below this; the end needs 5 samples [3.0] */
#define HUMP2_MIN_MM              (0.5f)        /* [P] a climb that never sums above this is a false slope, no event [15] */
#define HUMP2_MIN_CLIMB_MM        (15.0f)       /* [P] ... or is shorter than this (vibration spikes: 2-11 mm) [none] */
#define HUMP2_START_BLANK_MS      (500)         /* [P] no climb starts this soon after START: the start jolt [none] */
#define HUMP2_ALPHA               (0.15f)       /* [P] pitch EMA every 20 ms [0.3]; 0.15 halves the vibration (19 Nov run) */
#define HUMP_WHEELBASE_MM         (120)         /* [P] ESTIMATE: drive axle to caster contact (L) - MEASURE IT.
                                                   tilt height = L x sin(pitch). 120 = 10 mm / sin(4.4-5.0 deg),
                                                   the cleanest run (20:25), assuming the hump is 10 mm */
#define HUMP2_MAX_LEN_MM          (600.0f)      /* no drop within this: ends with the climb only [1500].
                                                   The 1 cm hump plus the car is under ~300 mm */

/* ---- Battery (NFR20): only if the VBAT jumper is soldered (spec 3.7) ---- */
#define VBAT_ENABLED              (0)
#define VBAT_LOW_MV               (4600)
#define VBAT_CRITICAL_MV          (4300)

/* ---- Safety (DR 5) ---- */
#define HEARTBEAT_TIMEOUT_MS      (300)         /* control task silent this long = watchdog E-stop */

#endif /* ROBOT_CONFIG_H */
