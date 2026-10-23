# Hardware validation record

Firmware cannot prove wiring. Each HV item is a no-power or bench procedure
that a named team member performs and signs here before the matching arming
switch in `app_program/shared/app_cal.h` is set to 1. Nothing in this file
is pre-filled as passed.

| ID | Procedure | Pass criterion | Unlocks | Owner | Result / date / signature |
|---|---|---|---|---|---|
| HV-1 | **No-power wiring check** against Rev B section 2 and the Robo Pico silkscreen: GP0/1 = left encoder A/B on GROVE 1 (3V3, GND), GP2/3 right encoder on GROVE 2, GP4/5 IMU on GROVE 3, GP7 barcode DOUT on GROVE 7 with AOUT and the GP28 wire disconnected, GP16/17 HC-SR04 on GROVE 4, GP26/GP27 line AOUT on GROVE 5/6 with the shared GP26 wire of GROVE 6 disconnected, servo on servo port 4 (GP15), motor leads on MOTOR 1/2 terminals, no Y-harness, common ground. Check VBAT jumper on GP28 (A13). | Every row ticked by two people; photo stored in `testing/system/logs/` | TEAM.md pin table marked "verified" | Integration owner (TBC) | NOT RUN |
| HV-2 | **Boot self-check**: flash the `SMP=0 NET=lwip MQTT=1` image, open the USB console. | Banner shows `hw self-check: 0x7 (timer 1, pwm 1, io 1)` and the controller reaches S1 (telemetry/console). If timer = 0 the car stays latched with CRITICAL_RUNTIME. | Everything else | Integration owner | NOT RUN |
| HV-3 | **Motion calibration (car on blocks, wheels free)**: measure wheel diameter and track width with calipers; count encoder edges per wheel revolution by hand-turning 10 turns while reading `enc_ticks` on the console (TEST image, S2-U tests in Phase 2); scope encoder A/B at 3V3 (A4). | Edge level <= 3.3 V; ticks/rev identical on 10 turns +-1; values entered in `app_cal.h` | `APP_CAL_MOTION_ARMED` (with HV-4) | Buddy 2 | NOT RUN |
| HV-4 | **Motor polarity and brake (on blocks)**: with the Phase 2 TEST image, command +20 % on each motor for 1 s, confirm the wheel turns forward and the encoder count increases; confirm BRAKE (both pins low) stops within the datasheet brake behaviour; STOP button during the pulse latches and brakes. Measure motor current < 1 A. | Direction, sign of counts and brake verified for both wheels | `APP_CAL_MOTION_ARMED` | Buddy 2 + integration owner | NOT RUN |
| HV-5 | **HC-SR04 ECHO level**: identify the regulated 5 V supply for the module; fit the divider/level shifter; with the ECHO output **not connected to the Pico**, trigger the module and measure the ECHO high level with a scope/meter. | <= 3.3 V (Robo Pico input max 3.6 V) | `APP_CAL_ECHO_SHIFTER_VERIFIED` | Buddy 5 + integration owner | NOT RUN |
| HV-6 | **Button polarity**: with the Phase 1 TEST image, read `app_hw_gpio_read(20/21)` on the console idle vs pressed (or watch the Robo Pico status LEDs). | Pressed = LOW with internal pull-up | `APP_CAL_BUTTON_ACTIVE_LOW` confirmed | Integration owner | NOT RUN |
| HV-7 | **Servo pulse limits**: at battery voltage, find the 0 deg and 180 deg pulse widths without stalling against the mechanical stop; time a 30 deg step with a scope (A11). | Limits entered in `app_cal.h`; settle time recorded | Phase 5 scan limits | Buddy 5 | NOT RUN |
| HV-8 | **Supply under load (A14)**: motors + servo + Wi-Fi publishing; log resets for 50 start-stop cycles; measure Grove 3V3 current. | No reset; < 300 mA | demo readiness | Integration owner | NOT RUN |
| HV-9 | **NFR1 brake latency**: logic analyser on GP21 (STOP) and GP8-GP11; press STOP while driving on blocks. | Motor pins in brake pattern <= 20 ms after the STOP edge, 20 trials | NFR1 PASS claim | Buddy 2 | NOT RUN |
| HV-10 | **IR line sensor polarity and references (A7)**: with the Phase 3 TEST image on the console (`s3_line_raw`), hold each line sensor over the white floor and over the black tape. | Black reading > white reading by >= 400 LSB on both channels (else set `APP_CAL_IR_DARK_IS_HIGH 0`); write the two values into `APP_CAL_LINE_INIT_WHITE/BLACK`. Then sweep the bracket so the tape drifts to the car's RIGHT: `line_error` must go positive. | `APP_CAL_IR_DARK_IS_HIGH`, FOLLOW steer sign | B3 Xu Zhiqiang | NOT RUN |
| HV-11 | **Barcode edge timing (NFR4, A6)**: logic analyser on GP7 while the car is pushed over sample 1 at the planned speed; compare the analyser element widths with the `[TEST] S3-U05 INFO` ring contents / telemetry. | Every element within 10 % of the analyser; narrow bars >= 2 ms (else lower the speed or raise `APP_CAL_T_GLITCH_US` margin); wide bars < `APP_CAL_T_QUIET_MS`. | barcode in the run profile | B3 Xu Zhiqiang | NOT RUN |

## Why motion is disarmed in the shipped foundation

`APP_CAL_MOTION_ARMED = 0`: `try_start_run()` in `vehicle_controller.c`
refuses to leave S1 and posts `CMD_REJECTED`, and the Phase 2 motion task
rejects every drive command while the switch is 0. Setting it to 1 is a
team decision recorded in this file (HV-3, HV-4).
