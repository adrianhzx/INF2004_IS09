# IS09 car software (port of 19 Nov 2026)

This folder is the base robot software, changed to run on the IS09 car's wiring, with the IS09 barcode, IMU and ultrasonic modules inside it.

It has been **built and host-tested only**. It has **not run on the car yet**. Test it with the wheels off the ground first.

## What comes from where

| Part | Source |
|---|---|
| Tasks, line following, junctions, turns, speed control, safety, MQTT, bypass manoeuvre | Base code (unchanged logic) |
| Pin map, motor/encoder/servo/I2C drivers | Base drivers, changed to the IS09 wiring |
| Line sensors (analog) | IS09 `line_core` (calibration and darkness 0..100) feeding the base follower |
| Barcode | Base analog reader on the sensor AO, GP28 (since 19 Nov; `BARCODE_AO 1`). The IS09 `barcode_core` on DO/GP7 is kept: `BARCODE_AO 0` in `robot_config.h` |
| IMU | IS09 `imu_core` (pitch, hump episodes, collision, motion class) |
| Ultrasonic | IS09 `scan_core` (forward watch, coarse/fine scan, profile, side choice) |

The IS09 modules are in `app_program/is09/`. They are **byte-identical** copies from `C:\pico\my-project` (16 Nov working copy):

| File | md5 (prefix) |
|---|---|
| `is09/subsystem3/barcode_core.c` | 23f3dac1 |
| `is09/subsystem3/barcode_core.h` | 8800c217 |
| `is09/subsystem3/line_core.c` | 03bbd6ab |
| `is09/subsystem4/imu_core.c` | 290d6942 |
| `is09/subsystem5/scan_core.c` | 3cfa9a49 |
| `is09/shared/app_cal.h` | ced1058d (changed here: ECHO gate set to 1, comments; not the team copy) |

The adapters that connect them to the base tasks are `app_program/is09_*.c` and `is09_glue.h`. If a buddy changes an IS09 module in the team repo, copy the new file into `app_program/is09/` and rebuild.

The base files that the IS09 modules replace are kept, not compiled, in `app_program/base_replaced/`.

## Pin map (IS09 wiring)

| Function | Pin | Base had |
|---|---|---|
| Left motor M1A/M1B | GP8 / GP9 | right motor |
| Right motor M2A/M2B | GP10 / GP11 | left motor |
| Left encoder A/B | GP0 / GP1 | right encoder |
| Right encoder A/B | GP2 / GP3 | GP16 / GP17 (left) |
| IMU SDA/SCL (I2C0) | GP4 / GP5 | I2C1 on GP2 / GP3 |
| IMU INT1 | none | GP6 |
| Barcode sensor AO | GP28 (ADC2), VBAT jumper must NOT be soldered | analog AO on GP26 |
| Barcode sensor DO | GP7 (only used with `BARCODE_AO 0`) | – |
| Servo | GP15 | GP12 |
| Ultrasonic TRIG / ECHO | GP16 / GP17 | GP7 / GP27 |
| Line LEFT / CENTRE (analog) | GP26 / GP27 | digital on GP4 / GP5 |
| NeoPixel, BTN1, BTN2, buzzer | GP18, GP20, GP21, GP22 | same |

The IS09 CENTRE line sensor sits to the right of the LEFT one, so it plays the base "right" sensor. IS09 calibration values are copied into `app_program/robot_config.h` and marked `[IS09]`.

## Build and flash (laptop)

1. In `build_make`, run:
   `make PICO_SDK_PATH=<path to Pico SDK 2.2.0>`
   The defaults are already `SMP=0 CONSOLE=usb_cdc WIFI=cyw43`. The output is `is09_car_smp0.uf2`.
2. There is no up-to-date prebuilt file, so build it as in step 1. An older `.uf2` from before 20 Nov uses the old MQTT topics and the old beacon text, so it cannot talk to the current dashboard.
3. Wi-Fi and MQTT: `config/wifi_credentials.h` and `config/mqtt_config.h` hold **placeholders**. The car runs without Wi-Fi. To use MQTT, put your 2.4 GHz hotspot name, password and laptop IP there and rebuild.

Host tests for the adapters: in `tests_is09`, run `make`. The last result (20 Nov) was 149 passed, 0 failed.

## Barcode on AO (19 Nov)

- **Wiring:** sensor AO to GP28. AO must stay at or below 3.3 V, so power the module from 3.3 V or measure AO over black first. The Robo Pico VBAT jumper must not be soldered, because it also uses GP28.
- **Calibrate:** in IDLE, hold the sensor over white paper and press "Use AO now as white" in the dashboard, then over a black bar and press "Use AO now as black". This sends `SET:IR_WHITE_ADC=` / `SET:IR_BLACK_ADC=`, which apply at the next START. Put the measured values into `robot_config.h` as well, so they survive a restart.
- **Sensor offset:** measure `BARCODE_SENSOR_Y_MM`, the sensor's distance to the left of the car's centre line. It is still the base value of 35 mm.
- **Readouts:** the dashboard shows the result, the levels that read it, and the bar widths in 0.1 mm, drawn to scale.

## Before the first run

- **Ultrasonic is ON (19 Nov).** `APP_CAL_ECHO_SHIFTER_VERIFIED` is now 1 in `app_program/is09/shared/app_cal.h`. The user said the ECHO divider was measured at or below 3.3 V on the GP17 side. The team repo (`my-project`) still has 0 until HV-5 is signed there.
- **Bench mode:** with `US_IDLE_PING` 1 (default), the car pings every 0.2 s in IDLE and the dashboard shows the distance. In IDLE there is no stop and no 8 cm E-stop. The 25 cm obstacle stop and the 8 cm E-stop work only while driving.
- **Motor PWM is capped at 50 %** (`MOTOR_MAX_DUTY`, from IS09 `APP_CAL_PWM_MAX_PCT`).
- **Barcode map at start-up:** A = LEFT, B = RIGHT, C = FORWARD, D = U-TURN (the IS09 table). A `MAP:` command over MQTT replaces it.
- The barcode is armed only in LINE_FOLLOWING, as IS09 arms it only in S2.

## Dashboard (live data in the browser)

`MQTT/is09_dashboard.py` is an MQTT broker and a web page in one program, using only the Python standard library.

1. Stop `is09_broker.py` or Mosquitto first, because the dashboard uses the same port 1883.
2. In the `MQTT` folder, run `py is09_dashboard.py`. The browser opens http://localhost:8080.
3. The car connects as before (`MQTT_BROKER_IP` is the laptop's IP).

The page shows:
- the hump detector, with a pitch chart and the start threshold;
- motion, the line sensors, the barcode pattern and widths, the ultrasonic, and all events.

It also has buttons for START, STOP, E-STOP, SET and MAP. "Download log" saves the telemetry as CSV and every message as text. A full log is also written to `MQTT/logs/`.

## Hump detector settings (19 Nov)

- **Axes:** the IMU's forward axis is Y and its sign is -1 (`is09/shared/app_cal.h`), from the 19 Nov lift test.
- **Start threshold:** `HUMP_ON_DEG` is 3.0 (IS09 uses 5.0) and `HUMP_OFF_DEG` is 1.5 (IS09 uses 2.0). Both can be changed at run time from the dashboard. A 1 cm ramp tilts a car with a ~10 cm wheelbase by only about 5 to 6 degrees, which the detector's slow filter does not keep above 5.0.
- **Start/stop acceleration:** with `IMU_ACCEL_COMP` 1, the acceleration measured by the wheel encoders is taken off the forward axis first, so starting or stopping does not look like a slope.

## Junctions with no command (19 Nov)

- **Problem (21:43–21:45 runs):** after both sensors saw a junction bar (both black), one sensor left the bar before the other. The other still saw black, so the follower steered hard towards it (the left wheel dropped to 12 mm/s) and the car zig-zagged off the line.
- **Fix:** after a junction with no stored barcode command, the car drives straight for `LINE_JUNC_CLEAR_MM` (80 mm, the IS09 `APP_CAL_D_JUNC_CLEAR_MM`) whatever the sensors see, then follows again. Seeing the bar again during the 80 mm restarts the 80 mm but is not a second junction. `SET:LINE_JUNC_CLEAR_MM=0` turns it off.
- **Event:** a `JUNCTION` event ("no command: straight on for 80 mm") is now posted for each junction without a command. A junction with a stored command still stops, drives `IR_TO_AXLE_MM` and turns, as before.
- **Branches (22:23 runs):** no `JUNCTION` was seen at all. At the junction the CENTRE sensor met the bar head-on (darkness 1 → 76), but the LEFT one only reached 20–26 %, never black. The follower took it as "line moved right" and turned into the branch.
- **First fix (22:27), withdrawn:** one sensor turning black within 6 mm counted as a junction. On the 22:46–22:50 runs the CENTRE sensor turned black within 1–3 mm at every line edge, so right curves became "junctions" and the car drove straight off them.
- **Fix (22:54):** one sensor black while the other is also darker than white is a junction. The sensors straddle the 18 mm line, so the line alone never darkens both; only something wider does. The other sensor must be at least `LINE_JUNC_OTHER_PCT` (15 %) dark, its dark patch must have begun within `LINE_JUNC_OTHER_MM` (40 mm), and no barcode may be in progress. Checked against the logged frames: the 22:23 branches at 279 and 630 mm (LEFT 20–24 %) count; the right curves (LEFT 0 %) and the barcode passes (LEFT grey for 48–280 mm) do not. The 22:23 branch at 1055 mm does not count either, because the LEFT sensor had been grey for 80 mm before it. `SET:LINE_JUNC_OTHER_PCT=0` turns it off.
- **Turns at junctions (22:51):** the encoders measured 90° (about 1306–1367 counts per wheel), but the car turned too far. `TURN_SNAP` is now on by default. The encoder turn stops 45° short (`TURN_SNAP_EARLY_DEG`), then the car pivots slowly until the new line is between the sensors: the leading sensor goes at least `TURN_SNAP_PCT` (25 %) dark and then white again. If it does not, it gives up after `TURN_SNAP_MAX_DEG` (90°) more. Darkness is used instead of the black level because the LEFT sensor only reaches about 50 %.
- **Dashboard:** the line card shows, for each sensor, the mm of black seen ("marks": dark mm, black mm, peak %, rise mm) and a list of the last marks.
- **Limit:** with the line between the two sensors, both read white on a straight, so the car cannot tell "line between the sensors" from "no line". At a junction with no straight branch (an L or a T end), it needs a barcode command; without one it drives straight off the end.
- PC test of `line_step()`: no steering for the whole 80 mm (bar seen at an angle, seen twice, other sensor black), one junction reported, normal steering after it.

## Steering while passing a barcode (19 Nov)

- **Problem (22:00 run):** while the barcode sensor was over the bars, the left line sensor slowly reached the line (darkness 0 to 47 over about 120 mm). The first touch gave the full `LINE_STEER` 0.40 plus its ramp. The car swung towards the barcode, overshot, and rocked from side to side (turn rate ±15–40°/s), sometimes with the left sensor over the bars.
- **Fix:** while a barcode pass is open (bars seen, the quiet gap after them not yet passed), the steering is limited to `BARCODE_STEER_MAX` (0.25, full steering = 1.0). `SET:BARCODE_STEER_MAX=0` turns this off.
- **Rocking in general:** the base gains were tuned on the original car. A first try for less rocking, live from the dashboard: `SET:LINE_STEER=0.25`, `SET:LINE_STEER_RAMP=1.5`, `SET:LINE_HOLD_STEER=0.15`, and a lower `LINE_BASE_MM_S` (60). Copy the values that work into `robot_config.h`.

## Hump height, base method (19 Nov)

`app_program/hump2.c` runs the base hump profiler (from `imu_logic.c`) beside IS09 imu_core, on the same accelerometer samples. It only measures. IS09 imu_core still sets the hump speed limit, the hump count and `MISSION_COMPLETE`'s maximum.

- **Logic kept from the base code:**
  - It runs every 20 ms on the mean of two 10 ms samples.
  - Pitch = atan2(forward − encoder acceleration, √(lateral² + up²)) − level offset, smoothed with an EMA (alpha 0.3). The axes are IS09's (`is09_imu_map_axes()`).
  - The level offset is the mean of 1 s of samples while IDLE and still.
  - The states are FLAT → ASCENT (pitch ≥ start for 3 samples) → CREST (pitch below level) → DESCENT (pitch ≤ −start for 3 samples) → end (|pitch| ≤ level for 5 samples).
- **Two heights per hump:**
  - **sum:** the base Σ sin(pitch) × encoder distance. It is the average height under the car, so it reads low on a hump shorter than the car.
  - **tilt:** L × (sin(highest pitch) + sin(−lowest pitch)) / 2, where L is the drive axle to caster distance. It is the peak height for a hump shorter than about 2 × L.
- **Settings:** these can be changed from the dashboard ("Hump height" card) or with `SET:`. Base values are in brackets.
  - `HUMP2_START_DEG` 2.0 [5.0]
  - `HUMP2_LEVEL_DEG` 1.0 [3.0]
  - `HUMP2_MIN_MM` 0.5 [15]
  - `HUMP2_MIN_CLIMB_MM` 15 [none]: the climb and the drop must each be at least this long.
  - `HUMP2_ALPHA` 0.15 [0.3]
  - `HUMP_WHEELBASE_MM` 120, an **estimate**: 10 mm / sin(4.4–5.0°) from the cleanest run (20:25), assuming the hump is 10 mm. **Measure L.**
  - `HUMP2_START_BLANK_MS` 500 [none]: no climb may start in the first 0.5 s of driving (about 40 mm).
  - `HUMP2_MAX_LEN_MM` 600 [1500], fixed at build time.
- **Changes after the first car run (19 Nov, 20:14):** on that run, pitch spikes of 60–80 ms (2–11 mm of travel) each gave a START/END pair, and a real climb ended at the first sample below level.
  - The climb now ends only after 3 samples below level. The base code ended it after 1.
- **Changes after the second run (19 Nov, 20:25):**
  - On that run, the real hump climbed only 2–4.4° (from about 170 to 213 mm), so the 3.5° start missed it. It then dropped to −2 to −5° (from about 214 to 360 mm). The stop at the end of the run gave a +8° jolt, which IS09's detector took as a second hump.
  - The thresholds were lowered (above).
  - There is now one event per hump, `HUMP2_END`, and only for a whole hump: a climb and a drop, each at least `HUMP2_MIN_CLIMB_MM` long. Everything else, including the stop jolt, is only counted as "rejected".
  - IS09's detector no longer posts `HUMP_START`. It posts `HUMP_END` only for a hump it counts; a rejected episode is only logged.
  - Replaying the car's own 20 ms pitch trace of that run through the new code gives one hump: sum 0.9 mm, tilt 8.2 mm (with L = 100 mm, not measured). The old settings gave none.
- **Runs of 20:42–20:48 (three runs, car started about 10 cm before the hump, 80–100 mm/s):**
  - 20:42 gave two `HUMP2_END`: tilt 10.1 mm for the start jolt (+9.5° in the first 15 mm), and 12.4 mm at 460–590 mm. The second was raised by an +11.4° spike at 575 mm while the right wheel slowed from 50 to 22 mm/s.
  - 20:47 (two runs) gave none.
  - Around 100–250 mm the pitch swung ±4–9° with a period of about 30 mm. The car seems to hit the hump hard and rock at this speed. At about 45 mm/s (20:25, after IS09 slowed the car) the trace was clean.
  - Added `HUMP2_START_BLANK_MS`. Replaying the three traces: the start jolt is no longer counted; the 460–590 mm event still is (15.0 mm with L = 120).
  - Hump measurements should be run at about 45 mm/s (`SET:LINE_BASE_MM_S=45`).
- **Results:**
  - Events: one `HUMP2_END` per hump ("sum … mm, tilt … mm, len … mm (max …/…)").
  - Telemetry: the `"h2"` object, including `"ps"`, the 20 ms pitch samples since the last frame (0.1°).
  - "Download log" on the dashboard also saves `pitch_trace_*.csv` (time, distance, pitch every 20 ms).
- **PC simulation:**
  - This used a 10 mm triangular hump, L = 100 mm and 48 mm/s, with no noise.
  - Tilt read 9.1–9.8 mm for hump lengths of 40–200 mm. Sum read 1.4–5.5 mm.
  - With added random noise, tilt reads high: about +10 % at 50 mg and +30–50 % at 100 mg. Alpha 0.15 roughly halves that, but it lowers the peak on a short hump by about 1 mm.
  - If L is set 20 mm too short, tilt reads about 17 % low.
  - With the 20:25 thresholds and random 70 ms spikes (one every ~0.4 s): spikes of up to 4° still gave one event per hump in most runs. Spikes of 8° gave extra events, especially at 80 mm/s. The real 20:25 trace was calm (about ±1.5°) between the start and the stop.
  - None of this has been checked on the car.

## Stop when picked up (20 Nov)

- **Rule:** while driving (LINE_FOLLOWING, MANEUVER or OBSTACLE_AVOIDANCE), if the LEFT and CENTRE line sensors and the barcode AO all read at least `PICKUP_ADC` (3000 raw) for `PICKUP_MM` (30 mm) of wheel travel or for `PICKUP_MS` (500 ms), the run ends. The code is `app_program/pickup_logic.c`, called from `tsk_line.c` every 10 ms.
- **Why this rule:** with no floor under them, no IR light comes back and all three read about 3300–4095 (19 Nov 21:04, car held in the air; 20 Nov 15:01, car lifted after a stop). On the track the three never read 2000 or more together in any logged frame; the weak LEFT sensor reached at most 2525 on black tape. The 30 mm / 500 ms hold means an 18 mm black bar does not count. Raw values are used, not the darkness %, because line_core rescales the darkness to a running black reference.
- **Action:** the state goes to MISSION_COMPLETE first (so the turn and bypass loops end quietly), then the wheels stop. Events: `PICKED_UP` (the three raw values, mm and ms), then the usual `MISSION_COMPLETE` summary. BTN1 then goes to IDLE as after any run.
- **Bench tests with the wheels in the air:** the car now stops itself. Send `SET:PICKUP_ADC=0` to turn it off.
- **Tested:** PC only. 9 host tests (`tests_is09`), and a replay of all 7 dashboard logs (4128 driving frames): it fired once, on the 19 Nov 21:04 run with the car in the air, and never on the track. Not built for the Pico and not car-tested yet.
- No IMU check is used. If the sensors ever give false stops, an IMU check (tilt above ~20°) could be added as a second condition.

## Car tests of 20 Nov afternoon (log `dashboard_20261120_145447.log`, 15:58-16:28)

- **Pick-up stop: works on the car** (user, 20 Nov). It ended 14 runs with `PICKED_UP`, after 24-32 mm or 310-500 ms.
- **Barcode then off the line:** the course barcode reads **W**, which is not in the map (start-up map is A/B/C/D only). At the next junction (CENTRE black, LEFT white: the line goes right) the car stored W -> NONE and went straight on, then drove 150-400 mm dead straight on white until it was picked up. Seen at 16:09:44, 16:17:31 and others. Not a code fault: send the map with `MAP:` (the demo brief gives the four letters just before the run), e.g. `MAP:W=RIGHT`.
- **S-curve (16:23 and 16:24 runs):** the LEFT sensor reads only 33-53 % darkness on the line, against a black level of 45 %. On left curves it often stayed "grey" (33-44 %), so the car did not steer, drifted off the outside of the curve, and read both sensors white (taken as "line between the sensors"). It then went straight until it met the line again (both black, `JUNCTION`). Over all driving frames LEFT was 0-4 % on white (1649 frames), 25-44 % in 100 frames and 45 % or more in only 72.
- **Fix (20 Nov, PC-tested only):** `LINE_L_DARK_PCT` 25, then 15 after the 16:47 run (below) `[P]`, a black level for the LEFT sensor alone (`is09_line_set_left_dark()` in `is09_line.c`); CENTRE keeps `LINE_DARK_PCT` 45. 0 = old behaviour. Host tests 75/75. Not car-tested.

- **16:45-16:49 (new firmware with `LINE_L_DARK_PCT` 25):** the S-curve run (16:47:27) failed at the same place. LEFT now read only 24-40 % on the line (raw 770-1530; white ~150; the shared black reference stays at ~4000 because lifting the car shows "no floor"). The last left arc gave a 22 mm mark with a 24 % peak: just under 25, so not black. Default lowered to 15. Also check the LEFT sensor's height against CENTRE's: CENTRE gives raw 1600-2900 on the same line.
- **Barcode W:** still no `MAP:` sent in these runs, so W -> NONE (straight on) every time.
- **Obstacle, 16:48:06:** the scan gave `w 48cm` (IS09: 63 cm, edges -26/+37), reversed 250 mm, then went about 500 mm to the left. The encoder pivots were 43-47° for the commanded 45°, so if the car visibly turns more than that, the real rotation per encoder count is bigger than the model (`TRACK_WIDTH_MM` 140 is marked [TBC]): the same suspicion as the junction over-turn.

## Pivot calibration: TURN_TEST (20 Nov, not car-tested)

The user saw the bypass pivots turn more than 45°, while the encoders measured 43-47°. So the car rotates more per encoder count than the model says; the model uses `TRACK_WIDTH_MM` 140, marked [TBC]. All pivots (junction turns, bypass legs) use it, so one value fixes both.

- In IDLE, send `TURN_TEST` (dashboard command box). The car pivots 360° by the encoders at `TURN_MM_S`, as MANEUVER (E-stop, BTN2 and the pick-up stop work). Events: `TURN_TEST start: ...` and `TURN_TEST done`.
- If it really turns A degrees, set `TRACK_WIDTH_MM` = current value × 360 / A, e.g. 420° -> 140 × 360 / 420 = 120: `SET:TRACK_WIDTH_MM=120`. Repeat until it ends where it started, then copy the value into `robot_config.h`.
- Note: the box width estimate (`w 40-48 cm`) is measured by the servo scan with the car standing still, before any pivot, so it is a separate error (wide ultrasonic beam). The over-turn adds to it: a "45°" leg that is really 55-60° carries the car further sideways.

## Measuring the box while driving: OBST_MEASURE (20 Nov, PC-tested only)

- **Why:** the front scan cannot measure the box. In a simulation of the car's beam (echo = nearest box point within ±45°, which matches the 15 cm box echoing at ±60°), boxes of 10, 15, 30 and 45 cm all give the same scan: 26 cm at every angle, 27 at the ends. The user chose software measurement over narrowing the beam.
- **How (`OBST_MEASURE` 1, default):**
  - The car reverses until the box front is `OBST_MEAS_FACE_MM` (500 mm) from the axle (`standoff_measure()`), pivots 45°, and drives the diagonal at 45 mm/s with the servo pointing along the original path (`leg_diag_measure()`).
  - While the sensor is in front of the box, the range falls in a straight line, r = a − k·s; the fitted k is the real angle (so an over-turning pivot does not matter). Past the edge, r² = e² + dx² with e = a − k·s. Two pings 20 mm above the line mark the edge; each ping 30 mm above gives where the sensor crossed it (median). Code: `edge_meas_t` in `obstacle_logic.c`.
  - The leg ends when the axle is half car + margin past the measured edge. Box front closer than 80 mm before the edge, or no edge within 900 mm: blocked (back out and retry, as before).
  - On the side pass, the 60° pings give the side gap (square-on point of the side, 3 pings within 8 mm) and then the back corner (r² = gap² + dy²), so the depth.
  - Events: `OBST_EDGE` (`x` near edge mm from the line, `face`, `deg` real angle, `kf`, `leg`, `w2`) and `OBST_SIDE` (`gap`, `x`, `back`, `depth`, `w2`). The dashboard shows them and draws the measured box.
- **Limits:** only the side the car passes is measured. The width shown is twice the near edge's distance from the line, so it assumes the box is centred on the line. A box narrower than about 11 cm is narrower than where the sensor already is after the pivot, so it reads up to ~9 cm too wide (the safe side). No 0/30/60 leg watch on the measuring diagonal (the 8 cm proximity stop still works).
- **PC tests (beam model, 3 mm noise, 8 runs each):** box ≥ 150 mm with the real pivot 45-52°: edge within ±62 mm; 300 mm box: within ±40 mm; the car never touched the box in any case. With a 58° real pivot the gap to the box shrank to 4-5 mm in some runs: run `TURN_TEST` first. Host tests 136/136. Not car-tested.
- `OBST_WIDTH_CM` is now 0 by default (not used).
- **First car run (19:44):** the dashboard's "58 cm" was the front scan (not used). The car reversed 240 mm (box front to 500 mm), then on the diagonal the range read 468 -> 447 mm for ~70 mm and then jumped to 1734, 2040, 3826 mm: at ~45 cm the beam is narrow and the box simply dropped out of it. The first version only looked for the gradual bend (wide-beam model), took the far readings into its fit, never found the edge, and drove 660 mm until the user's E-stop.
- **Fix:** a jump of more than 80 mm above the last range, or no echo, twice in a row = past the edge (crossing half-way between the last ping on the box and the first past it); later pings are ignored; a gradual-bend crossing only counts if it is at most 250 mm above the line. Replay of the 19:44 leg: edge at 98 mm, leg would stop at 356 mm, near edge 111 mm from the line (about 22 cm wide if centred; the box is 15 cm: beam lag at 45 cm, plus the log has only every second ping). Kept as a host test.
- **19:52 run (first full measured bypass):** edge found within ~20 mm of the diagonal (36.9 -> 112.6 cm jump), near edge 73 mm from the line, width 145 mm (box 150). Diagonal ended with the car's side 65 mm from the box. Bypass complete, rejoin by the line sensors worked (`BYPASS_REJOIN` heading -13°), line following went on. The side pings at 60° never saw the flat side (background 57-93 cm), so no `OBST_SIDE` (depth not measured).
- **Too long along the side:** the side leg used back + gap (498 mm) and the look-before-turn then added a 130 mm trail. Geometry: on the 45° return the car's side clears the back corner by the margin once the axle is (half + margin)/sin45 − gap past the back (`bypass_side_leg_mm()`): 414 mm for that run, and no trail in measure mode (the look still checks that the side has ended). About 210 mm less. The back is front + `OBST_DEPTH_MM` (150, assumed): set it to the real box depth.
- **20:00 runs (overdrive on the straight, stopped remotely):** run 1, a 10 x 15 cm box; run 2, the box turned. On the diagonal the sensor along the path never saw the box (70-360 cm background, no echo): with a 10 cm box the sensor is already past the edge after the pivot (~57 mm out), and a turned face sends the echo away. The code fitted its line to the background: front 1300 / 1362 mm, side leg 1293 / 1156 mm. Run 2 also fitted a 25° angle, so the diagonal was 456 mm.
- **Fix:** a ping counts as the box only within 200 mm of where the scan puts the front (`EDGE_GATE_MM`); the angle k is never fitted (encoder angle only: run `TURN_TEST`); a box not seen in the first 3 pings gives an assumed edge 50 mm beyond the sensor (`un=1` in `OBST_EDGE`, "ASSUMED" on the dashboard); the front used for the side leg is the pings' (3+ on the box) or else the scan's, and never more than 300 mm beyond the scan's. Replay of both runs: diagonal 349 mm, side leg 371 / 311 mm. Host tests 149/149 (both runs kept as tests).
- **Limit:** with the encoder angle only, a pivot that really turns 52-58° puts the edge estimate 2-9 cm inward (the car passes closer; in simulation the smallest gap was 3 mm at 58°). `TURN_TEST` first.
- **Closer start:** `OBST_MEAS_FACE_MM` 500 -> 350 (user asked for less reversing; less beam lag). Simulation, 15 cm box from 350 mm: edge within ±12 mm with a 45° pivot. A 30 cm box from 350 mm blocks the diagonal: each retry now starts 150 mm further back (`standoff_measure()`).
- **Build note:** never `#include <math.h>` in a file that includes `<tk/tkernel.h>`: on the Pico build the two define different `size_t` types ("conflicting types for 'size_t'"). Put the maths in a plain C file (`obstacle_logic.c`), as `obstacle_sin_deg()`, `edge_angle_deg()` and `obstacle_leg_of()` do. The 20 Nov 19:10 build failed on this; checked afterwards with arm-none-eabi-gcc 13.2.1 (xPack) on all 28 application files that do not need the Pico SDK.

## Rejoining the line after a bypass (20 Nov, PC-checked only)

- **Car data (17:56 run):** the bypass finished and found the line about 75-98 mm left of the original path (encoders). The car then turned "parallel to the path" by the encoders, but the pivots over-turn, so it was at an angle to the real line. Line following started, both sensors went black while crossing the line at that angle, and the follower took it for a junction ("both sensors black ... straight on for 80 mm") 4 times in 4 s, driving across the line instead of onto it.
- **Fix 1 (`tsk_obstacle.c`, `rejoin_snap()`):** after the line is found and the axle is moved onto it, the encoder pivot stops 30° short; then the car pivots slowly (45 mm/s) until the line is between the two sensors: the leading sensor (left one for a left pivot) goes black and then white, or the trailing one goes black. At most 75° more, then it keeps the encoder angle and line following searches. Event `BYPASS_REJOIN` gives the heading it ended at.
- **Fix 2 (`tsk_line.c`):** for 250 mm after OBSTACLE_AVOIDANCE -> LINE_FOLLOWING, both sensors black is the line, not a junction, and the follower keeps steering towards the side the line was last seen on.
- **Replay check:** the 17:57 line-sensor frames through `line_logic.c`: 3 false junctions before, 1 with the 250 mm window (the one left is 0.8 m on). The snap cannot be replayed (that run used the encoder turn). Not car-tested.

## Known obstacle width: OBST_WIDTH_CM (20 Nov, PC-tested only)

- **Car data (17:00-17:43):** the demo box is 15 cm wide (user). Every scan of it gave 47-66 cm (this car's profile and IS09's). In the coarse sweep the sensor returned 25-30 cm at every angle from -60 to +60 deg, with the box about 26 cm ahead, where a 15 cm box covers only about ±16 deg. So the HC-SR04 beam is far wider than the box at this range and the scan cannot measure its width; any box comes out as the whole ±60 deg sweep.
- **Fix:** `OBST_WIDTH_CM` 15 `[P]`. After the scan, `obstacle_set_width()` (`obstacle_logic.c`) replaces the width with this value, centred where the scan put the box, and moves the side clearances by the same amount (an "open" side stays open). 0 = use the scan as before. `OBST_BOX` now also gives `w0`, the scanned width; the dashboard shows both.
- **Effect (PC test on a scan like the car's):** sideways shift 391 -> 215 mm, diagonal leg 553 -> 304 mm, stand-off needed 449 -> 273 mm. Host tests 88/88. Not car-tested.
- **Misleading aborts:** all four `AVOIDANCE_ABORT` events in these runs came after the run had already ended (dashboard STOP or a pick-up). `abort_avoidance()` now says nothing when the state is no longer OBSTACLE_AVOIDANCE.
- `TURN_TEST` was not run yet, so the pivots still turn more than 45°.

## Obstacle card on the dashboard (20 Nov, PC-tested only)

- **Firmware events:** `OBST_BOX` (box as measured, relative to where the bypass starts: `r` cm to the front, edges `xl`/`xr` cm, width `w`, free space `cL`/`cR`, assumed depth `d` mm, sensor ahead of the axle `ua` mm) and `BYPASS_PLAN` (`side`, pivots `t1`/`t2`/`t3` deg, `d1` diagonal, `d2` minimum along the side, `sh` sideways shift, `ret` longest return, `m` margin; mm). Both are sent before each bypass attempt. `BYPASS_STEP` "n what" is sent as each of the 8 steps starts.
- **Telemetry:** `obst.p` = [x mm, y mm, heading deg] of the bypass dead reckoning, only while in OBSTACLE_AVOIDANCE.
- **Dashboard:** the ultrasonic card (now full width) shows the stop distance, both width estimates (this car's scan and IS09's), the edges, free space, depth, chosen side and margin; the numbered plan with done / current steps and any extend / blocked / retry notes; the result; and a top view (path, box, planned route, driven route).
- **Tested:** the card's code was run in Node on the 16:48 bypass's events plus a simulated plan and steps (also a retry), and the drawing was rendered and checked. The firmware side compiles on the PC but has not run on the car.
- The drawing uses encoder angles and distances, so a pivot that really turns more than the encoders say does not show there: calibrate with `TURN_TEST` first.

## Obstacle bypass, why it goes wide (20 Nov, from the 19 Nov 21:13 log)

Only one bypass in the logs finished (21:13:53, "passed on the left"). Its path, rebuilt from the encoders (2505 counts per wheel turn, track 140 mm), in the line's frame:
1. Stop with the box 25 cm ahead. The scan gave `w 40cm off -5` (IS09's own profile: 34 cm).
2. Reverse 218 mm (`OBSTACLE_BACKOFF`: the 45° leg would clip the corner).
3. Pivot 45° left, then 564 mm on the diagonal, to 398 mm left of the line.
4. Back to parallel, about 340 mm along the box, a look-before-turning pivot, then about 150 mm more.
5. Pivot 45° right and return; the line was found 260 mm left of the start line.

The 398 mm side shift is what the plan asks for with a 40 cm box: left edge 25 cm + half car 8 cm + `OBST_MARGIN_MM` 6 cm. So the main cause is the width estimate, not the legs. The profile puts each edge at the outermost scan angle that still echoes, and the ultrasonic beam is wide, so each edge comes out too far out. The other attempts measured 40–44 cm as well (one 6 cm). The encoder turns may also be bigger in reality (see the junction turn issue), which would push the car out further.

Next steps: measure the real box width and compare; then either shrink each edge by the beam half-angle (r × tan(beam)), or lower `OBST_MARGIN_MM` (`[P]`, try `SET:OBST_MARGIN_MM=20`). Not changed yet.

## Names (20 Nov)

Renamed so no file names the other team: MQTT topics are `is09/...`; the UDP beacon is `IS09_BROKER <port>`; the build output is `is09_car_smp0.uf2`; `hump2.c` / `hump2.h` (functions `hump2_*`); `app_program/base_replaced/`; `MQTT/robot_tool.py`; `MQTT/mosquitto_robot.conf`. Comments say "base code" for the code this port started from. The firmware and `is09_dashboard.py` must be from the same version, or they will not find each other.

## Serial monitor lines

Every 0.5 s (telemetry over MQTT is now 5 times a second):
- `[status] ... L=raw/darkness C=raw/darkness ref white-black ... bc X ...`
- `bar: pin=1 st=WAIT_QUIET edges=… results=…`

After every barcode capture (same format as the IS09 console):
- `bar: cap=29 n=…ms w=…ms stray=0 pat=nwnnwnwnn n wnnnnwnnw n nwnnwnwnn bc=A`
- `bar: widths #N (29, ms): b… s… …`
- `bar: decoded A (read forwards)` when a letter is read.

The `pat=` for `*A*` should be `nwnnwnwnn n wnnnnwnnw n nwnnwnwnn`.

## Bench test order

1. Wheels off the ground, USB connected. Check the banner, `[init] IMU: LSM303DLHC at 0x19`, and that the state goes to IDLE.
2. Move a black line under each line sensor by hand. The `L=` / `C=` darkness should go above 45 on black. `LR 1/0` must match the side.
3. Press BTN1. Check that both wheels turn forward and that `dist` rises. If a wheel runs backwards, set `MOTOR_L_REVERSE` / `MOTOR_R_REVERSE`. If its count falls, set `ENC_L_REVERSED` / `ENC_R_REVERSED`.
4. Slide the `*A*` sheets (100 %, 75 %, 50 %) under the barcode sensor while the car follows a line. Check the `bar:` lines.
5. Tilt the car nose-up while it drives (wheels off ground). A `HUMP_END` line should appear.
6. Only after HV-5: set the ECHO gate to 1 and test the obstacle stop at 25 cm.

## Not verified yet

- Nothing has run on the IS09 car.
- The base line-following gains (`LINE_STEER` and others) were tuned on the original car. They may need retuning.
- The speed feed-forward uses the IS09 value (2.5 % per cm/s, no offset). The PI loop adds the rest. Tune with the car on the track.
- `OBST_US_AHEAD_MM` (80) is still the base placeholder. Measure it on the IS09 car.
- The IS09 barcode sheet sits left of the line. If its bars pass under the LEFT line sensor, the follower will steer onto them, as seen on the 16 Nov 75 % run.
- MQTT topics are `is09/...` (renamed 20 Nov). The firmware, `is09_dashboard.py` and the laptop tools in `MQTT/` all use them, so update them together.

## Junction turn and rejoin pivot, rework (20 Nov, evening)

Car runs at 20:16 and 20:17 showed two faults with one cause.

- Barcode A (LEFT): the old turn did 45 deg on the encoders without looking
  at the line. The line passed under both sensors during that part (LEFT black,
  then CENTRE black). The slow part then swept up to 90 deg more and found
  nothing. Result: about 135 deg on the encoders, plus the pivot over-turn, so
  it looked like a U-turn.
- Rejoin after the bypass: the same thing. The line passed the sensors during
  the fast 15 deg encoder part, and the slow part swept away from it.

Changes:

- `turn_on_line()` (tsk_nav.c): one slow pivot (50 mm/s) for the whole turn.
  The sensors are watched once 40 % of the angle is done (so a straight-on
  branch does not stop it). It stops when the leading sensor has been black
  and is white again, or when the trailing sensor goes black. Limit: the
  angle + 45 deg (was + 90 deg after the encoder part). Uses the black flags,
  so the LEFT sensor's own level applies.
- `rejoin_snap()` (tsk_obstacle.c): one slow pivot, watched from the start.
  Limit: the angle + 45 deg (was + 75 deg).
- In both, a sensor counts only after it has read white once, because the car
  may start with a sensor on the line.
- New `TURN_CAL` command (IDLE, car on a straight line): the car pivots left
  slowly and notes the encoder angle at each line crossing. A sensor's 1st and
  3rd crossing are one real turn apart, so it sets
  `TRACK_WIDTH_MM = TRACK_WIDTH_MM x (encoder angle) / 360` for this power-on
  and posts the value to put in robot_config.h.

Checks (PC only, not car-tested yet):

- Host tests: 154/154 pass (new test_turn_cal: a simulated car that turns
  1.5 x the encoder angle gives 93.3 mm; 1.0 x gives 140 mm; an axle 8 mm off
  the line gives 93.3 mm).
- Replaying the 20:17 log with the new rule: it would have stopped at about
  44 deg on the encoders (CENTRE black), instead of about 135 deg.
- ARM compile of tsk_nav.c, tsk_obstacle.c, tsk_telemetry.c and
  obstacle_logic.c: no errors or warnings.

## Rejoin pivot, second fix (20 Nov, 20:35)

Car runs at 20:30 and 20:32 (both passed on the right, so the rejoin pivot was
to the right and CENTRE was the leading sensor):

- The axle-onto-line move left CENTRE on the line. The pivot started, CENTRE
  went white (the line was now between the sensors) and the car kept turning,
  because the 20:20 change ignored a sensor that was black at the start. At
  20:30 LEFT then went black too and was also ignored.
- The pivot ran to its limit (about 90 deg on the encoders, about 130 deg
  real), then line following searched: close to a 180 deg turn.

Changes (rule in `rejoin_watch_step()`, obstacle_logic.c):

- Leading sensor black at the start counts as "seen"; it going white stops
  the pivot.
- Trailing sensor going black (after it read white) stops the pivot, even if
  the leading sensor never saw the line.
- If the trailing sensor is on the line before the pivot, the axle has not
  reached the line yet: the car creeps forward (up to 40 mm) until it is
  white, then pivots.
- If the line is not found, the car turns back 45 deg to the encoder angle
  instead of staying 45 deg past it.

Checks (PC only): host tests 159/159, with the 20:30 and 20:32 sensor
sequences (both now stop as soon as CENTRE goes white). ARM compile of
tsk_obstacle.c, obstacle_logic.c and tsk_nav.c: no errors or warnings.
TURN_CAL has not been run on the car yet.

## Barcode F = U-turn (20 Nov, 20:39)

`BARCODE_DEFAULT_MAP` is now `A=LEFT,B=RIGHT,C=FWD,D=UTURN,F=UTURN`, and the
dashboard's map box shows the same text. The Code 39 decoder already reads F.
PC check: nav_map_parse() takes the new map (5 letters) and F gives UTURN
(180 deg). Not car-tested.

