# Subsystem 3 - Line sensing and barcode (Buddy 3)

## Host tests (`make -C testing/host test_line && ./testing/host/test_line`)

| Run | Date | Result | Log |
|---|---|---|---|
| Phase 3 | 2026-10-23 | 114 passed, 0 failed | `docs/evidence/phase3_host_tests.log` |

IDs H-LN-01..25 (line_core): normalisation and clamps, boot references
usable at once, flat signal collapsing the running references to an
IR_CONTRAST loss and immediate recovery, error sign (+ = line right of
centre) and steer polarity/clamp, seq modulo 16, JUNCTION on the 6th
both-dark cycle (T_JUNC 60 ms) reported once and re-armed after leaving
both-dark, a 3-cycle both-dark blip filtered, LOST on the 10th both-light
cycle (T_LOST 100 ms) with on_line held true until then, FOUND on the first
dark sample afterwards, LineState word round trip, raw > 4095 clamped.

IDs H-BC-00..23 (barcode_core): Code 39 A/B/C/D/Z decoded forwards and
reversed (post U-turn) at narrow = 6, 15, 30 ms (3 mm at 50 / 20 / 10 cm/s),
+-20 % width jitter, 28-element and corrupted-element rejection, no-contrast
rejection, unknown character; state machine DISARMED -> WAIT_QUIET ->
CAPTURING -> result, edges ignored while disarmed or before a quiet zone,
seq 1, 2, ... in 4 bits, a 6.7 cm/s capture (wide 135 ms) exact in 8 us
units, stray mark (5 elements) discarded silently, partial code (22
elements) reported as INV_NO_MATCH, a 100 us glitch inside a wide bar merged
back, a mid-capture gap >= T_QUIET closing the capture and starting the
next, ring overflow -> INV_OVERFLOW + fault, the 65th element ->
INV_OVERFLOW, DISARM discarding everything.

Known limits found by the tests (recorded, not hidden):

* T_QUIET (150 ms) bounds the slowest decodable speed: a wide element of
  3 x 3 mm = 9 mm must take < 150 ms, so the car must cross a barcode at
  >= 6 cm/s. The quiet timeout can be raised in `app_cal.h` if the bench
  speed is lower, at the cost of a slower result after the last bar.
* Element widths are stored in 8 us units (uint16, 524 ms range) to match the
  EdgeRing representation; a microsecond uint16 would have saturated at
  65 ms (found by H-BC-01 at 10 cm/s before the change).

## On-target (`make SMP=0 NET=lwip MQTT=1 TEST=1`, no sensor needed)

| ID | Check | Result | Log |
|---|---|---|---|
| S3-U01a-b | ADC0 (GP26) and ADC1 (GP27) convert, values <= 4095 | NOT RUN | |
| S3-U02a-b | LineSensingTask 100 +-5 cycles/s; longest cycle < 300 us | NOT RUN | |
| S3-U03 | LineState seq changes within 15 ms (NFR3) | NOT RUN | |
| S3-U04 | disarmed decoder ignores a forced GP7 edge | NOT RUN | |
| S3-U05a-e | ARM -> WAIT_QUIET; forced Code 39 'A' -> LEFT, 'D' -> UTURN, 'Z' -> INV_NO_MATCH through ISR -> EdgeRing -> task -> `g_barcode_byte` + EVT_BARCODE | NOT RUN | |
| S3-U06 | DISARM -> DISARMED | NOT RUN | |

### Run history (add a row for every run; never edit or delete old rows)

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S3-U01..U06 | `b0605d1` (feat/IRsensorandbarcode) | Adrian for Buddy 3 | REPORTED PASS 12/12, **console log not committed** | commit message only | Claimed in the commit message of `b0605d1`; no capture under `testing/subsystem3/logs/`. Treated as NOT RECORDED until the console text is added. Also reported: host `test_line` 114/114. |
| 2026-10-26 | S3-U07 live line loop, S3-U08 barcode loop | `b0605d1` | Adrian for Buddy 3 | diagnostic only | - | Both loops ship off (`S3_T_LIVE_LOOP 0`, `S3_T_BC_LOOP 0`); no readings recorded in the repo. HV-10 / HV-11 still unsigned. |
| 2026-10-27 | merge | `719e498` | Claude (for Buddy 4, integration) | BUILD PASS | `docs/evidence/merge_*` | Branch merged into the unified branch; 4 profiles build with 0 warnings; host 114/114; gate PASS. Nothing run on hardware here. |
| 2026-10-27 | S3-U01..U06 (two runs) | `57a1be1` tree (converged TEST image) | Wong Wei Qi | PASS 12/0 | `testing/system/logs/2026-10-27_test_image_full_run_converged.txt` | First recorded log. ADC raw at rest left 186/206, centre 295/281 (sensor on the bench, not over the line), task 100 cycles/s, longest cycle 85 / 78 us, forced 'A'/'D'/'Z' decode through ISR -> EdgeRing -> task, 91 edges, 0 ring overflow. HV-10/HV-11 still unsigned. |
| 2026-10-27 | demo-image telemetry (field, Phase 7 image) | `54c3299` | Wong Wei Qi | **DEFECT FOUND** (fixed next commit, D-32) | `../system/logs/2026-10-27_demo_image_phase7_idle_decoded.txt` | Stationary car, sensors on one shade: `faults=[IR_CONTRAST]`, `line_err` swinging -100..+100, `on_line=1`, one phantom JUNCTION event. Running references had collapsed; noise was being normalised. Fixed: uncalibrated = no line. |
| 2026-10-28 | fourth armed run (field, straight line, sensors set so both read white when centred) | `a794e8d` not yet merged by the user (pre-D-34 image) | Wong Wei Qi | FAIL (geometry + thresholds, D-35) | `../system/logs/2026-10-28_fourth_armed_run_decoded.txt` | 45 of 48 S2 snapshots `on_line=0`, `line_err=0`, `IR_CONTRAST`: with both sensors on white the firmware cannot tell centred from lost; LOST -> S5 -> `S7 cause=LINE_SEARCH_TIMEOUT`. Right encoder now counts (jumper refitted); left still counts down while driving. Fix: dead band 30-70 % and set the bracket so each sensor is half on the line. |
| 2026-10-27 | third armed run on a straight black line (field) | `2e8c52a` + local arming, ticks/rev still 20 | Wong Wei Qi | **DEFECT FOUND** (fixed next commit, D-34) | `../system/logs/2026-10-27_third_armed_run_decoded.txt` | On the line, `IR_CONTRAST` for 45 of 48 S2 snapshots: per-channel references relaxed away within seconds on a straight. Steer 0, LOST, search, `S7 LINE_SEARCH_TIMEOUT`. Also: right encoder now counts (jumper refitted), left counts DOWN while driving (ENC_L or MOTOR_L reversed). |
| 2026-10-28 | fifth run, straight line, straddle bracket (image before D-33..D-35) | `2e8c52a` + local arming | team | FAIL (design mismatch, D-36) | `../system/logs/2026-10-28_fifth_run_straddle_decoded.txt` | Both sensors on white when centred -> LOST after 100 ms -> S5 -> cascade (ticks/rev still 20) -> S7. Also: right encoder now counts (jumper), left counts down while pushed forward -> `APP_CAL_ENC_L_REVERSE 1`. D-36: both-light = straight ahead, LOST after 1.5 s. |

Known interaction (reported by Buddy 3 and Buddy 4): **S1-U06 fails once
LineSensingTask runs** because that test writes a pattern into
`g_line_state`, which the real task overwrites every 10 ms. Test design
issue, not a firmware bug; see `testing/subsystem1/TEST_LOG.md`.


S3-U05 forces the GP7 interrupt with 3 ms / 9 ms element timing from the
test task, so it exercises the real kernel path without a sensor; it does
not prove the optical side.

## Bench procedures owned by this subsystem (docs/HARDWARE_VALIDATION.md)

HV-10 IR analogue polarity, references (`APP_CAL_IR_DARK_IS_HIGH`,
`APP_CAL_LINE_INIT_WHITE/BLACK`) and the error-sign hand sweep on the real
bracket (left sensor lighter + centre darker = line right); HV-11 barcode
sensor DOUT edge timing at the planned speed (NFR4).
