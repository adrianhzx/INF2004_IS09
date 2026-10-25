# Subsystem 5 - Ultrasonic scan and obstacle profile (Buddy 5, Ho Zhi Xian Adrian)

## Host tests (`make -C testing/host test_scan && ./testing/host/test_scan`)

| Run | Date | Result | Log |
|---|---|---|---|
| Phase 5 | 2026-10-23 | 37 passed, 0 failed | `docs/evidence/phase5_host_tests.log` |

IDs H-SC-01..37: servo pulse map (ahead 1500 us, limits, clamp), settle time
(150 ms per 30 deg, 25 ms per 5 deg step, 20 ms floor, 900 ms full sweep),
sin/cos table within 2 LSB Q15, lateral geometry (LEFT positive, int7
clamp), forward watch (two consecutive readings at/inside D_STOP raise
EVT_OBSTACLE once, hysteresis D_STOP + 5 cm re-arms, a no-echo reading
breaks the pair, NO_ECHO fault on the 5th consecutive timeout and cleared by
a valid echo), coarse plan (9 points -60..+60 deg), fine window, profiles
for a synthetic world seen through a 14 deg beam: nothing ahead (CONTINUE,
no fine scan), centred 20 cm box at 22 cm (width 20 after beam correction,
edges both sides, closest ahead, a turn recommended), left-offset box ->
TURN_RIGHT, right-offset box -> TURN_LEFT, blocked left side -> TURN_RIGHT,
70 cm box -> STOP (offsets exceed Y_PATH_MAX), face at 12 cm -> REVERSE
(pivot guard), width clamps at 63, two-word round trip; side ranging
confirmation.

Limits recorded by the tests:

* The beam correction (`APP_CAL_US_BEAM_HALF_SIN_PERMILLE`, sin 7 deg) is
  calibrated for a sensor that sees a box when any ray within +-7 deg hits
  it. Against an ideal pencil beam the same correction under-reads a 20 cm
  box as 12 cm, so HV-15 must measure the real beam (A10) before NFR10/11
  claims.
* Profile time: 9 coarse + up to 19 fine points, each a servo settle plus a
  <= 30 ms ranging; about 1.1 s coarse-only, up to ~2.5 s with a wide fine
  window, bounded by T_SCAN_TIMEOUT 4 s (NFR14 T_profile is measured on
  target by `s5_last_scan_ms`).

## On-target (`make SMP=0 NET=lwip MQTT=1 TEST=1`, safe with both gates off)

| ID | Check | Result | Log |
|---|---|---|---|
| S5-U01a-b | slice 7B compare = ahead pulse; GP15 routed to PWM only when `APP_CAL_SERVO_ARMED` | NOT RUN | |
| S5-U02a-f | forced GP17 edges: ECHO_DONE, width 580 +-60 us, 10 cm, FLG_SCAN bit set; stray fall ignored; > 30 ms -> OVERRUN | NOT RUN | |
| S5-U03a-d | one REQ_PROFILE -> exactly one DONE or FAIL; profile copy under the mutex; scan_state matches the gates; scan time <= T_SCAN_TIMEOUT | NOT RUN | |
| S5-U04a-b | WATCH_ON: 10 +-2 rangings/s on the 100 ms tick; WATCH_OFF stops them | NOT RUN | |
| S5-U05 | TRIG stays low while `APP_CAL_ECHO_SHIFTER_VERIFIED == 0` | NOT RUN | |
| S5-U06 | no stop latched by the scan subsystem alone | NOT RUN | |

With both gates signed (HV-5, HV-7) the same image ranges for real; S5-U03
then prints the measured profile of whatever is in front of the car.

## Bench procedures owned by this subsystem (docs/HARDWARE_VALIDATION.md)

HV-5 ECHO level shifter, HV-7 servo pulse limits and settle (A11), HV-15
beam width / width correction against the course obstacles (A10, NFR10/11).
