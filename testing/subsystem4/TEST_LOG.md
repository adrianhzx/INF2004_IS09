# Subsystem 4 - IMU, hump and collision (Buddy 4, Wong Wei Qi)

## Host tests (`make -C testing/host test_imu && ./testing/host/test_imu`)

| Run | Date | Result | Log |
|---|---|---|---|
| Phase 4 | 2026-10-23 | 56 passed, 0 failed | `docs/evidence/phase4_host_tests.log` |

IDs H-IM-01..56: integer sqrt and atan2 (within 0.5 deg over the full
circle, large inputs prescaled), default axis map, pitch filter (alpha 1/4,
clamp at +-63.5 deg, state kept across an invalid sample), a scripted
20 cm/s car over a 15 deg trapezoid hump: EVT_HUMP_START after theta_ON held
50 ms, live peak while active, classes CLIMBING / DESCENDING, exactly one
EVT_HUMP_END carrying the same seq, NORMAL with the 51.8 mm peak estimated
within +-6 mm, run maximum kept across a smaller hump and raised by a larger
one; REJECTED by T_ABORT (false trigger) and by D_HUMP_MAX without descent;
DEGRADED when the descent never settles (peak reported, hump_max untouched);
INVALID at once on STOP_LATCH, stale MotionStatus or sensor failure with the
peak discarded; no start while stationary, latched or stale; a 257 mm climb
saturates to 127 mm and is reported DEGRADED, never as fact; 18 episodes wrap
the 4-bit seq; collision needs two consecutive >= 2.5 g spikes while moving,
is requested once, holds IMPACT for 500 ms, and is ignored at rest or after
the latch; class hysteresis (30 ms), ACCELERATING on a speed step, TURNING
from MotionStatus, STATIONARY on stop; ImuState word round trip.

Limits recorded by the tests:

* The height estimate uses distance x sin(pitch) with the filtered pitch;
  the filter lag costs a few mm on a 50 mm hump. K_CAL (A8, HV-14) is the
  bench correction; the field saturates at 127 mm.
* A hump with a flat crest longer than T_ABORT (300 ms = 6 cm at 20 cm/s)
  is REJECTED by the adopted statechart; raise `APP_CAL_T_ABORT_MS` if the
  reference humps have long crests.
* Collision and hump-entry shock are separated only by the 2.5 g threshold
  (A9); `APP_CAL_COLLISION_ENABLE 0` lets HV-13 record both without stops.

## On-target (`make SMP=0 NET=lwip MQTT=1 TEST=1`, car level and still)

| ID | Check | Result | Log |
|---|---|---|---|
| S4-U01 | LSM303DLHC answers: CTRL_REG1_A reads back 0x57, IRA_REG_M 'H'; abort/timeout counters printed | NOT RUN | |
| S4-U02a-b | ImuTask 100 +-5 cycles/s; longest cycle < 600 us | NOT RUN | |
| S4-U03 | gravity magnitude 1 g +-20 % at rest | NOT RUN | |
| S4-U04a-c | sensor healthy and IMU_FAULT clear; pitch within +-10 deg level (axis map HV-12); no episode, STATIONARY | NOT RUN | |
| S4-U05 | a tap at rest does not latch a stop (D-18) | NOT RUN | |
| S4-U06 | ImuState refreshed within 15 ms | NOT RUN | |

### Run history (add a row for every run; never edit or delete old rows)

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S4-U01..U03, U04a, U05, U06 | `8af4bdc` (feat/imu-hump-detection) | Wong Wei Qi (Buddy 4) | REPORTED PASS, **console log not committed** | commit message / zip notes only | Reported: ImuTask ~100 cycles/s, longest cycle 438 us, I2C scan finds 0x19 and 0x1E, host `test_imu` 56/56. **S4-U04b (level pitch) and S4-U04c are not listed as passed: treat as OPEN** (HV-12 axis signs unsigned). No capture under `testing/subsystem4/logs/`; treated as NOT RECORDED until added. |
| 2026-10-26 | behaviour note | `8af4bdc` | Wong Wei Qi | - | - | On that branch (no motion subsystem) ImuState quality stays INVALID and the STALE_MOTION fault is set: expected, the hump estimate needs wheel distance (D-18). Resolved by the unified branch, which carries subsystem2. |
| 2026-10-27 | merge | `c3920d3` | Claude (for Buddy 4, integration) | BUILD PASS | `docs/evidence/merge_*` | Branch merged into the unified branch; 4 profiles build with 0 warnings; host 56/56; gate PASS. Nothing run on hardware here. |
| 2026-10-27 | S4-U01..U06 incl. U04b/U04c (two runs) | `57a1be1` tree (converged TEST image) | Wong Wei Qi | PASS 9/0 | `testing/system/logs/2026-10-27_test_image_full_run_converged.txt` | First recorded log, and the first pass of S4-U04b (pitch within +-10 deg level) and S4-U04c (no episode, STATIONARY): the default axis map/signs read level on this car. Rest vector x=-39 y=63 z=878 mg (|a| = 0.88 g, within the 20 % window but low: check the +-2 g scale / HR mode reading at HV-12), longest cycle 467 / 445 us, 0 read errors, tap at rest does not latch. Motion subsystem present, so quality no longer INVALID by design. HV-12..14 signatures still needed. |

Known interaction (reported by Buddy 3 and Buddy 4): **S1-U06 fails once
ImuTask runs** because that test writes a pattern into `g_imu_state`, which
the real task overwrites every 10 ms. Test design issue, not a firmware bug;
see `testing/subsystem1/TEST_LOG.md`.


Also planned (needs the car moving, Phase 7 integration): forced collision
on blocks with the wheels spinning (COLLISION latched, IMU_FAULT set, cleared
after recovery), one reference hump per quality.

## Bench procedures owned by this subsystem (docs/HARDWARE_VALIDATION.md)

HV-12 axis mapping and signs, HV-13 collision threshold vs hump-entry shock
(A9), HV-14 K_CAL against reference humps (A8, NFR15).
