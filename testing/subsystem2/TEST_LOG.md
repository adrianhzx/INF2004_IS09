# Subsystem 2 - Motion and encoders (Buddy 2)

## Host tests (`make -C testing/host test_motion && ./testing/host/test_motion`)

| Run | Date | Result | Log |
|---|---|---|---|
| Phase 2 | 2026-10-23 | 56 passed, 0 failed | `docs/evidence/phase2_host_tests.log` |

IDs H-MO-01..43: geometry (um/tick, pivot ticks, speed from interval,
saturation), quadrature table (forward/reverse/glitch), boot BRAKED, reject
while disarmed / speed 0 / while MOVING, BRAKE pre-emption, latch -> PWM 0
same cycle + flush + ABORTED, stale command after latch, guarded recovery to
BRAKED only, 1 m forward and reverse MOVE across a uint16 counter wrap
(signed distance, direction bits), 90 deg pivots both ways (equal and
opposite wheel travel), until_line finish and 150 % TIMEOUT, FOLLOW steer
polarity, STALE_LINE / CTRL_HEARTBEAT / STALL / ENCODER_FAULT stops with
their timing, no false stop while idle, command TIMEOUT, PWM limiter.

The simulated car is a first-order lag (100 % PWM = 40 cm/s, tau 50 ms); it
proves the logic and the sign conventions, not the physical gains.

## On-target (`make SMP=0 NET=lwip MQTT=1 TEST=1`, car on blocks)

| ID | Check | Result | Log |
|---|---|---|---|
| S2-U01a-c | both slices in BRAKE (compares 0) after init; wrap 6249 | NOT RUN | |
| S2-U02a-b | 100 +-5 cycles/s; longest cycle < 1 ms | NOT RUN | |
| S2-U03a-b | no encoder edges / glitches at rest | NOT RUN | |
| S2-U04 | MOVE rejected while disarmed, compares stay 0 | NOT RUN | |
| S2-U05a-d | core on ARM: reverse MOVE signs, latch same-cycle brake, stale command rejected | NOT RUN | |
| S2-U06 | (ARMED only) MOVE 200 mm through the task on blocks completes OK | NOT RUN (disarmed) | |

### Run history (add a row for every run; never edit or delete old rows)

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-27 | S2-U01..U05 (two runs) | `57a1be1` tree (converged TEST image) | Wong Wei Qi | PASS 12/0, S2-U06 NOT RUN (disarmed) | `testing/system/logs/2026-10-27_test_image_full_run_converged.txt` | First recorded on-target run of the motion subsystem: compares 0 after init, 20 kHz wrap, task 100 cycles/s, longest cycle 110 / 125 us, no encoder edges or glitches at rest, MOVE rejected while disarmed, reverse MOVE path and latch handling. Motors were not energised (`APP_CAL_MOTION_ARMED 0`); HV-3/HV-4 still unsigned. |
| 2026-10-27 | demo-image telemetry (field) | `57a1be1` tree | Wong Wei Qi | observation | `testing/system/logs/2026-10-27_demo_image_telemetry_decoded.txt` | Hand-moved wheel: encoder word 579 -> 65390 in one 200 ms period, speed saturating at 127 cm/s on nudges, distance jumps of metres. Consistent with `APP_CAL_TICKS_PER_REV 20` being far from the real encoder (or bouncing inputs). S2-U07 live encoder loop added for the HV-3 count; see docs/HARDWARE_VALIDATION.md. |
| 2026-10-27 | first armed run (field, Phase 7 image, `APP_CAL_TICKS_PER_REV` still 20, track width unmeasured) | `2e8c52a` + local arming | Wong Wei Qi | **FAIL (two defects)** | `testing/system/logs/2026-10-27_first_armed_run_decoded.txt` | (1) **Right encoder never counted** (`enc=NNNN/0` for 8 s): wiring/channel on GP2/GP3 or the sensor itself (HV-1/HV-3). Speed loop then drove the right wheel at the 80 % ceiling and the left at the floor: fast, circling left. (2) The stall guard needed both wheels silent, so this was never caught (D-33, fixed). Left encoder ~5100 edges/s with `TICKS_PER_REV 20` saturated speed (127) and distance (32767 mm) at once: the real count per revolution is in the thousands; must be measured with S2-U07 before the next armed run. Line search cascaded through all four sweeps in 80 ms because each turn's tick target was ~10 ticks. |


## Bench procedures owned by this subsystem (docs/HARDWARE_VALIDATION.md)

HV-3 geometry and ticks/rev, HV-4 polarity and brake, HV-9 NFR1 brake
latency (logic analyser on GP21 and GP8-GP11; expected <= 10 ms cycle + ISR
latency, must be measured).
