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

## Bench procedures owned by this subsystem (docs/HARDWARE_VALIDATION.md)

HV-3 geometry and ticks/rev, HV-4 polarity and brake, HV-9 NFR1 brake
latency (logic analyser on GP21 and GP8-GP11; expected <= 10 ms cycle + ISR
latency, must be measured).
