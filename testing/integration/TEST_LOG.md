# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | mission statechart, every transition against a simulated car | Rev B section 5, 6.3; FR1, FR2, FR6, FR7, FR15, FR17-FR19, FR23-FR25 | auto (host) | `make -C testing/host test_mission && ./testing/host/test_mission` (H-MI-01..76) | `mission host tests: N passed, 0 failed` |
| INT-04 | START refused while motion is disarmed | HV-3/HV-4 gate | manual, demo image | Press START in S1 with `APP_CAL_MOTION_ARMED 0` | snapshot stays `S1_IDLE`, `EVT CMD_REJECTED payload 0x0001`, no wheel moves |
| INT-05 | local STOP from every mission state | FR24, NFR1 | manual, demo image, car on blocks, motion armed | Press STOP while S2 / S3 / S4 / S5 | wheels brake, snapshot `S7_SAFE_STOP cause=OPERATOR_STOP`; NFR1 latency by scope on M1A (HV-9) |
| INT-06 | guarded recovery | FR24 | manual | after INT-05 release STOP, hold START >= T_HOLD (1 s) | `EVT RECOVERY`, state S0 then S1, no motion until a fresh START press |
| INT-07 | line following start/stop on blocks | FR1 | manual, car on blocks over a printed line | START in S1 | S2, both wheels turn, `line_err` steers them; `MOTION_DONE` never fires for FOLLOW (D-31) |
| INT-08 | junction, no command (D-29) | FR3 | manual, on blocks | cover both IR sensors with black for > T_JUNC | S3, `EVT NAV payload 0x0300`, MOVE 80 mm, back to S2 |
| INT-09 | barcode -> junction -> turn | FR4-FR6 | manual, on blocks | pass barcode A under GP7, then cover both sensors | pending LEFT in snapshot; at the junction TURN +90 until_line (wheels counter-rotate), S2 after the line is seen |
| INT-10 | line loss -> bounded search -> timeout | FR2, FR25 | manual, on blocks, sensors over white | START, then keep both sensors over white | S5 sweeps (left/right wheel counter-rotation), then `S7 cause=LINE_SEARCH_TIMEOUT` within T_LINE_SEARCH |
| INT-11 | obstacle -> brake -> profile -> bypass | FR14-FR19, NFR2, NFR14 | manual, floor trial, gates signed (HV-5, HV-7) | box on the line at 40 cm | brake, scan (servo sweeps), six actions, line reacquired; `EVT BYPASS` steps 0..11 on the wire; T_total logged for NFR14 |
| INT-12 | remote START/STOP | FR23 | manual, broker | `telem_decode.py --encode-cmd start N`, publish to `is09/is09-car/cmd` in S1; `stop` while running | S2 on START (only from S1); `S7 cause=OPERATOR_STOP` on STOP |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-27 | INT-03 | Phase 7 commit | Claude (for Buddy 4, integration) | PASS 74/0 | `docs/evidence/phase7_host_tests.log` | simulated car: executor with the Rev B 6.2 acceptance rule (drive commands only from BRAKED), scanner, latch, buttons |
| | INT-04..INT-12 | | team | NOT RUN | | need HV-3/HV-4 (motion armed), HV-5/HV-7 for INT-11, broker for INT-12 |
