# Requirement traceability (FR1-FR25, NFR1-NFR29)

Status vocabulary: PASS (with evidence link), FAIL, BLOCKED (reason),
NOT RUN, PARTIAL (what is missing). Host simulation and target build/boot
never prove physical performance; such rows stay NOT RUN until a bench or
course trial is logged. A `[TBC]` threshold is never a PASS.

Evidence kinds: H = host test (`testing/host`), T = on-target self-test
(`make TEST=1`, log in `testing/*/logs`), B = bench procedure
(`docs/HARDWARE_VALIDATION.md`), C = course trial.

| ID | Requirement (short) | Owner | Implementation | Test | Status (Phase 1) |
|---|---|---|---|---|---|
| FR1 | line following | B3 + B2 | `line_core.c` error/steer (H-LN-09..12) feeding `motion_core` FOLLOW (H-MO) | C | NOT RUN (host logic PASS; HV-10 polarity pending) |
| FR2 | line loss + bounded search | B3 + ctrl | LOST after T_LOST, FOUND on return (H-LN-19..23); search = Phase 7 | C | NOT RUN (host logic PASS) |
| FR3 | junction detection | B3 | both-dark >= T_JUNC, once, re-armed (H-LN-14..18) | C | NOT RUN (host logic PASS) |
| FR4 | barcode detection while moving | B3 | GP7 ISR -> `g_edge_ring` -> BarcodeDecodeTask (`task_barcode.c`), S3-U05 forced-edge path | T | NOT RUN (H-ER + H-BC PASS) |
| FR5 | decode A/B/C/D or invalid | B3 | `barcode_core_decode` Code 39, forward + reversed, 3 speeds (H-BC-01..08) | H + C | HOST PASS; target NOT RUN |
| FR6 | execute nav command at junction | ctrl + B2 | Phase 7 | C | NOT RUN |
| FR7 | invalid barcode: continue + report | B3 + ctrl | INV_NO_MATCH / INV_OVERFLOW published with seq + `APP_TEV_BARCODE` (H-BC-17, 20, 21); controller reaction = Phase 7 | C | NOT RUN (host PASS) |
| FR8 | move/turn/stop by encoders, signal completion | B2 | `subsystem2/motion_core.c` MOVE/TURN/BRAKE, `done_seq`+`EVT_MOTION_DONE` (H-MO-23..33 PASS on a simulated car) | H PASS, B (HV-3/4) + C NOT RUN | PARTIAL |
| FR9 | hump detection | B4 | `imu_core.c` FLAT->CLIMBING after theta_ON held 50 ms while moving and fresh (H-IM-16) | H + B | HOST PASS; bench NOT RUN (HV-12) |
| FR10 | peak height estimate | B4 | distance x sin(pitch) x K_CAL, peak over the episode, 127 mm saturation -> DEGRADED (H-IM-22, 34, 41) | H + B (HV-14) | HOST PASS; NFR15 NOT MEASURED |
| FR11 | retain run maximum | B4 | `max_height_mm` from NORMAL episodes (H-IM-23, 26, 27; D-17) | H | HOST PASS |
| FR12 | six motion classes + change reports | B4 | `classify()` STATIONARY/ACCELERATING/TURNING/CLIMBING/DESCENDING/IMPACT (+CRUISING), `APP_TEV_MOTION_CLASS` on change (H-IM-18, 19, 44..55) | H | HOST PASS (TURNING comes from MotionStatus, no gyro) |
| FR13 | IMU collision -> latched stop | B4 + shared | two >= 2.5 g spikes while moving (H-IM-43..49) -> `app_safe_stop_request(COLLISION)` first, then IMU_FAULT (`task_imu.c`, D-18/D-19) | H PASS, T NOT RUN, B (HV-13) NOT RUN | PARTIAL (threshold A9 unvalidated) |
| FR14 | detect an obstacle ahead | B5 | forward watch every 100 ms, two consecutive readings at/inside D_STOP -> EVT_OBSTACLE (H-SC-12..17) | H + B | HOST PASS; bench NOT RUN (HV-5) |
| FR15 | stop before the obstacle | B5 + ctrl | EVT_OBSTACLE -> controller BRAKE (Phase 7); NFR2 budget: 2 x 100 ms + 10 ms | C | NOT RUN |
| FR16 | coarse + fine profile: location, width, closest, side clearances | B5 | `profile_scan` (9 coarse, 5 deg fine window), `scan_core_build_profile` y_min/y_max/width/closest with beam correction (H-SC-21..33) | H + B (HV-15) | HOST PASS; NFR10/11 NOT MEASURED |
| FR17 | choose a bypass side | B5 + ctrl | `action` LEFT/RIGHT/REVERSE/STOP from the Rev B validity rules (D-21, H-SC-26..31); controller executes (Phase 7) | H | HOST PASS |
| FR18 | side ranging during the pass | B5 | REQ_SIDE_L/R, two readings inside D_SIDE_UNSAFE -> EVT_SIDE_UNSAFE (H-SC-34..37) | H | HOST PASS |
| FR19 | reacquire the line after the bypass | ctrl + B3 | Phase 7 (actions 5-6, until_line) | C | NOT RUN |
| FR20 | Wi-Fi + MQTT telemetry of all listed items | B1 | `s1_snapshot_build` from every blackboard word (H-S1-SNAP, S1-U05/U06 PASS on target 22 Oct), bridge `mqtt_utk.c` publishes `MBF_TELEM_OUT` (D-10) | H PASS, T PASS, bench B1 FAIL (stall) -> fix2 NOT RUN | PARTIAL (radio path unverified since fix2) |
| FR21 | periodic heartbeat/status | B1 | 1 s HB with conn_state, back-off, dropped (S1-U07 PASS on target; B1 1.00 Hz while flowing) | T PASS, bench NOT RUN since fix2 | PARTIAL |
| FR22 | auto reconnect while autonomy continues | B1 | `link_lost()` -> back-off 1-2-4-8-10 s -> reconnect; 5 s connect deadline; 5 s publish stall -> link lost (fix2 `fc3109f`) | bench B3/B4/B9 NOT RUN | NOT RUN (code complete, untested) |
| FR23 | command topic, START/STOP only | B1 + ctrl | bridge subscribes `is09/is09-car/cmd` -> `MBF_TELEM_IN` -> `s1_cmd_filter` (H-S1-CMD, S1-U08 PASS on target) -> controller guard | H PASS, T PASS, bench B5/B6 NOT RUN | PARTIAL |
| FR24 | local START run / STOP latched stop | ctrl | `vehicle_controller.c` button ISR + debounce + S7 | T (SH-U08 NOT RUN), B (HV-6) | PARTIAL |
| FR25 | brake + latch on stall / stale / timeouts, report cause | all | motion producers STALL/STALE_LINE/CTRL_HEARTBEAT/ENCODER (H-MO-36..41 PASS); controller timeouts Phase 7 | H PASS | PARTIAL |
| NFR1 | brake <= 20 ms after latch | B2 | `task_motion.c` brakes on the latch byte before any other work, 10 ms cycle (H-MO-20 same-cycle brake) | B (HV-9) | NOT RUN (design margin: 10 ms cycle + handler latency) |
| NFR2 | brake cmd <= 250 ms inside D_STOP | B5 + ctrl | watch tick 100 ms x 2 confirmations + controller tick 20 ms + motion cycle 10 ms = 230 ms worst case by design; clearance [TBC] | C | NOT RUN (design budget only) |
| NFR3 | line update every 10 ms | B3 | CYC_10MS wake slot LINE, `s3_line_cycle` (S3-U02, S3-U03) | T | NOT RUN |
| NFR4 | barcode edge stamp error <= 10 % | B3 | dispatcher stamp at 1 us, stored in 8 us units (0.27 % of a 3 ms narrow bar); dispatcher latency is the real error term | B (HV-11) | NOT RUN (A6) |
| NFR5 | US trigger > 60 ms, echo timeout 30 ms | B5 | 100 ms tick between rangings, `range_once` waits 35 ms max, EchoIsr OVERRUN above 30 ms (S5-U02f, S5-U04) | T | NOT RUN |
| NFR6 | 1 s heartbeat, 200 ms snapshot | B1 | drift-free cadence (H-S1-CAD, S1-U05/U07 PASS); B1 measured 5.00 / 1.00 Hz while frames flowed | T PASS, bench re-run pending | PARTIAL |
| NFR7-NFR9, NFR15, NFR19 | thresholds [TBC] | - | - | - | BLOCKED (threshold TBC) |
| NFR10-NFR14 | obstacle performance bands | B5 + ctrl | width/position from `scan_core` (HV-15), T_profile from `s5_last_scan_ms`, bypass timing Phase 7 | B + C | NOT RUN |
| NFR16 | CPU idle >= 30 %, stacks <= 75 % | integration | Phase 8 measurement | T | NOT RUN |
| NFR17 | network never changes control periods / sends never wait | B1 + shared | `app_ipc_telem_post` TMO_POL (H-SF-17, SH-U06 PASS), S1-U04/U09/U10 PASS on target (cyclic 100/s under load), bridge polls with TMO_POL only; gate: no app code calls lwIP/cyw43 | H PASS, T PASS, B8 NOT RUN | PARTIAL |
| NFR18 | reconnect back-off capped at 10 s | B1 | `MQTT_UTK_RETRY_MAX_MS 10000` in the bridge, `APP_BACKOFF_S_MAX` clamp (H-BB-22) | H PASS, bench B3 NOT RUN | PARTIAL |
| NFR20 | micro T-Kernel on Pico on Robo Pico | all | template kept, 4 profiles build (ledger) | build | PASS (build) |
| NFR21 | C with Pico C SDK | all | ledger section 0 | build | PASS (build) |
| NFR22 | electrical limits | integration | HV-1, HV-5, HV-8; no firmware proof possible | B | NOT RUN |
| NFR23 | one owning task per output/bus | all | single-writer words, dispatcher refuses double attach (SH-U03c) | review + T | PARTIAL |
| NFR24 | WPA2 AP + standard broker | B1 | Phase 6 | bench | NOT RUN |
| NFR25 | BARR-C | all | `tools/barr-c/check_app.sh` PASS on Phase 1 files; manual review `docs/BARR_C_REVIEW.md` | tool + review | PARTIAL (Phase 1 files) |
| NFR26 | template structure | all | folders, `subsystemN_*` hooks kept; `shared/` added one level down (compiled by the template rule) | review | PASS |
| NFR27 | repeatable subsystem tests with logs | all | `testing/host`, `app_shared_test.c`, TEST_LOG.md per folder | - | PARTIAL |
| NFR28 | minimum-bit fields documented | all | `app_data.h`, `docs/DATA_REPRESENTATION.md` | H-PK/H-BB PASS | PASS (design + host), target measure Phase 8 |
| NFR29 | atomic / critical-section access to shared packed data | all | single-word publishes, `app_shared_bits_*`, mutex for 2-word profile | H-SF-02/03 PASS, SH-U04 NOT RUN | PARTIAL |

## Code-review rule (R5, D-09)

Every change to `app_program/shared/app_data.h` is reviewed by the consuming
buddy and must update `docs/DATA_REPRESENTATION.md`, the host test and
`tools/telem_decode.py` in the same commit.
