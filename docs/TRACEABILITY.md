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
| FR1 | line following | B3 + B2 | Phase 3 + Phase 2 | C | NOT RUN |
| FR2 | line loss + bounded search | B3 + ctrl | Phase 3 + Phase 7 | C | NOT RUN |
| FR3 | junction detection | B3 | Phase 3 | C | NOT RUN |
| FR4 | barcode detection while moving | B3 | Phase 3 (`g_edge_ring`, dispatcher GP7) | C | NOT RUN (ring + dispatcher: H-ER PASS) |
| FR5 | decode A/B/C/D or invalid | B3 | Phase 3 (Code 39, measured symbology) | H + C | NOT RUN |
| FR6 | execute nav command at junction | ctrl + B2 | Phase 7 | C | NOT RUN |
| FR7 | invalid barcode: continue + report | B3 + ctrl | Phase 3/7; event `APP_TEV_BARCODE` exists | C | NOT RUN |
| FR8 | move/turn/stop by encoders, signal completion | B2 | Phase 2; contract: `MotionCmd`, `done_seq`, `EVT_MOTION_DONE` (H-BB-09..11 PASS) | B + C | NOT RUN |
| FR9-FR12 | hump detect / peak / run max / motion class | B4 | Phase 4; `ImuState` word (H-BB-12..14 PASS) | B + C | NOT RUN |
| FR13 | IMU collision -> latched stop | B4 + shared | `app_safe_stop_request(COLLISION)` path (H-SF-04..07, SH-U08) | H PASS, T NOT RUN, C NOT RUN | PARTIAL |
| FR14-FR19 | obstacle detect/stop/profile/avoid/reacquire | B5 + ctrl | Phase 5/7; `ObstacleProfile` words (H-BB-15..19 PASS) | B + C | NOT RUN |
| FR20 | Wi-Fi + MQTT telemetry of all listed items | B1 | Phase 6; codecs SNAP/EVT/OBS (H-TF PASS, Python parity PASS) | H PASS, bench NOT RUN | PARTIAL (obstacle summary via OBS frame, D-06 pending ratification) |
| FR21 | periodic heartbeat/status | B1 | Phase 6; HB frame codec (H-TF-08 PASS) | NOT RUN | PARTIAL (frame defined under D-06) |
| FR22 | auto reconnect while autonomy continues | B1 | Phase 6 | NOT RUN | NOT RUN |
| FR23 | command topic, START/STOP only | B1 + ctrl | `app_cmd_frame_decode` + `remote_cmd()` guard (H-CMD-01..10 PASS) | H PASS | PARTIAL |
| FR24 | local START run / STOP latched stop | ctrl | `vehicle_controller.c` button ISR + debounce + S7 | T (SH-U08 NOT RUN), B (HV-6) | PARTIAL |
| FR25 | brake + latch on stall / stale / timeouts, report cause | all | cause codes + `app_safe_stop_request`; producers in Phases 2-7 | H PASS (contract) | PARTIAL |
| NFR1 | brake <= 20 ms after latch | B2 | Phase 2 motion cycle; latch check first | B (HV-9) | NOT RUN |
| NFR2 | brake cmd <= 250 ms inside D_STOP | B5 + ctrl | Phase 5/7 | C | NOT RUN ([TBC] clearance) |
| NFR3 | line update every 10 ms | B3 | CYC_10MS wake slot LINE (SH-U01) | T | NOT RUN |
| NFR4 | barcode edge stamp error <= 10 % | B3 | 1 us timer, 8 us units (ledger S6 analysis) | B | NOT RUN (A6) |
| NFR5 | US trigger > 60 ms, echo timeout 30 ms | B5 | `APP_CAL_T_US_TRIGGER_PERIOD_MS`, `APP_ECHO_WIDTH_MAX_US` | T | NOT RUN |
| NFR6 | 1 s heartbeat, 200 ms snapshot | B1 | Phase 6 | bench | NOT RUN |
| NFR7-NFR9, NFR15, NFR19 | thresholds [TBC] | - | - | - | BLOCKED (threshold TBC) |
| NFR10-NFR14 | obstacle performance bands | B5 | Phase 5 | C | NOT RUN |
| NFR16 | CPU idle >= 30 %, stacks <= 75 % | integration | Phase 8 measurement | T | NOT RUN |
| NFR17 | network never changes control periods / sends never wait | B1 + shared | `app_ipc_telem_post` TMO_POL (H-SF-17, SH-U06) | H PASS, T NOT RUN | PARTIAL |
| NFR18 | reconnect back-off capped at 10 s | B1 | `APP_BACKOFF_S_MAX`, comms_word clamp (H-BB-22 PASS) | H PASS | PARTIAL |
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
