# Subsystem 1 — test log

**Subsystem:** Wi-Fi / MQTT telemetry and remote commands (TelemetryCommsTask, priority 8)  ·  **Student:** Xavier Lam Jun Jie 2500887  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

Three levels, cheapest first. Each level only needs what is listed, so all of
them can run before any other subsystem exists:

| Level | Needs | Run with | Proves |
|---|---|---|---|
| Host (H-S1) | any PC with gcc | `make -C testing/subsystem1/host && ./testing/subsystem1/host/test_s1` | the logic: command filter, connection state, 200 ms / 1 s cadence, snapshot mapping, publish-request format |
| On-target (S1-U) | Pico W + USB console; no Wi-Fi, broker or other subsystem | `make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`, flash `_test.uf2` | the task on the real kernel: objects, loop, offline/online behaviour, non-blocking producers, command path to the controller, back-pressure, timing isolation |
| Bench (B) | Pico W, laptop broker on the same hotspot, `tools/telem_bench.py`; bridge patch applied | demo image `make SMP=0 NET=lwip MQTT=1 -j8` | the real radio path end to end: rates on the wire, byte budget, outages and recovery, commands from a laptop |

## Test cases

Define a test here before running it. Test IDs never change.

### Host (automatic)

| ID | What is tested | Requirement / interface | Expected result |
|---|---|---|---|
| H-S1-PUB-01..10 | publish request `[len][topic][payload]` packs and unpacks for every topic; oversize payload, bad topic and malformed requests rejected; topics are `is09/is09-car/{snap,evt,hb,obs,cmd}` | IF-NB, D-13 | all pass |
| H-S1-CON-01..10 | Wi-Fi + MQTT status maps to WIFI_CONNECTING / MQTT_CONNECTING / BACKOFF / ONLINE; WIFI_LOSS and BROKER_LOSS per state; both bits are in the diagnostic mask | Statechart 4, NFR17, D-08 | all pass |
| H-S1-CAD-01..04 | 60 s with a 20-30 ms loop gives 300 +-1 snapshots, gaps 170-230 ms; no drift across the 32-bit ms wrap; a 2 s stall gives one snapshot, not a burst | NFR6 | all pass |
| H-S1-CMD-01..15 | START rejected before ONLINE and inside the 3 s guard after every (re)connect; guard rejection does not consume the seq; replay rejected (also across reconnect); STOP accepted inside the guard; bad CRC, version, length (incl. truncated 8 B), type 0/3 rejected | FR23, D-02, R7 | all pass |
| H-S1-SNAP-01..08 | snapshot built from blackboard words carries every field (safety, motion, encoders, line, barcode, IMU, obstacle with 511 = no echo) and is 18 B | FR20, NFR28 | all pass |

### On-target (automatic, `subsystem1_test.c`)

The test detaches the real bridge and acts as it (reads MBF_TELEM_OUT, writes
MBF_TELEM_IN). It never sends STOP end to end, because that would latch S7
before the other suites run (STOP is H-S1-CMD-05/13 and bench B5).

| ID | What is tested | Requirement / interface | Expected result |
|---|---|---|---|
| S1-U01 | MBF_TELEM_EVT / OUT / IN exist with 4 / 43 / 8 B maximum message | IF-TE, IF-NB | PASS |
| S1-U02 | TelemetryCommsTask loop runs (20 ms requested; 20-30 ms on the 10 ms tick) | Rev B task table | 33-51 loops in 1 s (INFO prints the count) |
| S1-U03 | offline: comms_word WIFI_CONNECTING, WIFI_LOSS set, nothing published for 1 s | Statechart 4 | PASS |
| S1-U04 | 40 events posted while offline: no post takes more than 50 us; overflow counted | NFR17 | PASS (INFO prints the longest post) |
| S1-U05 | online: 10-12 snapshots in 2.1 s, sequence unbroken, every frame decodes; WIFI_LOSS / BROKER_LOSS cleared | NFR6, FR20 | PASS (INFO prints frame counts) |
| S1-U06 | values written into the line and IMU blackboards appear in the snapshot | FR20 | PASS |
| S1-U07 | heartbeat every 1 s with conn_state ONLINE; offline backlog (>= 32 events) drained; drop count reported | FR21, NFR6, Week 6 6.4 | PASS |
| S1-U08 | bad CRC, bad length, START inside the 3 s guard and replay are rejected; a valid START after the guard is accepted once and reaches the controller (which refuses it while motion is unarmed, so no latch) | FR23, D-02, R7 | PASS |
| S1-U09 | bridge stalled 500 ms: loop keeps running, OUT-full counted, all 10 marker events delivered afterwards (none lost between buffers) | NFR17 | PASS |
| S1-U10 | under telemetry load the 10 ms cyclic service still runs 100 +-2 times a second; loop body <= 2 ms | NFR17, A15 | PASS (INFO prints both) |

### Bench (manual, laptop broker + `tools/telem_bench.py`)

Needs the bridge change in `mqtt_utk.c` (agreed with the team and the
instructor, TEAM.md section 3), the demo image, `config/wifi_secrets.h`
pointing at the laptop, and the broker running. `telem_bench.py` uses only
the Python standard library.

| ID | What is tested | Requirement | Steps | Pass criterion |
|---|---|---|---|---|
| B1 | rates and integrity on the wire | NFR6, FR20, FR21 | `python3 tools/telem_bench.py --host <laptop IP> --secs 60` | B1a snapshot 5.0 +-0.1 Hz and B1b heartbeat 1.0 +-0.05 Hz (each over the time it flowed), B1c 0 decode errors, B1d 0 sequence gaps, B1e no silence > 1 s anywhere in the window (the tool prints PASS/FAIL) |
| B2 | message size (Week 13 item 3) | NFR28 | same 60 s run; record "payload total" B/s and frame sizes | snapshot 18 B, heartbeat 6 B, event 4 B, obstacle 10 B; record B/s next to the text baseline |
| B3 | broker outage and recovery | FR22, NFR17, NFR18 | stop the broker 30 s, restart it; watch console and heartbeat | console shows MQTT_CONNECTING / BACKOFF with delays 1, 2, 4, 8, 10 s (never > 10 s); ONLINE again within 10 s + connect time after restart; next heartbeat reports the drop count |
| B4 | Wi-Fi outage | FR22 | switch the hotspot off 30 s, then on | WIFI_CONNECTING while off, ONLINE again without a reset; no `[TEST]` / boot banner reprinted |
| B5 | remote commands | FR23, D-01, D-02, R7 | `--send start N` (fresh N), same again, `--send start M --bad-crc`, `--send start K --len 5`, `--send stop P` | first START: REMOTE_CMD then controller rejects "not armed"; repeat: REPLAY; bad CRC: BAD_CRC; length 5: BAD_LENGTH; STOP: snapshot shows S7_SAFE_STOP, cause OPERATOR_STOP |
| B6 | retained START after reconnect (Mosquitto only) | D-02 | `--send start N --retain`, then restart the car or the broker | START_GUARD rejection after each reconnect; the car never leaves S1 |
| B7 | soak (A3 MQTT spike) | A3, NFR16 | demo image publishing for 10 min with `--secs 600` | no reset, 0 decode errors, sequence gaps = 0 |
| B8 | timing isolation with real traffic | A15, NFR17 | test image while ONLINE to the broker; repeat S1-U10 and SH-U01 | cyclic 100 +-2 per s, SH-U04c still PASS |
| B9 | silent link loss (no FIN, no ACKs) | FR22, Statechart 4 | demo image ONLINE, serial console open, `telem_bench.py --secs 120` running; switch the **laptop's** Wi-Fi off for 20 s, then on | console prints `mqtt: publishing stalled for 5 s, link lost` within 7 s of switching off; back-off delays never > 10 s; ONLINE again within 16 s of Wi-Fi on; 10 s `telem:` line shows `stalls 1`; bench B1e longest silence <= outage + 16 s |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | H-S1 (62 checks) | Phase 6 commit | Claude (for Buddy 1) | PASS 62/0 | `make -C testing/subsystem1/host` | gcc 13.3, -std=c99 -pedantic; cadence 60 s -> 300 snapshots, gaps 173-228 ms |
| 2026-10-22 | pipeline dry run | Phase 6 commit | Claude (for Buddy 1) | PASS | host simulation | firmware encoders -> mqtt_broker.py -> telem_bench.py: 5.04 Hz snapshots, 0.99 Hz heartbeat, 0 decode errors, 0 gaps; command frames reached the car topic byte-exact. Not a hardware result. |
| 2026-10-22 | S1-U01..U10 (run 1) | `bbcaa13` | Xavier | FAIL 7/3 | `logs/2026-10-22_test_run1.txt` | S1-U07 FAIL: the drop report was posted into MBF_TELEM_EVT, which S1-U04 had just filled, so it was itself dropped. The other two failures were among S1-U01..U04, printed before the console was open: not identified. |
| 2026-10-22 | fix | `3f3cdd6` | Claude (for Buddy 1) | - | commit message | Reconnect reports go straight to MBF_TELEM_OUT; TEST image waits for the terminal; S1 tests wait for radio init; new INFO lines (loop gap, U03, U07). |
| 2026-10-22 | S1-U01..U10 (run 2) | `3f3cdd6` | Xavier | PASS 10/0 | `logs/2026-10-22_test_run2.txt` | Measured: loop 34 per s (about 29 ms: tk_dly_tsk(20) on the 10 ms tick, not the 20 ms in Rev B); longest gap between loops 30.4 ms incl. radio start-up; longest event post 37 us; cyclic 100 per s and telemetry loop 462 us under load. Run-1 failures in U01..U04 not reproduced, but start-up conditions changed (terminal open, radio up), so not proven fixed. |
| 2026-10-22 | B1 (run 1) | `77fea7a` on hold/mqtt-bridge (bridge not in this branch) | Xavier | FAIL | `logs/2026-10-22_bench_b1_run1.txt` | 244 snapshots + 49 heartbeats, all decoded, 0 sequence gaps, 5.00 / 1.00 Hz while flowing; then the car stopped publishing after 48.8 s with the session open and a silent console. Bridge only counted failed publishes; lwIP drops a silent session after 90 s. Fix written (fix2), not applied or tested. |
| 2026-10-23 | merge | `10ca7c8` + fix2 merge | Claude (for Buddy 4, integration) | BUILD PASS | `docs/evidence/phase6_*` | Bridge `5b60564` + fix2 `fc3109f` merged into the unified branch with Phases 2-5; strong bridge symbols confirmed in the demo image; 62 host checks PASS; gate PASS. No hardware here: S1-U01..U10 on the merged image and B1 (fix2) NOT RUN. |
| | B1 re-run (fix2), B2..B9 | | Xavier | NOT RUN | | run B1 against **Mosquitto** first (D-24: the laptop `mqtt_broker.py` is a stall suspect) |
| 2026-10-26 | S1-U06 interaction | `b0605d1`, `8af4bdc` | Buddy 3, Buddy 4 (reported) | FAIL (test design) | teammate commit messages | S1-U06 writes patterns into `g_line_state` / `g_imu_state` and expects them in the snapshot; once LineSensingTask and ImuTask run they overwrite both words every 10 ms, so the pattern never reaches the snapshot. Not a firmware bug. **Action (Xavier):** rewrite S1-U06 to compare the snapshot against the live blackboard words read at the same instant, or disarm the two producer tasks for the test. Open until re-run on the unified branch. |
| 2026-10-27 | merge | `c3920d3` | Claude (for Buddy 4, integration) | BUILD PASS | `docs/evidence/merge_*` | Teammate branches converged on the unified branch; 62 host checks PASS; gate PASS. S1-U01..U10 and B1..B9 on the converged image NOT RUN. |
