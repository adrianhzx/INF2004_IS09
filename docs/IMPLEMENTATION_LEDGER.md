# IS09 Implementation Ledger

Durable record of the Rev B / Tribunal-1 Version B implementation. Every phase
appends to this file. Nothing here is a claim of hardware behaviour unless the
row says MEASURED and names the log.

## 0. Baseline

| Item | Value |
|---|---|
| Repository | https://github.com/adrianhzx/INF2004_IS09 |
| Baseline commit (main) | `c3b0954` "Create object directories before compiling" |
| Working branch | `feat/revb-autonomous-integration` (branched from `c3b0954`) |
| Toolchain used for evidence | arm-none-eabi-gcc 13.2.1 (Ubuntu 15:13.2.rel1-2), GNU make 4.3 |
| Pico SDK | 2.2.0, submodules `lib/tinyusb`, `lib/cyw43-driver`, `lib/lwip` |
| Host tools | gcc (host), python3 3.13, clang-format 18.1.3, clang-tidy 18.1.3 |
| Adopted design | `Week_6_Design_Review_RevB.docx` (21 Oct 2026) + `IS09_ImmTri1.docx` Version B |
| Not adopted | Rev B "Open review points (Tribunal 1 Version C)"; `IS09_ImmTri1.docx` Version C is blank |

### Baseline builds (unchanged template, real output in `docs/evidence/`)

| Profile | Result | text | data | bss | Log |
|---|---|---|---|---|---|
| `make -j8` (SMP=1 usb_cdc wifi) | PASS, `.uf2` produced, 0 warnings | 285184 | 88 | 24476 | `baseline_build_smp1_default.log` |
| `make SMP=0 -j8` | PASS, `.uf2` produced | 277008 | 76 | 20344 | `baseline_build_smp0.log` |
| `make SMP=0 NET=lwip MQTT=1 TEST=1 -j8` | PASS, `.uf2` produced | 333212 | 84 | 78116 | `baseline_build_smp0_lwip_mqtt_test.log` |

`bss` excludes task stacks and kernel objects: those come from the kernel's
dynamic system area (`knl_Imalloc`), not from `.bss`. RAM evidence in Phase 8
must therefore report `.data`, `.bss`, and the system-area high-water mark
separately (the Tribunal method).

`v-prepack` tag: **NOT CREATED.** No authentic unpacked implementation exists in
the repository (the template contains only the LED/producer/consumer demo), so
tagging anything as a pre-packing baseline would be fabrication. The Week 13
"before" figure will be reported as a reasoned estimate (see
`docs/DATA_REPRESENTATION.md`, section "Estimated unpacked baseline") and the
first real measurement is of the packed implementation.

## 1. Source inventory and source-of-truth register

| # | Source | Status | Used for |
|---|---|---|---|
| S1 | `Week_6_Design_Review_RevB.docx` | ACCEPTED (primary) | FR1-25, NFR1-29, pin map, 7 tasks/priorities, S0-S7, data table 3.1, SafeStop contract, six-action bypass |
| S2 | `IS09_ImmTri1.docx` Version B | ACCEPTED | bit-width rationale, helper semantics, MBF record formats, test plan T1-T8, risk register R1-R10 |
| S3 | `IS09_ImmTri1.docx` Version C | EMPTY in the supplied file | nothing; the Rev B open-points list is the only Version C content |
| S4 | `Project Write-up (2).pdf` | ACCEPTED (mandatory) | micro T-Kernel, Pico C SDK, C, template repo, BARR-C; barcode A=Left B=Right C=Straight D=U-Turn (p.3) |
| S5 | `Barcode & Line Specification (2).pdf` | ACCEPTED (measured drawing) | 18 mm line, 20 mm gap, ~141 mm barcode |
| S6 | `Barcode Sample (2).pdf` (4 vector pages) | MEASURED (see below) | symbology and bar ratios |
| S7 | `Robo Pico Board Datasheet (1).pdf` | ACCEPTED (hardware) | M1A/M1B=GP8/GP9, M2A/M2B=GP10/GP11, truth table, 20 kHz PWM, buttons GP20/21, buzzer GP22, NeoPixel GP18, servos GP12-15, Grove map, VBAT/2 on GP28 via solder jumper, 1 A/1.5 A, 300 mA 3V3, input high max 3.6 V |
| S8 | `HCSR04 (1).pdf` | ACCEPTED | 10 us trigger, 5 V supply, >60 ms cycle, 58 us/cm |
| S9 | `lsm303dlhc (1).pdf` | ACCEPTED | accel SAD 0011001b = 0x19, CTRL_REG1_A 0x20, OUT_X_L_A 0x28, HR mode, no accelerometer WHO_AM_I (identity via magnetometer IRA/IRB/IRC) |
| S10 | Live repo `c3b0954` | ACCEPTED (build/API truth) | everything in section 2 |
| S11 | BARR-C:2018 kit `netrinomike/barr-c` (MIT) | ACCEPTED (tooling) | `.clang-format`, `.clang-tidy`, `check.sh`, rule coverage |
| S12 | `Week_6_Design_Review_Template (2).docx`, `IS09_Pre-Tribunal1.docx`, `IS09.txt`, run sheet | HISTORICAL | superseded by S1 where they differ |

### Barcode sample measurement (S6) - replaces any assumed A-D bit table

Extracted from the PDF vector rectangles (`python3 pdfplumber`, pages 1-4):

* Every page has 15 bars + 14 spaces = 29 elements, exactly two widths with
  ratio **3.00 : 1** (page 1: 19.82 / 59.44 pt; page 2: 9.95 / 29.85 pt).
* Element sequence page 1 = `n w n n w n w n n | n | w n n n n w n n w | n | n w n n w n w n n`
  (`b s b s ...` interleaved) = Code 39 `*` , `A` , `*` with narrow
  inter-character gaps. Page 3 middle character = `n w w n w n n n n` = Code 39 `Z`.
* 9 wide + 20 narrow = 47 narrow units over ~141 mm -> **narrow ~3.0 mm,
  wide ~9.0 mm** (S5 "approx" - confirm with a ruler on the printed course).
* Conclusion: the course barcodes are **Code 39, 3:1, one character per
  barcode**, start/stop `*`. Pages are labelled A, A, Z, Z (two print scales),
  so **no sample of B, C or D exists**; the decoder implements the Code 39
  table (A-D and `*`) and reports NO_MATCH for anything else.
* Physical timing: at 20 cm/s a 3 mm narrow bar is 15 ms, a 9 mm wide bar
  45 ms. 8 us EdgeRing units are 0.05 % of the narrow transit (NFR4 asks
  <= 10 %), and the 15-bit delta (262 ms) exceeds T_QUIET (150 ms), so the
  adopted 16-bit entry is consistent with the measured drawing. **A6 (real
  DOUT edge quality) remains open.**

## 2. Template / kernel facts verified by reading the port (not inferred)

| Fact | Where | Consequence |
|---|---|---|
| `tk_set_flg`, `tk_wup_tsk`, `tk_sig_sem` have no `CHECK_INTSK`/`CHECK_DISPATCH`; `tk_snd_mbf(..., TMO_POL)` passes `CHECK_DISPATCH_POL` | `kernel/tkernel/eventflag.c:133`, `messagebuf.c:291`, `check.h:206-228` | **A3 answered for the API half:** flag set / wake-up / polled message send are legal from a handler registered with `tk_def_int(TA_HLNG)`. `tk_wai_*`, `tk_loc_mtx`, `tk_dly_tsk` are not. |
| Handler entry wraps `ENTER_TASK_INDEPENDENT`; `END_CRITICAL_SECTION` defers dispatch through PendSV (`knl_dispatch` = `ICSR_PENDSVSET`) | `kernel/sysdepend/cpu/core/armv6m/interrupt.c:43`, `cpu_cntl.c:193` | A kernel call inside an ISR never context-switches inside the ISR; safe. |
| `DI(x)`/`EI(x)` = `disint()`/`set_primask()` (PRIMASK). No `save_and_disable_interrupts()` is linked (Pico SDK runtime is not part of this port) | `include/tk/sysdepend/cpu/core/armv6m/syslib.h` | **A3 helper question answered:** `shared_bits_set/clr` use `DI()/EI()` on SMP=0. On SMP=1 PRIMASK does not exclude core 1; the port offers `ISpinLock`/`atomic_bitset` (`include/tk/smp.h`, SIO spinlock + PRIMASK). |
| Vector table has `N_INTVEC = 32`; `EnableInt(intno, level)` with 2-bit priority, `INTPRI_MAX_EXTINT_PRI = 1` is the highest external level | `include/sys/sysdepend/cpu/rp2040/sysdef.h:93-117`, `lib/libtk/.../int_rp2040.c` | IO_IRQ_BANK0 = IRQ 13 (RP2040 datasheet 2.3.2) is registered with `tk_def_int` at level 1. |
| `CRASH_REPORT=0` defines `BOARD_UART0_FREE=1`, which removes the GP0/GP1 UART funcsel rows from `pinfnc_tbl`; the serial device driver `dev_init_ser(0)` touches no pin registers | `build_make/pico_rp2040.mk`, `kernel/sysdepend/pico_rp2040/hw_setting.c:93`, `device/ser/sysdepend/rp2040/ser_rp2040.c` | **GP0/GP1 encoders are safe with the default `CONSOLE=usb_cdc CRASH_REPORT=0`.** `CRASH_REPORT=1` or `CONSOLE=uart` must never be built while encoders are wired. The stale comment in `pico_rp2040.mk` ("usb_cdc still mirrors every line to UART0") is contradicted by the code; the code wins. |
| PWM block reset is released only `#if USE_PTMR`, and `USE_PTMR = 0` | `hw_setting.c:49`, `config/config.h` | The application must release `RESETS_RESET_PWM` before any motor/servo/buzzer PWM write (done once in `app_hw_init`). |
| TIMER block reset is never released and the watchdog tick generator is never started by the kernel; the Wi-Fi shim reads `timerawl` regardless | grep of `kernel/`, `lib/` for `RESET_TIMER`/`WATCHDOG` | `app_hw_init` releases `RESETS_RESET_TIMER`, writes `WATCHDOG.TICK = ENABLE | 12`, then **self-checks that the counter advances**; failure is a CRITICAL_RUNTIME fault and motion stays disarmed (fail closed). Bench item HV-2. |
| ADC reset released and `dev_init_adc(0)` runs at boot (`DEVCNF_USE_ADC = 1`), no pins claimed; I2C0 reset released, kernel I2C driver off (`DEVCNF_USE_IIC = 0`, default pins would be GP8/GP9 = motor pins) | `hw_setting.c`, `config/config_device.h`, `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h` | Kernel I2C driver must stay OFF. ADC ownership decision deferred to Phase 3 (kernel device vs. task-owned polled conversion). |
| Message buffer stores a 4-byte `INT` header per message and rounds each message to 4 bytes | `kernel/tkernel/messagebuf.h:58-62`, `messagebuf.c:65` | Open point (g) resolved: 4-byte records cost 8 bytes each. `MBF_MOTION_CMD` depth 4 = **32 B**, `MBF_TELEM_EVT` depth 32 = **256 B** (not 16 B / 128 B as estimated). |
| Mutex supports `TA_INHERIT` | `include/tk/syscall.h:93` | ObstacleProfile mutex is created with `TA_INHERIT`. |
| System tick `CNF_TIMER_PERIOD = 10` ms | `config/config.h` | A 10 ms cyclic handler is exactly one tick; 20 ms controller tick = every second call. |
| Task stacks are kernel-allocated (`TASK_STACK_SIZE 2048`), `tk_ref_tsk` available; no stack painting for application tasks | `app_main.c`, `config_func.h` | Stack high-water needs a painted user buffer (`TA_USERBUF`) - Phase 8. |
| MQTT client lives inside the kernel Wi-Fi service task `cyw43_utk_task` (priority 4, core 0); `mqtt_utk.c` polls `sys_check_timeouts()`; TEAM.md forbids any student task calling `cyw43_*`/lwIP/MQTT | `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `TEAM.md` section 3 | **Conflict with Rev B "TelemetryCommsTask owns the radio".** Resolution recorded as D-10 in `docs/DECISION_REGISTER.md`: TelemetryCommsTask (priority 8) owns framing, connection-state bookkeeping, command validation and the queues; the kernel service task does the socket I/O by polling an application message buffer with `TMO_POL`, exactly as TEAM.md prescribes. |
| `.clang-format` kit needs clang-format >= 16; installed 18.1.3 | kit README | OK. |
| Template file names `subsystem1.c`..`subsystem5.c` share their first 8 characters | template, BARR-C 4.1.b | Documented deviation DEV-1 (template structure NFR26 outranks a naming rule). |

## 3. Gap list (ranked)

| Sev | ID | Gap | Disposition |
|---|---|---|---|
| BLOCKER | G1 | HC-SR04 ECHO level shifting / 5 V supply unverified (A12, R3) | Hardware checklist HV-5; firmware keeps `obstacle` ranging disabled until `APP_CAL_ECHO_SHIFTER_VERIFIED` is set by the team after measurement. |
| BLOCKER | G2 | No measured V_NOM, ticks/rev, wheel geometry, D_STOP, T_HOLD, theta_ON... (all `[TBC]`) | All are calibration constants in `app_program/shared/app_cal.h` with conservative defaults and `APP_CAL_MOTION_ARMED = 0`; motors cannot drive until the team sets it after HV-3/HV-4. |
| BLOCKER | G3 | Default build profile is `SMP=1`; the adopted helper is single-core | Demo profile fixed to `SMP=0` (ledger section 4). SMP=1 still compiles with the spinlock variant but is NOT the validated image. |
| MAJOR | G4 | TelemetryCommsTask cannot own the radio in this port (see section 2) | D-10. |
| MAJOR | G5 | 18-byte snapshot omits obstacle profile; no heartbeat frame (FR20/FR21) | D-06: separate versioned frames on separate topics, decoder + tests; needs ratification. |
| MAJOR | G6 | MBF sizes in Rev B ignore the 4-byte header | Fixed in `app_ipc.h` with the measured header size. |
| MAJOR | G7 | TIMER/PWM blocks held in reset by the kernel | `app_hw_init` + boot self-check. |
| MAJOR | G8 | Remote STOP cause / replay / post-reconnect START guard unadopted | D-01, D-02. |
| MAJOR | G9 | MotionStatus two words, no common seq | D-04 (2-bit cycle tag in reserved bits). |
| MINOR | G10 | 8-bit event timestamp wraps at 256 ms | D-05. |
| MINOR | G11 | ObstacleProfile bit sum is 50, not 49 | Table corrected in `docs/DATA_REPRESENTATION.md`. |
| MINOR | G12 | Integration owner unnamed | D-09, TEAM.md row left for the team. |
| MINOR | G13 | Robo Pico button polarity (pull-up/active-low) not in the datasheet text | HV-6; firmware default active-low with internal pull-up, `APP_CAL_BUTTON_ACTIVE_LOW`. |

## 4. Build profile for the demo image

```
cd build_make
export PICO_SDK_PATH=/path/to/pico-sdk        # 2.2.0, submodules tinyusb cyw43-driver lwip
make SMP=0 NET=lwip MQTT=1 -j8                 # demo image (single core, Wi-Fi + MQTT)
make SMP=0 NET=lwip MQTT=1 TEST=1 -j8          # on-target self-test image
make SMP=0 -j8                                 # radio-less control-only image
```
Never add `CRASH_REPORT=1` or `CONSOLE=uart` while encoders occupy GP0/GP1.
`config/wifi_secrets.h` is gitignored; only the example is committed.

## 5. Phase status

| Phase | Status | Commit |
|---|---|---|
| 0 Audit | DONE (this file, sections 0-4) | - |
| 1 Shared foundation | DONE: host-verified; on-target SH-U01..U08 PASS 16/0 on 22 Oct after the SH-U04c SRAM fix (`0aafbaa`); HV bench items NOT RUN | section 6, testing/shared/TEST_LOG.md |
| 2 Motion | DONE (host-verified, target-built, bench NOT RUN, disarmed) | see section 7 |
| 3 Line + Barcode | DONE (host-verified, target-built, bench NOT RUN) | see section 8 |
| 4 IMU | DONE (host-verified, target-built, bench NOT RUN) | see section 9 |
| 5 Obstacle | DONE (host-verified, target-built, bench NOT RUN, outputs gated) | see section 10 |
| 6 Wi-Fi/MQTT | IN PROGRESS (Buddy 1 branch merged 23 Oct): TelemetryCommsTask + host tests (62 PASS); on-target S1-U01..U10 PASS 10/0 on 22 Oct (`3f3cdd6`). Radio bridge in `mqtt_utk.c` (D-10) still NOT in this branch: bench B1 on the held bridge delivered every frame at the right rate, then stalled silently after 49 s; fix written there, not yet tested | testing/subsystem1/TEST_LOG.md |
| 7 Integration | NOT STARTED | |
| 8 Evidence | NOT STARTED | |

## 6. Phase 1 record (shared interface, codecs, safety foundation)

### Files written (all BARR-C formatted, `tools/barr-c/check_app.sh` PASS)

| Path | Role |
|---|---|
| `app_program/shared/app_data.h` | single authority: every field position/width/unit/sentinel, enums, static-assert guard |
| `app_program/shared/app_pack.h` | unsigned mask/shift helpers, signed encode/decode without UB, saturation flags, LE byte helpers |
| `app_program/shared/app_cal.h` | every `[TBC]` constant + arming switches (`APP_CAL_MOTION_ARMED = 0`, `APP_CAL_ECHO_SHIFTER_VERIFIED = 0`) |
| `app_program/shared/app_hw.[ch]` | pin map, GPIO/IRQ/PWM/timer helpers, TIMER+PWM reset release, watchdog tick, boot self-check |
| `app_program/shared/app_gpio_irq.[ch]` | the one IO_IRQ_BANK0 dispatcher (`tk_def_int`, level 1), one callback per pin, refuses overwrite |
| `app_program/shared/app_safety.[ch]` | STOP_LATCH byte, safety word (first cause wins), fault mask, `app_shared_bits_set/clr`, controller-only clear |
| `app_program/shared/app_ipc.[ch]` | priorities, stacks, FLG bit maps, MBF sizing with the measured 4 B header, CYC_10MS (10/20/100 ms), non-blocking telemetry post |
| `app_program/shared/app_motion.h`, `app_line.h`, `app_barcode.h`, `app_imu.h`, `app_obstacle.h` | typed pack/unpack + publish/read per blackboard; EdgeRing SPSC |
| `app_program/shared/app_telemetry.[ch]` | SNAP 18 B / EVT / HB / OBS / CMD codecs, CRC-8, MissionStatus, comms_word, RemoteCmd |
| `app_program/shared/app_blackboard.c` | aligned storage with failsafe initial values, size guards |
| `app_program/shared/vehicle_controller.[ch]` | VehicleControllerTask: S0->S1, START debounce, STOP ISR -> SafeStop, S7 guard, heartbeat, motion sender (S2-S6 = Phase 7) |
| `app_program/shared/app_shared_test.c` | on-target A3 spikes SH-U01..08 (forced GPIO IRQ, 10 000-iteration race, cyclic rate) |
| `app_program/app.h`, `app_main.c` | template entry reformatted; bring-up order hw -> safety -> ipc -> irq -> inits -> controller -> starts -> cyclic last |
| `app_program/subsystem1..5/*` | motor-safe placeholders creating only their owned objects (MBF_TELEM_EVT 256 B, MBF_MOTION_CMD 32 B, FLG_BARCODE, FLG_SCAN, MTX_OBSTACLE TA_INHERIT) |
| `build_make/mtkernel_3/app_program/subdir.mk` | `APP_CFLAGS` compiler gates for application sources only |
| `testing/host/*` | 418-check host suite + kernel stubs + shared vectors |
| `tools/telem_decode.py` | laptop decoder/encoder with `--selftest` parity against the C vectors |
| `tools/barr-c/*`, `.clang-format`, `.clang-tidy` | kit at commit 75bcc16 (MIT) + `check_app.sh` |
| `docs/*` | this ledger, DECISION_REGISTER, DATA_REPRESENTATION, HARDWARE_VALIDATION, TRACEABILITY, BARR_C_REVIEW |

### Validation (what was actually run here)

| Check | Command | Result |
|---|---|---|
| Demo image | `make SMP=0 NET=lwip MQTT=1 -j8` | PASS, 0 app warnings, text 326200 / data 84 / bss 78432, `.uf2` produced |
| Test image | `make SMP=0 NET=lwip MQTT=1 TEST=1 -j8` | PASS, text 329484 / 84 / 78456 |
| Radio-less | `make SMP=0 -j8` | PASS, text 271684 / 76 / 20660 |
| SMP=1 variant (spinlock helper path compiles) | `make SMP=1 NET=lwip MQTT=1 TEST=1 -j8` | PASS (not the validated profile) |
| Host unit tests | `make -C testing/host && ./testing/host/test_shared` | 418 passed, 0 failed |
| Decoder parity | `python3 tools/telem_decode.py --selftest testing/host/vectors.txt` | 7 vectors ok |
| BARR-C gate | `tools/barr-c/check_app.sh` | PASS (format, lexical, naming, clang-tidy 0 findings) |
| On-target SH-U01..08 | flash `_test.uf2` | **NOT RUN** (no hardware in this session) |
| NFR1 brake latency, any bench item HV-1..9 | - | **NOT RUN** |

### Status statements (honest)

* BUILD PASS (4 profiles). HOST TEST PASS. TARGET BOOT NOT RUN. HARDWARE NFR1 NOT RUN. INTEGRATED AUTONOMY NOT VERIFIED.
* Motion is disarmed by configuration; no code path can drive GP8-GP11 in this phase.
* `v-prepack` not tagged (no authentic baseline, section 0).

### Next executable step

PHASE 2 (subsystem2, Buddy 2): `app_program/subsystem2/` - Robo Pico PWM
motor driver (slices 4/5, 20 kHz, brake = both low), quadrature EncoderIsr on
GP0-GP3 through `app_gpio_irq_attach`, 10 ms MotionControlTask that checks
`g_stop_latch` first, PID (integer), MOVE/TURN/FOLLOW/BRAKE, stall/stale/
heartbeat faults, `S2-U` on-target tests and host tests for the PID/odometry
maths. Gate: build + host + review, then STOP for team bench items HV-3/HV-4.

## 7. Phase 2 record (subsystem2 - motion and encoders, Buddy 2)

### Files written

| Path | Role |
|---|---|
| `app_program/subsystem2/motion_core.[ch]` | pure executor: BRAKED/FOLLOWING/MOVING/TURNING/STOP_LATCHED, latch-first ordering, heartbeat/stale-line/stall/glitch guards, one command per cycle, Q8 speed PID + feed-forward, straightness correction, pivot turns with `until_line`, wrap-safe signed odometry, status view with cycle tag |
| `app_program/subsystem2/drv_motor.[ch]` | Robo Pico PWM: slices 4/5 at 20 kHz (TOP 6249), brake = both compares 0, pins routed to PWM only after compares are 0, forward = A pulsed / B low |
| `app_program/subsystem2/drv_encoder.[ch]` | 4x quadrature table decode on GP0-GP3 through the dispatcher; ISR exports `g_enc_ticks_l/r` (uint16, +1/-1 wrap), per-wheel edge interval (uint16, 32 us units, saturating), saturating glitch byte |
| `app_program/subsystem2/task_motion.c` | 10 ms task: latch byte load first -> brake -> read ISR words -> dequeue one record (TMO_POL) -> core -> PWM -> flush -> SafeStop/fault -> publish two words + events |
| `app_program/subsystem2/subsystem2.[ch]`, `subsystem2_test.c` | MBF_MOTION_CMD (32 B), driver bring-up in brake, encoder attach, task start, S2-U01..06 |
| `app_program/shared/app_cal.h` | motion gains/limits (all `[TBC]`), `APP_CAL_ENC_STOP_MS`, `APP_CAL_ENC_GLITCH_MAX` |
| `testing/host/test_motion.c` | 56 checks with a simulated car |

### Decisions recorded

* Speed is measured from the inter-edge interval (A4), not ticks per cycle:
  with the provisional 20 ticks/rev and 65 mm wheel one tick is ~10.2 mm, so
  at 20 cm/s only ~0.2 ticks arrive per 10 ms. A5 risk stands: a 90 deg
  pivot is ~11 ticks; `until_line` is the real end condition on the track.
* Stall is counted only when the applied |PWM| is >= `APP_CAL_PWM_STALL_PCT`
  (a controller that legitimately drives 0 % is not a stall).
* A core-detected hazard brakes in the same cycle and the task then calls
  `app_safe_stop_request(cause)`; the latch propagates to every other task
  through the normal path. Guarded recovery leaves STOP_LATCHED only to BRAKED.
* `turn_ticks` limits the angle to 360 deg per command (uint32 arithmetic).

### Validation

| Check | Result |
|---|---|
| `make SMP=0 NET=lwip MQTT=1 -j8` | PASS 0 app warnings, text 330680 / data 124 / bss 78532 |
| `make SMP=0 NET=lwip MQTT=1 TEST=1 -j8` | PASS, 335812 / 124 / 78552 |
| `make SMP=0 -j8`, `make SMP=1 NET=lwip MQTT=1 TEST=1 -j8` | PASS |
| `./testing/host/test_motion` | 56 passed, 0 failed |
| `./testing/host/test_shared` (regression) | 418 passed, 0 failed |
| `tools/barr-c/check_app.sh` | PASS (subsystem2 sources and headers added to clang-tidy) |
| Linked sizes | `g_s2_core` 64 B (task-private), ISR words 2+2+1 B + 2x20 B wheel state, `docs/evidence/phase2_sizes.txt` |
| On-target S2-U01..06, HV-3/4/9, NFR1, NFR8/NFR9 accuracy | **NOT RUN** |

Status: BUILD PASS, HOST TEST PASS, TARGET NOT RUN, NFR1 NOT MEASURED, motion
still disarmed (`APP_CAL_MOTION_ARMED = 0`). Next: PHASE 3 (subsystem3, line
sensing + barcode).

## 8. Phase 3 record (subsystem3 - line sensing and barcode, Buddy 3)

### Files written

| Path | Role |
|---|---|
| `app_program/subsystem3/line_core.[ch]` | pure line algorithm: running black/white references (expand at once, relax 1 LSB/cycle) with the IR_CONTRAST check, `line_error = n_centre - n_left` (+ = line right), Q8 PD steer clamped +-100, hysteretic JUNCTION (T_JUNC, once, re-armed) / LOST (T_LOST) / FOUND, continuous `on_line`, 4-bit seq |
| `app_program/subsystem3/barcode_core.[ch]` | pure Code 39 decoder: DISARMED / WAIT_QUIET / CAPTURING, glitch (< T_GLITCH) folded into the running element, quiet zone or >= T_QUIET gap ends a capture, < N_MIN elements = stray (silent), min/max threshold with a 1.5..6.0 wide:narrow ratio check, forward then reversed pattern match, '*' at both ends, narrow inter-character gaps, A/B/C/D -> LEFT/RIGHT/STRAIGHT/UTURN, Z and anything else -> INV_NO_MATCH, 65th element or ring overflow -> INV_OVERFLOW |
| `app_program/subsystem3/drv_line_adc.[ch]` | task-owned polled one-shot ADC (START_ONCE, READY poll bounded, ERR check) on GP26/GP27; pads set to analogue |
| `app_program/subsystem3/task_line.c` | LineSensingTask, priority 4, woken by CYC_10MS slot LINE: two conversions (last sample held on error), core step, one aligned store of `g_line_state`, FLG_CTRL JUNCTION / LINE_LOST / LINE_FOUND + telemetry event, IR_CONTRAST fault set/clear (diagnostic) |
| `app_program/subsystem3/task_barcode.c` | BarcodeEdgeIsr via the shared dispatcher (stamp delta -> `app_edge_ring_push` -> `tk_set_flg`), BarcodeDecodeTask priority 7 waiting on FLG_BARCODE (EDGE_AVAIL / ARM / DISARM) with the T_QUIET timeout only while capturing; publishes `g_barcode_byte`, EVT_BARCODE, telemetry event, BARCODE_OVERFLOW fault; GP7 IRQ enabled only while armed |
| `app_program/subsystem3/subsystem3.[ch]`, `subsystem3_test.c` | FLG_BARCODE, ADC and ISR bring-up (DISARMED), two tasks, S3-U01..06 (barcode path driven by forced GP7 interrupts) |
| `config/config_device.h` | `DEVCNF_USE_ADC 0`: the kernel ADC device driver is off so subsystem3 owns the converter (TEAM.md section 3 decision closed) |
| `app_program/shared/app_cal.h` | `APP_CAL_IR_DARK_IS_HIGH`, `APP_CAL_LINE_*`, `APP_CAL_BC_*` (all `[TBC]` until HV-10/HV-11) |
| `testing/host/test_line.c` | 114 checks (line_core + barcode_core) |

### Decisions recorded

* ADC ownership: task-owned polled conversion, kernel `adca` driver OFF
  (D-16). Two one-shot conversions take ~4 us total at 48 MHz ADC clock; the
  kernel device would have added a device-open path and a driver task for no
  benefit, and `DEVCNF_USE_ADC` is a documented config switch, not a kernel
  source edit.
* Barcode element widths are kept in 8 us units (uint16) like the EdgeRing
  entry, not microseconds: a microsecond uint16 saturates at 65 ms, which a
  wide bar reaches below ~14 cm/s (found by the host test before commit).
* T_QUIET = 150 ms sets the slowest decodable speed at 6 cm/s (wide bar
  9 mm). Recorded in `testing/subsystem3/TEST_LOG.md` for the bench team.
* The ISR does not read the sensor level for decoding; elements alternate
  from the first edge after a quiet zone, so decoding is independent of the
  DOUT polarity (level is still stored in the ring entry for HV-11).
* Decoding is pattern-exact (all 29 elements must classify); no
  error-correcting guesswork, so an unreadable code is reported as
  INV_NO_MATCH (FR7) rather than mis-steering the car.

### Validation

| Check | Result |
|---|---|
| `make SMP=0 NET=lwip MQTT=1 -j8` | PASS 0 app warnings, text 332784 / data 124 / bss 78860 |
| `make SMP=0 NET=lwip MQTT=1 TEST=1 -j8` | PASS, 339792 / 124 / 78880 |
| `make SMP=0 -j8`, `make SMP=1 NET=lwip MQTT=1 TEST=1 -j8` | PASS, 0 warnings |
| `./testing/host/test_line` | 114 passed, 0 failed |
| `./testing/host/test_shared`, `./testing/host/test_motion` (regression) | 418 / 56 passed, 0 failed |
| `tools/barr-c/check_app.sh` | PASS (subsystem3 cores, drv_line_adc.h and test_line.c added to clang-tidy) |
| Linked sizes | `g_s3_line` 20 B, `g_s3_bc` 148 B (64 x 2 B widths + state), `g_edge_ring` 132 B, `g_line_state` 4 B, `g_barcode_byte` 1 B, `line_core_step` 732 B text, `docs/evidence/phase3_sizes.txt` |
| On-target S3-U01..06, HV-10, HV-11, NFR3, NFR4 | **NOT RUN** |

Status: BUILD PASS, HOST TEST PASS, TARGET NOT RUN, IR polarity and
references `[TBC]` (HV-10), barcode edge timing NOT MEASURED (HV-11). Next:
PHASE 4 (subsystem4, IMU).

## 9. Phase 4 record (subsystem4 - IMU, hump and collision, Buddy 4)

### Files written

| Path | Role |
|---|---|
| `app_program/subsystem4/imu_core.[ch]` | pure algorithm: axis map + signs, spike-rejecting EMA, integer atan2 pitch (0.5 deg), FLAT / CLIMBING / DESCENDING episodes with theta_ON confirm, T_ABORT false-trigger exit, D_HUMP_MAX / T_HUMP_MAX bounds, NORMAL / REJECTED / DEGRADED / INVALID terminals each with exactly one END(seq), height = sum(d x sin(pitch)) x K_CAL, peak and run maximum (D-17), 127 mm saturation -> DEGRADED, collision (D-18), six-class classifier with 30 ms hysteresis and 500 ms IMPACT hold |
| `app_program/subsystem4/drv_i2c.[ch]` | polled I2C0 master (DW_apb_i2c registers), 400 kHz timing as the Pico SDK derives it, every wait bounded by a 2 ms deadline, abort/timeout counters, RESTART reads |
| `app_program/subsystem4/drv_lsm303.[ch]` | LSM303DLHC accelerometer: CTRL_REG1_A 0x57 (100 Hz, XYZ), CTRL_REG4_A 0x88 (BDU, HR, +-2 g), read-back check, 7-byte STATUS+XYZ burst, 12-bit left-justified -> mg, identity via CTRL1 read-back + magnetometer IRA 'H' |
| `app_program/subsystem4/task_imu.c` | ImuTask, priority 5, CYC slot IMU: sensor read (3 consecutive errors -> IMU_FAULT, re-init every 500 ms, never a stop), MotionStatus freshness by cycle tag (STALE_MOTION diagnostic), distance delta, latch byte, core step, one aligned `g_imu_state` store, EVT_HUMP_START/END + telemetry, collision -> SafeStop_Request first then IMU_FAULT (D-19), MOTION_CLASS telemetry |
| `app_program/subsystem4/subsystem4.[ch]`, `subsystem4_test.c` | I2C/sensor bring-up (missing sensor is not a boot failure), task start, S4-U01..06 |
| `app_program/shared/app_hw.[ch]` | `app_hw_release_reset()`, `app_hw_gpio_function()` exported for task-owned peripherals |
| `app_program/shared/app_cal.h` | `APP_CAL_IMU_*`, `APP_CAL_COLLISION_*`, class and hump policy constants (all `[TBC]` until HV-12..14) |
| `testing/host/test_imu.c` | 56 checks with a scripted car |

### Decisions recorded

D-17 (hump_max from NORMAL only), D-18 (collision needs motion; spikes kept
out of the pitch filter), D-19 (IMU_FAULT on collision as Rev B writes it;
relabel suggested), D-20 (own polled I2C driver). Also:

* No WHO_AM_I exists on the LSM303DLHC accelerometer; presence is proven by
  the CTRL_REG1_A read-back and the magnetometer identification register.
* ODR 100 Hz equals the task rate, so a cycle may see no new sample
  (ZYXDA = 0); the previous values are reused and remain valid. A single bus
  error likewise reuses the last sample; only the third consecutive error
  declares the sensor faulty (and ends an active episode as INVALID).
* sin(pitch) is taken from the filtered vector (a_fwd / |a|), so the height
  integral needs no trigonometry table; atan2 is an integer rational
  approximation (max error 0.5 deg, H-IM-02).
* TURNING is taken from MotionStatus (no gyro on this part); ACCELERATING is
  |v - v_lp| >= 3 cm/s from the encoder speed, not from the accelerometer,
  so a slope does not read as acceleration.
* ImuTask blocks ~250 us per cycle in the polled I2C read (priority 5); the
  bounded deadline guarantees < 2 ms even with the sensor unplugged.

### Validation

| Check | Result |
|---|---|
| `make SMP=0 NET=lwip MQTT=1 -j8` | PASS 0 app warnings, text 337040 / data 128 / bss 78960 |
| `make SMP=0 NET=lwip MQTT=1 TEST=1 -j8` | PASS, 345596 / 128 / 78980 |
| `make SMP=0 -j8`, `make SMP=1 NET=lwip MQTT=1 TEST=1 -j8` | PASS, 0 warnings |
| `./testing/host/test_imu` | 56 passed, 0 failed |
| `./testing/host/test_line`, `test_shared`, `test_motion` (regression) | 114 / 418 / 56 passed, 0 failed |
| `tools/barr-c/check_app.sh` | PASS (imu_core, drv_i2c.h, drv_lsm303.h, test_imu.c added to clang-tidy) |
| Linked sizes | `g_s4_core` 56 B, `g_s4_sample` 8 B, `g_imu_state` 4 B, `imu_core_step` 1476 B text, `docs/evidence/phase4_sizes.txt` |
| On-target S4-U01..06, HV-12/13/14, NFR15 | **NOT RUN** |

Status: BUILD PASS, HOST TEST PASS, TARGET NOT RUN, axis mapping and signs
`[TBC]` (HV-12), collision threshold unvalidated (A9, HV-13), K_CAL and
NFR15 NOT MEASURED (HV-14). Next: PHASE 5 (subsystem5, obstacle scan).

## 10. Phase 5 record (subsystem5 - ultrasonic scan and obstacle profile, Buddy 5)

### Files written

| Path | Role |
|---|---|
| `app_program/subsystem5/scan_core.[ch]` | pure algorithm: servo index <-> pulse and settle, 5 deg sin/cos table, lateral geometry (LEFT positive), forward-watch confirmation with hysteresis and NO_ECHO counting, coarse (9 x 15 deg) and fine (5 deg window) plans, profile builder (y_min, y_max, width with beam correction, closest, action by the Rev B validity rules + pivot guard, D-21), side-unsafe confirmation |
| `app_program/subsystem5/drv_servo.[ch]` | PWM slice 7B, 50 Hz, 1 us per count; GP15 routed only when `APP_CAL_SERVO_ARMED` |
| `app_program/subsystem5/task_scan.c` | EchoIsr (dispatcher callback: rise stamp, fall width, OVERRUN > 30 ms, one flag set) and ObstacleScanTask: IDLE / WATCH / SIDE_L / SIDE_R modes on FLG_SCAN requests and the 100 ms tick, synchronous PROFILE scan bounded by T_SCAN_TIMEOUT and interruptible by ABORT between samples, profile published under MTX_OBSTACLE with a 4-bit seq, EVT_OBSTACLE / EVT_SCAN_DONE / EVT_SCAN_FAIL / EVT_SIDE_UNSAFE, NO_ECHO and SCAN_TIMEOUT faults (D-22), `s5_profile_copy` for readers |
| `app_program/subsystem5/subsystem5.[ch]`, `subsystem5_test.c` | FLG_SCAN, MTX_OBSTACLE, hardware bring-up behind the gates (D-23), S5-U01..06 |
| `app_program/shared/app_cal.h` | `APP_CAL_SERVO_ARMED`, servo direction, R_PROFILE, R_SIDE_FREE, Y_PATH_MAX, beam correction, D_SIDE_UNSAFE, watch hysteresis, NO_ECHO count (all `[TBC]`) |
| `testing/host/test_scan.c` | 37 checks against a synthetic world with a 14 deg beam |

### Decisions recorded

D-21 (action computed in the scanner from shared constants), D-22 (scan
failure -> fault + EVT_SCAN_FAIL, controller stops), D-23 (two hardware
gates; task answers honestly while disabled). Also:

* Ranging is synchronous inside the task (TRIG, then one bounded flag wait
  on ECHO_DONE | ABORT, 35 ms max); the ISR stores a halfword then a byte,
  no RMW crosses the boundary (Rev B echo protocol).
* The 100 ms watch tick comes from the shared cyclic handler (Phase 1), so
  the trigger period is 100 ms >= 60 ms (NFR5) without a timer of its own.
* Forward watch stops during a profile scan and after a request; the
  controller re-requests WATCH_ON when it resumes line following.
* `g_s5_samples` keeps all 37 angles (88 B) so the fine scan can reuse the
  coarse samples; the profile itself is the 8 B two-word blackboard.

### Validation

| Check | Result |
|---|---|
| `make SMP=0 NET=lwip MQTT=1 -j8` | PASS 0 app warnings, text 340762 / data 140 / bss 79088 |
| `make SMP=0 NET=lwip MQTT=1 TEST=1 -j8` | PASS, 351778 / 140 / 79112 |
| `make SMP=0 -j8`, `make SMP=1 NET=lwip MQTT=1 TEST=1 -j8` | PASS, 0 warnings |
| `./testing/host/test_scan` | 37 passed, 0 failed |
| `./testing/host/test_imu`, `test_line`, `test_shared`, `test_motion` (regression) | 56 / 114 / 418 / 56 passed, 0 failed |
| `tools/barr-c/check_app.sh` | PASS (scan_core, drv_servo.h, test_scan.c added to clang-tidy) |
| Linked sizes | `g_s5_samples` 88 B, `g_s5_watch` 3 B, `g_obstacle_profile` 8 B, `g_echo_width_us` 2 B + `g_echo_status` 1 B, `g_sin_q15` 38 B, `scan_core_build_profile` 978 B text, `docs/evidence/phase5_sizes.txt` |
| On-target S5-U01..06, HV-5, HV-7, HV-15, NFR2/5/10/11/14 | **NOT RUN** |

Status: BUILD PASS, HOST TEST PASS, TARGET NOT RUN, TRIG and servo outputs
gated off (`APP_CAL_ECHO_SHIFTER_VERIFIED 0`, `APP_CAL_SERVO_ARMED 0`), beam
correction and side-validity constants `[TBC]` (HV-15). Next: PHASE 6
(subsystem1, Wi-Fi / MQTT telemetry).
