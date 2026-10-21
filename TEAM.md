# Team and subsystems - IS09

Single place that says **who owns what**. Source: Week 6 Design Review Rev B
(21 Oct 2026) and Tribunal 1 Version B. Rule: **only the owner edits a
subsystem's folder.** Shared files (`app_program/app_main.c`, `app.h`,
`app_program/shared/`, this file) change only by team agreement; changes to
`app_program/shared/app_data.h` are reviewed by the consuming buddy
(`docs/TRACEABILITY.md` code-review rule).

## 1. Members and subsystems

| # | Subsystem | Student | ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Wi-Fi/MQTT telemetry and commands (TelemetryCommsTask) | Xavier Lam Jun Jie | 2500887 | *(fill in)* | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion and encoders (MotionControlTask, EncoderIsr); owner of `app_data.h`, `app_motion.h`, SafeStop helper coordination | S Reuben Kanna | 2500695 | *(fill in)* | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line sensing and barcode (LineSensingTask, BarcodeDecodeTask, BarcodeEdgeIsr) | Xu Zhiqiang | 2501297 | *(fill in)* | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU, hump, collision (ImuTask) | Wong Wei Qi | 2500430 | *(fill in)* | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic scan, obstacle profile (ObstacleScanTask, EchoIsr) | Ho Zhi Xian Adrian | 2500280 | adrianhzx | `app_program/subsystem5/` | `testing/subsystem5/` |
| shared | VehicleControllerTask, safety foundation, IPC catalogue, hardware helpers, data layer | **Integration owner: TBC (D-09) - name required** | | | `app_program/shared/` | `testing/host/`, `testing/shared/` |

## 2. Pins (Rev B section 2; verify silkscreen before power, HV-1)

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio (kernel) |
| GP0 | S2 / EncoderIsr | left encoder A (GROVE 1). **Build never uses CRASH_REPORT=1 / CONSOLE=uart** |
| GP1 | S2 / EncoderIsr | left encoder B (GROVE 1) |
| GP2 | S2 / EncoderIsr | right encoder A (GROVE 2) |
| GP3 | S2 / EncoderIsr | right encoder B (GROVE 2) |
| GP4 | S4 / ImuTask | I2C0 SDA, LSM303DLHC (GROVE 3) |
| GP5 | S4 / ImuTask | I2C0 SCL (GROVE 3) |
| GP6 | spare | keep unconnected (shares GROVE 5 with GP26) |
| GP7 | S3 / BarcodeEdgeIsr | barcode IR DOUT, both edges (GROVE 7); AOUT disconnected |
| GP8 | S2 / MotionControlTask | M1A (left motor) PWM slice 4A |
| GP9 | S2 / MotionControlTask | M1B PWM slice 4B |
| GP10 | S2 / MotionControlTask | M2A (right motor) PWM slice 5A |
| GP11 | S2 / MotionControlTask | M2B PWM slice 5B |
| GP12, GP13, GP14 | spare | servo headers 1-3 unused |
| GP15 | S5 / ObstacleScanTask | scan servo PWM 50 Hz (servo header 4), slice 7B |
| GP16 | S5 / ObstacleScanTask | HC-SR04 TRIG (GROVE 4) |
| GP17 | S5 / EchoIsr | HC-SR04 ECHO **through <= 3.3 V divider/shifter (HV-5)** |
| GP18 | shared / VehicleControllerTask | on-board NeoPixel (optional status) |
| GP19 | spare | used only by the TEST image as a forced-interrupt pin (SH-U03/04/08) |
| GP20 | shared / VehicleControllerTask | START button (active-low assumed, HV-6) |
| GP21 | shared / STOP ISR -> SafeStop_Request | STOP button |
| GP22 | shared / VehicleControllerTask | buzzer (optional), slice 3A |
| GP26 | S3 / LineSensingTask | ADC0 left line IR AOUT (GROVE 5) |
| GP27 | S3 / LineSensingTask | ADC1 centre line IR AOUT (GROVE 6) |
| GP28 | *reserved* | ADC2 VBAT/2 only if the solder jumper is fitted; never barcode AOUT |

## 3. Peripherals

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 | 4 | ImuTask (own driver in `subsystem4/drv_*`, kernel I2C driver stays OFF) | GP4, GP5 |
| ADC | 3 | LineSensingTask (decision Phase 3: kernel `adca` device vs task-owned polled conversion) | GP26, GP27 (GP28 reserved) |
| PWM slices 4, 5 | 2 | MotionControlTask | GP8-GP11 |
| PWM slice 7 (ch B) | 5 | ObstacleScanTask | GP15 |
| PWM slice 3 (ch A) | shared | VehicleControllerTask | GP22 |
| IO_IRQ_BANK0 | shared | `app_gpio_irq.c` dispatcher; one callback per pin, owners above | GP0-3, 7, 17, 20, 21 |
| TIMER (1 MHz) | shared | read-only by everyone (`app_hw_time_us`) | - |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in); S1 fills/empties message buffers that the kernel task polls (D-10) | GP23-25, GP29 |

## 4. Task priorities (Rev B section 3; 1 is highest; kernel uses 3 and 4)

| Task | Priority | Period / trigger | Subsystem | Stack (bytes, measured Phase 8) |
|---|---|---|---|---|
| MotionControlTask | 2 | 10 ms (CYC_10MS wake) | 2 | 2048 |
| VehicleControllerTask | 3 | FLG_CTRL events + 20 ms tick | shared | 2048 |
| LineSensingTask | 4 | 10 ms | 3 | 1536 |
| ImuTask | 5 | 10 ms | 4 | 1536 |
| ObstacleScanTask | 6 | 100 ms watch tick + requests | 5 | 2048 |
| BarcodeDecodeTask | 7 | FLG_BARCODE events, 150 ms quiet timeout | 3 | 1536 |
| TelemetryCommsTask | 8 | 20 ms loop; 200 ms / 1 s cadence | 1 | 2048 |

All on core 0 (`SMP=0` demo profile). Note: the kernel's USB console task is
priority 3 and the Wi-Fi service task priority 4, both above our tasks 4-8.

## 5. Interfaces between subsystems

| ID | From -> To | Mechanism | Object | Data | Agreed by |
|---|---|---|---|---|---|
| IF-SS | any -> Controller | `app_safe_stop_request(cause)` + FLG_CTRL `EVT_SAFE_STOP` | `g_stop_latch`, safety word | first cause wins; Motion reads latch each 10 ms | Rev B |
| IF-FM | any -> any | `app_fault_set/clr` | `g_fault_mask` | 13 bits, never a stop trigger | Rev B |
| IF-MC | Controller -> Motion | message buffer (owned by S2) | `mbf_motion_cmd` | 4 B packed `MotionCmd` | Rev B |
| IF-MS | Motion -> Controller, IMU, Comms | two lock-free words | `g_motion_status_a/b` | speeds, distance, state, seqs, tags | Rev B + D-04 |
| IF-LS | Line -> Motion, Controller, Comms | lock-free word + FLG_CTRL events | `g_line_state` | error, steer, on_line, event, seq | Rev B |
| IF-BC | Barcode -> Controller, Comms | byte + FLG_CTRL `EVT_BARCODE`; Controller -> Barcode `flg_barcode` ARM/DISARM | `g_barcode_byte`, `flg_barcode` | result + seq | Rev B |
| IF-IS | IMU -> Controller, Comms | lock-free word + `EVT_HUMP_START/END` | `g_imu_state` | 32-bit ImuState | Rev B |
| IF-OP | Scan -> Controller, Comms | mutex copy (owned by S5) + `EVT_OBSTACLE/SCAN_DONE/SCAN_FAIL/SIDE_UNSAFE`; Controller -> Scan `flg_scan` requests | `g_obstacle_profile[2]`, `mtx_obstacle`, `flg_scan` | 50-bit profile | Rev B |
| IF-TE | any -> Comms | message buffer (owned by S1), non-blocking | `mbf_telem_evt` | 4 B event records, drop counted | Rev B |
| IF-RC | Comms -> Controller | byte + FLG_CTRL `EVT_REMOTE_CMD` | `g_remote_cmd`, `g_remote_cmd_gen` | validated START/STOP | Rev B + D-11 |
| IF-CW | Comms -> Controller | lock-free halfword | `g_comms_word` | conn state, backoff, dropped | Rev B |
| IF-HB | Controller -> Motion | byte | `g_ctrl_heartbeat` | increments every 20 ms | Rev B |
| IF-CYC | shared -> Motion, Line, IMU, Scan, Controller | cyclic handler `tk_wup_tsk`/`tk_set_flg` | CYC_10MS | 10 ms wake, 20 ms and 100 ms ticks | Rev B |

## 6. Kernel object budget (`config/config.h` limits)

| Object | Limit | shared | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 2 | 1 | 1 |
| Event flag | 16 | 1 (FLG_CTRL) | | | 1 (FLG_BARCODE) | | 1 (FLG_SCAN) |
| Mutex | 12 | | | | | | 1 (MTX_OBSTACLE, TA_INHERIT) |
| Message buffer | 12 | | 1 (256 B) | 1 (32 B) | | | |
| Cyclic handler | 12 | 1 | | | | | |
| Interrupt handler | - | 1 (IO_IRQ_BANK0) | | | | | |

(The kernel's USB and Wi-Fi service tasks also use a few of these.)
