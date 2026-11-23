# IS09 car: code overview by buddy

This page explains, at overview level, what each buddy's part of the car software does and which files hold it. The detailed design notes, pin changes and car-test history are in [IS09_PORT_NOTES.md](IS09_PORT_NOTES.md).

## How the software is organised

The car runs on **micro T-Kernel 3.0** on a Raspberry Pi Pico W. The application is in `app_program/` and runs as seven tasks that talk through mailboxes and event flags (`os_glue.c`):

| Task | Period | Priority | Main owner |
|---|---|---|---|
| `TSK_SAFETY_MON` | 10 ms | 2 (highest) | Integration |
| `TSK_MOTION_CTRL` | 20 ms | 4 | Buddy 2 |
| `TSK_LINE_BARCODE` | 10 ms | 5 | Buddy 3 |
| `TSK_OBSTACLE_SCAN` | ~70 ms per ping | 6 | Buddy 5 |
| `TSK_TERRAIN_IMU` | 10 ms | 7 | Buddy 4 |
| `TSK_NAV_PLANNER` | event-driven | 8 | Integration |
| `TSK_TELEMETRY` | 500 ms | 10 (lowest) | Buddy 1 |

Each subsystem follows the same pattern:

- **Pure algorithm ("core")**: plain C with no kernel or hardware calls, so it can be tested on a PC. Buddies 3–5 keep theirs in `app_program/is09/subsystemN/`.
- **Adapter** (`is09_*.c`, declared in `is09_glue.h`): converts between the core's types and units and the rest of the software.
- **Task** (`tsk_*.c`): reads the hardware through the drivers (`hal_*.c`), calls the core, and sends the results to other tasks.

Shared data types and calibration constants are in `app_program/is09/shared/` (`app_data.h`, `app_cal.h`) and `app_program/robot_config.h`.

---

## Buddy 1: Telemetry and communication (Wi-Fi / MQTT)

**What it does:** connects the car to the laptop over Wi-Fi and MQTT. It sends live data, receives commands, and keeps working through network drop-outs.

**Files:** `app_net.c/.h`, `tsk_telemetry.c`, `jfmt.c/.h`, `app_status.c`, and the laptop tools in `MQTT/`.

**How it works:**
- `app_net.c` runs the MQTT client inside the Wi-Fi radio task. Other tasks never touch the network; they only queue messages.
- `tsk_telemetry.c` builds a JSON **telemetry frame every 500 ms (2 Hz)** with the state, wheel speeds, encoders, distance, line sensors, barcode, hump, obstacle, Wi-Fi signal and faults. It also sends a **1 Hz heartbeat**. Frames are built with `jfmt.c`, a small JSON writer that needs no `printf`.
- **Offline buffer:** while the link is down, up to 64 frames (~32 s) are kept and sent on reconnect. Wi-Fi and MQTT reconnect with back-off (1 → 2 → 4 → 8 → 16 s). Navigation never waits for the network.
- **Broker discovery:** if the built-in broker IP fails twice, the car listens for the laptop's UDP beacon (`IS09_BROKER`, port 47002), so a changed laptop IP needs no rebuild.
- **Commands** arrive on `is09/cmd`: `ESTOP`, `START`, `STOP`, `BARCODE:<A-Z>`, `PING`, `SET:NAME=VALUE`, `GET:PARAMS`, `MAP:...`. Each command is answered on `is09/ack`. `ESTOP` goes straight to the safety task.

**MQTT topics:** `is09/telemetry`, `is09/status`, `is09/event`, `is09/ack` (car → laptop); `is09/cmd`, `is09/config`, `is09/map` (laptop → car).

**Laptop side (`MQTT/`):** `is09_dashboard.py` (built-in broker and web dashboard), `dashboard.html`, `telemetry_monitor.py`, `robot_tool.py`, `robot_params.py` and `robot_map.py`. See [MQTT/README.md](../MQTT/README.md).

---

## Buddy 2: Motion control

**What it does:** drives the two wheels at the requested speeds with encoder feedback, and runs the movement primitives (forward, backward, turn, stop) that the other tasks ask for.

**Files:** `tsk_motion.c`, `speed_ctl.c/.h`, `motion_api.h`, and the motor and encoder parts of `hal_board.c` and `hal_isr.c`.

**How it works:**
- **API for other tasks (`motion_api.h`):** `motion_move_forward(d)`, `motion_move_backward(d)`, `motion_turn_left(a)`, `motion_turn_right(a)`, `motion_stop()`. Each call posts a command to `MBX_MOTION_CMD`. When a turn or bypass move finishes, `EVF_MOTION_DONE` is set.
- **Every 20 ms, `tsk_motion.c`:**
  1. updates **odometry** from the encoders (distance, heading, speed);
  2. takes the latest commands. **STOP always wins.** Line-following corrections are ignored while a turn or bypass is running, and a hump limits the speed to 60 %;
  3. runs the active primitive, counted in encoder ticks. If it times out, that is treated as an encoder fault and the car goes to ERROR;
  4. runs the **speed controller** per wheel and sets the PWM.
- **Speed controller (`speed_ctl.c`):** feed-forward plus PI, with anti-windup so a stalled wheel does not build up a huge correction. The gains (`SPEED_KP`, `SPEED_KI`) are in `robot_config.h` and can be changed over MQTT.
- **Encoders:** 4× quadrature decoding in the GPIO interrupt (`hal_isr.c`).
- **Safety hook:** when the E-stop latch is set, every output is forced to 0.

**Pins:** left motor GP8/GP9, right motor GP10/GP11; left encoder GP0/GP1, right encoder GP2/GP3.

---

## Buddy 3: Line following and barcode

**What it does:** keeps the car on the black line, detects junctions and a lost line, and reads the Code 39 barcodes that say where to turn.

**Files:** `is09/subsystem3/line_core.c/.h` and `barcode_core.c/.h` (the cores), `is09_line.c`, `is09_barcode.c` (adapters), `tsk_line.c` (task), `line_logic.c/.h`, `line_meas.c/.h`, `barcode_logic.c/.h`, `code39.c/.h`.

**Line following (every 10 ms):**
- Two **analog IR sensors** (LEFT GP26, CENTRE GP27) sit either side of the line.
- `line_core` learns one shared **black/white reference pair** from both sensors and turns each reading into a **darkness from 0 to 100**. `is09_line.c` converts that into "this sensor sees black", with hysteresis so a sensor on the line's edge does not flicker.
- `line_logic.c` turns the two sensors into a **steering value**:
  - both white → straight;
  - one black → turn towards it;
  - both black, or something wider than the line → **junction**.

  A "curve memory" keeps the turn rate steady in tight curves.
- **Lost line:** the car searches with alternating pivots of growing size, then reports `EVF_LINE_FOUND` or `EVF_LINE_LOST`.
- `line_meas.c` measures how much black each sensor passes, in mm of travel. This helps tell a junction bar met head-on from the line drifting under the sensor.

**Barcode:**
- **Default (`BARCODE_AO 1`):** the barcode sensor's analog output on GP28 is sampled at 1 kHz (`hal_isr.c`). `barcode_logic.c` measures bar widths **in distance, not time**, so a speed change does not upset the decode. `code39.c` decodes Code 39 **in both directions**.
- **Alternative (`BARCODE_AO 0`):** the digital output on GP7 is timed edge by edge in the interrupt. `barcode_core` decodes it, and `is09_barcode.c` reads the full letter.
- A decoded letter goes to the navigation task on `MBX_NAV_EVENT`. The direction for each letter comes from the barcode map (`nav_map.c`). The start-up map is `BARCODE_DEFAULT_MAP` in `robot_config.h`: **A = LEFT, B = RIGHT, C = FORWARD, D = U-TURN, F = U-TURN**. It can be replaced over MQTT. `barcode_core.c` has its own A–D table, but in this build only the letter is taken from it.

---

## Buddy 4: IMU and hump detection

**What it does:** uses the accelerometer to measure the car's pitch, detect and measure humps, detect collisions, and classify the motion.

**Files:** `is09/subsystem4/imu_core.c/.h` (core), `is09_imu.c` (adapter), `tsk_imu.c` (task), `hump2.c/.h` (second hump profiler, for height), `hal_imu.c` (driver).

**How it works:**
- **Sensor:** GY-511 (LSM303) accelerometer on I2C0 (SDA GP4, SCL GP5), read every 10 ms in milli-g under the I2C bus mutex. There is no INT1 wire on this car.
- **`imu_core`**, once per 10 ms:
  - low-pass filter, then **pitch = atan2(forward, up)**;
  - **hump episodes** FLAT → CLIMBING → DESCENDING → end, each ending with a quality: NORMAL, REJECTED, DEGRADED or INVALID;
  - **peak height** from distance × sin(pitch);
  - **collision** detection from acceleration spikes;
  - a **six-class motion classifier** (for example CRUISING, TURNING).
- **Adapter (`is09_imu.c`):** maps the axes and removes the forward acceleration measured by the wheel encoders, so starting or stopping is not read as a slope.
- **Events from `tsk_imu.c`:**
  - hump start → `EVF_HUMP_START` (navigation limits the speed to 60 %);
  - hump end → `EVF_HUMP_END` and the run maximum;
  - collision → `EVF_COLLISION` (safety E-stop while driving).
- **`hump2.c`** runs beside `imu_core` on the same samples and **only measures** the hump height. It integrates sin(pitch) over the distance travelled and adds a "tilt height" for short humps. It filters out vibration spikes and the start-up jolt. Its results go in the `h2` telemetry object.
- **Fault handling:** 3 failed reads in a row → I2C bus recovery. If the IMU is dead, the car keeps driving without hump or collision detection.

---

## Buddy 5: Ultrasonic obstacle detection and avoidance

**What it does:** watches ahead with the ultrasonic sensor, stops before an obstacle, scans it with the servo, chooses a side, drives round it and finds the line again.

**Files:** `is09/subsystem5/scan_core.c/.h` (core), `is09_scan.c` (adapter), `tsk_obstacle.c` (task), `obstacle_logic.c/.h` (profile and bypass planning).

**How it works:** `tsk_obstacle.c` runs in stages.

| Stage | What happens |
|---|---|
| **0: forward watch** | Servo straight ahead, one ping about every 70 ms. Two readings in a row at or inside **25 cm** → stop at once and switch to obstacle avoidance. Two readings under **8 cm** → emergency stop. |
| **1: coarse scan** | −60° to +60° in 15° steps. If nothing is within 45 cm, go back to line following. |
| **2: fine scan** | 5° steps across what the coarse scan found. |
| **3: profile** | `scan_core` finds the obstacle's edges, width and closest point, and recommends LEFT or RIGHT if there is room. `obstacle_logic.c` builds the profile the bypass planner needs. If neither side is clear, the car reverses and scans again. |
| **4: bypass** | Pivot 45° away, drive past, then follow the obstacle's side until it ends. Each leg is watched with the sensor; if a side is blocked, the car backs out and tries the other side. |
| **5: rejoin the line** | Using dead-reckoned position, drive diagonally back towards the original path until the line sensors see the line, then line up and resume line following (`EVF_BYPASS_COMPLETE`). |

- `is09_scan.c` converts between the core's angle frame (index 0–36, + = left) and the rest of the code (degrees, + = right).
- Anything that fails → `EVF_BYPASS_ABORT` → ERROR.

**Pins:** ultrasonic TRIG GP16 / ECHO GP17, servo GP15.

---

## Integration (shared, used by every buddy)

These parts tie the subsystems together:

| File(s) | What it does |
|---|---|
| `app_main.c` | Start-up: kernel objects → drivers (motors braked first) → IMU self-test → interrupts → the seven tasks. |
| `os_glue.c/.h` | Mailboxes, event flags (`EVF_*`), the I2C mutex, task priorities. |
| `tsk_nav.c` | **Mission state machine:** IDLE → LINE FOLLOWING → MANEUVER (turn at a barcode) / OBSTACLE AVOIDANCE → MISSION COMPLETE. Applies the hump speed limit. |
| `nav_map.c/.h` | Barcode letter → movement (FORWARD / LEFT / RIGHT / U-TURN). At start-up it loads `BARCODE_DEFAULT_MAP` from `robot_config.h` (`A=LEFT,B=RIGHT,C=FWD,D=UTURN,F=UTURN`), so the car works without MQTT. A `MAP:` command or the retained `is09/map` replaces it at run time. |
| `tsk_safety.c` | Highest-priority task: **E-stop** (collision, proximity < 8 cm, a missing task heartbeat, MQTT `ESTOP`, BTN2), the ERROR state, buttons and buzzer. Recovery needs BTN1. |
| `pickup_logic.c/.h` | Stops the run when the car is lifted off the track (all IR sensors read near full scale). |
| `app_params.c/.h`, `robot_config.h` | Every tuning value; can be changed over MQTT without recompiling. |
| `app_check.c` | Start-up check that the calibration values are present. |
| `hal.h`, `hal_board.c`, `hal_isr.c`, `pins.h` | Drivers and the pin map (motors, encoders, servo, sensors, buttons, NeoPixel, buzzer). |

**Tests:** the cores and adapters are tested on a PC in `tests_is09/` (`make`, then run `test_is09_glue`).
