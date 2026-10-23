# Data representation table (Week 13 evidence item 1, NFR28/NFR29)

Authority: `app_program/shared/app_data.h`. Every row below is implemented
there and round-trip tested in `testing/host/test_shared.c` (H-PK, H-BB,
H-ER, H-TF, H-CMD) and on target by `app_shared_test.c` (SH-U05).
"Phys" is the storage actually linked (`arm-none-eabi-nm -S`, see
`docs/evidence/phase1_blackboard_sizes.txt`).

Legend: W = writer, R = readers. Access rule numbers refer to app_data.h rules 1-6.

## Safety words

| Field | W / R | Valid range | Logical bits | Position | Phys | Units / sentinel | Rule |
|---|---|---|---|---|---|---|---|
| STOP_LATCH `g_stop_latch` | SafeStop (ISR+tasks) / Motion, Controller | 0,1 | 1 | own byte | 1 B | 1 = latched | 5: plain byte store; controller-only clear |
| stop_cause | SafeStop / Controller, Comms | 0..11 (NONE + 10 adopted + unadopted 11) | 4 | safety_word bits 0-3 | 1 B (shared with 4 reserved bits) | enum `app_stop_cause_t` | 4: write-if-NONE RMW under helper |
| fault_mask `g_fault_mask` | STOP ISR, Motion, IMU, Scan, Line, Comms / any | 13 named bits | 13 | bits 0-12 | 2 B | bit set = fault present; bits 4,5,11,12 diagnostic only | 4: `app_shared_bits_set/clr` |

## Controller

| Field | W / R | Range | Bits | Position | Phys |
|---|---|---|---|---|---|
| system_state | Controller / all | S0..S7 | 3 | MissionStatus 0-2 | in one 4 B word |
| pending_cmd | Controller / Comms | NONE,LEFT,RIGHT,STRAIGHT,UTURN | 3 | 3-5 | |
| retry_count | Controller / Comms | 0..3 | 2 | 6-7 | |
| mission seq | Controller / readers | 0..15 | 4 | 8-11 | |
| ctrl_heartbeat | Controller / Motion | 0..255 wrap | 8 | own byte | 1 B |
| RemoteCmd (D-11) | Comms / Controller | cmd 1..2, seq 0..63 | 2+6 | bits 6-7, 0-5 | 1 B (+1 B generation) |

MissionStatus: 12 meaningful bits in a 32-bit word (3+3+2 adopted + 4 seq).

## Motion (Buddy 2)

| Field | W / R | Range | Bits | Position | Phys | Units |
|---|---|---|---|---|---|---|
| enc_ticks_L / R | EncoderIsr / Motion | 0..65535 wrap | 16 each | two aligned halfwords | 2 B + 2 B | quadrature edges, wrap-safe delta |
| ticks per 10 ms | Motion private (signed delta per cycle) | +-32767 derived | 16 (int32 working) | private | | wrap-safe uint16 subtraction then sign decode |
| enc interval L / R | EncoderIsr / Motion | 0..65535 x 32 us (0 = none, saturating) | 16 each | own halfword | 2 B + 2 B | 32 us units; speed = um_per_tick x 100 / us |
| enc glitches | EncoderIsr / Motion | 0..255 saturating | 8 | own byte | 1 B | invalid 2-bit transitions; >= APP_CAL_ENC_GLITCH_MAX -> ENCODER_FAULT |
| speed_L / speed_R | Motion / IMU, Controller, Comms | 0..127 | 7 each | word A 0-6, 7-13 | 4 B | cm/s magnitude |
| distance_mm | Motion / IMU, Comms | -32768..32767 | 16 | word A 14-29 | | mm, signed two's complement |
| cycle_tag (D-04) | Motion / readers | 0..3 | 2 | word A 30-31 and word B 3-4 | | must match for a coherent pair |
| motion_state | Motion / Controller, Comms | BRAKED..STOP_LATCHED (5) | 3 | word B 0-2 | 4 B | |
| done_seq / active_seq | Motion / Controller | 0..15 | 4+4 | word B 5-8, 9-12 | | matches MotionCmd.seq |
| dir_L / dir_R | Motion / IMU | 0,1 | 1+1 | word B 13, 14 | | 1 = reverse (FR8 sign) |
| done_result | Motion / Controller | OK,REJECTED,ABORTED,TIMEOUT | 2 | word B 15-16 | | |
| motor_pwm L/R | Motion private | -100..100 | 8 each (int8) | private | | % |
| MotionCmd record | Controller -> MBF_MOTION_CMD -> Motion | type 0..3, speed 0..127, param int16, until_line, seq 0..15 | 2+7+16+1+4 = 30 | 0-1, 2-8, 9-24, 25, 26-29 | 4 B record; **8 B in the buffer** (kernel header, D-07); depth 4 = 32 B | param: mm (MOVE) or 0.1 deg (TURN) |

Signed reverse motion (master prompt question): speeds stay unsigned
magnitudes as adopted; the sign needed by FR8 is carried by `distance_mm`
(signed), `MotionCmd.param` (signed) and the two direction bits in word B,
which use reserved bits. No adopted width changed.

## Line + barcode (Buddy 3)

| Field | W / R | Range | Bits | Position | Phys | Units |
|---|---|---|---|---|---|---|
| line_error | Line / Motion, Controller, Comms | -100..100 | 8 (int8) | LineState 0-7 | 4 B word | + = line right of centre |
| steer | Line / Motion | -100..100 | 8 (int8) | 8-15 | | % demand |
| on_line | Line / Motion, Controller | 0,1 | 1 | 16 | | |
| line event | Line / Controller | NONE,JUNCTION,LOST,FOUND | 2 | 17-18 | | |
| line seq | Line / Motion (stale check) | 0..15 | 4 | 19-22 | | |
| adc_left / adc_centre | Line private | 0..4095 | 12 each | private | | raw ADC |
| EdgeRing entry | BarcodeEdgeIsr / BarcodeDecodeTask | level 0/1; delta 0..32767 x 8 us (262 ms) | 1+15 | bit 15, bits 0-14 | 64 x 2 B = 128 B + 4 B indices | saturates 0x7FFF, overflow byte |
| barcode result + seq | Barcode / Controller, Comms | 7 results; 0..15 | 3+4 | bits 0-2, 3-6 | 1 B | |
| decoder_state | Barcode private | 4 states | 2 | private | | |
| line references min/max x2 | Line private (`g_s3_line`) | 0..4095 | 4 x 16 | private | 20 B struct total | running, 1 LSB/cycle relax |
| dark_cycles / light_cycles | Line private | 0..65535 | 16 each | private | | saturating |
| prev_error | Line private | -100..100 | 16 (int16) | private | | for the D term |
| barcode element width | Barcode private (`g_s3_bc`) | 0..65535 x 8 us | 16 each, 64 entries | private | 148 B struct total | same unit as EdgeRing delta |
| barcode count / merge_pending | Barcode private | 0..64; 0,1 | 8; 1 | private | | |

LineState: 23 meaningful bits in a 32-bit word.

## IMU (Buddy 4)

| Field | W / R | Range | Bits | Position | Phys | Units |
|---|---|---|---|---|---|---|
| pitch | IMU / Comms | -63.5..63.5 deg | 8 (int8) | ImuState 0-7 | 4 B word | 0.5 deg |
| quality | IMU / Controller, Comms | NORMAL,REJECTED,DEGRADED,INVALID | 2 | 8-9 | | |
| episode seq | IMU / Controller | 0..15 | 4 | 10-13 | | |
| height_mm | IMU / Comms | 0..127 (A8 PROVISIONAL) | 7 | 14-20 | | mm, saturating -> quality DEGRADED |
| max_height_mm | IMU / Comms | 0..127 (A8 PROVISIONAL) | 7 | 21-27 | | mm |
| motion_class | IMU / Comms | 7 classes | 3 | 28-30 | | |
| hump_active | IMU / Controller | 0,1 | 1 | 31 | | |
| hump_state | IMU private | FLAT,CLIMBING,DESCENDING | 2 | private bit-field | | |
| lp_fwd / lp_lat / lp_up | IMU private (`g_s4_core`, 56 B) | +-2 g x 16 | 32 each | private | | mg x 2^2 EMA |
| confirm counters, episode cycles / distance | IMU private | 0..255 / 0..65535 | 8 / 16 | private | | cycles, mm |
| height_q8 / peak_q8 | IMU private | signed | 32 each | private | | mm x 256 |
| raw sample x/y/z | IMU private (`g_s4_sample`) | +-2048 | 16 each | private | 8 B | mg |

ImuState: exactly 32 meaningful bits.

## Obstacle (Buddy 5)

| Field | W / R | Range | Bits | Position | Phys | Units |
|---|---|---|---|---|---|---|
| echo_width_us | EchoIsr / Scan | 0..30000 | 16 | own halfword | 2 B | us |
| echo_status | task arms, ISR completes | IDLE,ARMED,RISING,DONE,OVERRUN | 3 (stored in 8) | own byte | 1 B | |
| range_cm | Scan / Controller, Comms | 0..511 | 9 | word0 0-8 | 2 x 4 B under MTX_OBSTACLE | cm, 511 = no echo / saturated |
| scan_angle_idx | Scan / Controller | 0..36 | 6 | word0 9-14 | | 5 deg steps |
| width_cm | Scan / Controller, Comms | 0..63 | 6 | word0 15-20 | | cm |
| y_min_cm / y_max_cm | Scan / Controller | -64..63 | 7+7 | word0 21-27, word1 0-6 | | cm lateral |
| closest_cm | Scan / Controller | 0..511 | 9 | word1 7-15 | | cm |
| action | Scan / Controller, Comms | 5 actions | 3 | word1 16-18 | | |
| scan_state | Scan / Controller | <= 8 | 3 | word1 19-21 | | |
| profile seq | Scan / Controller | 0..15 | 4 | word1 22-25 | | |

ObstacleProfile: **50** meaningful bits (D-12; Rev B said 49) + 4 seq in two words.

## Comms (Buddy 1)

| Field | W / R | Range | Bits | Position | Phys |
|---|---|---|---|---|---|
| conn_state | Comms / Controller, Comms | 4 states | 2 | comms_word 0-1 | 2 B |
| backoff_s | Comms / Comms | 0..10 | 4 | 2-5 | |
| dropped_events | Comms (mirrors `app_ipc_telem_dropped`) | 0..255 saturating | 8 | 6-13 | |
| Telemetry event record | any producer -> MBF_TELEM_EVT -> Comms | type 0..19 (<=32), source 0..7, payload 16, stamp 0..255 | 5+3+16+8 = 32 | 0-4, 5-7, 8-23, 24-31 | 4 B record; 8 B in buffer; 32 records = 256 B |

## Wire frames (little-endian, version byte first)

| Frame | Bytes | Meaningful bits | Topic | Status |
|---|---|---|---|---|
| SNAP (adopted) | 18 | 139 (+5 reserved zero) | `is09/<id>/snap` every 200 ms | implemented, tested |
| EVT | 4 | 32 | `is09/<id>/evt` on each dequeued record | implemented, tested |
| HB (D-06) | 6 | 8+8+16+2+4+8 = 46 | `is09/<id>/hb` every 1 s | implemented, NEEDS RATIFICATION |
| OBS (D-06) | 10 | 8+8+50(+4 seq) | `is09/<id>/obs` on EVT_SCAN_DONE | implemented, NEEDS RATIFICATION |
| CMD (inbound) | 3 | 8+2+6+8 = 24 | `is09/<id>/cmd` | implemented, tested incl. single-bit-flip rejection |

Snapshot bit offsets: ver 0, seq 8, sys 16, mot 19, cause 22, faults 26,
speed_l 42, speed_r 49, enc_l 56, enc_r 72, dist 88, line_err 104, on_line
112, barcode 113, hump_max 116, hump_active 123, class 124, range 127,
action 136, end 139.

## Estimated unpacked baseline (Week 13 items 2 and 3) - ESTIMATE, NOT MEASURED

No authentic unpacked implementation exists (ledger section 0), so this is
the design-estimate column the Tribunal asked for, to be replaced by the
first real measurement of the packed build:

| Object | Unpacked design (natural C types) | Packed (this implementation, linked) |
|---|---|---|
| MotionCmd record | struct {enum; int; int; bool} ~12-16 B (+4 B header) | 4 B (+4 B header) |
| LineState | struct of int, int, bool, enum, uint ~20 B | 4 B |
| MotionStatus | ~16 B | 8 B |
| ImuState | ~24 B | 4 B |
| ObstacleProfile | ~28 B | 8 B |
| MissionStatus | ~12 B | 4 B |
| EdgeRing | 64 x 4 B = 256 B | 128 B |
| Event record | ~12 B | 4 B |
| Snapshot payload | ~200-250 B text | 18 B |

Measured so far (packed): see `docs/evidence/phase1_blackboard_sizes.txt`
(the demo image links only the words already referenced; the TEST image
references all of them). Full RAM/stack/high-water comparison is Phase 8.
