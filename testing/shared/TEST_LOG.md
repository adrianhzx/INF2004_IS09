# Shared foundation tests (app_program/shared)

## Host tests (run on any PC: `make -C testing/host && ./testing/host/test_shared`)

| Run | Date | Toolchain | Result | Log |
|---|---|---|---|---|
| Phase 1 | 2026-10-21 | gcc (host), -std=c99 -pedantic | 418 passed, 0 failed | `docs/evidence/phase1_host_tests.log` |
| Decoder parity | 2026-10-21 | python3 3.13, `tools/telem_decode.py --selftest` | 7 vectors ok, 0 failed | `docs/evidence/phase1_host_tests.log` |

Test IDs: H-PK-01..25 (pack helpers), H-BB-01..24 (blackboards),
H-ER-01..09 (edge ring), H-TF-01..10 (frames), H-CMD-01..10 (command
frame), H-SF-01..17 (safety contract).

## On-target self-test (`make SMP=0 NET=lwip MQTT=1 TEST=1`, USB console, no external hardware)

| ID | Check | Result | Log |
|---|---|---|---|
| SH-U01 | cyclic handler 100 +-5 per second | PASS (runs A-C) | see runs below |
| SH-U02 | 1 MHz timer advances | PASS (runs B, C) | see runs below |
| SH-U03a-d | tk_set_flg from a forced GPIO IRQ reaches a task; double attach refused | PASS (runs B, C) | see runs below |
| SH-U04a-c | 10 000 ISR/task RMW iterations, no lost bit, masked window <= 5 us | U04c FAIL in run A; PASS after the SRAM fix (2 us run B, 1 us run C) | see runs below |
| SH-U05a-b | packed round trips on the ARM toolchain | PASS (runs B, C) | see runs below |
| SH-U06 | 100 event posts without waiting | PASS (runs B, C) | see runs below |
| SH-U07 | latch clear refused from a foreign task | PASS (runs B, C) | see runs below |
| SH-U08a-c | SafeStop from ISR: latch, first cause, controller in S7 <= 50 ms (leaves S7 latched) | PASS (runs A-C) | see runs below |

Procedure: flash the `_test.uf2`, open `screen /dev/ttyACM0 115200`, save the
console to `testing/shared/logs/<date>_test.txt`, grep `[TEST]`.

### On-target runs (Pico W on Robo Pico, `SMP=0 NET=lwip MQTT=1 TEST=1`, tester Xavier)

Add a new row for every run. Never edit or delete old rows.

| Run | Date | Commit | Result | Log | Notes |
|---|---|---|---|---|---|
| A | 2026-10-22 | `07fb89b` | FAIL 15/1 (SH-U04c) | `logs/2026-10-22_shared_runA_partial.txt` (partial) | Masked window over 5 us: it timed test set-up too, and the helpers ran from XIP flash, where a cache miss stalls the CPU with interrupts masked. INFO value not captured. |
| A2 | 2026-10-22 | `07fb89b` + SRAM patch (= `0aafbaa`) | PASS 16/0 | not saved | Reported by Xavier; console not kept. |
| B | 2026-10-22 | `bbcaa13` | PASS 16/0 | `../subsystem1/logs/2026-10-22_test_run1.txt` | SH-U04c helper window 2 us (set-up incl. 5 us). |
| C | 2026-10-22 | `3f3cdd6` | PASS 16/0 | `../subsystem1/logs/2026-10-22_test_run2.txt` | SH-U04c helper window 1 us (set-up incl. 3 us). Boot banner `hw self-check: 0x7 (timer 1, pwm 1, io 1)`. |
| D, D2 | 2026-10-27 | `57a1be1` tree (converged image, tester Wong Wei Qi) | PASS 16/0 (both runs) | `../system/logs/2026-10-27_test_image_full_run_converged.txt` | First run with all five subsystems' tasks live (motion, line, IMU, scan, barcode, telemetry) plus the dispatcher INTF fix: cyclic 100/s, SH-U04c helper window 3 us (set-up incl. 5 us, at the limit), 10000-iteration RMW with no lost bit, S7 within 50 ms. |
