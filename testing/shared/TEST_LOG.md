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
| SH-U01 | cyclic handler 100 +-5 per second | NOT RUN | |
| SH-U02 | 1 MHz timer advances | NOT RUN | |
| SH-U03a-d | tk_set_flg from a forced GPIO IRQ reaches a task; double attach refused | NOT RUN | |
| SH-U04a-c | 10 000 ISR/task RMW iterations, no lost bit, masked window <= 5 us | NOT RUN (SMP=0 only) | |
| SH-U05a-b | packed round trips on the ARM toolchain | NOT RUN | |
| SH-U06 | 100 event posts without waiting | NOT RUN | |
| SH-U07 | latch clear refused from a foreign task | NOT RUN | |
| SH-U08a-c | SafeStop from ISR: latch, first cause, controller in S7 <= 50 ms (leaves S7 latched) | NOT RUN | |

Procedure: flash the `_test.uf2`, open `screen /dev/ttyACM0 115200`, save the
console to `testing/shared/logs/<date>_test.txt`, grep `[TEST]`.
