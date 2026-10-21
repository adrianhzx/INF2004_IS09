# BARR-C:2018 compliance gate - IS09 application code

Standard: Barr Group Embedded C Coding Standard 2018 (167 rules, 45
bug-killing). Kit: `netrinomike/barr-c` commit `75bcc16` (MIT), vendored in
`tools/barr-c/` with its licence. A clean tool run is **not** proof of all
167 rules; the manual checklist below covers the review-only rules.

## Automated gate (run from the repo root: `tools/barr-c/check_app.sh`)

| Mechanism | Tool / version | Scope | Phase 1 result |
|---|---|---|---|
| clang-format (kit `.clang-format`: Allman, 4 spaces, 80 cols, no tabs) | clang-format 18.1.3 | all `app_program/**/*.[ch]`, `testing/host/*.[ch]` | PASS (0 findings) |
| Lexical gates 5.2.b, 1.7.a/b, 8.5.b, 3.5/3.6, 1.7.c advisory | kit `check.sh` logic | same | PASS |
| Module naming 4.1.a/c/d; 4.1.b with DEV-1 exemption | script | same | PASS |
| clang-tidy (kit `.clang-tidy`) | clang-tidy 18.1.3, `-std=c99 -Wall` | host-buildable sources + all shared headers + host tests | PASS (0 findings; `unused-macros` noise filtered) |
| Compiler gates (kit COMPILER_FLAGS.md minus `-Wundef`) | arm-none-eabi-gcc 13.2.1 via `APP_CFLAGS` | every `app_program/**/*.c` in every target build | PASS (0 warnings, 4 profiles) |
| Host build `-std=c99 -pedantic` | gcc | `testing/host` | PASS |

Not tool-gated on the target-only sources (`app_hw.c`, `app_gpio_irq.c`,
`vehicle_controller.c`, `app_shared_test.c`, `app_main.c`, `subsystemN*.c`)
because clang-tidy would need the kernel include tree with GNU extensions;
they are covered by the compiler gates and the manual review below.

## Manual review checklist (Phase 1 files)

| Rule(s) | Check | Result / note |
|---|---|---|
| 1.3 braces, 1.4 parentheses, 8.6.a constant-left | every `if/for/while` braced; comparisons written `0U != (x & m)`, `NULL != p` | Reviewed: yes in all shared files |
| 1.6.a casts commented | every cast that narrows carries a range argument or the field helper guarantees range | `app_pack.h` casts documented; `(uint8_t)` after `app_pack_get` is exact by construction (field width <= 8) |
| 1.7.c goto, 1.7.d recursion, 8.5.b | none used | grep clean |
| 1.8.a static | all module-private functions/variables `static` | yes; template-mandated `subsystemN_*` and `usermain` are `EXPORT` |
| 1.8.b const | pointer params to read-only data `const` | `app_telemetry.c`, pack helpers |
| 1.8.c volatile | only ISR-shared words and MMIO | `g_stop_latch`, counters, ring, blackboard words; MMIO through `in_w/out_w` |
| 2.1/2.2 comments | file headers, no commented-out code | yes |
| 4.2 headers | guard, one header per source (DEV-3 for header-only) | yes |
| 4.3.c source includes its own header first | `app_safety.c`, `app_ipc.c`, `app_hw.c`, `app_gpio_irq.c`, `app_telemetry.c`, `vehicle_controller.c` | yes |
| 5.1 typedef `_t`, enum UPPER | `app_*_t` types, UPPER enum members | yes |
| 5.2 fixed-width | `int32_t`/`uint8_t` etc.; kernel types (`ID`, `ER`, `UINT`) only at API boundaries with explicit casts | yes |
| 5.3.a no signed bit-fields | no bit-fields at all in shared data (masks/shifts) | yes |
| 5.3.b/c no bitwise on signed, no mixed signedness | enforced by clang-tidy `hicpp-signed-bitwise` + `-Wsign-compare`; promotions of `uint8_t` are cast to `uint32_t` first | PASS |
| 5.4 floating point | none in shared code | yes |
| 6.1 function naming, module prefix | `app_<module>_<verb>`; `vehicle_controller_*` | yes |
| 6.2.f unused params | `(void)` casts | yes |
| 6.3 ISRs | `*_isr`, static, bounded, no waits: `app_gpio_bank0_isr`, `button_isr`, `sh_test_isr`, `app_cyc_handler` (handler name mandated by kernel API shape) | yes; only ISR-legal kernel calls (ledger section 2) |
| 6.4 threads | tasks `static`, infinite loop, block on kernel object | `vehicle_controller_task` |
| 7.1 variable naming `g_`, `p_`, `b_` | `g_` for globals, `p_` for pointer params; booleans use descriptive names (7.1.m `b_` prefix NOT applied - DEV-6, see below) | partial |
| 7.2 initialise at declaration | enforced by clang-tidy on host-buildable files; target-only files reviewed | yes |
| 8.2 conditionals, 8.3 switch default | every `switch` has `default`; enum cases listed | yes (`-Wswitch-default`) |
| 8.4 loops bounded | all loops bounded by constants or array sizes; delay loop guarded (`app_hw_delay_us`) | yes |
| 8.5 no dynamic allocation in real-time paths | none in application code (kernel allocates stacks/buffers at init only) | yes |

### Additional documented deviations

| ID | Rule | Deviation | Reason |
|---|---|---|---|
| DEV-6 | 7.1.m boolean `b_` prefix | booleans are named `on_line`, `until_line`, `hump_active`, `first`, `cleared` | names mirror the adopted design table fields verbatim (`on_line`, `until_line`, `hump_active`) so the code and the Tribunal table match; local booleans keep short descriptive names. Can be revisited by team vote. |
| DEV-7 | 1.1 C99 on target | `-std=gnu11` from the template | kernel headers need GNU extensions; application sources also compile under `-std=c99 -pedantic` on the host |
| DEV-8 | 4.1.b | `subsystemN` 8-char prefix collision | template-mandated (same as DEV-1) |

## Provenance (Chapter 9)

Phase 1 application code was generated with Claude Fable 5.1 (claude-fable-5-1)
in Claude Code on 21 Oct 2026 and is to be read and owned line by line by
the named human owners in TEAM.md before merge. Every commit message names
the tool.
