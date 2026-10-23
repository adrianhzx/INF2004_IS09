#!/bin/sh
# BARR-C:2018 gate for IS09 team-authored code only (kernel/SDK excluded).
# Runs the kit's check.sh (format + lexical gates + clang-tidy) from the repo
# root on app_program/ and testing/host/. Documented deviations:
#   DEV-1 subsystemN.c share 8-char prefixes (template-mandated, NFR26);
#         the kit's 4.1.b gate is therefore reproduced here with that
#         exemption instead of calling check.sh's module-name gate blindly.
# Usage: tools/barr-c/check_app.sh            (from the repository root)
set -e
cd "$(dirname "$0")/../.."
FILES=$(ls app_program/*.c app_program/*.h app_program/shared/*.c app_program/shared/*.h \
           app_program/subsystem*/*.c app_program/subsystem*/*.h testing/host/*.c testing/host/*.h)

echo "== clang-format ($(clang-format --version | head -1)) =="
clang-format --style=file --dry-run --Werror $FILES

echo "== lexical gates (short/long, auto/register, abort/exit/setjmp/longjmp, goto, tabs/CR) =="
CTRL=$(printf '\001\002\003\004\005\006\007\010\011\013\015\016\017\020\021\022\023\024\025\026\027\030\031\032\033\034\035\036\037\177')
if grep -n "[$CTRL]" /dev/null $FILES; then echo "FAIL: control characters"; exit 1; fi
if grep -nE '(^|[^A-Za-z0-9_])(short|long)([^A-Za-z0-9_]|$)' /dev/null $FILES | grep -v 'barr-c:allow' | grep -vE '^[^:]*:[0-9]+:[[:space:]]*(//|\*|/\*)'; then
    echo "FAIL: short/long (5.2.b)"; exit 1; fi
if grep -nE '(^|[^A-Za-z0-9_])(auto|register)([^A-Za-z0-9_]|$)' /dev/null $FILES | grep -v 'barr-c:allow' | grep -vE '^[^:]*:[0-9]+:[[:space:]]*(//|\*|/\*)'; then
    echo "FAIL: auto/register (1.7.a/b)"; exit 1; fi
if grep -nE '(^|[^A-Za-z0-9_])(abort|exit|setjmp|longjmp)[[:space:]]*\(' /dev/null $FILES | grep -v 'barr-c:allow'; then
    echo "FAIL: abort/exit/setjmp/longjmp (8.5.b)"; exit 1; fi
grep -nE '(^|[^A-Za-z0-9_])goto([^A-Za-z0-9_]|$)' /dev/null $FILES && echo "ADVISORY: goto (1.7.c)" || true

echo "== module names (4.1.a/c/d; 4.1.b with DEV-1 exemption) =="
for f in $FILES; do
    b=$(basename "$f")
    case "$b" in *[!a-z0-9_.]*) echo "FAIL: $b not lowercase (4.1.a)"; exit 1;; esac
done
DUP=$(for f in $FILES; do basename "$f" | sed 's/\.[ch]$//'; done | sort -u | cut -c1-8 | sort | uniq -d | grep -v '^subsyste$' || true)
if [ -n "$DUP" ]; then echo "FAIL: module prefix collision (4.1.b): $DUP"; exit 1; fi

echo "== clang-tidy ($(clang-tidy --version | head -1 | sed 's/^ *//')) on sources =="
SRC="app_program/shared/app_safety.c app_program/shared/app_ipc.c app_program/shared/app_telemetry.c app_program/shared/app_blackboard.c \
     app_program/shared/app_pack.h app_program/shared/app_motion.h app_program/shared/app_line.h app_program/shared/app_barcode.h \
     app_program/shared/app_imu.h app_program/shared/app_obstacle.h app_program/shared/app_telemetry.h app_program/shared/app_data.h \
     app_program/shared/app_ipc.h app_program/shared/app_safety.h testing/host/test_shared.c testing/host/host_port.c \
     app_program/subsystem2/motion_core.c app_program/subsystem2/motion_core.h app_program/subsystem2/drv_encoder.c \
     app_program/subsystem2/drv_encoder.h app_program/subsystem2/drv_motor.h testing/host/test_motion.c \
     app_program/subsystem3/line_core.c app_program/subsystem3/line_core.h app_program/subsystem3/barcode_core.c \
     app_program/subsystem3/barcode_core.h app_program/subsystem3/drv_line_adc.h testing/host/test_line.c \
     app_program/subsystem4/imu_core.c app_program/subsystem4/imu_core.h app_program/subsystem4/drv_i2c.h \
     app_program/subsystem4/drv_lsm303.h testing/host/test_imu.c"
TIDY_OUT=$(clang-tidy --quiet $SRC -- -x c -std=c99 -Wall -DAPP_HOST_TEST=1 -Itesting/host -Iapp_program/shared -Iapp_program/subsystem2 -Iapp_program/subsystem3 -Iapp_program/subsystem4 2>&1 | grep -E "warning:|error:" | grep -v "unused-macros" || true)
if [ -n "$TIDY_OUT" ]; then echo "$TIDY_OUT"; echo "FAIL: clang-tidy findings"; exit 1; fi
echo "(no clang-tidy findings on host-buildable sources and shared headers)"
echo "NOTE: target-only sources (app_hw.c, app_gpio_irq.c, vehicle_controller.c, app_shared_test.c, app_main.c, subsystemN*.c, task_*.c, drv_line_adc.c, drv_i2c.c, drv_lsm303.c) need the kernel include tree; they are gated by the target build's APP_CFLAGS and by manual review (docs/BARR_C_REVIEW.md)."
echo "OK: BARR-C:2018 application gates passed"
