# Host unit tests for app_program/shared (no target hardware needed).
#   make -C testing/host && ./testing/host/test_shared
# Compiled as strict C99 with the BARR-C kit's compiler gates.

CC      ?= gcc
SHARED  := ../../app_program/shared
CFLAGS  := -std=c99 -pedantic -Wall -Wextra -Wshadow -Wswitch-default \
           -Wimplicit-fallthrough -Wvla -Wfloat-equal -Wsign-compare \
           -Werror=implicit-function-declaration -Werror=parentheses \
           -O1 -g -DAPP_HOST_TEST=1 -I. -I$(SHARED)

S2      := ../../app_program/subsystem2
S3      := ../../app_program/subsystem3
S4      := ../../app_program/subsystem4
S5      := ../../app_program/subsystem5
CFLAGS  += -I$(S2) -I$(S3) -I$(S4) -I$(S5)

SRCS := test_shared.c host_port.c \
        $(SHARED)/app_safety.c $(SHARED)/app_ipc.c $(SHARED)/app_telemetry.c \
        $(SHARED)/app_blackboard.c
MOTION_SRCS := test_motion.c host_port.c $(SHARED)/app_blackboard.c \
        $(S2)/motion_core.c $(S2)/drv_encoder.c
LINE_SRCS := test_line.c host_port.c $(SHARED)/app_blackboard.c \
        $(S3)/line_core.c $(S3)/barcode_core.c
IMU_SRCS := test_imu.c host_port.c $(SHARED)/app_blackboard.c $(S4)/imu_core.c
SCAN_SRCS := test_scan.c host_port.c $(SHARED)/app_blackboard.c $(S5)/scan_core.c

all: test_shared test_motion test_line test_imu test_scan

test_shared: $(SRCS) $(wildcard $(SHARED)/*.h) host_port.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

test_motion: $(MOTION_SRCS) $(wildcard $(SHARED)/*.h) $(wildcard $(S2)/*.h) host_port.h
	$(CC) $(CFLAGS) -o $@ $(MOTION_SRCS)

test_line: $(LINE_SRCS) $(wildcard $(SHARED)/*.h) $(wildcard $(S3)/*.h) host_port.h
	$(CC) $(CFLAGS) -o $@ $(LINE_SRCS)

test_imu: $(IMU_SRCS) $(wildcard $(SHARED)/*.h) $(wildcard $(S4)/*.h) host_port.h
	$(CC) $(CFLAGS) -D_DEFAULT_SOURCE -o $@ $(IMU_SRCS) -lm

test_scan: $(SCAN_SRCS) $(wildcard $(SHARED)/*.h) $(wildcard $(S5)/*.h) host_port.h
	$(CC) $(CFLAGS) -o $@ $(SCAN_SRCS) -lm

run: test_shared test_motion test_line test_imu test_scan
	./test_shared
	./test_motion
	./test_line
	./test_imu
	./test_scan

clean:
	rm -f test_shared test_motion test_line test_imu test_scan vectors.txt

.PHONY: all run clean
