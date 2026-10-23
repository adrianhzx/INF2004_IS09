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
CFLAGS  += -I$(S2)

SRCS := test_shared.c host_port.c \
        $(SHARED)/app_safety.c $(SHARED)/app_ipc.c $(SHARED)/app_telemetry.c \
        $(SHARED)/app_blackboard.c
MOTION_SRCS := test_motion.c host_port.c $(SHARED)/app_blackboard.c \
        $(S2)/motion_core.c $(S2)/drv_encoder.c

all: test_shared test_motion

test_shared: $(SRCS) $(wildcard $(SHARED)/*.h) host_port.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

test_motion: $(MOTION_SRCS) $(wildcard $(SHARED)/*.h) $(wildcard $(S2)/*.h) host_port.h
	$(CC) $(CFLAGS) -o $@ $(MOTION_SRCS)

run: test_shared test_motion
	./test_shared
	./test_motion

clean:
	rm -f test_shared test_motion vectors.txt

.PHONY: all run clean
