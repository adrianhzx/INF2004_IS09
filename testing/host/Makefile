# Host unit tests for app_program/shared (no target hardware needed).
#   make -C testing/host && ./testing/host/test_shared
# Compiled as strict C99 with the BARR-C kit's compiler gates.

CC      ?= gcc
SHARED  := ../../app_program/shared
CFLAGS  := -std=c99 -pedantic -Wall -Wextra -Wshadow -Wswitch-default \
           -Wimplicit-fallthrough -Wvla -Wfloat-equal -Wsign-compare \
           -Werror=implicit-function-declaration -Werror=parentheses \
           -O1 -g -DAPP_HOST_TEST=1 -I. -I$(SHARED)

SRCS := test_shared.c host_port.c \
        $(SHARED)/app_safety.c $(SHARED)/app_ipc.c $(SHARED)/app_telemetry.c \
        $(SHARED)/app_blackboard.c

all: test_shared

test_shared: $(SRCS) $(wildcard $(SHARED)/*.h) host_port.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

run: test_shared
	./test_shared

clean:
	rm -f test_shared vectors.txt

.PHONY: all run clean
