# Host unit tests for subsystem 1 pure logic (no target hardware needed).
#   make -C testing/subsystem1/host && ./testing/subsystem1/host/test_s1
# Same strict C99 gates as testing/host.

CC      ?= gcc
APP     := ../../../app_program
CFLAGS  := -std=c99 -pedantic -Wall -Wextra -Wshadow -Wswitch-default \
           -Wimplicit-fallthrough -Wvla -Wfloat-equal -Wsign-compare \
           -Werror=implicit-function-declaration -Werror=parentheses \
           -O1 -g -DAPP_HOST_TEST=1 -I../../host -I$(APP)/shared

SRCS := test_s1.c $(APP)/subsystem1/s1_telem.c $(APP)/shared/app_telemetry.c

all: test_s1

test_s1: $(SRCS) $(wildcard $(APP)/shared/*.h) $(wildcard $(APP)/subsystem1/s1_*.h)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

run: test_s1
	./test_s1

clean:
	rm -f test_s1

.PHONY: all run clean
