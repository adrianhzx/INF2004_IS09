# Host tests for the IS09 adapters (gcc on the laptop or Linux):  make
APP  := ../app_program
SRCS := test_is09_glue.c $(APP)/is09_barcode.c $(APP)/is09_line.c $(APP)/is09_scan.c \
        $(APP)/is09_imu.c $(APP)/code39.c $(APP)/pickup_logic.c $(APP)/obstacle_logic.c \
        $(APP)/is09/subsystem3/barcode_core.c $(APP)/is09/subsystem3/line_core.c \
        $(APP)/is09/subsystem4/imu_core.c $(APP)/is09/subsystem5/scan_core.c
CFLAGS := -std=gnu11 -O1 -Wall -Wextra -Wshadow -Wno-unused-parameter

test_is09_glue: $(SRCS)
	gcc $(CFLAGS) -o $@ $(SRCS) -lm
	./$@

clean:
	rm -f test_is09_glue test_is09_glue.exe
.PHONY: clean
