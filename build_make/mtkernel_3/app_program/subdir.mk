################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

# BARR-C:2018 compiler gates (kit COMPILER_FLAGS.md) applied to team-authored
# application code only; vendored kernel/SDK sources keep the port's flags.
# -Wundef is omitted: the kernel's own config headers test undefined macros.
APP_CFLAGS := -Wall -Wextra -Wshadow -Wswitch-default \
              -Wimplicit-fallthrough -Wvla -Wfloat-equal -Wsign-compare \
              -Werror=implicit-function-declaration -Werror=parentheses \
              -Werror=return-type -Werror=uninitialized

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) $(APP_CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
