################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# Existing flat subsystems plus subsystem 5's src/ layout.
SUBSYSTEM5_ALL_SRCS = $(wildcard ../app_program/subsystem5/src/*.c)
SUBSYSTEM5_TEST_SUPPORT_SRCS = \
    ../app_program/subsystem5/src/obstacle_test.c \
    ../app_program/subsystem5/src/scan_test_adapter.c \
    ../app_program/subsystem5/src/test_interface.c
SUBSYSTEM5_SRCS = $(filter-out $(SUBSYSTEM5_TEST_SUPPORT_SRCS), \
                              $(SUBSYSTEM5_ALL_SRCS))
SUBSYSTEM5_TEST_SRCS =
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c) \
            $(SUBSYSTEM5_SRCS)
# Subsystem 5 unit cases execute through scan_lifecycle_test on the Pico W.
ifeq ($(TEST),1)
SUBSYSTEM5_TEST_SRCS += $(SUBSYSTEM5_TEST_SUPPORT_SRCS)
SUBSYSTEM5_TEST_SRCS += $(wildcard ../testing/subsystem5/unit_tests/src/*.c)
TEMP_SRCS += $(SUBSYSTEM5_TEST_SRCS)
endif

TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)
SUBSYSTEM5_OBJS = $(SUBSYSTEM5_SRCS:.c=.o) $(SUBSYSTEM5_TEST_SRCS:.c=.o)
SUBSYSTEM5_BUILD_OBJS = $(subst ../, ./mtkernel_3/, $(SUBSYSTEM5_OBJS))
SUBSYSTEM5_BARR_CHECK = ../tools/check_subsystem5_barr_c.py
PYTHON ?= python3

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

# The Pico SDK requires C11 static assertions, but project-authored subsystem 5
# sources remain strictly C99 and treat all enabled diagnostics as errors. The
# kernel headers intentionally test optional undefined configuration macros, so
# -Wundef cannot be enabled at this integration boundary.
$(SUBSYSTEM5_BUILD_OBJS): CFLAGS += -std=c99 -Wall -Wextra -Wpedantic \
                                     -Wconversion -Wsign-conversion -Wshadow \
                                     -Werror
$(SUBSYSTEM5_BUILD_OBJS): | subsystem5_barr_check

# The other application files (app_main.c and subsystems 1 to 4) get the
# same warnings, as errors. Added 2026-11-25 after
# testing/host/target_static_check.py showed none of them warns; warning
# flags do not change the generated code (the mission UF2 is identical).
# They stay at the SDK's C dialect because the kernel headers need it.
APP_OTHER_BUILD_OBJS = $(subst ../, ./mtkernel_3/, \
                           $(filter-out $(SUBSYSTEM5_OBJS), $(TEMP_OBJS)))
$(APP_OTHER_BUILD_OBJS): CFLAGS += -Wall -Wextra -Wconversion \
                                    -Wsign-conversion -Wshadow -Werror

.PHONY: subsystem5_barr_check
subsystem5_barr_check:
	@$(PYTHON) $(SUBSYSTEM5_BARR_CHECK)

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/testing/subsystem5/unit_tests/src/%.o: ../testing/subsystem5/unit_tests/src/%.c
	@echo 'Building Pico W test: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
