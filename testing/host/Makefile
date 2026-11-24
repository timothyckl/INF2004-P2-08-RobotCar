################################################################################
# Host (computer) runs of the pure-logic unit cases.
#
#   make -C testing/host          build and run every suite
#   make -C testing/host clean
#
# These cases also run on the Pico W in the TEST=1 image where the owner
# wired them in. A host pass shows the logic is right for synthetic input;
# it is not evidence that the robot works (testing/README.md).
#
# Executables go to $(OUT) (default: a temporary directory), never the tree.
################################################################################

CC      ?= gcc
ROOT    := ../..
APP     := $(ROOT)/app_program
ifeq ($(strip $(OUT)),)
OUT     := $(shell mktemp -d 2>/dev/null || echo /tmp/robotcar-host)
endif
STRICT  := -std=c99 -Wall -Wextra -Werror -Wshadow -pedantic
STUBS   := -Istubs stubs/kernel_stub.c

S2_SRC  := $(APP)/subsystem2/motion_logic.c \
           $(APP)/subsystem2/controller_logic.c \
           $(APP)/subsystem2/line_control.c \
           $(APP)/subsystem2/scan_control.c \
           $(APP)/subsystem2/bypass_control.c \
           $(APP)/subsystem2/nav_control.c \
           $(APP)/subsystem2/pid.c \
           $(APP)/subsystem2/odometry.c \
           $(APP)/subsystem2/motion_api.c \
           $(APP)/subsystem2/test_motion.c \
           $(APP)/subsystem2/test_controller.c \
           $(APP)/subsystem2/test_line_control.c \
           $(APP)/subsystem2/test_scan_control.c \
           $(APP)/subsystem2/test_bypass_control.c \
           $(APP)/subsystem2/test_nav_control.c \
           s2_host_main.c

S1_SRC  := $(APP)/subsystem1/command_parser.c \
           $(APP)/subsystem1/telemetry_encoder.c \
           $(APP)/subsystem1/comms_timer.c \
           $(APP)/subsystem1/service_bridge.c \
           $(APP)/subsystem1/comms_test.c \
           $(ROOT)/testing/subsystem1/logic_main.c

S3_SRC  := $(APP)/subsystem3/s3_ir.c \
           $(APP)/subsystem3/s3_bar.c \
           $(APP)/subsystem3/s3_scan.c \
           $(APP)/subsystem3/s3_barcode.c \
           $(APP)/subsystem3/s3_code39.c \
           $(APP)/subsystem3/s3_nav.c \
           $(ROOT)/testing/subsystem3/test_main.c

S4_SRC  := $(APP)/subsystem4/terrain_logic.c \
           $(APP)/subsystem4/drv_lsm303dlhc.c \
           $(APP)/subsystem4/subsystem4_test.c \
           stubs/device_stub.c \
           s4_host_main.c

S5_SRC  := $(APP)/subsystem5/src/obstacle_test.c \
           $(APP)/subsystem5/src/scan_test_adapter.c \
           $(APP)/subsystem5/src/scan_engine.c \
           $(APP)/subsystem5/src/range_measurement.c \
           $(APP)/subsystem5/src/bypass_plan.c \
           $(APP)/subsystem5/src/scan_profile.c \
           $(APP)/subsystem5/src/forward_monitor.c \
           $(APP)/subsystem5/src/scan_look.c \
           $(wildcard $(ROOT)/testing/subsystem5/unit_tests/src/*.c) \
           s5_host_main.c

.PHONY: all s1 s2 s3 s4 s5 clean

all: s1 s2 s3 s4 s5
	@echo "[HOST] all suites passed"

s1:
	$(CC) $(STRICT) -Wconversion -Wsign-conversion -DS1_HOST_TEST \
	    -o $(OUT)/s1_logic $(S1_SRC)
	$(OUT)/s1_logic

s2:
	$(CC) $(STRICT) -DAPP_TEST=1 -o $(OUT)/s2_logic $(STUBS) $(S2_SRC)
	$(OUT)/s2_logic

s3:
	$(CC) $(STRICT) -Wconversion -I$(APP)/subsystem3 -o $(OUT)/s3_logic \
	    $(S3_SRC)
	$(OUT)/s3_logic

# Output goes to a file, not through a pipe: in a pipe the shell would
# report tee's exit status, so a crashed test (no FAIL line printed) would
# pass. The run's own status, a SUMMARY line and no FAIL line are all
# required.
#
# S4-H01 needs the sensor, so the host image is built without it
# (S4_HARDWARE_TEST=0) and reports it BLOCKED; only FAIL fails here.
s4:
	$(CC) $(STRICT) -DAPP_TEST=1 -DS4_HARDWARE_TEST=0 -o $(OUT)/s4_logic \
	    $(STUBS) $(S4_SRC)
	$(OUT)/s4_logic > $(OUT)/s4.txt 2>&1; status=$$?; \
	    cat $(OUT)/s4.txt; exit $$status
	@grep -q "SUMMARY" $(OUT)/s4.txt
	@! grep -qE " FAIL " $(OUT)/s4.txt

# The S5 runner prints one line per case; any FAIL or BLOCKED fails here.
s5:
	$(CC) $(STRICT) -DAPP_TEST=1 -o $(OUT)/s5_logic $(STUBS) $(S5_SRC)
	$(OUT)/s5_logic > $(OUT)/s5.txt 2>&1; status=$$?; \
	    cat $(OUT)/s5.txt; exit $$status
	@grep -q "SUMMARY" $(OUT)/s5.txt
	@! grep -qE " (FAIL|BLOCKED) " $(OUT)/s5.txt
	$(CC) $(STRICT) -DAPP_TEST=1 -Istubs -o $(OUT)/s5_view \
	    $(APP)/subsystem5/src/scan_telemetry.c s5_telemetry_test.c
	$(OUT)/s5_view

clean:
	rm -f $(OUT)/s1_logic $(OUT)/s2_logic $(OUT)/s3_logic \
	    $(OUT)/s4_logic $(OUT)/s5_logic $(OUT)/s5_view
