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

S1_SRC  := $(APP)/subsystem1/command_parser.c \
           $(APP)/subsystem1/telemetry_encoder.c \
           $(APP)/subsystem1/comms_timer.c \
           $(APP)/subsystem1/service_bridge.c \
           $(APP)/subsystem1/comms_test.c \
           $(ROOT)/testing/subsystem1/logic_main.c

S2_SRC  := $(APP)/subsystem2/motion_logic.c \
           $(APP)/subsystem2/controller_logic.c \
           $(APP)/subsystem2/pid.c \
           $(APP)/subsystem2/odometry.c \
           $(APP)/subsystem2/motion_api.c \
           $(APP)/subsystem2/test_motion.c \
           $(APP)/subsystem2/test_controller.c \
           s2_host_main.c

S3_SRC  := $(APP)/subsystem3/s3_ir.c \
           $(APP)/subsystem3/s3_bar.c \
           $(APP)/subsystem3/s3_nav.c \
           $(ROOT)/testing/subsystem3/test_main.c

S5_SRC  := $(APP)/subsystem5/src/obstacle_test.c \
           $(APP)/subsystem5/src/scan_test_adapter.c \
           $(APP)/subsystem5/src/scan_engine.c \
           $(APP)/subsystem5/src/range_measurement.c \
           $(APP)/subsystem5/src/bypass_plan.c \
           $(APP)/subsystem5/src/forward_monitor.c \
           $(wildcard $(ROOT)/testing/subsystem5/unit_tests/src/*.c) \
           s5_host_main.c

.PHONY: all s1 s2 s3 s5 clean

all: s1 s2 s3 s5
	@echo "[HOST] all suites passed"

s1:
	$(CC) $(STRICT) -Wconversion -Wsign-conversion -DAPP_TEST=1 \
	    -DS1_HOST_TEST -o $(OUT)/s1_logic $(S1_SRC)
	$(OUT)/s1_logic

s2:
	$(CC) $(STRICT) -DAPP_TEST=1 -o $(OUT)/s2_logic $(STUBS) $(S2_SRC)
	$(OUT)/s2_logic

S3_TRACK_SRC := $(APP)/subsystem3/s3_ir.c $(APP)/subsystem3/s3_bar.c \
           $(APP)/subsystem3/s3_track.c $(APP)/subsystem3/s3_scan.c \
           $(ROOT)/testing/subsystem3/test_track_scan.c

s3:
	$(CC) $(STRICT) -Wconversion -I$(APP)/subsystem3 -o $(OUT)/s3_logic \
	    $(S3_SRC)
	$(OUT)/s3_logic
	$(CC) $(STRICT) -Wconversion -I$(APP)/subsystem3 -o $(OUT)/s3_track \
	    $(S3_TRACK_SRC)
	$(OUT)/s3_track

# The S5 runner prints one line per case; any FAIL or BLOCKED fails here.
s5:
	$(CC) $(STRICT) -DAPP_TEST=1 -o $(OUT)/s5_logic $(STUBS) $(S5_SRC)
	$(OUT)/s5_logic | tee $(OUT)/s5.txt
	@! grep -qE " (FAIL|BLOCKED) " $(OUT)/s5.txt

clean:
	rm -f $(OUT)/s1_logic $(OUT)/s2_logic $(OUT)/s3_logic $(OUT)/s3_track \
	    $(OUT)/s5_logic
