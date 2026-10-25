# Build Pico W test firmware; tests execute only after flashing the board.
.PHONY: all test
.DEFAULT_GOAL := test

all: test

test:
	$(MAKE) -C ../../../build_make TEST=1
