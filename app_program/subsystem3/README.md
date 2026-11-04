# Subsystem 3: line sensing and barcode foundation

The current hardware stage verifies only the two front digital IR sensors:

- front-left DOUT: GP6;
- front-right DOUT: GP16;
- side barcode DOUT: GP4, deliberately excluded from steering and this test.

`s3_ir_gpio` configures GP16 and GP6 as SIO inputs, disables their output
drivers, and reads both pins into a two-bit mask. It does not enable internal
pull resistors or assume whether white produces a high or low signal.
`s3_ir` calibrates the raw polarity from stable white and black observations.

Subsystem 3 never writes motor PWM. In the line-test integration profile it
sends line positions to the Vehicle Controller, and only Motion Control writes
PWM. The existing consumer task and its mailbox/statistics interfaces remain
present for the other starter subsystems.

## On-car raw input monitor

1. Connect both sensor modules to 3V3 and a common ground. From behind the
   robot, the left DOUT is GP6 and the right DOUT is GP16. GP17 is unused;
   connect only DOUT for this test. Keep the Grove GP26 contact beside GP6
   disconnected because GP26 belongs to motor 1's encoder.
2. Build the dedicated monitor with `TEST=1`, `S3_HW_TEST=1`, and
   `CRASH_REPORT=0`:

   ```text
   make -C build_make TEST=1 S3_HW_TEST=1 CRASH_REPORT=0 -j8
   ```

3. Flash the generated `_test_s3hw.uf2` and open the serial console. This
   profile starts only the Subsystem 3 sensor monitor; the other application
   subsystems and their output are suppressed.
4. Cover and uncover either sensor with a finger and watch its raw DOUT value.
   The monitor continues at 10 Hz until the Pico is reset.

Example raw diagnostic output:

```text
s3 raw: left=0 right=1
```

`left` is GP6 and `right` is GP16, using robot-relative sides viewed from
behind. A value of `0` is LOW and `1` is HIGH. This
stage does not label readings as white or black and does not advance into a
calibration prompt. If a value never changes, check that sensor's threshold
adjustment, DOUT wiring, power, ground, and mounting height.

A normal `TEST=1` build without either S3-specific option reports the hardware
checks as blocked and runs the regular subsystem tests instead.

## Guided digital calibration

Keep the raw monitor available for wiring checks. For polarity calibration,
build the separate calibration profile:

```text
make -C build_make TEST=1 S3_CAL_TEST=1 CRASH_REPORT=0 -j8
```

Flash the generated `_test_s3cal.uf2`. Follow four independent prompts:

1. left sensor over white;
2. left sensor over black;
3. right sensor over white;
4. right sensor over black.

Only the named sensor needs to be positioned for each prompt. Hold it still at
its normal operating height. Each observation uses 50 samples and is accepted
when at least 45 agree. Both sensors must be stable and must change digital
level between white and black. A failed sequence repeats automatically.

After S3-H01 through S3-H03 pass, the monitor continuously labels each input
as `WHITE` or `BLACK`, reporting changes immediately and unchanged readings
once per second. Calibration is held in RAM and is repeated after a reset.

This learns DOUT polarity; it cannot adjust the module's comparator threshold.
Use the raw monitor and the physical potentiometer to obtain a clean transition
before running calibration.

## Button-controlled line-follow bench trial

Only run this after S3-H01 through S3-H03 pass. Raise the wheels clear of the
floor and build:

```text
make -C build_make TEST=1 S3_LINE_TEST=1 CRASH_REPORT=0 -j8
```

Flash the generated `_test_s3line.uf2`. This profile starts only Subsystems 2
and 3 and loads the verified digital polarity from firmware: LOW is WHITE and
HIGH is BLACK for both sensors. It does not repeat the four calibration prompts.
It waits for Motion Control to become ready, then continuously reports fresh
line positions while STOP remains latched. Place the car on the track and press
the Robo Pico programmable button on GP20 to start. Press GP20 again to stop;
after the wheels stop, another press resets and starts a new run. The input is
active-low and debounced for 30 ms by the Vehicle Controller.

The saved value is polarity, not an optical threshold. The modules' physical
potentiometers retain their comparator thresholds. If a sensor module or its
wiring changes, rerun `S3_CAL_TEST=1`, verify the new WHITE/BLACK levels, and
update the saved constants before using the line profile again.

There is no automatic run-time limit. Its verified sensor mapping is:

| Left | Right | Request |
|---|---|---|
| WHITE | WHITE | straight |
| BLACK | WHITE | left correction |
| WHITE | BLACK | right correction |
| BLACK | BLACK | STOP |

It also stops if reports become invalid or stale, a queue/API operation fails,
or the GP20 button is pressed while running. The
outer/straight speed is 30 mm/s and the inner correction speed is 15 mm/s;
these are provisional commands, not measured wheel speeds. Keep the wheels
raised for the first run because the motor polarity, encoder polarity and PID
tuning still require physical validation.

WHITE/WHITE is ambiguous: it means centred for the current sensor placement,
but it can also mean the car has completely lost the line. Supervise the whole
run and keep a hand near GP20; do not use this as an unattended line follower.

## Save every serial line

Close VS Code Serial Monitor first because only one program can own COM4, then
run the logger from the project root:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem3/capture_serial.ps1 -Port COM4
```

It displays every received line and writes it immediately, with a host
timestamp, under `testing/subsystem3/logs/`. Start the logger first and then
reset the Pico if the boot messages must be included. Press Ctrl+C to close the
port and finish the log. Change `-Port` if Windows assigns a different COM
number.

## Host tests

From the project root, with GCC on `PATH`:

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem3/run_tests.ps1
```

These tests cover two-sensor calibration, polarity normalization, filtering,
the verified four-state steering table, and the existing synthetic
barcode/navigation foundation. They do not verify
wiring, optical distance, surface contrast, ambient-light immunity, sampling
performance, line following, motor behaviour, or the printed course barcode.

## Existing software foundation

- `s3_bar` provides bounded edge capture, timeout handling, and a configurable
  synthetic one-symbol decoder. Its framing, width windows, alphabet, reverse
  travel, and speed tolerance are not validated against the course barcode.
- `s3_nav` provides the current synthetic A/B/C/D command mapping, duplicate
  suppression, expiry, STOP cancellation, and controller-authority gating. It
  contains no motor actions or validated turn angles.

Before barcode integration, confirm the printed symbology and measured timing,
keep raw edge capture separate from filtered steering samples, and use the
team's shared GPIO interrupt dispatcher. Barcode state must be task-owned or
passed through an agreed bounded queue; do not share mutable capture state with
an ISR or another core without concurrency protection.

Line following should be added only after both front sensors pass the physical
white/black test. Steering requests must go through the Vehicle Controller;
only Motion Control may write motor PWM.
