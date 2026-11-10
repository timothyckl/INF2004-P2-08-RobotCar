# Subsystem 3: line sensing and barcode foundation

Every build uses the agreed pin mapping in `AGENTS.md`, viewed from behind:

- front-left AO: GP26/ADC0;
- front-right AO: GP27/ADC1;
- side barcode DOUT: GP16.

The front sensors have analogue outputs only; both front DOUT wires and the
barcode AO wire are disconnected, and GP6 is unused.

Subsystem 3 never writes motor PWM. In the line-test integration profile it
sends line positions to the Vehicle Controller, and only Motion Control writes
PWM. The existing consumer task and its mailbox/statistics interfaces remain
present for the other starter subsystems.

## On-car raw input monitor

1. Connect all three sensor modules to 3V3 and a common ground. From behind
   the robot, connect front-left AO to GP26, front-right AO to GP27, and the
   side barcode DOUT to GP16. Disconnect both front DOUT wires and the barcode
   AO wire.
2. Build the dedicated monitor with `TEST=1`, `S3_HW_TEST=1`, and
   `CRASH_REPORT=0`:

   ```text
   make -C build_make TEST=1 S3_HW_TEST=1 CRASH_REPORT=0 -j8
   ```

3. Flash the generated `_test_s3hw.uf2` and open the serial console. This
   profile starts only the Subsystem 3 sensor monitor; the other application
   subsystems and their output are suppressed.
4. Move each front sensor over representative white, solid black, and faded
   black sections at its installed height. The monitor prints the 12-bit ADC
   readings at 10 Hz until the Pico is reset. Cover or uncover the side sensor
   to check the barcode DOUT separately.

Example raw diagnostic output:

```text
s3 raw: left_adc=742 right_adc=801 barcode_do=1
```

`left_adc` and `right_adc` are raw values from 0 through 4095; the example is
illustrative and is not a validated threshold. `barcode_do` is the raw GP16
digital level. Record stable ranges for every tested surface. This stage does
not label readings as white or black and does not command the motors.

`S3_LINE_TEST=1` uses these same two AO inputs for line following. The former
`S3_CAL_TEST=1` digital-output calibration profile was removed on 2026-11-10
because the front sensors have no digital outputs; the makefile refuses it.

A normal `TEST=1` build without either S3-specific option reports the hardware
checks as blocked and runs the regular subsystem tests instead.

## Button-controlled line-follow bench trial

First verify both AO channels with `S3_HW_TEST=1`. Raise the wheels clear of
the floor for the first moving test and build:

```text
make -C build_make TEST=1 S3_LINE_TEST=1 CRASH_REPORT=0 -j8
```

Flash the generated `_test_s3line.uf2`. This profile starts only Subsystems 2
and 3. It samples front-left AO on GP26 and front-right AO on GP27 every 10 ms.
The drivetrain follows the agreed pin mapping used by every build.
It waits for Motion Control to become ready, then continuously reports fresh
line positions while STOP remains latched. Place the car on the track and press
the Robo Pico programmable button on GP20 to start. Press GP20 again to stop;
after the wheels stop, another press resets and starts a new run. The input is
active-low and debounced for 30 ms by the Vehicle Controller.

Each channel has an independent filtered threshold because the measured sensor
ranges differ. The initial white baselines are 136 counts on the left and 180
on the right. While a channel remains white, its baseline moves by at most one
count per eight samples and the threshold follows it. The left baseline is
bounded to 96..168 counts and uses 32-count detection and 20-count release
margins. The right baseline is bounded to 100..300 counts and uses 500-count
detection and 350-count release margins. A black reading freezes baseline
adaptation so the algorithm cannot learn the line as white. These values come
from the 2026-11-05 bench observations and still require on-track validation.

The serial heartbeat prints raw ADC values, filtered values and the current
thresholds, plus a 0..100 steering strength for a single-black observation.
The strength is normalised independently against the measured solid-black
references of 1500 counts on the left and 3300 counts on the right. If a sensor
or mounting height changes, rerun `S3_HW_TEST=1` and review all white,
solid-black and faded-black ranges before changing them.

There is no automatic run-time limit. Its verified sensor mapping is:

| Left | Right | Request |
|---|---|---|
| WHITE | WHITE | centred straight |
| BLACK | WHITE | left correction |
| WHITE | BLACK | right correction |
| BLACK | BLACK | crossing/junction candidate |

BLACK/BLACK must persist for 100 ms before the controller confirms a junction;
brief BLACK/BLACK observations retain the preceding curve command. A confirmed
junction crosses straight and must clear within 1500 ms. Its first clear reading
starts a straight exit of at least 600 ms. After that minimum, a side-black
reading that persists for 150 ms resumes normal curve tracking, while
WHITE/WHITE must remain stable for 100 ms before normal following resumes. The
exit requests STOP after 2500 ms if neither pattern becomes stable.

It also stops if reports become invalid or stale, a queue/API operation fails,
or the GP20 button is pressed while running. The supervised USB-logging profile
samples the front sensors every 10 ms, requires three consecutive matching
samples, and refreshes the Vehicle Controller report every 30 ms. It allows at
most 500 ms between valid line reports. The
outer/straight speed is 40 mm/s. During an ordinary correction, the inside
wheel scales from 38 down to 32 mm/s as analog contrast rises from 0 to 100,
while the other wheel remains at 40 mm/s. This replaces the immediate fixed
32/40 step that caused excessive left-right oscillation on straights. `LEFT`
slows the left wheel and drives the right wheel; `RIGHT` drives the left wheel
and slows the right wheel. A single-black observation must persist for 200 ms
before the sustained curve target becomes a rolling 5/35 mm/s LEFT arc or
35/5 mm/s RIGHT arc. If WHITE/WHITE appears during a sustained curve, LEFT is
retained for at most 250 ms and RIGHT for 150 ms before returning to straight.
The lower forward targets provide additional margin below the test profile's
PWM ceiling, while the larger wheel-speed ratio produces a tighter rolling
curve without braking a wheel. The sensor labels and GPIO assignments are
unchanged. The serial monitor prints the classified sensor state at least every
500 ms, even when it has not changed.
The earlier 30/15 mm/s target did not reduce the observed physical speed and
the controller entered Fault. These are provisional commands, not measured
wheel speeds. Keep the wheels raised for the first run because the motor
polarity, encoder polarity and PID tuning still require physical validation.

The dedicated line-follow image also limits Motion Control to 20% PWM. This
provisional ceiling prevents an uncalibrated speed loop or missing encoder
feedback from increasing the motors to full power. A zero wheel target clears
that wheel's PID history and applies the motor driver's brake state. If the
outside motor cannot start at 20%, Motion Control reports a stall fault and
stops; do not increase the ceiling until the encoder counts and measured wheel
speeds are verified.

WHITE/WHITE is intentionally the centred pattern for the separated-sensor
geometry, but it is also possible after the car loses the line completely.
The bounded curve hold cannot distinguish those two physical situations.
Supervise the whole run and keep a hand near GP20; do not use this as an
unattended line follower.

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

These tests cover the adaptive analogue thresholds, filtering, the verified
four-state steering table, and the existing synthetic
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
