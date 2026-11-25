# Subsystem 3: line sensing and barcode decoding

Every build uses the agreed pin mapping in `AGENTS.md`, viewed from behind:

- front-left AO: GP26/ADC0;
- front-right AO: GP27/ADC1;
- side barcode AO: GP28/ADC2 (Grove 7).

All three sensors have analogue outputs only; their DOUT wires are
disconnected, and GP6 is unused. Keep the GP28 battery-sense jumper open.

Subsystem 3 never writes motor PWM. In the mission image it sends line
positions (IF-06) and decoded letters (IF-21) to the Vehicle Controller,
and only Motion Control writes PWM. The starter consumer task and its
mailbox and statistics interfaces were removed on 2026-11-20 (S2 decision
29).

## On-car raw input monitor

1. Connect all three sensor modules to 3V3 and a common ground. From behind
   the robot, connect front-left AO to GP26, front-right AO to GP27, and the
   side barcode AO to GP28 (Grove 7). Disconnect all three DOUT wires.
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
   readings at 10 Hz until the Pico is reset. Move the side sensor over white
   and black to check the barcode AO the same way.

Example raw diagnostic output:

```text
s3 raw: left_adc=742 right_adc=801 barcode_adc=760
```

`left_adc`, `right_adc` and `barcode_adc` are raw values from 0 through 4095;
the example is illustrative and is not a validated threshold. Record stable ranges for every tested surface. This stage does
not label readings as white or black and does not command the motors.

The mission image uses these same two AO inputs for line following. The former
`S3_CAL_TEST=1` digital-output calibration profile was removed on 2026-11-10
because the front sensors have no digital outputs; the makefile refuses it.

A normal `TEST=1` build without either S3-specific option reports the hardware
checks as blocked and runs the regular subsystem tests instead.

## Line following in the mission image

First verify both AO channels with `S3_HW_TEST=1`. Then build the mission
(S2 decision 28); `TEST=1 S3_LINE_TEST=1` builds the same mission:

```text
make -C build_make NET=lwip MQTT=1 -j8
```

The loop is `s3_follow_run()` in `s3_follow.c`, run by `usermain()` after
Subsystems 2, 4 and 5 start. It samples front-left AO on GP26 and
front-right AO on GP27 every 10 ms.
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
bounded to 96..168 counts and uses 100-count detection and 80-count release
margins (raised from 32 and 20 on 2026-11-16, decision 8, so that the car
rolling on the hump does not read white as black). The right baseline is bounded to 100..300 counts and uses 500-count
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

These tests cover the adaptive analogue thresholds, filtering, the
four-state steering table, the letter-to-command table and Code 39 A--Z
decoding in both travel directions, including the recorded car captures,
one misread element and speed changes (S3-U02), noise blips and broken
frames (S3-U03), stray marks and double faults (S3-U04), and a replay of
the 19 captures logged on the car through `s3_barcode_update()` (S3-U05).
On macOS or Linux, `make -C testing/host s3` runs the same cases. The
summary line counts passed checks, failures and known defects.

F-18 (fixed after the demo, S3 decision 9): in the demonstrated firmware a
single 20 ms blip inside the code gave a wrong letter in 25 of 234
synthetic payload cases. That was a count of test cases, not a real-world
error rate, and no wrong letter was seen on the car. S3-U03 now checks
10 ms and 20 ms blips at three places in every run of every letter (4,524
cases): none gives a wrong letter and none is rejected.

S3-U04 reports **one known defect, F-25**: a capture that has lost two runs
at one end *and* holds a blip can still give a wrong letter (39 of 702
synthetic cases). A 29-run capture cannot tell such a blip from a genuine
short element, and repairing it made blurred narrow elements decode wrongly,
so it is left as it is ([log](../../testing/subsystem3/logs/2026-11-25_s3-f18-fix.txt)).
The host tests do not verify wiring, optical distance,
surface contrast, ambient light, sampling timing, line following, motor
behaviour or the printed course barcodes.

## Modules

- `s3_bar` provides bounded edge capture, timeout handling, and a configurable
  one-symbol decoder. Sentinels and gaps must match exactly; the payload may
  differ from one dictionary entry by one element only when that entry is
  uniquely closest.
- `s3_scan` frames captures from timestamped side-sensor edges, derives the
  narrow/wide boundary from each capture, and decodes forwards then reversed.
  A 24--29 run capture is accepted when one intact end supplies a
  sentinel, both gaps and the full payload. A 31-run capture has one extra
  bar and space: if its shortest run is at either end it is a stray mark
  and is dropped; if it is inside the code it is a noise blip, and the
  element it split is rebuilt from the blip and its two neighbours (F-18).
  Only that one window is decoded.
- `s3_code39` holds the standard Code 39 A--Z patterns and the `*` sentinel.
- `s3_barcode` classifies each ADC2 sample with an unfiltered adaptive
  threshold (white baseline 154..184, black at baseline + 400, release at
  baseline + 300) and feeds level changes, timestamped from the 1 MHz
  `TIMERAWL`, to `s3_scan`. A run over 2.5 s aborts a capture (raised from
  800 ms on 2026-11-17, see `s3_barcode.c`) and 700 ms of white ends one.
  All of these values are provisional.
- `s3_scan`, `s3_code39`, the `s3_bar` near-match rule and the barcode
  thresholds were ported from `feature/subsystem3-ir-line-barcode` (commit
  `e6a7027`), where they were measured on 2026-11-11 at about 40 mm/s.
  Printed barcodes were read on this car from 2026-11-17 (`TEST_LOG.md`)
  and on demo day; reliability stays below 4 of 5 (SYS-02).
- `s3_nav` maps a decoded letter to LEFT, RIGHT, STRAIGHT, U-TURN or none.
  It does not hold or time commands: the Vehicle Controller does that in
  `nav_control.c` (S2 decisions 26 and 27).

## Barcode reporting in the line-follow image

The mission image reads ADC2/GP28 in the same loop as the front sensors,
so one task owns the ADC and the barcode state. A 10 ms cyclic handler
(`S3_IR_SAMPLE_MS`) wakes the loop (decision 10). Until 2026-11-25 it slept
with `tk_dly_tsk(10)` and really ran every 20 ms, because the kernel adds a
tick to every delay (F-01). The steering filter (60 ms) and the
white-baseline step (160 ms) keep the durations used on the car; every 10 s
the loop prints its period, worst pass time and overruns (`s3 loop:`).
The 10 ms period is implemented; a car run to confirm it is pending. The barcode
never steers; each decoded letter goes to the Vehicle Controller, which
acts on a mapped command at the next junction (below). Once a capture looks like a barcode (three
runs of at least 60 ms), each line report carries `S2_LINE_FLAG_BARCODE`
until it ends, and the controller replaces strong curves with gentle
corrections (S2 decision 25). A failed ADC2 read abandons the capture
without stopping line following. Serial lines:

```text
s3 bar: DECODED A (decoded=1 errors=0)
s3 bar: capture rejected, runs=25 threshold=569 (decoded=1 errors=1)
s3 bar: single bar (crossing line), ignored
```

To validate on the car:

1. With the `S3_HW_TEST=1` raw monitor, hold the car still and record
   `barcode_adc` on white, solid black and faded black at the mounted height.
   White must stay within 154..184 and black above about 600, or the
   thresholds in `s3_barcode.c` need re-measuring.
2. With the mission image, drive past each course barcode in both
   directions and record each `s3 bar:` line with the expected letter.
3. Record the results in `testing/subsystem3/TEST_LOG.md`.

## Acting on a barcode (S2 decision 26)

A decoded letter is mapped by `s3_nav_command()`. At boot the table is
the brief's example A-D, plus the test letter F, set at build time:

```sh
make NET=lwip MQTT=1 S3_NAV_F=1 -j8   # F = left
```

`S3_NAV_F` is 0 (none), 1 (left), 2 (right), 3 (straight) or 4 (U-turn);
the image name ends `_navF<n>`. Other letters are reported and ignored.
When the course letters are announced, set them from the dashboard's
Barcode letters table instead of rebuilding: it sends `L=<letter>` and so
on over MQTT, and `s3_nav_set_letter()` makes that letter the only one for
the command (S1 decision 6). The table returns to the boot map on a
restart; the dashboard sends it again when it sees one.
The mapped command goes to the Vehicle Controller, which carries it out at
the next junction. Expect these serial lines:

```text
s3 bar: DECODED F (decoded=1 errors=0)
[S2] nav: LEFT held for the next junction (scan 1)
[S2] line: junction confirmed; crossing straight
[S2] nav: junction; stopping to turn
[S2] nav: step 118 mm onto the junction
[S2] nav: spinning left until a sensor sees the line
[S2] nav: on the new line; following
```
