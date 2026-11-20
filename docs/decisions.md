# Decision Resolutions

## Subsystem 1 — WiFi/MQTT (Hong Yih)

### 1. Telemetry frame version 2 (2026-11-17)

Approved by Dalton on the integration branch.

**Problem.** The version 1 frame carried only time, run distance and three
flags, so the dashboard could not show most of the telemetry the project
brief lists for subsystem 1: robot state, motor speed, line sensor status,
hump measurement and obstacle scan results.

**Decision.** A 27-byte version 2 frame, little-endian, published on the
same topic every 1 s (period unchanged and still unvalidated). Bytes 0–7
keep their version 1 positions, so a decoder can read the version and the
old fields before it knows the rest.

| Byte | Field | Range | Source |
|---:|---|---|---|
| 0 | version | 2 | S1 |
| 1 | flags0: bit 0 distance valid, 1 stopped, 2 MQTT link up, 3 STOP latched, 4 motion valid, 5 line fresh, 6 hump valid, 7 zero | — | S2, S4, S1 |
| 2–5 | `captured_ms`, `uint32_t` | wraps | S2 controller clock |
| 6–7 | run distance mm, `uint16_t` | 0..65535, saturated | S2 |
| 8 | flags1: bit 0 forward valid, 1 profile valid, 2 shape valid, 3–7 zero | — | S5 |
| 9 | state (bits 0–3), mode (bits 4–7) | `S2_SYS_*` 1..5, `S2_MODE_*` 0..5 | S2 |
| 10 | fault (bits 0–3), line (bits 4–7) | `S2_FAULT_*` 0..4, `S2_LINE_*` 0..4 | S2 |
| 11 | scan result (bits 0–3), bypass side (bits 4–7) | `SCAN_TELEMETRY_*` 0..5, side 0..2 | S5 |
| 12–13 | left wheel mm/s, `int16_t` | motion range | S2 |
| 14–15 | right wheel mm/s, `int16_t` | motion range | S2 |
| 16–17 | heading, centidegrees, `int16_t`, left positive | −17999..18000 | S2 odometry |
| 18–19 | ultrasonic forward range mm, `uint16_t` | 20..4000 | S5 |
| 20–21 | nearest obstacle ray mm, `uint16_t` | as above | S5 profile |
| 22–23 | observed obstacle span mm, `uint16_t` | as above | S5 shape estimate |
| 24–25 | obstacle centre offset mm, `int16_t`, left positive | sensor-relative | S5 shape estimate |
| 26 | highest hump this run mm, `uint8_t` | 0..255, saturated | S4 (IF-18) |

Codes (wire value = provider's constant):

- state: 1 Initialising, 2 Ready, 3 Running, 4 Stopped, 5 Fault
- mode: 0 none, 1 Follow, 2 Navigate, 3 Scan, 4 Bypass, 5 Reacquire
- fault: 0 none, 1 init, 2 motion, 3 scan, 4 delivery
- line: 0 centre, 1 left, 2 right, 3 both black, 4 stop
- scan result: 0 none, 1 plan, 2 clear, 3 no route, 4 fault, 5 cancelled
- bypass side: 0 none, 1 left, 2 right

**Rules.**

- A field whose validity flag is clear is sent as zero; a decoder must test
  the flag, never the value.
- An enumerated code above 14 is sent as 15 (invalid) rather than masked,
  so a provider bug cannot pass as a valid code.
- Enumerated fields share a byte in nibbles. 16-bit fields stay
  byte-aligned: packing the 12-bit ranges more tightly would save 2 bytes
  of a 27-byte frame sent once a second, at the cost of a harder decoder.
- Barcode is not in the frame. It is added (as version 3) when S3's
  barcode decoder is connected to the controller; until then there is no
  source for it.

**Concurrency.** The frame is built in the WiFi service task, which must
never wait. Motion Control's `motion_get_status()` waits up to 10 ms, so
the Vehicle Controller copies wheel speeds and heading into `robot_state`
each cycle (`S2_RS_MOTION_VALID`), together with the latest line position
while it is fresh (`S2_RS_LINE_FRESH`, line-follow image only).
`robot_read_telemetry()` then polls `state_mutex`, S4's mutex and S5's mutex
with `TMO_POL`, one after another and never nested. A busy `state_mutex`
skips that publication; a busy S4 or S5 lock clears only that provider's
flags. `s2_robot_state_t` grows from 20 to 28 bytes (S2-U39).

**One type, not two.** The decoded snapshot is defined once, by S1, in the
kernel-free `subsystem1/telemetry_snapshot.h`, and S2 fills it directly.
This removes S2's duplicate `s2_telemetry_t` and the mapping file
`s2_s1_telemetry.c`. The kernel/`<stddef.h>` `size_t` conflict only
applies to `telemetry_encoder.h`, which kernel-side code does not include.

**Tests.** S1-U05 (golden 27-byte frame), S2-U37 (state, codes, motion,
line), S2-U72 (obstacle fields), the S2-U39 size and
`test_ctrl_robot_state` (speeds, heading truncation, unreadable status) on
host. S1-H02 must be repeated on the car with the version 2 decoder.

### 2. Merge into the integration branch (2026-11-17)

Approved by Dalton on the integration branch.

Subsystem 1 comes from `feature/subsystem1-integration` (`b21d964`), the
newest S1 code and one built on this branch, rather than
`feature/full-system-integration`, whose S1 code is older and whose other
subsystems predate this branch. On merge:

- The `s1_compat_task` that kept `tid_led` for IF-05 is removed: nothing on
  this branch uses IF-05, so S1 creates no task or kernel object, and GP19
  is free.
- Committed `__pycache__` files and dashboard session logs are removed and
  ignored. The dashboard is to be redesigned against telemetry version 2.

### 3. Commands are stamped with the controller's clock (2026-11-17)

Approved by Dalton on the integration branch.

**Problem.** After the merge (decision 2), remote `RESET` sent while the
car was in Fault was acknowledged by the car (PUBACK) but had no effect.
`b21d964` stamps each command's `received_ms` with lwIP's `sys_now()`,
which is the kernel's operating time (`tk_get_otm`, ms since boot). The
Vehicle Controller judges command age against `tk_get_tim`, a different
base (telemetry time read about 3.36e9 ms), so every START and RESET
appeared about 38 days old and was dropped as expired. STOP was unaffected:
it uses the urgent flag, which has no age check.

**Decision.** Restore the design of `feature/full-system-integration`,
which passed S1-H01 on the car on 2026-11-04: S2 exports `s2_clock_ms()`
(`tk_get_tim`, kernel-free declaration in `s2_s1_adapter.h`), and
`mqtt_service_data()` stamps every command with it. `mqtt_utk.c` no longer
passes a time; lwIP's own timers keep using `sys_now()`. `s2_local_command()`
uses the same function, so GP20 and MQTT commands share one clock.

**Tests.** S1-H01 (START, RESET) must be repeated on the car.

### 4. Radio power save off, and telemetry drop counters (2026-11-17)

Approved by Dalton on the integration branch. Not yet validated on the car.

**Problem.** On a 140 s lap only 76 of about 141 telemetry frames arrived,
remote STOP was acknowledged 11 s late, and even with the car stationary
frames reached the broker in bursts (several frames 1 s apart in robot time
arriving within 60 ms). The radio runs the SDK default `CYW43_PERFORMANCE_PM`
(PM2 power save), which sleeps between AP beacons; that delays incoming ACKs,
stalls the TCP send window, and `mqtt_publish()` then refuses frames. Its
return value was ignored, so the drops were silent.

**Decision.**

- `cyw43_utk_task` sets `CYW43_NONE_PM` on every link-up (a new association
  restores the default). The cost is higher radio current while connected,
  acceptable on the car's battery for a run; not measured.
- `telemetry_timeout()` counts each 1 s period as sent, refused by
  `mqtt_publish()` (with the last error), no snapshot, or not connected, and
  prints and publishes (topic `p2-08/p2-08-car-01/stats`, cumulative since
  boot) the counts every 10 periods. This separates link congestion from
  a provider lock that is busy.

**Tests.** Repeat S1-H02 driving and S1-H01 STOP while running: pass if
frames stay about 1 s apart and the counters show no refused frames.

### 5. Telemetry frame version 3: barcode and stop cause (2026-11-20)

Approved by Dalton on the integration branch. Host-tested only; not yet
run on the car.

**Problem.** In the 2026-11-20 demo the dashboard could not show three
items of the brief: the barcode was detected, which letter it was, and
which command the car took from it. Nor could it tell a pick-up (the end
of the demo) from a dashboard STOP or the GP20 button: each showed only
STOPPED with STOP latched. These were on the serial console only.

**Decision.** Version 3 is version 2 with two bytes appended (29 bytes).
Bytes 1–26 keep their meanings, so the version 2 decoder logic is reused
for them.

| Byte | Field | Range | Source |
|---:|---|---|---|
| 0 | version | 3 | S1 |
| 8 | flags1 bit 3: barcode valid (bits 0–2 as version 2) | — | S2 |
| 27 | barcode letter (bits 0–4), its command (bits 5–7) | letter 0..25 = `A`..`Z`; command `S2_NAV_*` 0..4 | S3 via S2 |
| 28 | stop cause (bits 0–3), barcode count (bits 4–7) | `S2_STOP_*` 0..11; count 0..15, wraps | S2 |

- **Widths.** The decoder reads A–Z only (`s3_code39.c`), 26 values, so
  the letter takes 5 bits; 31 means a letter outside A–Z, sent rather than
  masked (the decision 1 rule). Commands are 0..4 in 3 bits, 7 invalid.
  The stop cause fits the existing nibble rule (above 14 sent as 15).
- **Barcode.** The last letter decoded since power-up, sent while flags1
  bit 3 is set. The letter is sent even when it maps to no command (0).
  The count is the low 4 bits of the number of letters decoded, so a
  dashboard can see the same letter read twice. A frame every 1 s cannot
  miss a change unless 16 letters are read in that second; a barcode
  takes well over a second to cross.
- **Stop cause.** Sent only while STOP is latched (flags0 bit 3); zero
  otherwise. Codes and rules are S2 decision 27.
- The decoder `testing/subsystem1/decode_telemetry_v2.py` reads versions
  2 and 3, so the 2026-11-17 logs still decode; it keeps its name because
  earlier test records cite it.

**Tests.** S1-U05 (revised golden 29-byte frame; invalid letter and
command, count wrap, cause only while latched), S2-U37, S2-U39 and S2-U40
(revised), S2-U78 (new), and the dashboard unit tests. INT-07 must run on
the car.

## Subsystem 2 — Vehicle and Motion Control (Dalton)

### 1. Encoder and IMU pins

The agreed mapping assigns motor 1's encoder to GP26 (A) / GP27 (B), motor 2's
encoder to GP7 (A) / GP28 (B), and the IMU to I2C0 at GP0 (SDA0) / GP1
(SCL0). Motor 1 (GP8/GP9) drives the right wheel and motor 2 (GP10/GP11) the
left. Consequence: `CRASH_REPORT=1` and `CONSOLE=uart` cannot be used because
UART0 would take the IMU pins. ADC pin initialisation stays disabled on
GP26-GP28, and the GP28 battery-sense jumper stays open.

### 2. PWM block reset

Motion Control releases the PWM block from reset in its own driver instead of
setting `USE_PTMR`, which would make the kernel use PWM as its physical timer.
It only clears that reset and never asserts it, because the servo (subsystem
5) shares the PWM block.

### 3. Motor protection

The RP2040 hardware watchdog is kicked only by Motion Control, once per control
period, and is armed only outside `TEST=1` and bench builds; a hung or starved
Motion Control task resets the chip, which stops the motors. Motion Control
ramps duty increases (full scale in 200 ms, provisional); decreases, STOP and
Fault are never ramped.

### 4. Encoder sides, signs and counts per revolution (2026-11-04)

Measured on the bench (S2-H01 and S2-H03, `testing/subsystem2/TEST_LOG.md`)
and agreed by the subsystem 2 owner. This revises the encoder sides in
decision 1 and the pin mapping of 2026-11-03.

- Motor 1 drives encoder 1 and motor 2 drives encoder 2. With motor 1 on the
  right wheel, encoder 1 (GP26/GP27) is the right encoder and encoder 2
  (GP7/GP28) the left. Before this correction, each wheel's speed loop read
  the other wheel's encoder.
- Going forward, the right encoder counts up and the left counts down
  (`MC_LEFT_ENC_INVERT 1`, `MC_RIGHT_ENC_INVERT 0`). These signs were measured
  before the GPIO remap and must be reconfirmed on the new pins.
- `MC_COUNTS_PER_REV` is provisionally 1330 with x2 decoding: the median of
  the single hand-turned revolutions. A 10-revolution count will replace it.
- RP2040 latches rising and falling GPIO edge events independently. A
  coalesced A rise+fall pair is decoded from the current A/B levels as two
  steps instead of being recorded as an encoder error. Invalid snapshots and
  genuinely undecodable events retain the Motion Control safety fault.

### 5. Dedicated subsystem 3 installed-wiring profile (2026-11-05)

Edmund approved a build-profile exception for the supervised line-follow trial.
Only `S3_LINE_FOLLOW_TEST` uses the currently installed wiring: motor 1 drives
the physical left wheel, motor 2 drives the physical right wheel, encoder 2 on
GP7/GP28 measures the left wheel, and encoder 1 on GP0/GP1 measures the right
wheel. Subsystem 4 is not started and UART0 is disabled in this image. The motor
inversions reproduce the previously observed forward motion but remain
provisional until checked with the wheels raised.

Normal and integrated firmware retain decisions 1 and 4. This exception does
not assign replacement subsystem 4 pins and cannot become the integrated
mapping until those pins are agreed, checked against the hardware references
and recorded in `TEAM.md`.

### 6. Installed wiring becomes the only mapping (2026-11-10)

Supersedes decision 5 and the drivetrain parts of decisions 1 and 4. The
`S3_LINE_TEST` image completed a full lap on 2026-11-10, which confirms the
installed wiring. Viewed from behind (ultrasonic sensor forward), motor 1
(GP8 forward) and encoder 2 (GP7/GP28, counts up going forward) drive the left
wheel; motor 2 (GP11 forward) and encoder 1 (GP0/GP1, counts down going
forward) drive the right wheel. The front line sensors' analogue outputs use
ADC0/GP26 (left) and ADC1/GP27 (right); the GP26 sensor and the motor 1 wheel
are on the same side, which the lap requires. Every build now uses this
mapping, ADC0/ADC1 pin initialisation is always enabled, and the makefile
refuses `CRASH_REPORT=1` and `CONSOLE=uart`. The front sensors have no
digital outputs and GP6 is unused, so subsystem 3 always reads ADC0/ADC1 plus
the GP16 barcode input, and the `S3_CAL_TEST` digital-calibration profile is
removed (the makefile refuses it). The IMU (I2C1, GP2/GP3), HC-SR04
(GP4/GP5) and servo (GP12) keep the 2026-11-03 team mapping and are not yet
exercised on this branch. `AGENTS.md`, *Agreed pin mapping*, is authoritative.

### 7. Kernel sets the 1 us timer tick (2026-11-10)

After a cold power-up the RP2040 system timer ran at 23.4 kHz instead of
1 MHz: the watchdog TICK register's CYCLES field resets to 0 (RP2040
datasheet 4.7.2, Table 550), and only the boot ROM's BOOTSEL path left it at
12. Every TIMERAWL user (Motion Control speed, stop and timeout timing, the
hardware watchdog, GPIO interrupt timestamps) was then 43 times slow.
`startup_clock()` in `kernel/sysdepend/pico_rp2040/cpu_clock.c` now writes
CYCLES = `XOSC_MHz` with ENABLE once clk_ref runs from the crystal. S2-H06
verifies it after a cold power-up.

### 8. Line-follow inside wheel and PWM cap (2026-11-10)

The strong-curve inside wheel stays at 5 mm/s. At 5 mm/s the speed loop
averages correctly but the wheel moves jerkily (about one encoder count per
40 ms window, plus stick-slip below breakaway duty). On the track, 10 mm/s
widened the arc so the car could not hold the loop, and 0 (brake) stalled
the outer wheel at the 20% cap while pivoting (Fault STALL); 5 mm/s is the
only value of the three that completes the lap (S2 test log, 2026-11-10).
The `_s3line` PWM cap stays at 20%: line following peaks at duty 1120 of
1250, and steady 200 mm/s needs about 1800, so the cap is raised only
together with faster line-follow speeds or pivot turns, and retested then.

### 9. Subsystem 5 integration, stage 1: obstacle stop (2026-11-10)

Subsystem 5 is integrated from `feature/full-system-integration` (11-10)
in stages. Stage 1: S2 creates `scan_q` (IF-11) and publishes
`robot_state.mode` (IF-14, from the former reserved bytes; still 20
bytes). Mode is FOLLOW while Running in the `_s3line` image and NONE
otherwise, because Running is line following only there. S5 raises
`CTRL_FLG_HAZARD` (IF-12) when the forward range falls inside its
provisional 250 mm hazard distance in a moving mode, and the controller
latches STOP, as before. The `_s3line` image now also starts subsystem 5.
Stage 2 adds the scan request, authorise and stop-confirm handshake
(IF-11, IF-13); stage 3 the bypass and line reacquisition.

### 10. A late forward echo means nothing in range (2026-11-10)

Deviation from subsystem 5 decision 3 ("an invalid forward reading requests
a stop"), approved by Dalton on the integration branch. On the track, the HC-SR04 facing open space or an angled surface
returned echoes that rose but were still high at the 30 ms deadline: one
such reading stopped the car, and a sustained run (134 in a row) latched a
scan fault. The datasheet gives a 4 m maximum range (23.5 ms); it does not
specify the output when no echo returns. A late echo (a rise with no fall
by 30 ms, or a fall after it) and an in-time echo longer than the 4000 mm
duration are therefore reported as a valid reading at the 4000 mm range
bound (clear), both by the forward monitor and by the scan engine
(`range_measurement_is_beyond()`, S5-U20 revised). An attempt with no
rising edge and ECHO stuck high past 130 ms remain invalid and still
fault. The first stage 2 trial needed the scan-engine half: all four scans
faulted while the forward monitor already had the rule. Real readings below 250 mm still stop the car (four such stops in
the same lap, including one at 248 mm beside the track on a curve).

### 11. Subsystem 5 integration, stage 2: scan handshake (2026-11-10)

In the `_s3line` image, a hazard while Running in FOLLOW no longer latches
STOP. The pure `scan_control` module (S2-U60..U63) stops the wheels with a
motion STOP, sends REQUEST and sets mode SCAN; once both wheels are still
(`motion_stop_confirmed()`), it sends STOP_CONFIRM and AUTHORISE (IF-11).
It then waits up to 20 s (`CTRL_SCAN_LIMIT_MS`, as on full-system) for
subsystem 5's result for that request (IF-13); a timeout sends CANCEL. A
STOP or fault from elsewhere sends S5 a STOP. Outcomes: CLEAR resumes line
following; PLAN is printed and, until stage 3 drives it, stops the run;
no route, fault or timeout stop the run. Line following is paused (its
queue still drained) while the mode is not FOLLOW.

### 12. Clear to resume after a scan (2026-11-10)

Approved by Dalton on the integration branch. Subsystem 5 reported CLEAR
only when nothing was within 4 m straight ahead, which indoors almost never
happens: with the obstacle removed, scans read 1134 mm and 682 mm ahead and
returned a plan or no route, so following never resumed. A scan now
reports CLEAR when the 0 degree median is at least 500 mm (twice the
provisional 250 mm hazard distance); the forward monitor still stops the
car if it later comes within 250 mm. Closer readings are planned around as
before. 500 mm is provisional, like the hazard distance.

### 13. Grove 4 and Grove 7 swapped (2026-11-11)

Approved by Dalton on the integration branch. The Grove 4 and Grove 7 cables
were swapped on the robot. Grove 4 (GP16/GP17, Robo Pico datasheet Grove
table) now carries the left wheel encoder, A on GP16 and B on GP17. Grove 7
(GP7/GP28) now carries the side barcode sensor, with its DOUT removed and only
AO connected, on GP28/ADC2; GP7 is unused. ADC2 pin initialisation is enabled
for it, and the GP28 battery-sense jumper must stay open. The barcode input
becomes analogue: subsystem 3 reads it on ADC channel 2 instead of as a GPIO
level. The A/B order assumes the Grove cable keeps its wire order. S2-H03
confirmed it on 2026-11-11: the left encoder still counts up going forward,
with no decode errors.

### 14. Subsystem 5 integration, stage 3: bypass and reacquisition (2026-11-11)

Approved by Dalton on the integration branch. In the `_s3line` image a PLAN
no longer stops the run. The pure `bypass_control` module (S2-U64..U67)
drives subsystem 5's plan: each segment as one MOVE or TURN, then the
approach turn and a forward search at 100 mm/s (UNMEASURED) for at most the
plan's search distance or 5 s (S5 decision 9). A front sensor on black for
100 ms (S5 decision 9) ends the search; the car then moves its axle to
where the sensors first saw the line (they are 133 mm ahead of the axle,
measured by ruler) and turns by the align angle. Following resumes only if
the heading is within 15 degrees of the start (S5 decision 9) and nothing
is within the hazard distance ahead. The mode is BYPASS during the
segments and REACQUIRE afterwards. Subsystem 5 raises its hazard flag only
on an edge, and the bypass starts within the hazard distance, so the
controller consumes the flag and the bypass checks the forward verdict
itself: ignored during in-place turns, a stop during forward travel. A
snapshot older than 150 ms (S5 decision 3) counts as a hazard. Any rejected
or failed command, hazard, exhausted search or heading error stops the run.

On the robot (2026-11-11) the segments ran in order, but each time the plan
took the car off the mat and the run was stopped by hand during the move
past the obstacle, so the search, the step onto the line and the align turn
are not yet validated. The plan is sized by subsystem 5's provisional
80 mm half width and 200 mm assumed obstacle depth; the car is 127 mm wide
(64 mm half width), and the mat is as little as 35 mm from the line, so no
bypass can be guaranteed to stay on the mat without knowing where it is.

### 15. Fresh forward clearance after a scan (2026-11-11)

Deviation from subsystem 5 decision 4's implementation, approved by Dalton
on the integration branch. After a scan, subsystem 5 turned the sensor
forward and accepted the plan only if the fresh reading was at least the
250 mm hazard distance. A hazard stop always ends inside that distance
(245 mm in the first stage 3 run), so every scan faulted. The fresh reading
may now be at most 20 mm (twice the 10 mm ranging target) closer than the
nearest of the -15, 0 and +15 degree medians, the obstacle's closest seen
point; the bypass starts with an in-place turn. The nearest of three, not
the 0 degree median alone, because one run's 0 degree median was 296 mm
while +/-15 degrees and the fresh reading were 245 mm. The engine no
longer uses its hazard distance, so that setting was removed from its
configuration. S5-U56 is revised and fails against both earlier rules.

### 16. Bypass prefers left; side-specific vehicle extents (2026-11-11)

Deviation from subsystem 5 decision 5 ("choose the shorter valid route, left
on a tie"), approved by Dalton on the integration branch. The planner now
takes the left route whenever it is valid and the right route only when
left is blocked, unobserved or beyond the search bound. The side barcode
sensor sticks out on the left, so passing the obstacle on the car's right
keeps it away from the obstacle; the team also judged left to be the safer
side on the mat. The single 80 mm half width is replaced by the measured
extents from the car's centre line (ruler, 2026-11-11): 82 mm on the left
(63 mm to the outside of the left tyre plus 19 mm of barcode sensor) and
63 mm on the right (outside of the right tyre). A left bypass adds the
right extent to the obstacle's edge, a right bypass the left extent. This
replaces the 127 mm width noted in decision 14. The sensor axis is taken as
the car's centre line. The controller now prints the observed profile
(distance ahead, nearest point, both edges, and width and centre when both
edges were seen) when every scan ends. S5-U36, S5-U53, S5-U60 and S5-U62
are revised; the old rule and swapped extents each fail them. Not yet run
on the robot.

### 17. 45 degree bypass with swept-body clearance (2026-11-11)

Deviation from subsystem 5 decisions 5 and 9, approved by Dalton on the
integration branch (option A). The team asked that the car never turn
90 degrees after sensing an obstacle. Subsystem 5's planner now returns
five segments: reverse (often 0), turn 45 degrees to the bypass side, a
diagonal until the axle is the sideways offset across, turn straight,
forward past the obstacle. The controller then turns 45 degrees back,
searches diagonally, steps onto the line and aligns, as in decision 14.
Each length is the shortest, in 10 mm steps, that keeps every part of the
body clearance + margin (50 + 30 mm) from the obstacle rectangle (edge
to nearest point plus the 200 mm assumed depth): the diagonals and the
straight by perpendicular distance, each in-place turn by sweeping the
body rectangle in 5 degree steps against the obstacle outline sampled
every 20 mm, plus an allowance for both steps. Reversing is allowed up
to 150 mm (UNMEASURED) along the path the car just drove. Body extents by
ruler (Dalton, 2026-11-11): 132 mm from the axle to the front, 56 mm to
the rear, 81 mm to the ultrasonic sensor face, sides as in decision 16;
nothing at the rear sticks out past the tyres.

Subsystem 5 decision 9 bounds the search at 500 mm. A 45 degree search
covers only 0.71 of its length sideways, so the bound now applies to the
sideways offset: offset + margin <= 500 mm, a diagonal search of up to
708 mm. The 5 s search limit becomes 7.5 s, the time for 708 mm at the
search speed plus 6%. A float simulation sweeping the body outline every
2 mm along 17,178 planned routes for random obstacles found no approach
closer than 80.1 mm. S5-U36, S5-U53 and S5-U63 are revised and S5-U64 to
S5-U66 added; removing each clearance check fails at least one of them.
Not yet run on the robot. Only the observed edge and nearest point are
modelled; anything beyond the assumed depth is not.

### 18. Ultrasonic beam allowance on obstacle edges (2026-11-11)

Revision of subsystem 5 decision 5's edge estimate, approved by Dalton on
the integration branch. The HC-SR04 datasheet gives a 15 degree measuring
angle, but a centred 144 mm wide box 240 mm ahead (corners at +/-16.7
degrees, measured by ruler) was still seen out to +30 and -40 degrees, and
the scan reported it 340 to 370 mm wide. Together the two sides imply an
effective half-beam of about 21 degrees. The 10 degree left/right
imbalance fits the car having stopped about 5 degrees off the line, or a
servo centre about 5 degrees off (its mapping is still provisional). A
ray hits anything within its beam, so the first ray that misses shows
nothing reaches within the half-beam of it. Each hit is now placed at
most the beam allowance inside the first miss.

The allowance is 10 degrees. Three bypasses on the robot with 15 degrees
(same centred box, `logs/2026-11-11_s5-beam15.txt`) all returned to the
line, but replaying all five centred-box scans of the day put the left
edge at 50 and 59 mm on two of them, inside the real 72 mm: the angle at
which the box vanished varied from 20 to 35 degrees between stops,
likely with the car's heading at the stop. With 10 degrees the five left
edges are 74 to 148 mm, all outside the box, and the sideways offset
falls by about 20 to 45 mm against no allowance. The coarse-only planner
is unchanged, since 15 degree steps make the allowance a no-op. S5-U60
and S5-U62 are revised and S5-U67 added (synthetic 15 degrees). Not yet
validated for an off-centre obstacle, which the run day may bring.

Also observed: with the obstacle far up the start straight, the bypass
ran off the end of the mat, and it completed only after the obstacle was
moved closer to the start. The route reaches roughly the sideways offset
plus the forward leg plus the return diagonal beyond the stop, about
0.8 m here, much of it from the 200 mm assumed depth. The space left on
the straight ahead of the obstacle is not yet measured or checked.

### 19. Passing the obstacle by measurement (2026-11-11)

Approved by Dalton on the integration branch (proposed by a teammate);
replaces the assumed-depth forward leg of decision 17. A front scan
cannot see an obstacle's depth, and assuming 200 mm made the forward leg
long enough to leave the mat. After the turn back to straight the car
now passes the obstacle by measurement: it looks sideways at 60 degrees
(the servo's limit) towards the obstacle, and while the reading is
within clear_mm it steps 50 mm forward and looks again. clear_mm is the
farthest the side of the obstacle can read through a 50 to 70 degree
beam (sideways gap / sin 50 degrees) plus 100 mm (UNMEASURED). The first
look beyond it bounds the far end to within gap x cot(60 + beam
allowance) ahead of the sensor; the car then drives extra_mm, which puts
the turn back as far past that bound as the assumed-depth route put it
past its far end, and turns back, searches and aligns as before.
Obstacles up to 300 mm deep are handled; stepping beyond that stops the
run, as does a look that faults or gets no answer within 3 s.

Each look is stationary, keeping stop-before-scan: subsystem 5 waits up
to 1 s for the wheels to be confirmed still, turns the servo, waits the
scan settling time, takes the median of three valid readings, turns the
servo forward, waits again and answers only after one fresh forward
reading, so the controller never moves on a stale forward verdict. A
look needs RUNNING, mode BYPASS and no latched STOP; movement, three
invalid readings or 2.5 s overall fault it and return the servo
forward. Only subsystem 5 drives the servo. The controller sends LOOK on
scan_q (new kind 8, angle in the former reserved byte); subsystem 5
answers in look_id, look_result and look_mm. For the 144 x 96 mm test
box the forward leg is expected to fall from about 37 to 31 cm. Pure
logic in scan_look (S5-U68 to S5-U70) and bypass_control (S2-U68,
S2-U69); S5-U53 revised. Not yet run on the robot.

### 20. Stop before stepping onto the found line (2026-11-11)

Approved by Dalton on the integration branch; revises the step of
decision 14. The first run passing by measurement (decision 19) found
the line, stepped and aligned, but ended past the line. The step had
been worked out the moment the line was confirmed, while the car was
still searching at 100 mm/s, and the step MOVE counts from wherever the
car is when it starts, so the distance rolled in between was never
subtracted. Once the line is confirmed the controller now sends the
motion STOP command (not the safety STOP latch), waits for
motion_stop_confirmed() (both wheels still for 100 ms and motion Idle),
and only then sets the step from the travel since the sensors first saw
the line. The command's own STOPPED result is its success there. Not
still within 3 s stops the run. A first build braked with a zero SPEED
target instead; motion stays in its moving SPEED state then, so the stop
was never confirmed and two runs on the robot stopped just short of the
line and failed after 3 s (`logs/2026-11-11_s5-pass-halt.txt`). S2-U66
and S2-U67 are revised; the old order fails S2-U67. Not yet run on the
robot with the STOP command.

### 21. Align by spinning onto the line (2026-11-11)

Approved by Dalton on the integration branch; revises the align turn of
decision 14. With the stop before the step (decision 20) three bypasses
found the line, stepped and turned 45 degrees, but the car then drove on
with both sensors white and had to be stopped near the mat edge; in one
run the right sensor met the tape only about 4 s later. The front sensors
straddle the tape and are 133 mm ahead of the axle, so a few degrees of
turn error or a centimetre of step error leaves the tape outside both,
and "both white" then reads the same as centred. The align is now closed
on the line: the car spins in place towards the original direction (60
mm/s per wheel, UNMEASURED) and stops as soon as a fresh line report has
a sensor on black, then line following takes over and steers onto it.
Black is accepted only after turning the align angle less twice the 15
degree heading tolerance (15 degrees of the 45), so the crossing just
made is not taken; past the align angle plus 90 degrees (135 in all, agreed
with Dalton after the runs below) without the line
the run stops (line not found). Once still, the heading check of
decision 14 still applies. S2-U66 and S2-U67 are revised. The first run
on the robot (`logs/2026-11-11_s5-pass-spin.txt`) spun about 60 degrees
without a sensor on black, so the limit was widened from plus 15 to plus
45 degrees and the controller now prints the odometry at each bypass
command and every line-sensor change during reacquisition, to find out
whether the step leaves the axle past the line.

### 22. Turn to look, a 45 degree servo, and the line sets the heading (2026-11-12)

Approved by Dalton on the integration branch; revises decisions 19 and
21, S5 decisions 4 and 9 and the servo mapping of S5-U47. Evidence is in
`testing/subsystem5/logs/2026-11-12_*.txt`.

- **Servo.** The mounted sensor can turn 45 degrees each way before it
  meets the chassis. With the SG90 calibration image (`S5_SERVO_CAL=1`
  was used and then removed) the sensor faced forward at 1400 to 1450 us
  and was last clear at 950 and 1850 us. The centre is now 1400 us and
  angles are limited to +/-45 degrees (950 to 1850 us) at 10 us per
  degree. The 10 us per degree is an estimate from a photo, not a
  protractor reading (UNMEASURED). At the old 1500 us centre a +45 degree
  command pressed the sensor against the chassis and left looks never saw
  the box.
- **Scan.** The coarse scan covers -45 to +45 degrees in 15 degree steps
  (seven angles, 19 five-degree bins). The 60 degree rays are gone, so a
  box wider than about 25 to 30 cm at 250 mm shows no edge and the run
  stops with no route. A scan is CLEAR only if every measured bin within
  +/-15 degrees reads at least 500 mm (decision 12 checked 0 degrees only).
- **Turn to look.** A flat side reflected the pulse away from a 60 degree
  look, so every look read past the obstacle. Each look now turns the car
  35 degrees towards the obstacle and the servo 45 degrees more, 80
  degrees in all, then turns back. The turned front corner reaches 128 mm
  towards the obstacle, so the sideways offset is widened to keep the 50
  mm clearance. A miss bounds the far end where the look's axis meets the
  side; a pass counts only after pass_min_mm of stepping, which puts that
  point at least 100 mm (UNMEASURED) behind the near face; and a passing
  reading must also be at least 100 mm (UNMEASURED) beyond the nearest
  side reading so far, because the scan overstates widths (287 mm against
  a 269 mm threshold once passed a box still beside the car).
- **Headings.** The turn back after each look and the approach turn are
  to the heading at the start of the bypass, not relative: each 50 mm step
  curved about 2 degrees, and the car had drifted 17 degrees by the fifth
  look.
- **Align spin.** 30 mm/s per wheel (was 60): a spin at 60 swept past the
  tape with no report, as the three-sample mask filter and averaging take
  about 40 to 70 ms of the ~140 ms on the tape. It gives up past 170
  degrees (was 135), the most the wrapped heading change allows with 10
  degrees in hand.
- **Final check.** The line sets the heading: after the spin the bypass
  fails only if the car is more than 90 degrees from the start heading,
  that is, it took the line the wrong way. The 15 degree check of S5
  decision 9 failed cars already on a line that curved beyond the box.

On the robot, three straight-on bypasses returned to the line and resumed
following (`2026-11-12_s5-recentred.txt`, `2026-11-12_s5-angled.txt`,
`2026-11-12_s5-straight-final.txt`). Approaches from a curve, at an angle
to the box, are on hold: the professor has said the obstacle will be on
a straight with room before and after it. Squaring to the face before
the scan was tried and removed.

### 23. Subsystem 4 integration, stage 1: terrain monitor alongside line following (2026-11-12)

Subsystem 4 is integrated from `feature/subsystem4-imu-terrain` (`ba1fb62`)
in stages. Stage 1 enables the kernel I2C driver for unit 1 only (`iicb`,
GP2/GP3 per the agreed pin mapping; unit 0 is not started, so its pins are
untouched) and starts subsystem 4 in the `_s3line` image as well as the
full image. S4 calibrates while the car waits for GP20 and then prints
calibration, hump, impact and fault events. It has no interface to the
controller or motion yet: no movement is authorised or stopped by it, and
a hump has no height because no distance is supplied. On the robot
(`testing/subsystem4/logs/2026-11-12_s4-stage1-lap.txt`) S4 calibrated in
5890 ms and stayed silent and fault-free from START to STOP through a
scan, a bypass and a CLEAR scan; it reported humps and impacts only while
the car was carried. The S3 starter
consumer still sends IF-03 lines, so S4 keeps creating `mbf_summary` and
`flg_events`. Stage 2 supplies wheel distance from Motion Control for the
height estimate and a new-run reset at START (decision 24); stage 3 reports terrain
events to the controller and telemetry.

### 24. Subsystem 4 integration, stage 2: hump height (2026-11-12)

S4 measures a hump from tilt and the contact geometry (S4 decision 8).
It reads `motion_get_status()` each IMU sample (IF-17, read-only) for
`run_id`, so START, which already sends `NEW_RUN`, clears the run
maximum, and for the wheel speeds, so pitch is ignored while the car
speeds up or brakes. A completed hump prints its height, the contact that
crossed it and the run maximum. S4 still does not affect motion; stage 3
reports terrain events to the controller and telemetry.

For the bench, `S2_CAL_TEST=3` drives 450 mm forward per GP20 press and
also starts subsystem 4, as does `S2_CAL_TEST=1` and 2.

### 25. Soften strong curves while a barcode is read (2026-11-17)

On 2026-11-17, passes over barcode F decoded once in 5, then 0 of 11
captures after the stack fix. In the rejected captures, the 5/35 mm/s
curve corrections fired while the side sensor was over the barcode.
The inside wheel nearly stopped, so a run took longer than the 800 ms
or 700 ms scan limits, and the sensor swept across the bars at an angle,
which corrupted the run widths. One full 29-run capture still had the
correct payload (`testing/subsystem3/TEST_LOG.md`).

The Line Sensor now sets `S2_LINE_FLAG_BARCODE` in each line report while
a capture is in progress (the first reserved byte of `s2_line_report_t`
becomes `flags`). While the flag is set, the controller replaces a strong
curve with the gentle correction in the same direction at its strongest
setting, 32/40 mm/s (`line_control_soften`). Straight, gentle corrections,
junction handling and STOP are unchanged. Each capture gets at most
`CTRL_LINE_BARCODE_SOFT_MS`, 10 s; a 29-run frame is about 290 mm, about
7.3 s at 40 mm/s. The flag cannot outlive the reports: a report older
than 100 ms already stops the car. This assumes barcodes are beside
straight track, where gentle corrections hold the line; a lap must
confirm that the car still follows the line through softened periods.

Revised 2026-11-17: the flag is set only once a capture looks like a
barcode, with at least three runs, each at least 60 ms. Later the same
day, 19..40 ms blips and single crossing bars under the side sensor
started a capture on most of the track. Each one softened the curves,
and the car ran off the line on bends (runs of 243..504 mm). Real runs
measured 99..600 ms. Softening now starts a few runs into a barcode;
the decoder already tolerates a damaged first sentinel.

### 26. Barcode command at the next junction (2026-11-17)

Approved by Dalton on the integration branch. The brief requires the car
to carry out the barcode's command (left, right, straight, U-turn) with
encoder-based motion. The course letters are announced on the day, so
the mapping is one table in `s3_nav_command()`: the brief's example A-D,
plus the test letter F, whose command is chosen at build time with
`S3_NAV_F` (0 none, 1 left, 2 right, 3 straight, 4 U-turn). A letter with
no command is reported and ignored. A dashboard to set the mapping is
deferred.

The Line Sensor sends each mapped command once on `nav_q`
(`s2_nav_request_t`). The controller holds it while following, a newer
command replacing an older one, and drops it after 60 s without a
junction, or at once if the car is not running. The junction is the
existing both-black confirmation (`line_control`, JUNCTION_CONFIRM to
CROSSING). There, `nav_control`:

- STRAIGHT lets the normal crossing proceed.
- LEFT, RIGHT and UTURN send the motion STOP command and wait until the
  wheels are still. Then the car steps forward 133 mm (sensors to axle),
  less the travel since both sensors went black (decision 20), so that the
  axle is on the junction. It then spins in place at 30 mm/s per wheel,
  left for UTURN, until a fresh report has a sensor on black. Black is
  ignored until 30 degrees short of the turn (90 or 180 degrees), so the
  line just crossed, and a side branch during a U-turn, are not taken. More
  than 45 degrees past the turn, or 12 s in any phase, fails and latches
  STOP (decision 21). After a stop on the line, line following restarts.
- The robot mode is NAVIGATE while it runs. STOP, not running, and a
  scan for an S5 hazard all abandon it.

The values are UNMEASURED. S2-U74 to S2-U77 cover the policy on the host.
It has not yet run on the car.

Revised 2026-11-17 after the first runs: the hold is 60 s, not 20 s. On
the course, the + after barcode F came 21.4 s after the decode, and one
held LEFT expired 1.3 s before it. The barcode run limit is 2.5 s, not
800 ms (`s3_barcode.c`). In three passes the car paused on a wide bar of
the closing sentinel for over 800 ms, which discarded captures whose F
was intact.

### 27. Every barcode letter and the cause of each STOP (2026-11-20)

Approved by Dalton on the integration branch, for telemetry version 3
(S1 decision 5). Host-tested only; not yet run on the car.

**Barcode letters (IF-21).** `s2_nav_request_t` carries the decoded
letter in the byte that was `reserved[0]` (size still 12 bytes), and
`s2_nav_submit()` takes it. S3 now sends a record for every decoded
letter, with `S2_NAV_NONE` when the letter has no command; before, an
unmapped letter never left S3. The controller keeps the last letter, its
command and a count for telemetry, and stages only a mapped command, as
in decision 26. An unmapped letter therefore costs one queue slot, which
the controller drains each cycle.

**Stop cause.** `s2_robot_state_t.stop_cause` (`S2_STOP_*`) says what
latched STOP: 1 start-up, 2 remote, 3 button, 4 pick-up, 5 hazard (one a
scan did not take), 6 line, 7 scan (failed, timed out or no route), 8
bypass, 9 junction manoeuvre, 10 fault, 11 starter demo script finished.

- The first cause after STOP was clear is kept, so a dashboard STOP after
  a pick-up still reports the pick-up. A fault replaces any cause. START
  clears it; RESET from Stopped keeps it; RESET from Fault gives
  start-up, as STOP stays latched.
- If several STOP bits arrive in one cycle, the pick-up wins, then
  remote, on-board, line and hazard.
- Line failures (S3's sensing errors and the controller's line checks)
  had shared `CTRL_FLG_LOCAL_STOP` with the GP20 button. `s2_line_stop()`
  now sets a new `CTRL_FLG_LINE_STOP` (bit 6, `CTRL_FLG_ALL` 0x7F).
- Scan, bypass and junction failures also raise `CTRL_FLG_LOCAL_STOP`,
  and all run in the controller task. That task notes the cause in a
  task-private variable before it raises the flag, and passes it to
  `ctrl_flags()` in `ctrl_input_t.local_cause`. No other task writes it,
  so it needs no lock. A LOCAL_STOP with no noted cause is the button.
- `s2_robot_state_t` grows from 28 to 32 bytes (S2-U39).

**Tests.** S2-U78 (host and target) covers each cause, first-cause
retention, the shared-cycle order, RESET and a fault. S2-U37 covers the
copy into the telemetry snapshot, and S2-U40 the new flag bit.

### 28. The normal image runs the mission (2026-11-20)

Approved by Dalton on the integration branch. Builds without warnings
and the host suites pass; not yet run on the car.

Until now the mission (line following, barcode commands, humps, Scan &
Plan, bypass) ran only in the `TEST=1 S3_LINE_TEST=1` image, as the S3
"line-follow trial" called from `subsystem3_test()`. A plain `make`
started the starter Producer and Consumer and the Week 10 demo script
instead. The graded firmware must not be a test image.

- The makefile derives `APP_MISSION`: 1 when `TEST=0`, or `TEST=1
  S3_LINE_TEST=1`; 0 in the self-test and bench images. It replaces the
  C macro `S3_LINE_FOLLOW_TEST`. `S3_LINE_TEST=1` stays as the same
  mission in a `_test` image, so the commands in earlier test-log rows
  still build.
- The S3 loop moved unchanged from `subsystem3_test.c` to
  `subsystem3/s3_follow.c` (`s3_follow_run()`). It still runs in the
  initial task, after S2, S4 and S5 start, so its timing is as tested.
  Only its start-up console lines changed.
- The 20% PWM cap, the GP20 button and the controller's line, scan,
  bypass and junction steps follow `APP_MISSION`, so a plain `make`
  image behaves as the tested `S3_LINE_TEST` image.
- **Watchdog.** `task_motion.c` armed the RP2040 watchdog in every
  `TEST=0` image, so this change would have armed it for the first time
  on the car. It is now behind `S2_WATCHDOG` (default 0). No tested image
  has armed it, and the controller's console prints run above Motion
  Control on the same core (`docs/subsystem2.md`), so a blocking print
  could reset the board mid-run. Arm it only after S2-H05 and a car run
  with `S2_WATCHDOG=1`.

The self-test image (`TEST=1` alone) still starts the starter Producer
and Consumer for INT-01; their removal is a separate change.

## Subsystem 3 — Line Following and Barcodes (Edmund)

### 1. Front-left sensor signal assignment

Edmund confirmed the installed wiring and robot-relative viewpoint on
2026-11-04. Viewed from behind the robot, the front-left DOUT uses GP6 and the
front-right DOUT uses GP16. The side barcode sensor uses GP4. GP17 is unused,
and the Grove GP26 contact beside GP6 remains disconnected because GP26 belongs
to encoder 1. This corrects the earlier face-to-face left/right naming.
Subsystem 3 reads only the two front DOUT signals for steering; GP16 must not
be driven by the demo liveness LED.

### 2. Initial two-sensor line-follow trial

Edmund and Dalton approved IF-06 on 2026-11-04. Subsystem 3 sends timestamped
line positions through S2's bounded `line_q`; the Vehicle Controller alone
converts those positions to Motion Control speed commands. Subsystem 3 never
writes motor PWM. The observed centred geometry is WHITE/WHITE. The trial
therefore maps WHITE/WHITE to straight, BLACK/WHITE to a left correction,
WHITE/BLACK to a right correction, and BLACK/BLACK to STOP.

The initial 70 mm/s outer-wheel and 35 mm/s inner-wheel targets were reduced
to 40/20 mm/s on 2026-11-04. A subsequent 30/15 mm/s target did not reduce the
observed physical speed and the controller entered Fault, so neither target is
a validated minimum. These speeds, the 50 ms report interval and the 200 ms
stale-report stop remain provisional safety values, not validated performance
or acceptance criteria.
After the physical speed remained too high despite lower speed targets, Edmund
requested a much slower trial. The dedicated line-follow profile now caps the
Motion Control speed PID output at 20% PWM; normal firmware retains the full
range. This is a provisional safety ceiling, not a measured minimum duty or a
substitute for validating encoder feedback and tuning the speed PID.
Because the sensors classified left and right surfaces correctly but the
20/40 mm/s arc did not produce a reliable left correction, Edmund requested a
stronger correction. The inside wheel target is now 0 mm/s (brake) while the
outside wheel remains at 40 mm/s. Motion Control clears a wheel's PID history
and duty when its speed target becomes zero so an earlier integral term cannot
continue driving the braked wheel.
Physical testing then showed that the left/right actuator responses were
reversed even though the IR classifications were correct. At Edmund's request,
the Vehicle Controller wheel commands are swapped: `LEFT` drives the left wheel
and brakes the right, while `RIGHT` brakes the left wheel and drives the right.
The GP6/GP16 sensor identities and the BLACK/WHITE decision table are unchanged.
Edmund also requested a supervised continuous trial with no automatic run-time
limit.
The Robo Pico GP20 programmable button is debounced and routed through the
Vehicle Controller: a press starts, a press while running latches STOP, and a
later press performs the controller RESET/START sequence.
Because WHITE/WHITE is also possible after completely losing the line, this
two-sensor rule remains a supervised test until line-loss detection or
additional evidence resolves that ambiguity. Invalid or stale sensor reports,
failed delivery, and controller/motion faults still stop it.

On 2026-11-04 Edmund approved the first bounded recovery revision. This revises
BLACK/BLACK from an immediate STOP to straight travel for at most 750 ms so the
car can cross a wide line or junction. A single-black correction that persists
for 600 ms changes to an on-the-spot pivot in that remembered correction
direction. WHITE/WHITE resumes straight following, an opposite single-black
observation resumes normal correction, and BLACK/BLACK enters the crossing
state. A crossing that remains BLACK/BLACK for 750 ms or a pivot search that
lasts 1500 ms latches STOP. These times and the 30 mm/s pivot target are
provisional safety values requiring physical calibration; they are not validated
performance criteria. The policy is owned by the Vehicle Controller and only
Motion Control writes PWM.

This revision does not claim general line-loss detection. WHITE/WHITE remains
ambiguous with only two sensors, so the pivot is triggered only by a persistent
single-black correction. Reliable recovery from an arbitrary WHITE/WHITE loss
still requires extra evidence such as the agreed encoder corridor and heading.

The Vehicle Controller drains and discards `line_q` observations whenever it is
not Running. Subsystem 3 publishes continuously, so retaining reports while
Ready or Stopped fills the bounded queue and makes the next START consume stale
observations. Only reports drained while Running may update the active line
state; invalid or stale Running reports still latch STOP.

On 2026-11-04 Edmund requested recovery from the GP20 line-test stall indication.
In the dedicated line-follow test, pressing GP20 while the Vehicle Controller is
in Fault now requests RESET only after the encoder status confirms both wheels
have been stopped for 100 ms. The controller retains the latched STOP during
reset, waits for Motion Control to return healthy, and only then issues the
pending START. The motor-stall detector remains enabled; a one-shot diagnostic
records the indicated left/right stalled wheel, preceding PWM duty, encoder
speed, counts and decode errors so recurring hardware or calibration faults are
not hidden by automatic recovery.

Repeated successful calibration on 2026-11-04 established LOW on white and
HIGH on black for both front modules. The dedicated line-test image stores that
digital polarity in firmware and skips guided calibration at boot. The separate
calibration image remains the validation path after any sensor or wiring change;
the firmware value does not replace adjustment of the modules' potentiometers.

### 3. Progressive curve following (2026-11-05)

Edmund approved a curve-following revision after physically confirming that
straight following, left/right correction, sensor classification and
BLACK/BLACK straight crossing work on the laboratory track. This supersedes
the earlier 0 mm/s inside-wheel correction and the 600 ms pivot search.

The forward target remains 40 mm/s. Physical testing of the initial 40/30
single-black correction showed that it could lose ordinary curves before the
150 ms sustained-curve transition, while the post-junction 40/0 curve worked.
The inside wheel is therefore braked at a 0 mm/s target immediately for every
single-black observation while the outside target remains 40 mm/s. The 150 ms
classification is retained only to distinguish a sustained loop curve for the
bounded junction-exit memory; it no longer changes wheel speeds. The controller
keeps this strong correction for as long as that sensor remains black and does
not pivot or stop merely because the track has a long curve.

The first 40/20 curve trial could turn but lost the line halfway through, after
which WHITE/WHITE caused premature 40/40 straight travel. Because WHITE/WHITE
is both the centred pattern and the two-sensor lost-line pattern, the revised
policy remembers a sustained curve for 100 ms after WHITE/WHITE first appears.
It then resumes 40/40 if neither sensor reacquires black, which bounds the
previous-direction assumption. BLACK/BLACK remains straight crossing for at
most 750 ms. The 0 mm/s inside-wheel target, 150 ms sustained-curve marker and
100 ms hold are provisional until repeated physical trials are recorded.

The loop trial then showed a separate junction-exit failure: the first
single-black observation after BLACK/BLACK could be the incoming branch. A
directional probe incorrectly steered the vehicle back onto that branch. A
subsequent 250 ms straight-exit trial was too short and also showed that brief
BLACK/BLACK observations occur on ordinary tight curves. The controller now
qualifies BLACK/BLACK for 100 ms before declaring a junction. During this
qualification it retains the preceding curve command, so a brief observation
does not suppress ordinary curve steering. A confirmed junction drives 40/40,
and its first clear observation starts a bounded 600 ms 40/40 exit. Sensor
steering resumes when that window expires. The 100 ms and 600 ms values are
provisional until repeated physical trials confirm both normal-curve tracking
and clearance of the incoming branch.

After that junction revision, Edmund requested that the line-following
forward/outside-wheel target increase from 40 mm/s to 45 mm/s. The inside-wheel
target initially remained 0 mm/s during corrections and curves. Junction
timing and all other steering decisions remained unchanged.

The 45 mm/s trial logs then showed correct repeated LEFT classifications and
curve actions, but physical LEFT turns still dropped to straight partway
through while the equivalent RIGHT loop completed. Ordinary corrections remain
0/45 or 45/0. A sustained LEFT curve now commands -10/45 mm/s and retains that
direction for 300 ms through WHITE/WHITE; the working sustained RIGHT curve
remains 45/0 mm/s with its 100 ms hold. This asymmetric compensation and the
45 mm/s target are provisional until the complete track is physically retested.

The next physical run confirmed normal left and right curves, but either loop
could intermittently select a side branch after its junction. A fixed 600 ms
exit can expire while one sensor still sees that branch. The exit therefore
keeps 45/45 mm/s for at least 600 ms and then requires WHITE/WHITE to remain
stable for 100 ms before normal steering resumes. Later logs showed that merely
ignoring a side observation could still carry the vehicle off the forward
route: after the 600 ms minimum, LEFT black therefore requests a RIGHT
correction and RIGHT black requests a LEFT correction, steering away from the
side branch. Either observation resets the centred-stability interval. The exit
stops at 1500 ms if it cannot establish the centred pattern, preserving a
bounded recovery.

The same run recorded `stale report; STOP` between otherwise valid changing S3
observations. USB serial logging can delay the supervised test task beyond its
original 200 ms watchdog. The dedicated line-test profile now permits 500 ms
between reports; at 45 mm/s this still bounds unobserved travel to 22.5 mm.
Production timing remains subject to measured scheduling and stopping tests.

### 4. Two-front-sensor on-line geometry trial (2026-11-05)

Edmund changed the physical mounting so both front sensors reliably observe
black while centred on the normal line; the left-side sensor remains reserved
for barcode capture. For this supervised trial, BLACK/BLACK now means centred
straight, BLACK/WHITE requests LEFT, WHITE/BLACK requests RIGHT, and
WHITE/WHITE means lost. This removes the previous ambiguity in which
WHITE/WHITE could mean either centred or completely off-track.

Because BLACK/BLACK is now the ordinary centred pattern, the two front sensors
cannot independently identify a junction. The trial therefore continues
straight through BLACK/BLACK and removes the junction confirmation and exit
states. This is suitable for testing straight-through track following but does
not close the project's future junction-detection requirement.

When WHITE/WHITE follows a steering observation no more than 500 ms old, the
controller searches using that most recent direction for at most 800 ms. It
requests STOP when no recent direction exists or the bound expires. The
existing 45 mm/s target, 150 ms sustained-curve transition, -10/45 mm/s
sustained LEFT target, 45/0 mm/s sustained RIGHT target and 500 ms report
watchdog remain provisional.

### 5. Gentler centred-line correction trial (2026-11-05)

After the front sensors were repositioned over the line, Edmund observed the
vehicle alternating LEFT and RIGHT while trying to regain BLACK/BLACK. The
first 30/45 mm/s reduction still produced excessive steering on a straight.
The ordinary single-black correction therefore uses a 38 mm/s inside-wheel
target and a 45 mm/s outside-wheel target instead of braking the inside wheel
to 0 mm/s. The straight target remains 45/45 mm/s. An opposite-direction
observation must remain stable for 100 ms before the controller changes from
LEFT to RIGHT or RIGHT to LEFT; it requests straight during confirmation. The
sustained-curve transition increases from 150 ms to 250 ms. Existing strong
curve targets remain unchanged pending separate physical curve testing.

### 6. Restore separated front sensors (2026-11-05)

Edmund moved the two front sensors back to either side of the black line and
requested the behavior used before the side-by-side BLACK/BLACK trial. The
centred pattern is therefore WHITE/WHITE again. BLACK/WHITE requests LEFT,
WHITE/BLACK requests RIGHT, and BLACK/BLACK is a junction candidate.

This supersedes sections 4 and 5 for the active supervised test. It restores
the last pre-trial 45 mm/s profile: ordinary corrections are 0/45 or 45/0,
sustained LEFT is -10/45, sustained RIGHT is 45/0, and the sustained marker is
150 ms. A sustained curve is retained through WHITE/WHITE for at most 300 ms
on LEFT or 100 ms on RIGHT.

BLACK/BLACK must persist for 100 ms before junction confirmation. A confirmed
junction crosses straight and must clear within 750 ms. Clearance starts a
straight exit of at least 600 ms; afterward, a side-black observation steers
away from that branch and WHITE/WHITE must remain stable for 100 ms before
normal following resumes. The exit requests STOP at 1500 ms if it cannot
establish the centred pattern. The 500 ms report watchdog and GP20 behavior
remain unchanged.

### 7. Dampen separated-sensor straight corrections (2026-11-05)

The first separated-sensor retest repeatedly entered a strong LEFT curve,
returned to WHITE/WHITE, then entered a strong RIGHT curve. Because the prior
LEFT curve was retained for 300 ms through WHITE/WHITE, it could carry the car
past centre and trigger the opposite sensor.

For the next supervised straight-line trial, ordinary corrections use 35/45
or 45/35 mm/s, a strong curve requires 300 ms of continuous single-black input,
and strong curve targets are 15/45 or 45/15 mm/s. The WHITE/WHITE curve hold is
50 ms in either direction. Straight speed remains 45/45 mm/s; junction timing,
the 500 ms report watchdog and GP20 behavior remain unchanged. These values
supersede only the steering values in section 6 and require separate curve
testing after straight stability is confirmed.

The straight-line trial reduced oscillation, but 15/45 sustained steering
activated after 300 ms could not follow an ordinary course curve. The gentle
35/45 ordinary correction and 50 ms WHITE/WHITE hold remain unchanged. A
continuous single-black observation now enters the sustained stage after
200 ms, where the inside wheel target becomes 0 mm/s and the outside remains
45 mm/s. This restores curve authority without restoring the earlier long
post-curve hold.

Intermittent curve acquisition then showed that S3's 20 ms sampling period and
three-sample filter could delay a new stable steering observation by about
60 ms. The supervised line-test path now samples every 10 ms while retaining
three consecutive matching samples, reducing the nominal filter response to
about 30 ms. Fresh unchanged reports are submitted every 30 ms instead of
50 ms. The calibration monitor retains its existing timing, and the controller
still rejects stale or invalid reports.

Logs from the next track runs identified two independent stop paths. Repeated
`junction/recovery bound exceeded` messages occurred when the post-junction
state waited for stable WHITE/WHITE while the loop already produced a valid
persistent side reading. After the 600 ms straight minimum, a single-black side
may now resume normal following when it remains stable for 150 ms. The crossing
bound increases from 750 ms to 1500 ms and the exit bound from 1500 ms to
2500 ms. These remain bounded supervised recoveries. The sustained LEFT hold
increases from 50 ms to 150 ms; RIGHT remains 50 ms.

The same logs also contain `RIGHT stall/no feedback` for encoder GP0/GP1. That
is a distinct Motion Control safety fault, so the stall detector remains
enabled. The wiring, wheel freedom and right encoder feedback must be checked
if that diagnostic recurs.

The following normal-right-curve log repeatedly reached `sustained RIGHT`, but
the vehicle continued forward. Sensor classification and curve escalation were
therefore working; the remaining problem was actuator duration/authority. The
sustained RIGHT target changes from 45/0 to 45/-10 mm/s and its WHITE/WHITE
hold increases from 50 ms to 150 ms. LEFT, ordinary correction, straight,
junction and report timing remain unchanged. Strong-curve logs now include the
actual left/right target pair for physical verification.

The next fault log showed `stalled L/R=0/1`, right PWM at the line-test ceiling
of 1250, and right speed at 0 mm/s during a strong LEFT target of 0/45. Sensor
classification had continued normally up to the fault. Braking the inside
wheel can increase the load on the outside wheel during a pivot, so sustained
curves now use rolling arcs: 10/40 mm/s for LEFT and 40/10 mm/s for RIGHT. The
stall detector remains enabled because a loose GP0/GP1 encoder connection or a
physically jammed wheel must still stop the vehicle.

S3 previously printed only when the classified state changed, which could make
an unchanged reading after a fault look like sensor reporting had stopped. The
supervised line test now also prints a sensor-state heartbeat every 500 ms;
the 30 ms controller-report rate is unchanged.

In the next two-run log, the first run ended through an explicit GP20 STOP after
10/40 LEFT arcs failed to remain on the loop; it was not an automatic fault.
The second run latched a genuine dual-wheel stall with PWM already at 1250/1250
and measured speed at 0/7 mm/s. Sensor heartbeats continued afterward.

For the next supervised trial, straight speed reduces from 45/45 to 40/40
mm/s, ordinary corrections become 32/40 or 40/32, and sustained curves become
rolling 5/35 or 35/5 arcs. The larger curve speed ratio gives a tighter radius
at a lower load. The LEFT WHITE/WHITE hold increases from 150 ms to 250 ms;
RIGHT remains 150 ms. Stall protection and its 20% PWM ceiling remain enabled.

### 3. Analog front sensors and adaptive thresholds (2026-11-05)

Edmund installed front-left AO on GP26/ADC0, front-right AO on GP27/ADC1 and
the side barcode DOUT on GP16. Both front DOUT wires and the barcode AO wire
are disconnected. This mapping is restricted to the dedicated S3 hardware and
line-follow profiles because the integrated mapping assigns GP26/GP27 to motor
encoder 1.

Measured white ranges were 118..154 counts on the left and 152..194 on the
right. Solid black measured 1459..1724 and 3322..3397 respectively. On isolated
faded patches, left measured 206..310 and right measured 2137..2942. The line
profile therefore filters each channel independently and adapts only its white
baseline. Baseline updates stop while black is classified, preventing the
track from being learned as white. Hysteresis provides separate black-entry
and white-release margins. The constants are bench-derived implementation
values, not validated course acceptance thresholds.

### 4. Analog-strength steering for smoother straights (2026-11-05)

Edmund requested smoother tracing after observing excessive zig-zag motion.
The line report remains eight bytes but now carries a 0..100 steering-strength
field for LEFT and RIGHT observations. S3 derives that value from filtered
contrast above its adaptive white baseline, normalised independently against
1500 left and 3300 right ADC counts. Straight and crossing reports carry zero.

The Vehicle Controller retains movement authority and maps ordinary correction
strength to an inside-wheel target from 38 down to 32 mm/s while holding the
outside wheel at the existing 40 mm/s. Thus a faint line edge causes only a
small speed difference. A single-side reading that persists for 200 ms still
uses the existing 5/35 or 35/5 sustained-curve target so curve authority is not
reduced. These mappings are provisional until verified on the physical track.

### 8. Wider left black margin for the hump (2026-11-16)

Approved by Dalton on the integration branch. The right front sensor worked
loose and was refitted; the sensors are now 30 mm apart, centre to centre,
on 18 mm tape, so each sits about 6 mm outside the tape edge when centred.
The raw ranges did not change (left white 113..164, black 1467..1532; right
white 168..221, black 3345..3378).

With the right wheel on the 10 mm hump the car rolls left and the left sensor
comes closer to the floor: over white it read 175..210, above its threshold
of about 170, so it reported BLACK and steered left (S3 log, 2026-11-16).
Two of three hump runs then lost the line. The right sensor on the hump still
read about 3290 over the tape.

`S3_LEFT_BLACK_MARGIN` goes from 32 to 100 counts and `S3_LEFT_WHITE_MARGIN`
from 20 to 80, giving thresholds of about 236 (black) and 216 (release) at
the 136 baseline, both above the tilted white. Both are kept above 210 so a
tilt can neither set nor hold BLACK. Cost: faded left patches, measured at
206..310 in decision 3, are only detected above about 236. The right sensor
is unchanged. Not yet validated on the track.

## Subsystem 4 — IMU and Terrain Monitoring (Khang)

### 1. Standalone ownership and wiring

Subsystem 4 owns the GY-511 / LSM303DLHC and I2C1. Use GP2 for SDA and GP3
for SCL, power the module from 3V3, and use a common ground. Only
`s4_imu_task` accesses I2C1; the standalone implementation has
no dependency on motor, encoder, controller, or network subsystems.

### 2. Sensor mode and calibration

Configure the accelerometer for 100 Hz, all axes enabled, +/-2 g,
high-resolution mode, and block-data update. Verify the magnetometer identity
bytes and read back both accelerometer control registers before accepting the
device. Require fresh stationary calibration at startup and after local sensor
recovery. Assume the installed sensor X axis points forward and Z points up;
physical orientation must be recorded before validation.

### 3. Terrain and fault processing

Use integer milligravity and millimetre units. Confirm ascent, descent, and
return-to-level over consecutive samples before reporting a hump. A direct
ascent-to-level transition is incomplete. Treat repeated invalid samples,
stale timestamps, unstable calibration, I2C failures, and data overruns as
faults. Local recovery closes and revalidates the sensor and recalibrates,
while retaining the run maximum; only an explicit new-run reset clears it.

### 4. Experimental measurements

Candidate height is the accumulated forward distance multiplied by the
small-angle gravity projection. It is valid only when distance increments are
provided for the active ascent. The standalone task has no encoder input, so
its height result remains invalid even when a hump is detected. The
LSM303DLHC has no gyroscope; no angular-rate result may be claimed.

All current calibration spans, tilt thresholds, impact thresholds, and sample
confirmation counts are provisional engineering estimates. They are not
acceptance criteria and must be replaced or approved using recorded physical
evidence.

### 5. Test evidence

Run synthetic logic and raw-decode cases in the Pico W `TEST=1` image. Enable
the live startup check only with `TEST=1 S4_HW_TEST=1` and connected hardware.
A successful cross-build is not a passed on-board or physical test. Record the
commit, wiring, mounting, reference instruments, serial log, raw observations,
and repeated measurements before marking a hardware case passed.

### 6. An overrun is not a fault at the 10 ms kernel tick (2026-11-12)

Approved by Dalton on the integration branch.
Revises decision 3 for overruns. The kernel tick is 10 ms
(`CNF_TIMER_PERIOD`), so each `tk_dly_tsk()` lasts 10 to 20 ms and the
task cannot read every sample of the 100 Hz output. On the robot every
read soon found `ZYXOR` set (datasheet table 35); decision 3 made that a
fault, so S4 restarted every second and never calibrated
(`testing/subsystem4/logs/2026-11-12_s4-h01-2.txt`, `IMU read failed
(-43)`). Polling every 5 ms was tried first and could not help, because a
delay cannot be shorter than a tick. Now:

- An overrun is not an error: the outputs hold the newest sample and it is
  read. A task held off too long is still caught by the 50 ms sample-gap
  check (stale fault).
- Opening the device reads the outputs once and discards them, because
  the sensor keeps sampling while closed and, with BDU, could otherwise
  hand over a sample from before the close.
- The calibration line prints how long the 200 samples took, which gives
  the real read rate for S4-H05. Calibration, confirmation counts and the
  50 ms gap are in samples, so their time depends on that rate.
- S4-H01 waits up to 10 s for calibration (was 3.5 s).

### 7. Installed IMU orientation (2026-11-12)

Approved by Dalton on the integration branch. Seen from behind the
car with the ultrasonic sensor facing away, the GY-511's X points left,
Y backwards and Z up. This replaces decision 2's assumption of X forward.
The terrain logic works in the vehicle's axes (x forward, y left, z up),
so `task_imu.c` turns each sample into them: forward = -Y, left = +X,
up = +Z. The decoded driver values stay in the sensor's axes. Confirmed
with `S4_RAW_LOG=1` (`testing/subsystem4/logs/2026-11-12_s4-h02-raw.txt`):
level reads sensor z +900 mg, nose up gives forward +190 mg (sensor y
-190) and left side up gives left +145 mg (sensor x +145).

### 8. Hump height from tilt and contact geometry (2026-11-12)

Approved by Dalton on the integration branch. Replaces the
pitch-then-descent state machine of decision 3 and the distance-based
height of the brief: the course's speed humps are crossed by one wheel
(the bench ramps are 50 mm wide; other groups test the same way), and a
one-wheel hump mostly rolls the car, which the pitch method never saw.

- **Geometry.** The car stands on its left wheel, right wheel (track
  119 mm, S2 odometry) and front ball caster (88 mm ahead of the axle,
  ruler). Filtered roll and pitch fix the plane through them: wheels at
  +/-roll x 59.5 mm, caster at pitch x 88 mm. With the lowest contact on
  the floor, the height is highest minus lowest. One wheel raised h gives
  roll h/119 and pitch -h/176; both wheels, pitch -h/88; the caster,
  +h/88. Each reads h. A hump the whole car climbs onto reads level on
  top and is not measured.
- **Detection.** A hump starts at 4 mm and ends below 2 mm, each held for
  3 samples; its height is the peak, and the highest contact at the peak
  is reported (left wheel, right wheel, both wheels, caster). A hump not
  ended within 5 s is dropped and the next waits for level. The tilt
  filter is halved each sample (was a quarter), because a heavier filter
  lowered a short hump's peak. All of these are provisional.
- **Acceleration.** Speeding up or braking reads as pitch (200 mg, 17 mm
  at the caster). The task reads Motion Control's snapshot (IF-17) and
  ignores pitch while the mean wheel speed changes by more than 15 mm/s
  between samples and for 8 samples after; the caster is then taken as
  level with the lower wheel, leaving the roll estimate.
- **Run.** A new `run_id` (START) clears the run maximum.

Test cases S4-U07, S4-U15 and S4-U22 are retired with the old method;
S4-U23 to S4-U27 cover the four contacts and the pitch gate.

### 9. Short calibration, retried in place (2026-11-12)

Approved by Dalton on the integration branch. In the first hump-pass run
(`testing/subsystem4/logs/2026-11-12_s4-h08-one-wheel.txt`) S4 never
calibrated: 200 samples took about 6 s, the car was being handled, and
each calibration fault also closed the sensor for 1 s. Calibration now
takes 50 samples (about 1.5 s), and a calibration fault prints each
axis's span and starts again without restarting the sensor. Sensor, I2C
and stale-data faults still restart it.

### 10. Tilt moving average and a longer hump limit (2026-11-13)

Approved by Dalton on the integration branch. Revises decision 8's tilt
filter and hump limit. Evidence is in `testing/subsystem4/TEST_LOG.md`
(2026-11-13 rows) and `testing/subsystem4/logs/2026-11-13_*.txt`.

- **Why.** Driving shakes the car. In 450 mm passes at about 180 mm/s
  every pass gave a false hump of 18 to 24 mm. At line-follow speed
  (about 40 mm/s, the speed on the course) the vertical spread fell from
  107 to about 10 mg, but four runs of 43 to 46 s still gave two false
  humps (6 and 7 mm) with the filter that halved each change. A 25 Hz
  output rate was tried and reverted (worse).
- **Average.** Roll and pitch are now the mean of the last
  `average_samples` samples (`S4_AVERAGE_SAMPLES` 16, about 470 ms at
  the measured 29.5 ms per sample; at most `S4_AVERAGE_MAX_SAMPLES`).
  Replayed on the four line-follow runs, the steady-driving maximum is
  2.4 to 3.4 mm, under the 4 mm rise. A hump crossed at 40 mm/s lasts
  seconds, so its peak is kept (S4-U29); the hump thresholds are
  unchanged. The window is in samples, so it shortens if the read rate
  rises (about 20 ms per sample without line-sensor logging).
- **Hump limit.** A hump not ended within 12 s is dropped (was 5 s): a
  195 mm hump takes about 5 s at 40 mm/s.

On the robot (`testing/subsystem4/logs/2026-11-13_s4-h07-line-avg.txt`),
about 82 s of line following gave no vibration hump and one 4 mm hump
from a smooth floor tilt held about 0.6 s; height p99 3.4 to 3.6 mm. The
image read about 21 ms per sample, so the window was about 340 ms.

Not yet addressed: pressing GP20 on the car and hard braking at a hazard
stop also read as humps, and a 4 mm rise may catch floor undulations.
The rise threshold should be set from S4-H08 with a measured hump.

### 11. Right-wheel humps and a 6 mm rise (2026-11-13)

Approved by Dalton on the integration branch. Revises decision 8's rise
threshold. Evidence: `testing/subsystem4/logs/2026-11-13_s4-h08-10mm.txt`
(S4-H08).

- **Course.** The professor has said a hump passes only under the
  robot's right side, so S4-H08 tested the right wheel only.
- **Measured.** A 10 mm hump (ruler; 195 mm across the path, 50 mm
  along it), line followed at about 40 mm/s: four clean crossings read
  11, 12, 12 and 12 mm under the right wheel, each detected once, and
  the car stopped on top read a steady 11.2 mm. Height reads 1 to 2 mm
  high; with one hump height it is not known whether this is an offset
  or a scale error, so no correction is applied.
- **Rise.** A hump now starts at 6 mm (was 4 mm); it still ends below
  2 mm. Floor undulations read up to about 4 mm while driving
  (decision 10), and a 10 mm hump reads 11 to 12 mm, so 6 mm leaves
  margin both ways. Humps under about 6 mm are not reported.

### 12. Height scaled to the 10 mm hump; humps only while driving (2026-11-13)

Approved by Dalton on the integration branch. The course uses one hump,
10 mm high, under the right wheel.

- **Scale.** S4-H08 read a mean of 11.75 mm for the 10 mm hump (decision
  11), so heights are multiplied by `height_scale_permille` 851
  (10 / 11.75; at most 2000). With one hump height a scale and an offset
  are indistinguishable, and only that height matters on the course.
  Thresholds compare the scaled height.
- **Only while driving.** Lifting the car, pressing GP20 on it and
  braking at a stop all read as humps. The IMU task now enables hump
  detection only while subsystem 2 reports Running in FOLLOW, NAVIGATE,
  BYPASS or REACQUIRE (`s2_get_robot_state()`, IF-14, the check
  subsystem 5 uses), and only after 32 samples there (about 0.7 s, which
  covers the GP20 press and the 16-sample average). A hump in progress
  when driving stops is dropped unreported; the tilt average keeps
  running. A hazard stop changes the mode to SCAN, so its braking is not
  a hump, and carrying the car between runs no longer changes the run
  maximum. The `S2_CAL_TEST` images never enter a driving mode, so they
  report no humps. The `S4_RAW_LOG` line ends with `humps on` or `off`.

Correction to the S4-H08 record: the line-follow stops in runs 6 to 8
came from line sensors that had been moved and not adjusted back, not
from line sensing.

### 13. Subsystem 4 integration, stage 3: highest hump in telemetry (2026-11-13)

Approved by Dalton on the integration branch. The brief requires the
highest hump of the run to be reported.

- **IF-18.** `subsystem4_get_status()` now copies S4's terrain result
  with `TMO_POL`: subsystem 2's telemetry reader runs in the WiFi service
  task and must never wait. Its only other caller (S4-H01) already
  polls.
- **Telemetry.** `s2_read_telemetry()` reads the run maximum after
  releasing `state_mutex`, so the two locks never nest, and
  `s2_telemetry_add_hump()` fills `s2_telemetry_t.hump_max_mm` (uint8_t,
  saturating at 255 mm; humps are about 10 mm) and `b_hump_valid`. A busy
  or missing S4 snapshot reports no hump for that publication.
- **Run.** The maximum is cleared at START (IF-17) and only humps found
  while driving count (decision 12), so it holds the run's value after
  STOP until the next START.
- **Run summary.** On a GP20 STOP the controller prints the run distance
  and highest hump read through `s2_read_telemetry()`, the path the MQTT
  bridge will use.
- **Not yet.** Subsystem 1's `telemetry_encoder_snapshot_t` has no hump
  field; `robot_read_telemetry()` maps it when subsystem 1 is merged. The
  controller does not act on humps.

### 14. Impacts only while driving (2026-11-13)

Approved by Dalton on the integration branch. Extends decision 12 to
impacts: placing and carrying the car raised impact events between runs.
`s4_terrain_set_detect_enable()` (renamed from
`s4_terrain_set_hump_enable()`) now gates both. While disabled the impact
edge is still tracked, so enabling during a shake does not report it;
the next crossing while driving does (S4-U34). The `S4_RAW_LOG` line ends
with `detect on` or `off`.

### 15. Stop when the car is picked up (2026-11-16)

Approved by Dalton on the integration branch. The demo rubric requires the
motors to stop when the car is picked up, which ends the demo. Subsystem 4
reports a lift (`S4_TERRAIN_EVENT_LIFT`) when either:

- one sample reads at least 200 mg of upward acceleration above rest (a hand
  lifting the car); or
- either unaveraged tilt is at least 200 mg for 3 samples in a row, about
  70 ms (the car tipped as it is lifted); or
- either averaged tilt (the 16-sample hump average) reaches 300 mg, about
  17 degrees (a car carried at an angle).

Lift detection uses the same gate as humps and impacts (decisions 12 and
14): only while driving and after the settle time, so a GP20 press, which
read up to +280 mg up, is not a lift. It is reported once per drive and
re-armed when detection is next disabled (S4-U35 to S4-U39). Dropping off a
hump reads below one g, so only upward acceleration counts.

The IMU task sets `CTRL_FLG_PICKUP` on subsystem 2's `controller_flags`
(IF-19), the way subsystem 5 raises `CTRL_FLG_HAZARD`. The Vehicle
Controller treats it like a local STOP: it latches STOP, goes from Running
to Stopped, and prints `[S2] S4 PICKUP: car lifted, STOP latched` and the
run summary. GP20 then resets and starts a new run as after any STOP.

Thresholds come from `testing/subsystem4/logs/2026-11-16_s4-raw-d14.txt`:
about 2900 gated driving samples, including a 10 mm hump, read at most
+106 mg up and 97 mg of averaged tilt. Seven handling episodes read
+209 to +622 mg up on a single sample (two in a row only once, so a
two-sample rule was rejected), and most reached 309 to 478 mg of averaged
tilt. Provisional: the margin is about two times on one log only. A hump
taller than about 36 mm under one wheel would reach the tilt limit. A car
lifted slowly and level may be caught only by the tilt rule, or not at
all. Lifting during a scan, when the gate is off, is not detected.

The first car trial (S4-H11, without the unaveraged rule) stopped two
deliberate lifts: one by the upward rule in about 200 ms, one by the
averaged tilt in about 530 ms, because the average needs about 0.5 s to
reach 300 mg. A light lift (about 100 mg of tilt, +184 mg up) was not
detected; it reads like a hump. The unaveraged rule was added for the
delay: replayed on that log it stops the second lift after about 290 ms.
Driving jolts, such as leaving the hump, held 200 mg for at most 2 samples
in about 3300 driving samples, so the margin is one sample; obstacle
bypass manoeuvres are not in that data.

### 16. No detection during the obstacle bypass (2026-11-20)

Approved by Dalton on the integration branch. Revises decision 12's
driving modes.

**Problem.** In the SYS-05/06/07 runs of 2026-11-20 every bypass ended in
a false pick-up, with no one touching the car: at 15:49:52 and 15:51:26
in BYPASS, at 15:50:34 in REACQUIRE
(`testing/system/logs/2026-11-20_system-dashboard.jsonl`). The same
bypasses reported humps of 7 and 16 mm on a flat floor. The bypass drives
at about 130 mm/s with spins in place (wheel speeds such as +132/-106
mm/s), over three times line-following speed. Decision 15 set its
thresholds from line-following data with a one-sample margin and noted
that bypass manoeuvres were not in it.

**Decision.** Humps, impacts and lifts are detected only in FOLLOW and
NAVIGATE (`s4_detect_gate()`), no longer in BYPASS or REACQUIRE. SCAN was
already excluded.

- Lifting the car during a bypass is not detected, as during a scan. The
  bypass still ends on its own limits, and line following after it
  re-arms detection after the 32-sample settle.
- The course hump is on the line, so a bypass does not cross it.
- NAVIGATE stays: its spins run at 30 mm/s, and the junction runs of
  2026-11-17 and 2026-11-20 had no false stop.
- Raising the thresholds was rejected: it needs bypass data not yet
  logged, and it would weaken detection of real lifts everywhere.

**Tests.** Repeat SYS-05/06/07 (no false pick-up or hump during the
bypass) and SYS-10 (lifts while following still stop the car).

## Subsystem 5 — Obstacle Scanning and Bypass Planning (Timothy)

### 1. Test-first approach

Write specifications and executable C tests for the Pico W before production code. Build with `TEST=1`, flash the firmware and collect results from the Pico W serial console. Run logic cases on the Pico W using controlled timestamps and injected inputs, followed by real RTOS/ISR, sensor, servo and integrated bypass tests. A computer builds the firmware and saves logs; it does not execute the test cases.

### 2. Hardware test setup

Use TRIG GP2, ECHO GP3 and servo GP15. Power the HC-SR04 from 5 V and
level-shift ECHO to 3.3 V before it reaches the Pico. Verify the assembled
wiring, supply voltages and actual servo model before powered tests. Obtain the
matching servo datasheet before choosing PWM limits. Use a ruler, angle
reference, logic analyser and voltage measurement for calibration.

### 3. Ranging rules

- Use integer millimetres and microsecond echo timing, with the datasheet's 340 m/s reference for conversion tests.
- Accept nominal distances of 20–4000 mm; establish usable course coverage through physical tests.
- Use a trigger pulse of at least 10 microseconds, at least 65 ms between triggers and a 30 ms echo-completion deadline. Never retrigger while ECHO is high.
- Monitor forward every 65 ms; reject data older than 150 ms.
- An unsafe, invalid or stale forward reading requests a stop immediately. Retry invalid readings only while stationary; three consecutive invalid attempts or stuck ECHO causes FAULT.

### 4. Scan parameters

Scan −60° to +60° in 15° coarse steps, then use 5° fine steps around detected edges and candidate gaps. Define 0° as forward and positive angles as left. Use the median of three valid readings per angle.

Calibrate pulse-to-angle mapping and wait the worst measured settling time plus 25% before sampling. Block this configuration if the servo cannot safely reach the required angles. Limit a complete scan to 15 s. Return forward and obtain fresh clearance before bypass movement.

### 5. Profile and bypass acceptance

Convert samples to vehicle-relative points and estimate width from fine-scan edges. Treat incomplete edges and unobserved space as unknown.

Evaluate left/right routes against the whole vehicle's swept footprint. Choose the shorter valid route, left on a tie. Require 50 mm physical clearance plus planning margins for measured sensing and motion errors. No safe route causes FAULT.

Set the hazard threshold from measured sensor offset, worst braking distance, travel during 65 ms sampling plus 100 ms response, 50 mm clearance and measurement uncertainty. Reject operating speeds beyond validated detection coverage.

### 6. Stop, cancellation and recovery contract

Match scan authorisation and stop confirmation to the active request ID. Confirm both wheels stationary for 100 ms using calibrated encoder thresholds; allow at most 2 s for confirmation.

Distinguish existing motor inhibition from a new STOP event. A new STOP, cancellation, vehicle movement or withdrawn authorisation aborts the scan and invalidates its echo generation, profile and plan. Acknowledge cancellation and reject late results. A rejected initial servo command prevents scan entry. A rejected trigger command aborts the active scan and requests a best-effort return to forward; the request remains invalid even if that return command is also rejected. Do not recursively publish a fault through a failed output path. Recovery requires controller-authorised RESET and fresh sensor validation.

### 7. Messages and compact formats

Allow one active scan request and one pending result. Reject busy, duplicate and expired requests. Unstarted requests expire after 2 s; unaccepted plans expire after 2 s or a reference-pose change.

Include message type, request ID, capture time and validity. Plans additionally contain reference pose, selected side, move/turn segments, minimum clearance and failure reason. Keep continuous clearance in protected shared state and discrete results in queues. Use non-blocking sends; an undeliverable result is invalidated and raises a delivery fault.

Use explicit masks/shifts, little-endian multi-byte fields and separate validity flags. Derive widths from valid ranges and precision; document the field table and golden messages before implementation. Telemetry contains obstacle status, essential measurements and bypass outcome; detailed scans stay local.

### 8. ISR/task handoff

Use the coordinated GPIO dispatcher. Capture native timestamps in the ISR; snapshot, validate and pack them in the scan task. Protect the handoff against both interrupts and cross-core access. Clear old completion signals before arming and associate accepted edges with the active trigger generation.

Give each hazard/fault its own event bit. Clear only handled bits. Test timestamp wraparound, stale completion signals, simultaneous events and cancellation during handoff; no lost or torn updates are acceptable.

### 9. Original-line confirmation

Combine line validity with encoder pose and the recorded original line corridor and direction. Require an unambiguous corridor match, heading within 15° of the original travel direction and stable valid line detection for 100 ms. Calibrate corridor tolerance from track width and odometry error.

Bound the controller's search to 5 s or 500 mm, whichever comes first. Ambiguous branches or exhausted bounds cause a stopped fault.

### 10. Acceptance and evidence

Repeat each physical measurement condition 10 times. Target distance error no greater than the larger of 10 mm or 2%, and width error no greater than the larger of 20 mm or 5%. Require 10/10 safe returns for each passable layout and safe stopped outcomes for every blocked, invalid or cancelled case.

Test under concurrent encoder, IR, ECHO, servo and WiFi load. Require control cycles ≤20 ms, stop/slowing initiation ≤100 ms after unsafe confirmation, and STOP-flag-to-motor-off ≤20 ms. Measure STOP reception-to-flag and physical braking separately.

Compare actual RAM, queue/payload sizes and timing against an unpacked reference using identical data and workload. Record calibration, tested commit, inputs, results and saved evidence. Numerical targets remain unvalidated until tested.

### 12. Ultrasonic power-up allowance (2026-11-17)

Approved by Dalton on the integration branch.

**Problem.** Every cold start from the power bank (Robo Pico VIN) came up
in Fault with a scan fault; boots after flashing or a reset button press
did not. A temporary diagnostic that published S5's first 16 forward
attempts over MQTT showed the cause: the first attempt is 60 ms after
boot, the first six attempts got no echo at all, the forward monitor
faulted on the third (200 ms after boot, three consecutive invalid
attempts, decision 3), and from the seventh (about 480 ms) every echo was
valid. On a USB boot after flashing, the sensor was already powered and
the first attempt (30 ms) was valid. The HC-SR04 datasheet gives no
power-up time; the module runs at 3V3 (decision 13).

**Decision.** `scan_task.c` sends no forward trigger until 1 s after
`scan_task_start()` (about twice the measured 480 ms). The allowance is
latched once passed, so the 32-bit microsecond timer wrap cannot reopen it.
Decision 3 is unchanged after that: three consecutive invalid attempts or
stuck ECHO still faults. During the allowance there is no forward reading;
the controller is in Ready with STOP latched at that time, so no movement
depends on it.

**Evidence.** `testing/subsystem5/logs/2026-11-17_s5-bootfault-*.txt`
(diagnostic image, not committed). To be confirmed by a cold power-bank
boot that comes up Ready.

### 13. The HC-SR04 stays on 3V3 (2026-11-17)

Approved by Dalton on the integration branch.

The module is powered from the 3V3 rail, as wired in the team pin mapping of
2026-11-03, and that is the agreed supply; the datasheet's 5 V figure is not
a requirement for this car. On 3V3, ECHO is already at a level the Pico's
GPIO accepts, so no level shifter is fitted. On the car on 2026-11-17 it read
203 mm at 200 mm and 482 mm at 500 mm with every reading valid, and it
detected an obstacle at 373 mm during a lap. Its only 3V3-specific
behaviour seen so far is the slow power-up handled by decision 12.

### 11. C module templates

Use these maintained templates for new subsystem 5 modules. Replace `module`
consistently and choose a root name that is unique in its first eight characters.

Header template:

```c
/**
 * @file module.h
 * @brief Describe the single entity exposed by this module.
 */
#ifndef MODULE_H
#define MODULE_H

/** Perform the documented module operation. */
void module_run (void);

#endif /* MODULE_H */

/* End of file. */
```

Source template:

```c
/**
 * @file module.c
 * @brief Implement the single entity exposed by this module.
 */
#include "../include/module.h"

void
module_run (void)
{
    /* Implement the documented operation. */
}

/* End of file. */
```
