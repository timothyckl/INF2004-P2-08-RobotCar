# Decision Resolutions

## Subsystem 1 — WiFi/MQTT (Hong Yih)

_No decisions recorded yet._

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
a stop"), decided by Dalton on the integration branch; to be confirmed with
Timothy. On the track, the HC-SR04 facing open space or an angled surface
returned echoes that rose but were still high at the 30 ms deadline: one
such reading stopped the car, and a sustained run (134 in a row) latched a
scan fault. The datasheet gives a 4 m maximum range (23.5 ms); it does not
specify the output when no echo returns. A late echo is therefore reported
as a valid forward reading at the 4000 mm range bound (clear). An attempt
with no rising edge and ECHO stuck high past 130 ms remain invalid and still
fault. Real readings below 250 mm still stop the car (four such stops in
the same lap, including one at 248 mm beside the track on a curve).

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

## Subsystem 4 — IMU and Terrain Monitoring (Khang)

### 1. Standalone ownership and wiring

Subsystem 4 owns the GY-511 / LSM303DLHC and I2C0. Use GP0 for SDA and GP1
for SCL, power the module from 3V3, and use a common ground. Only
`s4_imu_task` accesses I2C0; the standalone implementation has
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
