# Subsystem 5 — test log

**Owner:** Timothy (2501530). **Scope:** ultrasonic scanning and obstacle avoidance planning.

Requirements come from the Design Review (FR6, FR7, FR10, NFR2–NFR8), PostTri1 Versions B–C and the subsystem 5 sections of [`docs/decisions.md`](../../docs/decisions.md). Decision references below use `D3` for subsystem 5 decision 3, not another subsystem's section.

## Status and execution

All test cases execute on the **Raspberry Pi Pico W**, not on the build computer. The computer only cross-compiles firmware, copies the UF2 and saves serial output.

- **Ready / blocked:** S5-U03–S5-U17 have executable on-device test code. Until actual subsystem logic is bound, each reports `BLOCKED`, never PASS. The serial summary includes the blocked count.
- **Planned:** the case is specified but its code, interface or hardware prerequisites are not ready. Planned cases are not counted as executed.
- Automated logic cases run through the existing `subsystem5_test()` call in `usermain()` when built with `TEST=1`. They use controlled timestamps and injected inputs on the Pico W; these inputs do not validate real GPIO timing, servo movement or ISR concurrency.
- Real sensor, servo, RTOS/ISR and integration cases require separate on-board measurements. No motor or servo movement is enabled by this test scaffold.

Test C sources are in `testing/subsystem5/unit_tests/src/` and their headers in `testing/subsystem5/unit_tests/include/`; the subsystem test interfaces are in `app_program/subsystem5/include/test_interface.h`. The firmware build includes the unit source only for `TEST=1`. Normal firmware excludes it. All new/modified C requires BARR-C:2018 review; there is no custom BARR checker.

1. Build with `make -C build_make TEST=1 -j8`, following the SDK/toolchain setup in the project README. `make -C testing/subsystem5/unit_tests` forwards to the same firmware build; it does not run a native executable.
2. Flash `build_make/mtk3pico_smp1_usb_cdc_wifi_test.uf2` to the Pico W using BOOTSEL. This is the default build profile's filename; other profile options change it.
3. Capture the Pico W USB serial console at 115200 baud, then reset the board. Follow [`testing/README.md`](../README.md) for the platform-specific serial port and evidence procedure.
4. Record the committed firmware revision, test ID and saved serial result. A successful build is not an on-board test result, and blocked cases do not constitute a passing suite.

## Retired IDs

| ID | Previous placeholder | Disposition |
|---|---|---|
| S5-U01 | Runs in a task context | Retired; not evidence of any robot feature. Do not reuse. |
| S5-U02 | Empty manual-test placeholder | Retired without execution. Do not reuse. |

## Executable Pico W logic cases

All cases below execute on the Pico W and are **ready / blocked** until an adapter binds to real S5 logic. Every stateful case uses an isolated on-device fixture with controlled inputs, starting ready and forward-facing with no echo in flight. The synthetic fixture uses a 1000 µs effective settling interval and a 200 mm hazard threshold; neither is physical calibration evidence.

| ID | Reference | Initial state / inputs | Expected result |
|---|---|---|---|
| S5-U03 | D3 | Convert 1000, 2000 and 5000 µs pulse widths. | Valid outputs of 170, 340 and 850 mm, independently derived from 340 m/s. |
| S5-U04 | D3 | Convert 117 and 118 µs. | Reject the below-minimum physical pulse; accept the first above-minimum pulse with an in-range integer result. |
| S5-U05 | D3 | Convert 23529 and 23530 µs. | Accept the below-maximum physical pulse; reject the above-maximum pulse. |
| S5-U06 | D3 | Convert zero width with an existing output value. | Invalid result; no plausible measurement replaces the existing value. |
| S5-U07 | D3; invalid input handling | Supply a null conversion output pointer. | Reject safely without a crash. |
| S5-U08 | D3, D8 | Convert 30000 µs and `UINT32_MAX`. | Invalid; multiplication/overflow cannot produce a valid near obstacle distance. |
| S5-U09 | D6; FR10 | Request 7 and authorise it, but supply no stop confirmation. Advance 1000 µs. | No non-forward scan command or bypass plan. |
| S5-U10 | D6 | Request/authorise 7; confirm stop for request 8. Advance 1000 µs. | Wrong-request confirmation does not start scanning. |
| S5-U11 | D6 | Request 7 and confirm its stop, but provide no scan authorisation. | No non-forward scan command or bypass plan. |
| S5-U12 | D6; FR6 | Request 7, authorise 7 and confirm its stop. Advance the effective settling interval. | Scan starts; no premature plan. This positive control prevents a no-op subject from passing the gate suite. |
| S5-U13 | D6, D8 | Start an authorised scan; cancel 7; inject late echo edges. | Acknowledge 7; no subsequent sweep or plan from cancelled work. Forward monitoring may continue. |
| S5-U14 | D6 | Start an authorised stationary scan, then inject a new STOP event. | No subsequent sweep or bypass plan. Existing motor inhibition is not itself this new event. |
| S5-U15 | D4; FR6 | Start a matched, authorised scan. | First non-forward coarse command is −60°. |
| S5-U16 | D4 | Start the scan at time zero; tick at 999 and 1000 µs with the synthetic effective settling interval. | No scan trigger before settling; first trigger when the interval completes. |
| S5-U17 | D6 | Authorise request 7, withdraw authorisation, then confirm its stop. | Withdrawn permission cannot start scanning. |

The physical boundaries 20 and 4000 mm are not exactly representable as integer microsecond echo widths. U04/U05 therefore bracket them with adjacent pulse widths rather than invent exact pulses. Fractional-millimetre rounding is not asserted by U03; the fixtures there are exact integers.

## Remaining Pico W logic cases

These cases are **planned**, not executed. Each implementation must exercise just-below, exact and just-above boundaries where representable. Decoded plan/protocol tests require the complete field/range/width and result contract before becoming executable; the current seam is not a packed message format.

| ID | Reference | Procedure / inputs | Expected result |
|---|---|---|---|
| S5-U18 | D3 | Capture trigger transitions with ticks around 10 µs. | Trigger remains high for at least 10 µs. |
| S5-U19 | D3 | Supply valid echoes; inspect repeated trigger starts around 65000 µs spacing. | No trigger interval below 65 ms; forward monitoring follows its agreed sampling interval. |
| S5-U20 | D3 | Complete echoes below, at and above 30 ms from trigger start. | Completion within the deadline is eligible; later completion cannot supply the measurement. |
| S5-U21 | D3, D8 | Inject fall-without-rise, unsolicited edges, stuck-high input and malformed pulses. | No usable measurement from malformed capture; stuck ECHO faults; no retrigger while high. |
| S5-U22 | D6, D8 | Cancel/expire a capture, arm later work and inject old completions. | Old completion signals or timestamps cannot complete the new capture. |
| S5-U23 | D4 | Supply valid profile samples of 170, 850 and 340 mm plus separate invalid-sample cases. | Median is 340 mm; invalid samples are not treated as valid clear distances. |
| S5-U24 | D3 | While stationary, supply one, two and three consecutive invalid attempts; separately supply valid recovery readings. | Bounded retry policy; the third consecutive invalid attempt faults and invalidates the profile. |
| S5-U25 | D3 | Provide safe, unsafe and invalid forward data; advance freshness across 150 ms. | Unsafe/invalid data requests a stop without filtering delay; data older than 150 ms is unusable. |
| S5-U26 | D4; FR6 | Feed a complete deterministic coarse scan. | Coarse angles are −60, −45, −30, −15, 0, 15, 30, 45 and 60 degrees; three valid samples per angle. |
| S5-U27 | D4 | Feed profiles with known obstacle edges and candidate gaps. | Fine refinement uses 5° steps within ±60°; each sample follows effective settling. |
| S5-U28 | D4; FR10 | Prevent scan completion across the 15 s limit, measured from scan entry. | Scan times out, reports fault and invalidates partial profile/plan. |
| S5-U29 | D6 | Withhold request-matched stop confirmation across the 2 s wait limit. | Scan never starts; stop timeout faults. |
| S5-U30 | D7 | Send repeated/busy requests; delay an unstarted request across its 2 s expiry. | Reject duplicates/busy/expired requests; preserve the active request rather than replace it silently. |
| S5-U31 | D7 | Delay plan acceptance across 2 s; separately change reference pose first. | Expired or pose-invalidated plan cannot be accepted for movement. |
| S5-U32 | D7 | Reject delivery of a completed result through the fake queue. | Send does not block; result is invalidated and delivery fault is signalled. |
| S5-U33 | D6 | Enter FAULT; try unauthorised RESET, authorised RESET with invalid data, then valid fresh data. | No unauthorised recovery; readiness only after validation; no clearing system STOP or initiating movement. |
| S5-U34 | D5; FR6 | Use independent known-position/width fixtures, including incomplete edges. | Correct position/width within declared uncertainty; incomplete or unobserved space remains unknown. |
| S5-U35 | D5 | Evaluate vehicle swept footprints around turns at clearance boundaries with synthetic uncertainty margins. | Require 50 mm physical clearance plus the configured planning margins; a ray alone cannot establish clearance. |
| S5-U36 | D5 | Supply two safe routes of unequal length, then equal length. | Select the shorter route; select left on a tie. |
| S5-U37 | D5; FR10 | Supply blocked routes, incomplete coverage and no safe candidate. | FAULT, no usable plan and no movement request. |
| S5-U38 | D5 | Use independent sensor-offset, speed, response, braking and uncertainty fixtures. | Hazard threshold includes all agreed terms; reject speed beyond validated detection coverage. |
| S5-U39 | D7 | After field contract completion, use golden bytes, boundary values, invalid flags and malformed payloads. | Explicit masks/shifts and little-endian representation decode correctly; no truncation or invalid-as-clear interpretation. |
| S5-U40 | D6, D8 | Inject STOP, cancellation, movement or withdrawn permission during every active state and result handoff. | Cancelled work cannot reappear as a valid result or plan. |
| S5-U41 | D8 | Inject simultaneous independent hazard/fault events and clear only one handled event. | Other pending bits remain visible; no lost event. |
| S5-U42 | D7, D8 | Repeat capture, freshness and expiry fixtures across native timer wraparound. | Correct bounded elapsed-time handling; no stale result becomes fresh. |

## Pico W RTOS/ISR cases

These are **planned**, require actual S5 kernel objects/driver code and must run in isolated fixtures or explicit test mode. Passing the injected-input logic cases does not close these cases.

| ID | Reference | Procedure / inputs | Expected result / evidence |
|---|---|---|---|
| S5-U43 | NFR7; D8 | Start an isolated target fixture and inspect S5 kernel objects and echo signalling. | Valid object setup, bounded waits and expected completion signalling; serial log. |
| S5-U44 | D8 | Exercise actual ISR/task snapshots concurrently on both cores, including cancellation and wraparound. | No lost/torn capture or revived cancelled work; timestamped trace and captured reference pulses. |
| S5-U45 | NFR6; D7 | Saturate S5 request/result paths in an isolated fixture. | Non-blocking rejection/delivery-fault behaviour; recorded call durations and events. |

## Pico W hardware cases

These are **planned** and opt-in. Verify wiring, electrical levels, the actual device datasheets and safe travel before powering a test. Servo pulse limits must come from the correct servo datasheet and calibration, not guessed values.

| ID | Reference | Procedure / inputs | Expected result / evidence |
|---|---|---|---|
| S5-U46 | D2, D3 | With a known target, capture TRIG/ECHO using a logic analyser; repeat missing/late-echo conditions safely. | Verify ≥10 µs pulse, ≥65 ms spacing, echo deadline and invalid-capture handling; annotated captures. |
| S5-U47 | A10; D2, D4 | Measure commanded angles and safe travel repeatedly; measure worst settling time. | Required ±60° travel is safe or configuration is blocked; calibrated mapping and effective settling = worst observed +25%, rounded up to ms. |
| S5-U48 | A9; D3, D10 | Measure known distances for representative surfaces/angles, 10 repetitions per condition. | Error ≤ greater of 10 mm or 2%; preserve every measurement and invalid reading. |
| S5-U49 | A10; D4, D10 | Scan obstacles of known width/position, 10 repetitions per condition. | Width error ≤ greater of 20 mm or 5%; coarse/fine sequence visible in angle/range logs. |
| S5-U50 | A9; D5 | Characterise small, angled and poorly reflecting obstacles at the intended approach conditions. | Document validated detection coverage and limitations; unsupported speeds/coverage remain blocked. |

## Joint integration specifications

Allocate unused `INT-` IDs in the shared log with the other owners before implementation/execution; do not reuse S5 IDs for integration tests.

| Owners | Case | Expected outcome / evidence |
|---|---|---|
| Timothy + Dalton | Confirmed stop, bypass execution, cancellation and new hazards during bypass. | No sweep before request-matched stop; ≥50 mm physical clearance; unsafe/unknown clearance stops the vehicle. Timestamped echo/controller/motion logs and video. |
| Timothy + Edmund + Dalton | Original line, wrong branch, heading/stability boundaries, search timeout/distance. | Unambiguous original corridor, heading within 15°, valid line for 100 ms; stop on ambiguity or at 5 s / 500 mm. Calibrated corridor/odometry, logs and video. |
| Timothy + Dalton + Edmund + Hong Yih | Combined encoder/IR/ECHO/servo/WiFi load, queue saturation and concurrent hazard/fault/remote STOP. | Control cycles ≤20 ms; stop/slowing initiation ≤100 ms after unsafe confirmation; STOP flag to motor-off ≤20 ms. Measure reception-to-flag and physical braking separately; no stale restart or lost events. |
| Timothy + Dalton + Hong Yih | Shared A11 power validation within device/current limits. | No unintended reset or unreliable sensing under representative combined loads; supply/current captures. |
| Timothy + Hong Yih + Dalton | Packed/unpacked correctness and memory/timing comparison. | Identical workload and essential decoded data; actual RAM/queue/payload savings measured without broken timing. Raw profiles remain local. |

Repeat passable layouts for 10/10 safe bypass/original-line returns. Every invalid, blocked or cancelled scenario must end safely stopped. Full-course evidence belongs in the shared system log; A9/A10/A12 and A11 remain open until physical evidence supports them.

## Results

No Pico W executions have been recorded. Build verification does not count as subsystem acceptance or hardware validation. Do not enter PASS for blocked/planned cases; keep actual results append-only and record the committed revision as required by the repository testing guide.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
