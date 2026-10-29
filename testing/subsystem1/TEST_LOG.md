# Subsystem 1 — test log

**Owner:** Hong Yih (2501110)  ·  **Planned subsystem:** WiFi/MQTT communication  ·  **Code:** `app_program/subsystem1/`

S1-U01–S1-U03 test the LED starter example. S1-U04–S1-U07 test the
hardware-independent communication logic using provisional message formats.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | cyclic handler is running | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks (not busy) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | LED on GP19 toggles every 250 ms | | manual | 1. Flash normal image. 2. Watch LED / measure with scope. | 2 Hz blink, 50 % duty |
| S1-U04 | Bounded command parsing | Command syntax, provisional contract | auto / host | `comms_test_commands()`; valid tokens, malformed input, nulls, explicit lengths and oversized input | All cases pass; unsupported bytes rejected |
| S1-U05 | Binary telemetry encoding | Provisional codec v1 | auto / host | `comms_test_telemetry()`; golden bytes, each flag, integer limits, valid zero, invalid distance, nulls and every undersized buffer | Exact bytes; no overwrite on failure |
| S1-U06 | Heartbeat and retries | Isolated scheduling logic | auto / host | `comms_test_timers()`; initialisation bounds, pending attempts, retry deadlines, reconnect, clock wrap and delayed polling | Correct actions without busy retries or catch-up bursts |
| S1-U07 | MQTT service bridge | Planned service-task integration boundary | auto / host | Fragmentation, exact topic, malformed lengths, STOP routing, regular routing, failed delivery, telemetry provider and counters | STOP never uses regular path; invalid input rejected; failures reported |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

### Communication logic run — 2026-10-27

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-27 | S1-U04–S1-U06 | Working changes based on `f5a0213` | Codex | PASS, host only | [Host output](logs/2026-10-27_logic-host.txt) | MSVC C11 mode compiling C99-compatible code, `/W4 /WX /analyze`; three groups passed |

The four new C modules also compile for Cortex-M0+ with ARM GCC 10.3.1,
`-std=c99 -Wall -Wextra -Werror -Wconversion -Wsign-conversion -Wshadow
-Wstrict-prototypes -pedantic`, with tests enabled. The disabled-test module
also compiles with `APP_TEST=0`. This is compilation evidence, not execution
on the Pico W. No complete firmware build, RTOS/concurrency, live MQTT,
STOP timing or hardware validation was performed in this run.

Reproduce the host run with `run_host.ps1` as documented in
`app_program/subsystem1/README.md`. Record on-target results separately.

### Service bridge run — 2026-10-29

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-29 | S1-U04–S1-U07 | Working changes based on `75a488f` | Codex | PASS, host only | [Host output](logs/2026-10-29_service-bridge-host.txt) | MSVC C11 mode, `/W4 /WX /analyze`; four groups passed |

The new bridge and the existing logic modules also pass ARM GCC 10.3.1
syntax checking for Cortex-M0+ with strict C99 warnings. No full firmware,
Pico W, broker, RTOS concurrency or physical STOP timing was exercised.
