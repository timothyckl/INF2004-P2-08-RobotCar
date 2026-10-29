# Buddy 1: independent communication logic

These modules are a first implementation for isolated testing. They do not
start networking, create controller objects, or replace the LED starter task.
The existing subsystem 1 self-test now also runs four communication logic
groups when built with `TEST=1`.

The service-task integration boundary and the exact questions for Buddy 2,
the sensor owners and the team are in [INTEGRATION.md](INTEGRATION.md).
Recommended starting values for team review are in
[PROPOSAL.md](PROPOSAL.md); they remain explicitly unapproved.

## MQTT service bridge

`service_bridge` accepts an exact command topic at runtime, assembles bounded
MQTT fragments and dispatches a complete validated command through callbacks.
STOP has its own callback; START and RESET share the regular-command callback.
The bridge also asks a provider for one consistent telemetry snapshot before
encoding it. It contains no network or kernel calls and is intended to run
inside the existing WiFi service task after shared integration is approved.

The final broker, topic, authentication, timing, QoS, retain, command-ID and
telemetry-field decisions are intentionally absent. The starter values are
not treated as project requirements.

## Command parser

`command_parser_parse(payload, length)` recognises exactly the ASCII bytes
`START`, `STOP` and `RESET`. No terminator is required. Whitespace, lowercase,
embedded null bytes, trailing bytes and unknown tokens are rejected. The
maximum payload is five bytes; oversize input is rejected before reading it.
Returned command IDs use a byte of local storage and are not a wire protocol.

This grammar is **provisional**, pending agreement with Dalton. Parsing says
only that the command syntax is recognised. It does not authorise movement,
reset STOP, check mission state, authenticate a sender, or detect replay.

For eventual MQTT integration, assemble a complete, bounded message before
parsing: MQTT callbacks may provide fragments. Do not parse each fragment as
a complete command. Agree topics, retained-command handling, expiry/request
IDs, duplicate handling and delivery failures before accepting live commands.
Urgent STOP must use `controller_flags` outside `remote_q`; START/RESET use
the agreed normal interface. The controller owns state validation, latched
STOP and recovery. Those objects are not created by this code.

## Telemetry encoder

`telemetry_encoder_encode()` consumes a consistent caller-owned snapshot and
returns eight bytes, or zero if arguments/capacity are invalid. Failure leaves
the output untouched. Output and input must not overlap. This is a binary
payload, not a C string: publish the returned length, never `strlen()`.

The **provisional codec version 1** is a test fixture, not an agreed complete
robot telemetry schema:

| Byte | Meaning | Representation |
|---|---|---|
| 0 | Format version | 1 |
| 1 | Flags | Bit 0: distance valid; bit 1: stopped; bit 2: link up |
| 2–5 | Capture timestamp | Unsigned 32-bit milliseconds, little-endian |
| 6–7 | Distance | Unsigned 16-bit millimetres, little-endian |

Unused flag bits are zero. An invalid distance is encoded as zero with its
validity bit clear; a valid zero remains distinguishable. The codec accepts
0–65535 mm as its representable range, **not a sensor operating range**.
The producer must establish physical validity. Timestamps wrap modulo 2^32.
`link_up` should mean WiFi/MQTT/subscriptions ready; `stopped` must come from
controller-confirmed state, not be inferred from a STOP command.

Golden example: timestamp `0x12345678`, distance `0xabcd`, all three flags set
encodes to `01 07 78 56 34 12 cd ab`.

Only the meaningful flag bits are assigned; timestamp/distance widths are
provisional until owners agree ranges, precision and freshness rules. No
packed C structures or bit-fields are transmitted. Hump, barcode, obstacle
and bypass summaries still need owner-approved fields. Do not publish raw
scan arrays by default. No RAM or timing savings are claimed yet.

## Heartbeat and retry scheduler

Initialise one `comms_timer_t` with heartbeat/retry periods of 1 through
2147483647 ms. No period is hard-coded as a validated requirement.
Call `comms_timer_poll()` with a monotonic modulo-32-bit millisecond clock,
connection status and whether a connection attempt is still pending.

- Offline: request WiFi first, then MQTT when WiFi is up. The first attempt
  is immediate; later attempts share the configured retry interval.
- A pending attempt suppresses new attempts. The network adapter must bound
  attempts with timeouts and update the pending/status inputs accurately.
- Online means WiFi and MQTT subscriptions are ready. The first online poll
  requests a heartbeat immediately; subsequent requests follow the period.
- Disconnecting resets the heartbeat schedule. After an observed online
  state, the first reconnect attempt is immediate. No catch-up burst occurs.
- An action records its issue time, not successful delivery. Failed heartbeat
  delivery is retried on the next heartbeat period. This is a fixed retry
  scheduler, not a complete WiFi/MQTT connection state machine.

Poll at least once per 2147483647 ms and use one continuous clock source.
Reinitialise after a clock reset. A scheduler has one owning task; protect it
externally if shared. Snapshots likewise require a consistent copy under the
provider's lock before encoding. The modules contain no mutable global state.

Only the built-in WiFi service task may execute the returned network actions.
No action can start motion or clear STOP. Actual network integration requires
the shared-library agreement described in `TEAM.md`.

## Testing and next steps

The tests use injected bytes, snapshots, status and timestamps. On a Pico W,
`make TEST=1` runs them through `subsystem1_test()` as S1-U04–S1-U07. Their
host runner uses the same test functions; it does not simulate the RTOS,
MQTT callbacks, motor safety or physical hardware.

On Windows with a host GCC on PATH:

```powershell
./testing/subsystem1/run_host.ps1
```

Or in Visual Studio Developer PowerShell:

```powershell
./testing/subsystem1/run_host.ps1 -Compiler cl
```

On Linux/WSL with a host GCC:

```sh
gcc -std=c99 -Wall -Wextra -Werror -Wconversion -Wsign-conversion \
  -Wshadow -pedantic -DAPP_TEST=1 -DS1_HOST_TEST \
  app_program/subsystem1/command_parser.c \
  app_program/subsystem1/telemetry_encoder.c \
  app_program/subsystem1/comms_timer.c \
  app_program/subsystem1/comms_test.c testing/subsystem1/logic_main.c \
  -o /tmp/robotcar_s1_logic
/tmp/robotcar_s1_logic
```

Next, review the provisional contracts with Dalton and sensor owners. Record
actual agreements in `docs/decisions.md` and `TEAM.md`, then add the approved
service-task adapter. Test full-queue STOP, stale commands, reconnection,
shared-state consistency and timing under combined load before live use.
