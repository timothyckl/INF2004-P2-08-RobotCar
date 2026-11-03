# Subsystem 1 integration contract

Status: implementation-ready boundary; final MQTT and cross-subsystem values
remain unapproved. This document records proposals and review questions. It is
not an agreed decision and must not be copied into `docs/decisions.md` until
the named owners agree.

The concrete defaults proposed for that review are in
[PROPOSAL.md](PROPOSAL.md).

## What is ready

Subsystem 1 now supplies four hardware-independent modules:

| Module | Responsibility | Intended execution context |
|---|---|---|
| `command_parser` | Recognise the provisional `START`, `STOP`, `RESET` grammar | WiFi service task |
| `service_bridge` | Match the command topic, assemble MQTT fragments, reject malformed input and route commands | WiFi service task |
| `telemetry_encoder` | Encode a bounded provisional snapshot explicitly in little-endian order | WiFi service task |
| `comms_timer` | Testable reference logic for heartbeat/retry deadlines | Test/reference only while the existing service owns live timers |

The bridge contains no lwIP, MQTT, CYW43 or T-Kernel calls. This keeps it
host-testable and prevents application tasks from touching the network stack.
One bridge instance should be owned by `cyw43_utk_task`; its callbacks must be
nonblocking because they execute inside that service task.

## Contract with Buddy 2

Buddy 2 owns the Vehicle Controller objects and must provide two adapter
functions when its real interfaces exist:

```c
bool controller_raise_remote_stop(void * p_context, uint32_t received_ms);
bool controller_send_remote_command(void * p_context,
    command_parser_command_t command, uint32_t received_ms);
```

`controller_raise_remote_stop()` must set the agreed urgent remote-STOP bit in
`controller_flags` without waiting. It must not place STOP in `remote_q`.
Returning `true` means the urgent signal was accepted; it does not mean the
wheels have stopped. The receipt timestamp must be retained so tests can
measure network-receipt-to-flag latency separately from flag-to-motor-off and
physical braking.

`controller_send_remote_command()` handles START and RESET only. It must send
the agreed command record to the controller-owned `remote_q` with `TMO_POL`.
It returns `false` if the queue is unavailable or full. The controller still
validates the command against mission state, latched STOP, recovery rules and
freshness. Receiving START must never authorise motion directly.

Buddy 2 needs to agree these missing details before its adapter is coded:

- the event-flag bit name and value for remote STOP;
- the `remote_q` object ID name, capacity and maximum message size;
- the START/RESET message structure, including request ID, receipt time,
  expiry, byte order and duplicate/stale-command rules;
- when the controller becomes ready during boot and what happens to a command
  received earlier;
- how receipt, rejection and completion are acknowledged for telemetry.

The current bridge deliberately returns `UNDELIVERED` instead of buffering a
command whose controller adapter rejects it. Silent retry could execute a
stale command later. Whether an explicit rejection acknowledgement is
published remains to be agreed.

## Contract with telemetry providers

The service bridge calls one provider function:

```c
bool robot_read_telemetry(void * p_context,
    telemetry_encoder_snapshot_t * p_snapshot);
```

The provider must copy one internally consistent snapshot and release every
mutex before returning. It must not publish, allocate memory, wait for sensor
input or hold a mutex across the later MQTT call. It returns `false` when no
safe snapshot is available; the service then skips that publication.

The present eight-byte codec is only a boundary test. It lacks motion, line,
barcode, terrain, obstacle and bypass data. Before replacing it, each owner
must provide the following for every proposed telemetry field:

| Required fact | Why Buddy 1 needs it |
|---|---|
| Meaning and unit | Prevent ambiguous interpretation |
| Valid physical/application range | Select the smallest safe width |
| Required precision/resolution | Avoid losing required information |
| Invalid, unknown and stale representation | Keep failure distinct from zero |
| Producer update period and maximum age | Define freshness |
| Concurrent writer and protection method | Prevent torn snapshots |
| Whether it is continuous state or a discrete event | Choose snapshot or queue |
| Whether it is essential remotely | Keep raw captures and scans local |

Buddy 2 supplies controller state, motion summary and STOP/fault status.
Buddy 3 supplies compact line/barcode/navigation results. Buddy 4 supplies
terrain/hump/highest-peak results. Buddy 5 supplies obstacle status, essential
measurements and bypass outcome. Detailed ultrasonic scans remain local as
already agreed.

## MQTT service adapter

The final adapter belongs in the existing WiFi service task. Adding it changes
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` and build include paths, so
`TEAM.md` requires team and instructor agreement first.

The current `subsystem1_start()` retains a sleeping compatibility task because
subsystem 4 directly uses its `tid_led` starter interface. The task no longer
configures or writes GP19. Deleting the compatibility task requires coordinated
removal of example IF-05 by Buddy 4 and an agreed update to shared startup
documentation; subsystem 1 cannot delete it independently without breaking the
current firmware link/runtime.

After that approval, the adapter should perform this sequence:

1. Initialise one `service_bridge_t` with the agreed command topic and the
   controller/telemetry adapter functions. Early calls must fail safely until
   application objects are ready because the WiFi task starts before
   `usermain()` creates subsystem objects.
2. Subscribe only to the command topic. Heartbeat and telemetry must publish
   to separate topics so the client cannot parse its own outbound packets as
   commands.
3. In the MQTT topic callback, call `service_bridge_begin()` with the exact
   topic length and lwIP's total payload length.
4. In each data callback, call `service_bridge_data()` with the fragment,
   `MQTT_DATA_FLAG_LAST` state and a monotonic local receipt timestamp.
5. At the agreed telemetry period, call `service_bridge_telemetry()` and
   publish exactly the returned byte count. Binary payloads are not strings.
6. Publish command rejection/delivery counters from
   `service_bridge_get_stats()` only after their telemetry representation is
   agreed.

The adapter must keep every lwIP/MQTT call in `cyw43_utk_task`, use bounded
callbacks, avoid blocking kernel waits and never log raw untrusted payload
bytes. It must not interpret reconnection as START, clear STOP, replay an old
command or retain command publications unless the team explicitly defines a
safe retained-message policy.

## MQTT decisions still required

Buddy 1 cannot safely finalise the shared service adapter until the team gives
these values:

- broker IPv4 address or approved hostname mechanism, port and whether the
  course broker requires authentication or TLS;
- unique client ID rules when multiple cars or developers connect;
- command, telemetry, heartbeat and acknowledgement topics;
- MQTT QoS and retain setting for each topic;
- exact command payload, request ID, timestamp/expiry and acknowledgement
  format; the current plain tokens cannot detect replay or duplicates;
- heartbeat and telemetry periods, maximum acceptable telemetry latency and
  offline buffering/drop policy;
- maximum payload and topic lengths;
- behaviour for broker rejection, subscription failure and publish failure.

The existing demo uses one topic, QoS 0, no retain flag, a 5 s heartbeat, a
3 s MQTT retry and a 5 s WiFi rejoin delay. These are starter-code values,
not validated project requirements.

## Verification required before integration is claimed

Host tests cover fragmentation, wrong topics, malformed and oversized input,
separate STOP routing, controller delivery failure, telemetry snapshot failure,
buffer limits and clock wraparound. Remaining evidence must come from the Pico
W and integrated robot:

- build and run with `NET=lwip MQTT=1 TEST=1`;
- disconnect/reconnect WiFi and the broker without restarting movement;
- send STOP while `remote_q` is full and measure receipt-to-flag and
  flag-to-motor-off separately;
- reject stale, duplicate, retained, malformed and oversized commands;
- confirm START/RESET remain subject to controller state validation;
- measure control-cycle and STOP timing during reconnect and telemetry load;
- test snapshot consistency under concurrent producer updates;
- compare packed/unpacked RAM, payload size and execution time with identical
  data before claiming savings;
- record broker captures, serial logs, tested commit and configuration with
  secrets removed.

