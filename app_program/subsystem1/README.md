# Subsystem 1: WiFi, MQTT and telemetry

Subsystem 1 runs its network-facing code inside the kernel-owned
`cyw43_utk_task`. No application task calls CYW43, lwIP or MQTT APIs.

## MQTT contract

The defaults below may be overridden in `config/wifi_secrets.h`:

| Direction | Topic | QoS | Retained | Payload |
|---|---|---:|---:|---|
| Broker to robot | `p2-08/p2-08-car-01/command` | 1 | no | Exactly `START`, `STOP` or `RESET`, or a barcode letter assignment `L=A`, `R=A`, `S=A` or `U=A` (any letter `A`–`Z`; S1 decision 6) |
| Robot to broker | `p2-08/p2-08-car-01/telemetry` | 0 | no | 31-byte binary frame, version 4 |
| Robot to broker | `p2-08/p2-08-car-01/heartbeat` | 0 | no | ASCII `hb N`, every 5 s |
| Robot to broker | `p2-08/p2-08-car-01/stats` | 0 | no | ASCII telemetry counters since boot, every 10 s (S1 decision 4) |

`STOP` raises Buddy 2's urgent, latched STOP flag. `START` and `RESET` are
offered to Buddy 2's non-blocking remote-command queue. A letter
assignment makes that letter the only one for left (`L`), right (`R`),
straight (`S`) or U-turn (`U`) in subsystem 3's table, until the car
restarts. A received command does not write motor PWM and does not bypass
the Vehicle Controller.

The telemetry frame is version 4 (`docs/decisions.md`, subsystem 1,
decisions 1, 5 and 7), little-endian. The authoritative byte layout is the comment in
`telemetry_encoder.h`; the decision lists every code. Summary:

| Bytes | Content |
|---:|---|
| 0 | Format version (`4`) |
| 1 | flags0: distance valid, stopped, MQTT link up, STOP latched, motion valid, line fresh, hump valid, last hump valid |
| 2-5 | Snapshot time in milliseconds (`uint32_t`) |
| 6-7 | Run distance in millimetres (`uint16_t`) |
| 8 | flags1: forward range valid, obstacle profile valid, obstacle shape valid, barcode valid, width raw (bytes 22-23 hold the unrefined span; display only) |
| 9-11 | Nibble pairs: state/mode, fault/line, scan result/bypass side |
| 12-17 | Left and right wheel mm/s, heading in centidegrees (`int16_t`) |
| 18-25 | Forward range, nearest obstacle ray, obstacle span (`uint16_t`), obstacle centre offset (`int16_t`) |
| 26 | Highest hump this run in millimetres (`uint8_t`) |
| 27 | Last barcode letter (5 bits, `A` = 0) and its command (3 bits) |
| 28 | Stop cause while STOP is latched (4 bits), barcode count (4 bits, wraps) |
| 29 | Last completed hump in millimetres (`uint8_t`, saturated) |
| 30 | Humps measured this run (`uint8_t`, saturates) |

A field is zero whenever its validity flag is clear. Bytes 0-7 keep their
version 1 meaning and bytes 1-26 their version 2 meaning.

Subsystem 2 fills `telemetry_encoder_snapshot_t`
(`telemetry_snapshot.h`, kernel-free) through `robot_read_telemetry()`,
which never waits.

The stats message reads `periods P sent S refused R err E nosnap N nolink L`:
of `P` one-second periods, `S` frames were handed to MQTT, `R` were refused
by `mqtt_publish()` (last error `E`), `N` had no snapshot (a provider lock
was busy) and `L` had no broker connection.

The WiFi task turns radio power save off at every link-up (S1 decision 4);
with the SDK default, frames and command ACKs were delayed by seconds.

Commands are stamped with the controller's clock, `s2_clock_ms()`, never
lwIP's `sys_now()` (S1 decision 3).

`testing/subsystem1/decode_telemetry_v2.py` is the reference decoder for
versions 2 and 3; its docstring has the `mosquitto_sub` command line.

The one-second telemetry period is an initial integration value and still
requires validation under combined robot load.

## Build and test

From `build_make` in WSL:

```sh
PICO_SDK_PATH="$HOME/pico/pico-sdk" make \
  WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 CONSOLE=usb_cdc -j8
```

Run all hardware-independent suites from the repository root:

```sh
make -C testing/host -j4
```

The production UF2 is
`build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.

