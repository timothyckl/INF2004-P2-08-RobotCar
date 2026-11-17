# Subsystem 1: WiFi, MQTT and telemetry

Subsystem 1 runs its network-facing code inside the kernel-owned
`cyw43_utk_task`. No application task calls CYW43, lwIP or MQTT APIs.

## MQTT contract

The defaults below may be overridden in `config/wifi_secrets.h`:

| Direction | Topic | QoS | Retained | Payload |
|---|---|---:|---:|---|
| Broker to robot | `p2-08/p2-08-car-01/command` | 1 | no | Exactly `START`, `STOP` or `RESET` |
| Robot to broker | `p2-08/p2-08-car-01/telemetry` | 0 | no | 27-byte binary frame, version 2 |
| Robot to broker | `p2-08/p2-08-car-01/heartbeat` | 0 | no | ASCII `hb N` |

`STOP` raises Buddy 2's urgent, latched STOP flag. `START` and `RESET` are
offered to Buddy 2's non-blocking remote-command queue. A received command
does not write motor PWM and does not bypass the Vehicle Controller.

The telemetry frame is version 2 (`docs/decisions.md`, subsystem 1,
decision 1), little-endian. The authoritative byte layout is the comment in
`telemetry_encoder.h`; the decision lists every code. Summary:

| Bytes | Content |
|---:|---|
| 0 | Format version (`2`) |
| 1 | flags0: distance valid, stopped, MQTT link up, STOP latched, motion valid, line fresh, hump valid |
| 2-5 | Snapshot time in milliseconds (`uint32_t`) |
| 6-7 | Run distance in millimetres (`uint16_t`) |
| 8 | flags1: forward range valid, obstacle profile valid, obstacle shape valid |
| 9-11 | Nibble pairs: state/mode, fault/line, scan result/bypass side |
| 12-17 | Left and right wheel mm/s, heading in centidegrees (`int16_t`) |
| 18-25 | Forward range, nearest obstacle ray, obstacle span (`uint16_t`), obstacle centre offset (`int16_t`) |
| 26 | Highest hump this run in millimetres (`uint8_t`) |

A field is zero whenever its validity flag is clear. Bytes 0-7 keep their
version 1 meaning.

Subsystem 2 fills `telemetry_encoder_snapshot_t`
(`telemetry_snapshot.h`, kernel-free) through `robot_read_telemetry()`,
which never waits.

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
`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.

