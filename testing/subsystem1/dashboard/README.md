# Subsystem 1 MQTT diagnostic dashboard

This local-only dashboard subscribes to the integrated firmware's MQTT topics,
decodes the provisional eight-byte telemetry frame and records every event to a
JSON-lines session log. It detects robot-time gaps independently of broker
arrival jitter and exports the browser view as CSV or JSON.

The dashboard does not invent values that are absent from the wire format. The
current frame contains only format version, flags, capture time and run
distance. Controller state/fault, line/barcode, terrain/peak and
obstacle/bypass fields remain visibly marked **not published** until the owners
agree and implement a telemetry schema.

## 1. Firmware and broker

Build and flash the integration firmware from WSL:

```sh
cd /mnt/c/SIT/Y2S1/EmbedSystems/INF2004-P2-08-RobotCar/build_make
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
make WIFI=cyw43 NET=lwip MQTT=1 TEST=0 -j8
```

The broker must listen on an address reachable from the Pico W. The dashboard
uses normal MQTT/TCP; the broker does not need WebSocket support. Match these
values with `config/wifi_secrets.h` without copying any WiFi or broker password
into this directory:

- broker IPv4 address and port;
- telemetry topic (default `p2-08/p2-08-car-01/telemetry`);
- heartbeat topic (default `mtk3smp-pico/heartbeat`);
- command topic (default `p2-08/p2-08-car-01/command`).

## 2. Install the dashboard dependency

Run from Ubuntu/WSL:

```sh
cd /mnt/c/SIT/Y2S1/EmbedSystems/INF2004-P2-08-RobotCar/testing/subsystem1/dashboard
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
```

If `venv` is unavailable, install it with the package matching the installed
Python version, for example `sudo apt install python3-venv`.

## 3. Start recording

Replace the example address with the broker address used by the firmware:

```sh
python server.py --broker 172.20.10.4
```

Then open <http://127.0.0.1:8765> in Windows. Stop the server with `Ctrl+C`.
Each run is written to `logs/dashboard-YYYYMMDD-HHMMSS.jsonl`; that directory is
ignored by Git. CSV/JSON export in the page exports the events currently held by
the browser.

Override a non-default topic when starting the server, for example:

```sh
python server.py --broker 172.20.10.4 \
  --heartbeat-topic p2-08/p2-08-car-01/heartbeat
```

If the broker requires credentials, set them only in the shell environment:

```sh
export ROBOT_MQTT_USERNAME='test-user'
export ROBOT_MQTT_PASSWORD='test-password'
python server.py --broker 172.20.10.4
```

Do not commit credentials or session logs.

## Command safety

STOP can be published directly. START and RESET stay disabled until the command
panel is armed and require a second confirmation. The dashboard's arm control
is not a safety interlock: use the team's approved physical test setup, and rely
on the Vehicle Controller to authorise or reject movement.

## Decoder tests

The tests require only Python's standard library:

```sh
python -m unittest discover -s tests -v
```

