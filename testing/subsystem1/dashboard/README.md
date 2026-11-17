# P2-08 robot car dashboard

A local web dashboard for the car's MQTT telemetry, version 2 (S1 decision
1). It runs on the laptop that runs the broker, decodes each frame with the
shared reference decoder (`../decode_telemetry_v2.py`), and writes every
message to a JSON-lines session log. It shows only what the car publishes.
Fields whose validity flag is clear are shown as "—", not as zero.

## What it shows

| Panel | Contents | Source |
|---|---|---|
| Header | broker link, frame age, state, mode, fault, STOP latched; START / STOP / RESET buttons | telemetry, command topic |
| Motion (S2) | left/right wheel speed, distance, heading, last 60 s of wheel speeds | telemetry |
| Line & barcode (S3) | which front sensor is on black | telemetry |
| Terrain (S4) | highest hump of the run | telemetry |
| Obstacle (S5) | forward range, last scan result and bypass side, nearest point, width, centre offset | telemetry |
| Events | state, mode, fault, STOP, scan and hump changes, commands sent, broker link | derived |
| Link | good/bad frames, heartbeat, the car's telemetry counters, session log file | heartbeat, stats |

The barcode letter, its command and the junction action are not in telemetry
v2. They are on the car's serial log only, and the page says so.

The buttons only publish `START`, `STOP` or `RESET` to the command topic.
The Vehicle Controller still decides whether to act, exactly as for the GP20
button.

## Set up (once)

```sh
cd testing/subsystem1/dashboard
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run

Start the broker that the car publishes to (the address set in
`config/wifi_secrets.h`), then:

```sh
. .venv/bin/activate
python server.py --broker <broker address>
```

Open <http://127.0.0.1:8765>. Stop with Ctrl+C. Options:

| Option | Default | Meaning |
|---|---|---|
| `--port` | 1883 | broker port |
| `--prefix` | `p2-08/p2-08-car-01` | topic prefix of `telemetry`, `stats` and `command` |
| `--heartbeat` | `mtk3smp-pico/heartbeat` | heartbeat topic (`MQTT_TOPIC` in `wifi_secrets.h`) |
| `--http-port` | 8765 | page port; the server binds to 127.0.0.1 only |

Each session is logged to `logs/dashboard-YYYYMMDD-HHMMSS.jsonl` (ignored by
Git), one JSON object per line: `telemetry` (the decoded fields), `stats`,
`heartbeat`, `bad_frame`, `event` and `other`.

## Tests

```sh
python3 -m unittest discover tests
```

They cover frame parsing, flag and name decoding, and the event rules, using
synthetic frames; they do not need paho-mqtt. On 2026-11-17 the server was
also checked end to end against a local mosquitto with synthetic frames:
decoding, events, a bad frame, stats, heartbeat, a START reaching the
command topic, a cross-site command refused, an unknown command rejected.
It has not yet been run against the car.
