# P2-08 robot car dashboard

A local web dashboard for the car's MQTT telemetry, versions 2 and 3 (S1
decisions 1 and 5). It runs on the laptop that runs the broker, decodes each frame with the
shared reference decoder (`../decode_telemetry_v2.py`), and writes every
message to a JSON-lines session log. It shows only what the car publishes.
Fields whose validity flag is clear are shown as "—", not as zero.

## What it shows

| Panel | Contents | Source |
|---|---|---|
| Header | broker link, frame age, state, mode, fault, STOP latched and its cause; START / STOP / RESET buttons | telemetry, command topic |
| Motion (S2) | left/right wheel speed, distance, heading, last 60 s of wheel speeds | telemetry |
| Line & barcode (S3) | which front sensor is on black; last barcode letter, its command, letters read | telemetry (barcode: version 3) |
| Barcode letters | which letter means left, right, straight and U-turn; Send to car, A–D (fills in the boot letters) | command topic (S1 decision 6) |
| Terrain (S4) | highest hump of the run | telemetry |
| Obstacle (S5) | forward range, last scan result and bypass side, nearest point, width, centre offset | telemetry |
| Events | state, mode (with the barcode command on NAVIGATE), fault, STOP and its cause, each barcode read, scan and hump changes, commands sent, broker link | derived |
| Link | good/bad frames, heartbeat, the car's telemetry counters, session log file | heartbeat, stats |

A version 2 car shows no barcode and no stop cause. The junction outcome
(turn finished, expired or failed) is not in telemetry; a failed turn
shows as the stop cause "junction".

The buttons only publish `START`, `STOP` or `RESET` to the command topic.
The Vehicle Controller still decides whether to act, exactly as for the GP20
button.

**Barcode letters.** Pick a different letter for each command and press
Send to car. The page refuses to send two commands with the same letter.
The server publishes `L=G`, `R=H`, `S=C`, `U=Z` (for example) to the
command topic, one per command. The car does not reply to these, so
"sent" means the broker accepted them. To check that the car took them,
drive over a barcode: the Line & barcode panel shows the letter and the
command the car used. The car goes back to A–D whenever it restarts, so
when the server sees the heartbeat count go back (`hb 0` after a higher
count), it sends the last table again and logs `car restarted` and
`barcode letters re-sent after restart`. If the dashboard was restarted
instead, press Send to car again.

## Set up (once)

```sh
cd testing/subsystem1/dashboard
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run

Start a broker on this laptop that listens on the network, at the address
the car is built with (`MQTT_BROKER_IP` in `config/wifi_secrets.h`; this must
be the laptop's address on the car's WiFi). For example, with mosquitto:

```sh
printf 'listener 1883 0.0.0.0\nallow_anonymous true\n' > /tmp/p2-08-broker.conf
mosquitto -c /tmp/p2-08-broker.conf -v
```

Then, in another terminal:

```sh
. .venv/bin/activate
python server.py --broker <broker address>
```

Open <http://127.0.0.1:8765>. Stop with Ctrl+C. Options:

| Option | Default | Meaning |
|---|---|---|
| `--port` | 1883 | broker port |
| `--prefix` | `p2-08/p2-08-car-01` | topic prefix of `telemetry`, `stats` and `command` |
| `--heartbeat` | `<prefix>/heartbeat` | heartbeat topic (`MQTT_TOPIC` in `wifi_secrets.h`) |
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
