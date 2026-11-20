"""Local MQTT dashboard for the P2-08 robot car (telemetry versions 2, 3).

Subscribes to the car's topics, decodes telemetry with the shared reference
decoder (../decode_telemetry_v2.py), records every message to a JSON-lines
session log and serves a single page that polls /api/state. The page can
send START, STOP and RESET to the command topic; the Vehicle Controller
still decides whether to act on them.

Usage:
    python server.py --broker 192.168.1.10
    then open http://127.0.0.1:8765
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlparse

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - setup error only
    sys.exit("paho-mqtt is missing: pip install -r requirements.txt")

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import decode_telemetry_v2 as decoder  # noqa: E402  (shared reference decoder)

COMMANDS = ("START", "STOP", "RESET")
STATIC = {"/": "index.html", "/app.js": "app.js", "/styles.css": "styles.css"}
TYPES = {".html": "text/html; charset=utf-8",
         ".js": "text/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8"}

# flags0 / flags1 bits (telemetry_encoder.h)
F0 = {"distance_valid": 0x01, "stopped": 0x02, "link_up": 0x04,
      "stop_latched": 0x08, "motion_valid": 0x10, "line_fresh": 0x20,
      "hump_valid": 0x40}
F1 = {"forward_valid": 0x01, "profile_valid": 0x02, "shape_valid": 0x04,
      "barcode_valid": 0x08}


def describe(fields: dict[str, Any]) -> dict[str, Any]:
    """Add names and validity flags to a parsed frame."""
    out = dict(fields)
    out.update({k: bool(fields["flags0"] & m) for k, m in F0.items()})
    out.update({k: bool(fields["flags1"] & m) for k, m in F1.items()})
    out["state_name"] = decoder.STATE.get(fields["state"], str(fields["state"]))
    out["mode_name"] = decoder.MODE.get(fields["mode"], str(fields["mode"]))
    out["fault_name"] = decoder.FAULT.get(fields["fault"], str(fields["fault"]))
    out["line_name"] = decoder.LINE.get(fields["line"], str(fields["line"]))
    out["scan_name"] = decoder.SCAN.get(fields["scan_result"],
                                        str(fields["scan_result"]))
    out["side_name"] = decoder.SIDE.get(fields["bypass_side"],
                                        str(fields["bypass_side"]))
    out["barcode_text"] = decoder.letter(fields["barcode_letter"])
    out["command_name"] = decoder.COMMAND.get(fields["barcode_command"],
                                              str(fields["barcode_command"]))
    out["stop_cause_name"] = (
        decoder.STOP.get(fields["stop_cause"], str(fields["stop_cause"]))
        if out["stop_latched"] and fields["stop_cause"] != 0 else None)
    return out


def changes(old: dict[str, Any] | None, new: dict[str, Any]) -> list[str]:
    """Human-readable events between two consecutive frames."""
    if old is None:
        return [f"first frame: {new['state_name']}/{new['mode_name']}"]
    events = []
    if new["state"] != old["state"]:
        events.append(f"state {old['state_name']} -> {new['state_name']}")
    if new["mode"] != old["mode"]:
        turn = (f" ({new['command_name']})"
                if new["mode_name"] == "NAVIGATE" and new["barcode_valid"]
                else "")
        events.append(f"mode {old['mode_name']} -> {new['mode_name']}{turn}")
    if new["fault"] != old["fault"] and new["fault"] != 0:
        events.append(f"FAULT: {new['fault_name']}")
    if new["barcode_valid"] and (
            not old["barcode_valid"]
            or new["barcode_count"] != old["barcode_count"]):
        events.append(f"barcode {new['barcode_text']} decoded: "
                      f"{new['command_name']}")
    if new["stop_latched"] and not old["stop_latched"]:
        cause = new["stop_cause_name"]
        events.append(f"STOP latched: {cause}" if cause else "STOP latched")
    if new["scan_result"] != old["scan_result"] and new["scan_result"] != 0:
        side = f" side {new['side_name']}" if new["bypass_side"] else ""
        events.append(f"scan result: {new['scan_name']}{side}")
    if new["hump_valid"] and new["hump_max_mm"] > old["hump_max_mm"]:
        events.append(f"highest hump now {new['hump_max_mm']} mm")
    return events


class Store:
    """Everything the page shows, guarded by one lock; also writes the log."""

    def __init__(self, log_path: Path) -> None:
        self.lock = threading.Lock()
        self.events: deque[dict[str, Any]] = deque(maxlen=2000)
        self.seq = 0
        self.connected = False
        self.telemetry: dict[str, Any] | None = None
        self.telemetry_at = 0.0
        self.stats = ""
        self.stats_at = 0.0
        self.heartbeat = ""
        self.heartbeat_at = 0.0
        self.frames = 0
        self.bad_frames = 0
        # Last 60 s of wheel speeds, so a page opened mid-run has a chart.
        self.speeds: deque[tuple[float, int, int]] = deque(maxlen=600)
        self.log_path = log_path
        log_path.parent.mkdir(exist_ok=True)
        self._log = log_path.open("a", encoding="utf-8")

    def record(self, kind: str, data: Any) -> None:
        line = {"host": datetime.now().isoformat(timespec="milliseconds"),
                "kind": kind, "data": data}
        self._log.write(json.dumps(line) + "\n")
        self._log.flush()

    def event(self, text: str) -> None:
        """Caller holds the lock."""
        self.seq += 1
        self.events.append({"seq": self.seq, "host": time.time(),
                            "text": text})
        self.record("event", text)

    def snapshot(self, after: int) -> dict[str, Any]:
        with self.lock:
            now = time.time()
            return {
                "now": now,
                "connected": self.connected,
                "telemetry": self.telemetry,
                "telemetry_age": now - self.telemetry_at
                if self.telemetry_at else None,
                "frames": self.frames,
                "bad_frames": self.bad_frames,
                "stats": self.stats,
                "stats_age": now - self.stats_at if self.stats_at else None,
                "heartbeat": self.heartbeat,
                "heartbeat_age": now - self.heartbeat_at
                if self.heartbeat_at else None,
                "events": [e for e in self.events if e["seq"] > after],
                "speeds": [p for p in self.speeds if now - p[0] <= 60],
                "log": self.log_path.name,
            }


class Bridge:
    """MQTT side: subscribe, decode, and publish commands."""

    def __init__(self, args: argparse.Namespace, store: Store) -> None:
        self.args = args
        self.store = store
        self.topic_telemetry = f"{args.prefix}/telemetry"
        self.topic_stats = f"{args.prefix}/stats"
        self.topic_command = f"{args.prefix}/command"
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="p2-08-dashboard")
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(1, 10)

    def start(self) -> None:
        self.client.connect_async(self.args.broker, self.args.port, 30)
        self.client.loop_start()

    def stop(self) -> None:
        self.client.loop_stop()
        self.client.disconnect()

    def send(self, command: str) -> tuple[bool, str]:
        if command not in COMMANDS:
            return False, "unsupported command"
        info = self.client.publish(self.topic_command, command.encode(), 1)
        ok = info.rc == mqtt.MQTT_ERR_SUCCESS
        with self.store.lock:
            self.store.event(f"command {command} "
                             f"{'sent' if ok else 'NOT sent'}")
        return ok, "sent" if ok else f"publish failed ({info.rc})"

    def _on_connect(self, client, userdata, flags, reason, props) -> None:
        ok = not reason.is_failure
        with self.store.lock:
            self.store.connected = ok
            self.store.event(f"broker {self.args.broker}:{self.args.port} "
                             f"{'connected' if ok else f'refused ({reason})'}")
        if ok:
            client.subscribe([(f"{self.args.prefix}/#", 0),
                              (self.args.heartbeat, 0)])

    def _on_disconnect(self, client, userdata, flags, reason, props) -> None:
        with self.store.lock:
            self.store.connected = False
            self.store.event(f"broker disconnected ({reason})")

    def _on_message(self, client, userdata, msg) -> None:
        now = time.time()
        s = self.store
        if msg.topic == self.topic_telemetry:
            fields = decoder.parse(msg.payload)
            with s.lock:
                if fields is None:
                    s.bad_frames += 1
                    s.record("bad_frame", msg.payload.hex())
                    return
                new = describe(fields)
                for text in changes(s.telemetry, new):
                    s.event(text)
                s.telemetry = new
                s.telemetry_at = now
                if new["motion_valid"]:
                    s.speeds.append((now, new["left_mm_s"],
                                     new["right_mm_s"]))
                s.frames += 1
                s.record("telemetry", fields)
        elif msg.topic == self.topic_stats:
            text = msg.payload.decode("ascii", "replace")
            with s.lock:
                s.stats, s.stats_at = text, now
                s.record("stats", text)
        elif msg.topic == self.args.heartbeat:
            text = msg.payload.decode("ascii", "replace")
            with s.lock:
                s.heartbeat, s.heartbeat_at = text, now
                s.record("heartbeat", text)
        elif msg.topic != self.topic_command:
            with s.lock:
                s.record("other", {"topic": msg.topic,
                                   "hex": msg.payload.hex()})


def make_handler(store: Store, bridge: Bridge):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:  # quiet
            pass

        def _send(self, status: int, body: bytes, ctype: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, data: Any) -> None:
            self._send(status, json.dumps(data).encode(),
                       "application/json")

        def do_GET(self) -> None:
            url = urlparse(self.path)
            if url.path == "/api/state":
                try:
                    after = int(parse_qs(url.query).get("after", ["0"])[0])
                except ValueError:
                    after = 0
                self._json(HTTPStatus.OK, store.snapshot(after))
            elif url.path in STATIC:
                path = HERE / "static" / STATIC[url.path]
                self._send(HTTPStatus.OK, path.read_bytes(),
                           TYPES[path.suffix])
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self) -> None:
            # A custom header keeps cross-site form posts out (CSRF).
            if (urlparse(self.path).path != "/api/command"
                    or self.headers.get("X-Dashboard") != "1"):
                self._json(HTTPStatus.FORBIDDEN, {"error": "forbidden"})
                return
            length = min(int(self.headers.get("Content-Length", "0")), 64)
            command = self.rfile.read(length).decode("ascii", "replace")
            ok, text = bridge.send(command.strip().upper())
            self._json(HTTPStatus.OK if ok else HTTPStatus.BAD_REQUEST,
                       {"ok": ok, "message": text})

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--broker", required=True,
                        help="broker address the car publishes to")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--prefix", default="p2-08/p2-08-car-01",
                        help="topic prefix of telemetry/stats/command")
    parser.add_argument("--heartbeat", default=None,
                        help="heartbeat topic (MQTT_TOPIC in wifi_secrets.h); "
                             "default <prefix>/heartbeat")
    parser.add_argument("--host", default="127.0.0.1",
                        help="HTTP bind address; keep it local")
    parser.add_argument("--http-port", type=int, default=8765)
    args = parser.parse_args()
    if args.heartbeat is None:
        args.heartbeat = f"{args.prefix}/heartbeat"

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    store = Store(HERE / "logs" / f"dashboard-{stamp}.jsonl")
    bridge = Bridge(args, store)
    bridge.start()
    server = ThreadingHTTPServer((args.host, args.http_port),
                                 make_handler(store, bridge))
    print(f"dashboard: http://{args.host}:{args.http_port}  "
          f"broker {args.broker}:{args.port}  log {store.log_path}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.stop()


if __name__ == "__main__":
    main()
