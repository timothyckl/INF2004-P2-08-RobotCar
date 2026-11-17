"""Local MQTT flight recorder and HTTP dashboard for robot telemetry."""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import secrets
import threading
from typing import Any
from urllib.parse import parse_qs, urlparse

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:  # pragma: no cover - exercised during setup only.
    raise SystemExit(
        "paho-mqtt is missing. Activate the dashboard virtual environment and "
        "run: pip install -r requirements.txt"
    ) from exc

from telemetry_codec import decode_frame


ROOT = Path(__file__).resolve().parent
STATIC_ROOT = ROOT / "static"
DEFAULT_COMMAND_TOPIC = "p2-08/p2-08-car-01/command"
DEFAULT_TELEMETRY_TOPIC = "p2-08/p2-08-car-01/telemetry"
DEFAULT_HEARTBEAT_TOPIC = "mtk3smp-pico/heartbeat"
ALLOWED_COMMANDS = frozenset(("START", "STOP", "RESET"))


def local_timestamp() -> str:
    """Return an ISO timestamp with the computer's local UTC offset."""

    return datetime.now().astimezone().isoformat(timespec="milliseconds")


class EventStore:
    """Thread-safe event ring plus an append-only JSON-lines recording."""

    def __init__(self, log_path: Path, capacity: int = 10000) -> None:
        self._lock = threading.Lock()
        self._events: deque[dict[str, Any]] = deque(maxlen=capacity)
        self._sequence = 0
        self._log_path = log_path
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = log_path.open("a", encoding="utf-8", buffering=1)
        self.status: dict[str, Any] = {
            "connected": False,
            "connection_count": 0,
            "disconnect_count": 0,
            "telemetry_frames": 0,
            "invalid_frames": 0,
            "gap_count": 0,
            "estimated_missing": 0,
            "heartbeat_frames": 0,
            "last_telemetry_at": None,
            "last_heartbeat_at": None,
        }

    @property
    def log_path(self) -> Path:
        return self._log_path

    def append(
        self, kind: str, data: dict[str, Any], severity: str = "info"
    ) -> dict[str, Any]:
        with self._lock:
            self._sequence += 1
            event = {
                "sequence": self._sequence,
                "host_time": local_timestamp(),
                "kind": kind,
                "severity": severity,
                "data": data,
            }
            self._events.append(event)
            self._log_file.write(json.dumps(event, separators=(",", ":")) + "\n")
            return dict(event)

    def update_status(self, **values: Any) -> None:
        with self._lock:
            self.status.update(values)

    def increment(self, name: str, amount: int = 1) -> None:
        with self._lock:
            self.status[name] = int(self.status.get(name, 0)) + amount

    def read_after(self, sequence: int) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(event) for event in self._events if event["sequence"] > sequence]

    def status_copy(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self.status)
            result["latest_sequence"] = self._sequence
            result["log_path"] = str(self._log_path)
            return result

    def close(self) -> None:
        with self._lock:
            self._log_file.close()


class MqttRecorder:
    """Subscribe to robot topics and record decoded diagnostic events."""

    def __init__(self, args: argparse.Namespace, store: EventStore) -> None:
        self.args = args
        self.store = store
        self._previous_robot_ms: int | None = None
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=args.client_id,
            protocol=mqtt.MQTTv311,
        )
        self._client.on_connect = self._on_connect
        self._client.on_connect_fail = self._on_connect_fail
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=1, max_delay=10)

        username = os.environ.get("ROBOT_MQTT_USERNAME")
        password = os.environ.get("ROBOT_MQTT_PASSWORD")
        if username:
            self._client.username_pw_set(username, password)

        self.store.update_status(
            broker=args.broker,
            broker_port=args.port,
            telemetry_topic=args.telemetry_topic,
            heartbeat_topic=args.heartbeat_topic,
            command_topic=args.command_topic,
            expected_period_ms=args.period_ms,
        )

    def start(self) -> None:
        self.store.append(
            "broker",
            {"message": "connecting", "broker": self.args.broker, "port": self.args.port},
        )
        self._client.connect_async(self.args.broker, self.args.port, keepalive=60)
        self._client.loop_start()

    def stop(self) -> None:
        self._client.disconnect()
        self._client.loop_stop()

    def publish_command(self, command: str) -> tuple[bool, str]:
        if command not in ALLOWED_COMMANDS:
            return False, "unsupported command"
        if not self.store.status_copy().get("connected", False):
            return False, "MQTT broker is not connected"

        info = self._client.publish(
            self.args.command_topic,
            payload=command.encode("ascii"),
            qos=1,
            retain=False,
        )
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            return False, f"publish failed with MQTT error {info.rc}"

        self.store.append(
            "command",
            {"command": command, "topic": self.args.command_topic, "mid": info.mid},
            severity="warning" if command != "STOP" else "critical",
        )
        return True, "published"

    def _on_connect(
        self,
        client: mqtt.Client,
        userdata: Any,
        flags: mqtt.ConnectFlags,
        reason_code: mqtt.ReasonCode,
        properties: mqtt.Properties | None,
    ) -> None:
        del userdata, flags, properties
        if reason_code == 0:
            topics = [
                (self.args.telemetry_topic, 0),
                (self.args.heartbeat_topic, 0),
            ]
            result, message_id = client.subscribe(topics)
            self.store.update_status(connected=True)
            self.store.increment("connection_count")
            self.store.append(
                "broker",
                {
                    "message": "connected and subscribing",
                    "subscriptions": [topic for topic, _qos in topics],
                    "subscribe_result": result,
                    "mid": message_id,
                },
            )
        else:
            self.store.update_status(connected=False)
            self.store.append(
                "broker",
                {"message": "connection rejected", "reason": str(reason_code)},
                severity="error",
            )

    def _on_connect_fail(self, client: mqtt.Client, userdata: Any) -> None:
        del client, userdata
        self.store.update_status(connected=False)
        self.store.append(
            "broker", {"message": "TCP connection failed; retrying"}, severity="error"
        )

    def _on_disconnect(
        self,
        client: mqtt.Client,
        userdata: Any,
        disconnect_flags: mqtt.DisconnectFlags,
        reason_code: mqtt.ReasonCode,
        properties: mqtt.Properties | None,
    ) -> None:
        del client, userdata, disconnect_flags, properties
        self.store.update_status(connected=False)
        self.store.increment("disconnect_count")
        self.store.append(
            "broker",
            {"message": "disconnected", "reason": str(reason_code)},
            severity="warning",
        )

    def _on_message(
        self, client: mqtt.Client, userdata: Any, message: mqtt.MQTTMessage
    ) -> None:
        del client, userdata
        if message.topic == self.args.telemetry_topic:
            self._record_telemetry(bytes(message.payload))
        elif message.topic == self.args.heartbeat_topic:
            text = bytes(message.payload).decode("utf-8", errors="replace")
            self.store.increment("heartbeat_frames")
            self.store.update_status(last_heartbeat_at=local_timestamp())
            self.store.append(
                "heartbeat",
                {"topic": message.topic, "payload": text, "raw_hex": message.payload.hex()},
            )

    def _record_telemetry(self, payload: bytes) -> None:
        try:
            decoded = decode_frame(
                payload,
                previous_robot_ms=self._previous_robot_ms,
                expected_period_ms=self.args.period_ms,
            )
        except ValueError as exc:
            self.store.increment("invalid_frames")
            self.store.append(
                "telemetry_error",
                {"message": str(exc), "raw_hex": payload.hex(), "length": len(payload)},
                severity="error",
            )
            return

        self._previous_robot_ms = decoded["captured_ms"]
        self.store.increment("telemetry_frames")
        if decoded["gap"]:
            self.store.increment("gap_count")
            self.store.increment("estimated_missing", decoded["estimated_missing"])
        self.store.update_status(last_telemetry_at=local_timestamp())

        severity = "warning" if decoded["gap"] else "info"
        if not decoded["version_supported"] or decoded["unknown_flags"]:
            severity = "error"
            self.store.increment("invalid_frames")
        self.store.append("telemetry", decoded, severity=severity)


class DashboardServer(ThreadingHTTPServer):
    """HTTP server carrying the runtime services used by request handlers."""

    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        store: EventStore,
        recorder: MqttRecorder,
        command_token: str,
    ) -> None:
        super().__init__(address, DashboardHandler)
        self.store = store
        self.recorder = recorder
        self.command_token = command_token


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve static assets and a small same-origin diagnostic API."""

    server: DashboardServer

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API.
        parsed = urlparse(self.path)
        if parsed.path == "/api/config":
            status = self.server.store.status_copy()
            self._send_json(
                {
                    "command_token": self.server.command_token,
                    "broker": status.get("broker"),
                    "broker_port": status.get("broker_port"),
                    "topics": {
                        "telemetry": status.get("telemetry_topic"),
                        "heartbeat": status.get("heartbeat_topic"),
                        "command": status.get("command_topic"),
                    },
                    "expected_period_ms": status.get("expected_period_ms"),
                    "schema": {
                        "published": [
                            "format version",
                            "distance validity",
                            "stopped flag",
                            "link-up flag",
                            "robot capture time",
                            "run distance",
                        ],
                        "not_published": [
                            "controller state and fault",
                            "line and barcode status",
                            "terrain and peak status",
                            "obstacle and bypass status",
                        ],
                    },
                }
            )
        elif parsed.path == "/api/status":
            self._send_json(self.server.store.status_copy())
        elif parsed.path == "/api/events":
            query = parse_qs(parsed.query)
            try:
                after = max(0, int(query.get("after", ["0"])[0]))
            except ValueError:
                self._send_json({"error": "after must be an integer"}, HTTPStatus.BAD_REQUEST)
                return
            self._send_json({"events": self.server.store.read_after(after)})
        else:
            self._serve_static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API.
        if urlparse(self.path).path != "/api/command":
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        if self.headers.get("X-Dashboard-Token") != self.server.command_token:
            self._send_json({"error": "invalid command token"}, HTTPStatus.FORBIDDEN)
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0 or length > 1024:
            self._send_json({"error": "invalid request size"}, HTTPStatus.BAD_REQUEST)
            return

        try:
            request = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json({"error": "invalid JSON"}, HTTPStatus.BAD_REQUEST)
            return

        command = str(request.get("command", "")).upper()
        confirmed = request.get("confirmed") is True
        if command not in ALLOWED_COMMANDS:
            self._send_json({"error": "unsupported command"}, HTTPStatus.BAD_REQUEST)
            return
        if command != "STOP" and not confirmed:
            self._send_json(
                {"error": "START and RESET require explicit confirmation"},
                HTTPStatus.CONFLICT,
            )
            return

        ok, message = self.server.recorder.publish_command(command)
        status = HTTPStatus.ACCEPTED if ok else HTTPStatus.SERVICE_UNAVAILABLE
        self._send_json({"ok": ok, "message": message}, status)

    def log_message(self, format_string: str, *args: Any) -> None:
        del format_string, args

    def _serve_static(self, request_path: str) -> None:
        relative = "index.html" if request_path == "/" else request_path.lstrip("/")
        target = (STATIC_ROOT / relative).resolve()
        try:
            target.relative_to(STATIC_ROOT.resolve())
        except ValueError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        content = target.read_bytes()
        mime_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"{mime_type}; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "img-src 'self' data:; connect-src 'self'",
        )
        self.end_headers()
        self.wfile.write(content)

    def _send_json(
        self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK
    ) -> None:
        content = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(content)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record and inspect P2-08 robot MQTT telemetry."
    )
    parser.add_argument("--broker", required=True, help="MQTT broker IPv4/hostname")
    parser.add_argument("--port", type=int, default=1883, help="MQTT TCP port")
    parser.add_argument("--listen", default="127.0.0.1", help="dashboard bind address")
    parser.add_argument("--http-port", type=int, default=8765, help="dashboard HTTP port")
    parser.add_argument("--period-ms", type=int, default=1000, help="expected test period")
    parser.add_argument("--telemetry-topic", default=DEFAULT_TELEMETRY_TOPIC)
    parser.add_argument("--heartbeat-topic", default=DEFAULT_HEARTBEAT_TOPIC)
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC)
    parser.add_argument("--client-id", default="p2-08-diagnostic-dashboard")
    parser.add_argument("--log-file", type=Path)
    args = parser.parse_args()
    if not (1 <= args.port <= 65535 and 1 <= args.http_port <= 65535):
        parser.error("ports must be between 1 and 65535")
    if args.period_ms <= 0:
        parser.error("--period-ms must be positive")
    return args


def main() -> None:
    args = parse_args()
    if args.log_file is None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        args.log_file = ROOT / "logs" / f"dashboard-{stamp}.jsonl"

    store = EventStore(args.log_file)
    recorder = MqttRecorder(args, store)
    command_token = secrets.token_urlsafe(24)
    httpd = DashboardServer(
        (args.listen, args.http_port), store, recorder, command_token
    )

    recorder.start()
    print(f"Dashboard: http://127.0.0.1:{args.http_port}")
    print(f"Session log: {store.log_path}")
    print("Press Ctrl+C to stop.")
    try:
        httpd.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        recorder.stop()
        store.close()


if __name__ == "__main__":
    main()

