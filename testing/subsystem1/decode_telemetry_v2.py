"""Decode telemetry version 2 frames (S1 decision 1) for S1-H02.

Reference decoder used to record the 2026-11-17 S1-H01/H02 results. The
authoritative layout is the comment in
app_program/subsystem1/telemetry_encoder.h.

Usage (mosquitto clients installed, broker on this machine):

    mosquitto_sub -h 127.0.0.1 -t '#' -v -F '%U %t %x' \
        | python3 -u decode_telemetry_v2.py | tee logs/<date>_s1-h02.txt

Each telemetry line shows the robot time, the gap since the previous frame
(robot clock, wrap-safe), flags0 as DSLXMNH (distance valid, stopped, link
up, STOP latched, motion valid, line fresh, hump valid; '.' when clear),
flags1 as FPW (forward, profile, shape valid) and the decoded fields.
Other topics (heartbeat, stats, command) are printed as raw bytes.
"""
import struct
import sys

FRAME_SIZE = 27
VERSION = 2

STATE = {1: "INIT", 2: "READY", 3: "RUNNING", 4: "STOPPED", 5: "FAULT"}
MODE = {0: "-", 1: "FOLLOW", 2: "NAVIGATE", 3: "SCAN", 4: "BYPASS",
        5: "REACQUIRE"}
FAULT = {0: "none", 1: "init", 2: "motion", 3: "scan", 4: "delivery"}
LINE = {0: "CENTRE", 1: "LEFT", 2: "RIGHT", 3: "BOTH", 4: "STOP"}
SCAN = {0: "none", 1: "plan", 2: "clear", 3: "no-route", 4: "fault",
        5: "cancelled"}
SIDE = {0: "-", 1: "L", 2: "R"}

LINE_FRESH = 0x20
HUMP_VALID = 0x40


def decode(data, previous_ms):
    """Return (text, robot_ms) for one frame, or (error text, None)."""
    if len(data) != FRAME_SIZE or data[0] != VERSION:
        return (f"BAD len={len(data)} ver={data[:1].hex()} {data.hex()}",
                None)
    (_, f0, t, dist, f1, sm, fl, ss, left, right, heading, fwd, near,
     width, centre, hump) = struct.unpack("<BBIHBBBBhhhHHHhB", data)
    gap = "" if previous_ms is None else f" dt={(t - previous_ms) & 0xFFFFFFFF}ms"
    flags0 = "".join(c if f0 & (1 << i) else "." for i, c in enumerate("DSLXMNH"))
    flags1 = "".join(c if f1 & (1 << i) else "." for i, c in enumerate("FPW"))
    line = LINE.get(fl >> 4, fl >> 4) if f0 & LINE_FRESH else "-"
    hump_text = str(hump) if f0 & HUMP_VALID else "-"
    text = (f"t={t}{gap} [{flags0}|{flags1}] "
            f"{STATE.get(sm & 15, sm & 15)}/{MODE.get(sm >> 4, sm >> 4)} "
            f"fault={FAULT.get(fl & 15, fl & 15)} line={line} dist={dist} "
            f"L={left} R={right} hdg={heading / 100:.2f} fwd={fwd} "
            f"scan={SCAN.get(ss & 15, ss & 15)}/{SIDE.get(ss >> 4, ss >> 4)} "
            f"near={near} w={width} c={centre} hump={hump_text}")
    return (text, t)


def main():
    previous_ms = None
    for raw in sys.stdin:
        parts = raw.split()
        if len(parts) < 3:
            print(raw.rstrip(), flush=True)
            continue
        host_time, topic, payload_hex = parts[0], parts[1], parts[2]
        data = bytes.fromhex(payload_hex)
        if topic.endswith("/telemetry"):
            text, robot_ms = decode(data, previous_ms)
            if robot_ms is not None:
                previous_ms = robot_ms
            print(f"{host_time} {text}", flush=True)
        else:
            print(f"{host_time} {topic} {data!r}", flush=True)


if __name__ == "__main__":
    main()
