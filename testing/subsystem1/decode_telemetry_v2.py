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


def parse(data):
    """Return the fields of one frame as a dict, or None if it is not v2.

    Field names follow telemetry_encoder.h. Raw codes are kept; the name
    tables above turn them into text.
    """
    if len(data) != FRAME_SIZE or data[0] != VERSION:
        return None
    (_, f0, t, dist, f1, sm, fl, ss, left, right, heading, fwd, near,
     width, centre, hump) = struct.unpack("<BBIHBBBBhhhHHHhB", data)
    return {
        "flags0": f0, "flags1": f1, "captured_ms": t, "distance_mm": dist,
        "state": sm & 15, "mode": sm >> 4, "fault": fl & 15, "line": fl >> 4,
        "scan_result": ss & 15, "bypass_side": ss >> 4,
        "left_mm_s": left, "right_mm_s": right, "heading_cdeg": heading,
        "forward_mm": fwd, "nearest_mm": near, "width_mm": width,
        "centre_offset_mm": centre, "hump_max_mm": hump,
    }


def decode(data, previous_ms):
    """Return (text, robot_ms) for one frame, or (error text, None)."""
    fields = parse(data)
    if fields is None:
        return (f"BAD len={len(data)} ver={data[:1].hex()} {data.hex()}",
                None)
    f0, f1, t = fields["flags0"], fields["flags1"], fields["captured_ms"]
    dist, sm = fields["distance_mm"], fields["state"] | (fields["mode"] << 4)
    fl = fields["fault"] | (fields["line"] << 4)
    ss = fields["scan_result"] | (fields["bypass_side"] << 4)
    left, right = fields["left_mm_s"], fields["right_mm_s"]
    heading, fwd = fields["heading_cdeg"], fields["forward_mm"]
    near, width = fields["nearest_mm"], fields["width_mm"]
    centre, hump = fields["centre_offset_mm"], fields["hump_max_mm"]
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
