"""Decode telemetry frames, versions 2 to 5 (S1 decisions 1, 5, 7 and 8).

Reference decoder used to record the 2026-11-17 S1-H01/H02 results; it
still reads those version 2 logs. Version 3 adds the last barcode letter,
its command and count, and the cause of a STOP latch. Version 4 adds the
last hump's height, the hump count and a raw (unconfirmed) width. Version
5 (post-demo, 2026-11-26) adds a diagnostics byte: the motion fault detail,
the barcode command held for the next junction and a count of junctions
confirmed (low 2 bits). The authoritative
layout is the comment in app_program/subsystem1/telemetry_encoder.h. The
file keeps its name because earlier test records cite it.

Usage (mosquitto clients installed, broker on this machine):

    mosquitto_sub -h 127.0.0.1 -t '#' -v -F '%U %t %x' \
        | python3 -u decode_telemetry_v2.py | tee logs/<date>_s1-h02.txt

Each telemetry line shows the robot time, the gap since the previous frame
(robot clock, wrap-safe), flags0 as DSLXMNH (distance valid, stopped, link
up, STOP latched, motion valid, line fresh, hump valid, last hump valid;
'.' when clear), flags1 as FPWBR (forward, profile, shape, barcode valid,
width raw) and the decoded
fields.
Other topics (heartbeat, stats, command) are printed as raw bytes.
"""
import struct
import sys

# Version -> frame length. Version 3 appends bytes 27-28 to version 2,
# version 4 bytes 29-30 to version 3, version 5 byte 31 to version 4.
SIZES = {2: 27, 3: 29, 4: 31, 5: 32}

STATE = {1: "INIT", 2: "READY", 3: "RUNNING", 4: "STOPPED", 5: "FAULT"}
MODE = {0: "-", 1: "FOLLOW", 2: "NAVIGATE", 3: "SCAN", 4: "BYPASS",
        5: "REACQUIRE"}
FAULT = {0: "none", 1: "init", 2: "motion", 3: "scan", 4: "delivery"}
LINE = {0: "CENTRE", 1: "LEFT", 2: "RIGHT", 3: "BOTH", 4: "STOP"}
SCAN = {0: "none", 1: "plan", 2: "clear", 3: "no-route", 4: "fault",
        5: "cancelled"}
SIDE = {0: "-", 1: "L", 2: "R"}
COMMAND = {0: "none", 1: "LEFT", 2: "RIGHT", 3: "STRAIGHT", 4: "UTURN"}
# MOTION_FAULT_* (motion_api.h), sent while the fault class is motion.
MOTION_FAULT = {0: "-", 1: "init", 2: "encoder", 3: "stall", 4: "timeout",
                5: "stop timeout", 6: "external", 7: "task stalled"}
STOP = {0: "-", 1: "start-up", 2: "remote", 3: "button", 4: "pick-up",
        5: "hazard", 6: "line", 7: "scan", 8: "bypass", 9: "junction",
        10: "fault", 11: "script done"}

STOP_LATCHED = 0x08
LINE_FRESH = 0x20
HUMP_VALID = 0x40
HUMP_LAST_VALID = 0x80   # flags0, version 4
BARCODE_VALID = 0x08     # flags1, version 3
WIDTH_RAW = 0x10         # flags1, version 4
LETTER_INVALID = 31


def letter(code):
    """Barcode letter text for a 5-bit code."""
    return chr(ord("A") + code) if code < 26 else "?"


def parse(data):
    """Return the fields of one frame as a dict, or None if malformed.

    Field names follow telemetry_encoder.h. Raw codes are kept; the name
    tables above turn them into text. Fields an older version lacks are
    zero (barcode and last hump not valid, no stop cause, no humps).
    """
    if not data or SIZES.get(data[0]) != len(data):
        return None
    (_, f0, t, dist, f1, sm, fl, ss, left, right, heading, fwd, near,
     width, centre, hump) = struct.unpack("<BBIHBBBBhhhHHHhB", data[:27])
    bc, sc = (data[27], data[28]) if data[0] >= 3 else (0, 0)
    last, count = (data[29], data[30]) if data[0] >= 4 else (0, 0)
    diag = data[31] if data[0] >= 5 else 0
    return {
        "version": data[0],
        "flags0": f0, "flags1": f1, "captured_ms": t, "distance_mm": dist,
        "state": sm & 15, "mode": sm >> 4, "fault": fl & 15, "line": fl >> 4,
        "scan_result": ss & 15, "bypass_side": ss >> 4,
        "left_mm_s": left, "right_mm_s": right, "heading_cdeg": heading,
        "forward_mm": fwd, "nearest_mm": near, "width_mm": width,
        "centre_offset_mm": centre, "hump_max_mm": hump,
        "barcode_letter": bc & 31, "barcode_command": bc >> 5,
        "stop_cause": sc & 15, "barcode_count": sc >> 4,
        "hump_last_mm": last, "hump_count": count,
        "motion_fault": diag & 7, "nav_held": (diag >> 3) & 7,
        "junction_count": diag >> 6,
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
    flags0 = "".join(c if f0 & (1 << i) else "." for i, c in enumerate("DSLXMNHV"))
    flags1 = "".join(c if f1 & (1 << i) else "." for i, c in enumerate("FPWBR"))
    line = LINE.get(fl >> 4, fl >> 4) if f0 & LINE_FRESH else "-"
    hump_text = str(hump) if f0 & HUMP_VALID else "-"
    text = (f"t={t}{gap} [{flags0}|{flags1}] "
            f"{STATE.get(sm & 15, sm & 15)}/{MODE.get(sm >> 4, sm >> 4)} "
            f"fault={FAULT.get(fl & 15, fl & 15)} line={line} dist={dist} "
            f"L={left} R={right} hdg={heading / 100:.2f} fwd={fwd} "
            f"scan={SCAN.get(ss & 15, ss & 15)}/{SIDE.get(ss >> 4, ss >> 4)} "
            f"near={near} w={width} c={centre} hump={hump_text}")
    if fields["version"] >= 3:
        cause = (STOP.get(fields["stop_cause"], fields["stop_cause"])
                 if f0 & STOP_LATCHED else "-")
        bar = "-"
        if f1 & BARCODE_VALID:
            cmd = fields["barcode_command"]
            bar = (f"{letter(fields['barcode_letter'])}/"
                   f"{COMMAND.get(cmd, cmd)}#{fields['barcode_count']}")
        text += f" bar={bar} stop={cause}"
    if fields["version"] >= 4:
        last = (str(fields["hump_last_mm"]) if f0 & HUMP_LAST_VALID
                else "-")
        text += f" hump_last={last} humps={fields['hump_count']}"
    if fields["version"] >= 5:
        detail = MOTION_FAULT.get(fields["motion_fault"],
                                  fields["motion_fault"])
        held = COMMAND.get(fields["nav_held"], fields["nav_held"])
        text += (f" detail={detail} held={held}"
                 f" junctions={fields['junction_count']}")
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
