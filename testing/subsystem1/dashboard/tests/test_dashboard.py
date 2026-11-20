"""Unit tests for the dashboard's decoding and event logic.

Run from testing/subsystem1/dashboard: python -m unittest discover tests
Frames are synthetic, packed in the telemetry_encoder.h version 3 layout
(and version 2, which the decoder still reads).
"""
import struct
import sys
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
# server.py imports paho at module level; the logic under test does not
# use it, so a stub keeps these tests runnable without the dependency.
sys.modules.setdefault("paho", types.ModuleType("paho"))
sys.modules.setdefault("paho.mqtt", types.ModuleType("paho.mqtt"))
sys.modules.setdefault("paho.mqtt.client", types.ModuleType("client"))

import decode_telemetry_v2 as decoder  # noqa: E402
import server  # noqa: E402


def frame(f0=0x3F, t=1000, dist=10, f1=0x07, state=3, mode=1, fault=0,
          line=0, scan=0, side=0, left=40, right=38, heading=0, fwd=500,
          near=300, width=80, centre=-5, hump=0, letter=0, command=0,
          cause=0, count=0, version=3):
    v2_part = struct.pack("<BBIHBBBBhhhHHHhB", version, f0, t, dist, f1,
                          state | (mode << 4), fault | (line << 4),
                          scan | (side << 4), left, right, heading, fwd,
                          near, width, centre, hump)
    if version == 2:
        return v2_part
    return v2_part + bytes([letter | (command << 5), cause | (count << 4)])


class ParseTest(unittest.TestCase):
    def test_fields(self):
        f = decoder.parse(frame(heading=-1234, line=1, scan=1, side=2))
        self.assertEqual(f["heading_cdeg"], -1234)
        self.assertEqual((f["line"], f["scan_result"], f["bypass_side"]),
                         (1, 1, 2))

    def test_rejects_wrong_length_or_version(self):
        self.assertIsNone(decoder.parse(b"\x02" * 26))
        self.assertIsNone(decoder.parse(b"\x01" + frame()[1:]))
        self.assertIsNone(decoder.parse(b"\x03" + frame(version=2)[1:]))
        self.assertIsNone(decoder.parse(b""))

    def test_version_3_fields(self):
        f = decoder.parse(frame(f1=0x08, letter=5, command=1, cause=4,
                                count=15))
        self.assertEqual((f["barcode_letter"], f["barcode_command"],
                          f["stop_cause"], f["barcode_count"]),
                         (5, 1, 4, 15))
        self.assertEqual(decoder.letter(5), "F")
        self.assertEqual(decoder.letter(31), "?")

    def test_version_2_still_reads(self):
        f = decoder.parse(frame(version=2))
        self.assertEqual(f["version"], 2)
        self.assertEqual((f["barcode_letter"], f["stop_cause"]), (0, 0))
        text, _ = decoder.decode(frame(version=2), None)
        self.assertNotIn("bar=", text)

    def test_text_shows_barcode_and_cause(self):
        text, _ = decoder.decode(frame(f0=0x08, f1=0x08, letter=5,
                                       command=1, cause=4, count=2), None)
        self.assertIn("bar=F/LEFT#2", text)
        self.assertIn("stop=pick-up", text)

    def test_text_output_unchanged(self):
        text, t = decoder.decode(frame(), None)
        self.assertEqual(t, 1000)
        self.assertIn("RUNNING/FOLLOW", text)


class DescribeTest(unittest.TestCase):
    def test_flags_and_names(self):
        d = server.describe(decoder.parse(frame(f0=0x08, f1=0x01, fault=2)))
        self.assertTrue(d["stop_latched"])
        self.assertFalse(d["motion_valid"])
        self.assertTrue(d["forward_valid"])
        self.assertFalse(d["shape_valid"])
        self.assertEqual(d["fault_name"], "motion")
        self.assertIsNone(d["stop_cause_name"])   # cause 0, latched: "-"
        d = server.describe(decoder.parse(frame(f0=0x08, cause=4)))
        self.assertEqual(d["stop_cause_name"], "pick-up")
        d = server.describe(decoder.parse(frame(f0=0x00, cause=4)))
        self.assertIsNone(d["stop_cause_name"])   # not latched


class ChangesTest(unittest.TestCase):
    def d(self, **kw):
        return server.describe(decoder.parse(frame(**kw)))

    def test_first_frame(self):
        self.assertEqual(server.changes(None, self.d()),
                         ["first frame: RUNNING/FOLLOW"])

    def test_no_change_no_event(self):
        self.assertEqual(server.changes(self.d(), self.d()), [])

    def test_mode_scan_hump_and_stop(self):
        old = self.d(f0=0x37, hump=3)            # STOP latched (0x08) clear
        new = self.d(f0=0x37 | 0x40 | 0x08, mode=3, scan=1, side=1, hump=11)
        ev = server.changes(old, new)
        self.assertIn("mode FOLLOW -> SCAN", ev)
        self.assertIn("scan result: plan side L", ev)
        self.assertIn("highest hump now 11 mm", ev)
        self.assertIn("STOP latched", ev)

    def test_barcode_and_stop_cause_events(self):
        old = self.d(f0=0x37)
        new = self.d(f0=0x37 | 0x08, f1=0x0F, letter=5, command=1,
                     cause=4, count=1)
        ev = server.changes(old, new)
        self.assertIn("barcode F decoded: LEFT", ev)
        self.assertIn("STOP latched: pick-up", ev)
        # Same letter read again: the count changes, so it is reported.
        again = self.d(f0=0x37 | 0x08, f1=0x0F, letter=5, command=1,
                       cause=4, count=2)
        self.assertIn("barcode F decoded: LEFT",
                      server.changes(new, again))
        self.assertEqual(server.changes(again, again), [])

    def test_navigate_names_the_command(self):
        old = self.d(f1=0x0F, letter=5, command=2, count=1)
        new = self.d(f1=0x0F, letter=5, command=2, count=1, mode=2)
        self.assertIn("mode FOLLOW -> NAVIGATE (RIGHT)",
                      server.changes(old, new))

    def test_fault_reported_not_cleared(self):
        self.assertIn("FAULT: motion",
                      server.changes(self.d(), self.d(state=5, fault=2)))
        self.assertNotIn("FAULT: none",
                         server.changes(self.d(fault=2), self.d()))


if __name__ == "__main__":
    unittest.main()
