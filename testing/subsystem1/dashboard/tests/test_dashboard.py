"""Unit tests for the dashboard's decoding and event logic.

Run from testing/subsystem1/dashboard: python -m unittest discover tests
Frames are synthetic, packed in the telemetry_encoder.h version 2 layout.
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

import decode_telemetry_v2 as v2  # noqa: E402
import server  # noqa: E402


def frame(f0=0x3F, t=1000, dist=10, f1=0x07, state=3, mode=1, fault=0,
          line=0, scan=0, side=0, left=40, right=38, heading=0, fwd=500,
          near=300, width=80, centre=-5, hump=0):
    return struct.pack("<BBIHBBBBhhhHHHhB", 2, f0, t, dist, f1,
                       state | (mode << 4), fault | (line << 4),
                       scan | (side << 4), left, right, heading, fwd, near,
                       width, centre, hump)


class ParseTest(unittest.TestCase):
    def test_fields(self):
        f = v2.parse(frame(heading=-1234, line=1, scan=1, side=2))
        self.assertEqual(f["heading_cdeg"], -1234)
        self.assertEqual((f["line"], f["scan_result"], f["bypass_side"]),
                         (1, 1, 2))

    def test_rejects_wrong_length_or_version(self):
        self.assertIsNone(v2.parse(b"\x02" * 26))
        self.assertIsNone(v2.parse(b"\x01" + frame()[1:]))

    def test_text_output_unchanged(self):
        text, t = v2.decode(frame(), None)
        self.assertEqual(t, 1000)
        self.assertIn("RUNNING/FOLLOW", text)


class DescribeTest(unittest.TestCase):
    def test_flags_and_names(self):
        d = server.describe(v2.parse(frame(f0=0x08, f1=0x01, fault=2)))
        self.assertTrue(d["stop_latched"])
        self.assertFalse(d["motion_valid"])
        self.assertTrue(d["forward_valid"])
        self.assertFalse(d["shape_valid"])
        self.assertEqual(d["fault_name"], "motion")


class ChangesTest(unittest.TestCase):
    def d(self, **kw):
        return server.describe(v2.parse(frame(**kw)))

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

    def test_fault_reported_not_cleared(self):
        self.assertIn("FAULT: motion",
                      server.changes(self.d(), self.d(state=5, fault=2)))
        self.assertNotIn("FAULT: none",
                         server.changes(self.d(fault=2), self.d()))


if __name__ == "__main__":
    unittest.main()
