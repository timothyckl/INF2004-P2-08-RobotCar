"""Unit tests for the dashboard's decoding and event logic.

Run from testing/subsystem1/dashboard: python -m unittest discover tests
Frames are synthetic, packed in the telemetry_encoder.h version 5 layout
(and versions 2 to 4, which the decoder still reads).
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
          cause=0, count=0, last=0, humps=0, fault_detail=0, held=0,
          junctions=0, version=5):
    v2_part = struct.pack("<BBIHBBBBhhhHHHhB", version, f0, t, dist, f1,
                          state | (mode << 4), fault | (line << 4),
                          scan | (side << 4), left, right, heading, fwd,
                          near, width, centre, hump)
    if version == 2:
        return v2_part
    v3_part = v2_part + bytes([letter | (command << 5), cause | (count << 4)])
    if version == 3:
        return v3_part
    v4_part = v3_part + bytes([last, humps])
    if version == 4:
        return v4_part
    return v4_part + bytes([fault_detail | (held << 3) | (junctions << 6)])


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

    def test_version_4_fields(self):
        f = decoder.parse(frame(f0=0x80, f1=0x10, width=412, last=9,
                                humps=3))
        self.assertEqual((f["hump_last_mm"], f["hump_count"],
                          f["width_mm"]), (9, 3, 412))
        text, _ = decoder.decode(frame(f0=0x80, last=9, humps=3), None)
        self.assertIn("hump_last=9 humps=3", text)
        text, _ = decoder.decode(frame(f0=0x00, last=9, humps=3), None)
        self.assertIn("hump_last=- humps=3", text)

    def test_version_5_fields(self):
        f = decoder.parse(frame(fault=2, fault_detail=7, held=2,
                                junctions=3))
        self.assertEqual((f["version"], f["motion_fault"], f["nav_held"],
                          f["junction_count"]), (5, 7, 2, 3))
        text, _ = decoder.decode(frame(fault=2, fault_detail=3, held=1,
                                       junctions=1), None)
        self.assertIn("detail=stall held=LEFT junctions=1", text)
        # The car's golden frame (comms_test.c, S1-U05) decodes as built.
        golden = bytes([5, 0xff, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab,
                        0x0f, 0x43, 0x12, 0x21, 0x6a, 0xff, 0x40, 0x01,
                        0xb0, 0xb9, 0xa0, 0x0f, 0x02, 0x01, 0x04, 0x03,
                        0xfe, 0xff, 0xff, 0x99, 0xf4, 0xfe, 0x21, 0x57])
        g = decoder.parse(golden)
        self.assertEqual((g["motion_fault"], g["nav_held"],
                          g["junction_count"], g["barcode_letter"],
                          g["hump_count"]), (7, 2, 1, 25, 0x21))

    def test_version_4_still_reads(self):
        f = decoder.parse(frame(version=4, last=9, humps=3))
        self.assertEqual((f["version"], f["hump_count"], f["motion_fault"],
                          f["nav_held"], f["junction_count"]),
                         (4, 3, 0, 0, 0))
        self.assertIsNone(decoder.parse(b"\x05" + frame(version=4)[1:]))
        text, _ = decoder.decode(frame(version=4), None)
        self.assertNotIn("held=", text)

    def test_version_3_still_reads(self):
        f = decoder.parse(frame(version=3))
        self.assertEqual((f["version"], f["hump_last_mm"], f["hump_count"]),
                         (3, 0, 0))
        self.assertIsNone(decoder.parse(b"\x04" + frame(version=3)[1:]))
        text, _ = decoder.decode(frame(version=3), None)
        self.assertNotIn("humps=", text)

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
        self.assertFalse(d["width_raw"])
        self.assertFalse(d["hump_last_valid"])
        self.assertEqual(d["fault_name"], "motion")
        d = server.describe(decoder.parse(frame(f0=0x80, f1=0x12)))
        self.assertTrue(d["width_raw"])
        self.assertTrue(d["hump_last_valid"])
        self.assertIsNone(d["stop_cause_name"])   # cause 0, latched: "-"
        d = server.describe(decoder.parse(frame(f0=0x08, cause=4)))
        self.assertEqual(d["stop_cause_name"], "pick-up")
        d = server.describe(decoder.parse(frame(f0=0x00, cause=4)))
        self.assertIsNone(d["stop_cause_name"])   # not latched


class SubscriptionsTest(unittest.TestCase):
    def test_heartbeat_under_prefix_is_not_subscribed_twice(self):
        self.assertEqual(server.subscriptions("p2-08/car", "p2-08/car/heartbeat"),
                         [("p2-08/car/#", 0)])

    def test_heartbeat_elsewhere_is_added(self):
        self.assertEqual(server.subscriptions("p2-08/car", "robot/heartbeat"),
                         [("p2-08/car/#", 0), ("robot/heartbeat", 0)])

    def test_similar_prefix_is_not_covered(self):
        self.assertEqual(server.subscriptions("p2-08/car", "p2-08/car2/hb"),
                         [("p2-08/car/#", 0), ("p2-08/car2/hb", 0)])


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

    def test_hump_count_event(self):
        old = self.d(f0=0x37 | 0x40 | 0x80, hump=11, last=11, humps=1)
        new = self.d(f0=0x37 | 0x40 | 0x80, hump=11, last=6, humps=2)
        ev = server.changes(old, new)
        self.assertIn("hump 2 measured: 6 mm", ev)
        self.assertNotIn("highest hump now 11 mm", ev)
        self.assertEqual(server.changes(new, new), [])

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

    def test_v5_fault_detail_held_and_junctions(self):
        events = server.changes(self.d(), self.d(state=5, fault=2,
                                                 fault_detail=7))
        self.assertIn("FAULT: motion (task stalled)", events)
        self.assertIn("held for the next junction: RIGHT",
                      server.changes(self.d(), self.d(held=2)))
        self.assertEqual(server.changes(self.d(held=2), self.d()), [])
        self.assertIn("junction confirmed",
                      server.changes(self.d(junctions=3), self.d(junctions=0)))
        self.assertIn("2 junctions confirmed",
                      server.changes(self.d(junctions=1), self.d(junctions=3)))
        d = server.describe(decoder.parse(frame(fault=2, fault_detail=3)))
        self.assertEqual(d["motion_fault_name"], "stall")
        self.assertIsNone(server.describe(decoder.parse(frame()))["held_name"])

    def test_v4_frames_raise_no_v5_events(self):
        self.assertEqual(server.changes(self.d(version=4),
                                        self.d(version=4)), [])

    def test_fault_reported_not_cleared(self):
        self.assertIn("FAULT: motion",
                      server.changes(self.d(), self.d(state=5, fault=2)))
        self.assertNotIn("FAULT: none",
                         server.changes(self.d(fault=2), self.d()))


class LetterTest(unittest.TestCase):
    """Barcode letter table (S1 decision 6)."""

    def test_valid_table(self):
        self.assertIsNone(server.check_letters(
            {"L": "G", "R": "H", "S": "C", "U": "Z"}))
        self.assertIsNone(server.check_letters(server.BOOT_LETTERS))

    def test_invalid_tables(self):
        bad = [None, [], {"L": "A", "R": "B", "S": "C"},
               {"L": "A", "R": "B", "S": "C", "U": "D", "X": "E"},
               {"L": "a", "R": "B", "S": "C", "U": "D"},
               {"L": "AB", "R": "B", "S": "C", "U": "D"},
               {"L": 1, "R": "B", "S": "C", "U": "D"},
               {"L": "A", "R": "A", "S": "C", "U": "D"}]
        for table in bad:
            self.assertIsNotNone(server.check_letters(table), table)

    def test_payload_format_matches_the_car(self):
        for good in ("L=A", "R=Z", "S=F", "U=D"):
            self.assertTrue(server.ASSIGN.fullmatch(good))
        for bad in ("L=a", "X=A", "L=AB", "L:A", "l=A", "L="):
            self.assertIsNone(server.ASSIGN.fullmatch(bad))

    def test_restart_detection(self):
        self.assertEqual(server.restarted(None, "hb 0"), (0, False))
        self.assertEqual(server.restarted(0, "hb 0"), (0, False))  # dup
        self.assertEqual(server.restarted(0, "hb 1"), (1, False))
        self.assertEqual(server.restarted(41, "hb 0"), (0, True))
        self.assertEqual(server.restarted(7, "garbage"), (7, False))


if __name__ == "__main__":
    unittest.main()
