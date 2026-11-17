"""Unit tests for the provisional 8-byte telemetry decoder."""

import sys
from pathlib import Path
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from telemetry_codec import decode_frame  # noqa: E402


class TelemetryCodecTests(unittest.TestCase):
    def test_golden_frame(self) -> None:
        frame = bytes.fromhex("010778563412cdab")
        decoded = decode_frame(frame)

        self.assertEqual(decoded["version"], 1)
        self.assertEqual(decoded["captured_ms"], 0x12345678)
        self.assertEqual(decoded["distance_mm"], 0xABCD)
        self.assertTrue(decoded["distance_valid"])
        self.assertTrue(decoded["stopped"])
        self.assertTrue(decoded["link_up"])

    def test_invalid_distance_is_none(self) -> None:
        decoded = decode_frame(bytes.fromhex("010200000000ffff"))

        self.assertIsNone(decoded["distance_mm"])
        self.assertEqual(decoded["encoded_distance_mm"], 0xFFFF)

    def test_gap_estimate_uses_robot_time(self) -> None:
        current = bytes.fromhex("0101b80b00006400")
        decoded = decode_frame(current, previous_robot_ms=0, expected_period_ms=1000)

        self.assertEqual(decoded["robot_delta_ms"], 3000)
        self.assertTrue(decoded["gap"])
        self.assertEqual(decoded["estimated_missing"], 2)

    def test_robot_time_wrap_is_safe(self) -> None:
        current = bytes.fromhex("0101e80300000000")
        decoded = decode_frame(
            current, previous_robot_ms=0xFFFFFFF0, expected_period_ms=1000
        )

        self.assertEqual(decoded["robot_delta_ms"], 1016)
        self.assertFalse(decoded["gap"])

    def test_wrong_length_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            decode_frame(b"\x01\x02")


if __name__ == "__main__":
    unittest.main()

