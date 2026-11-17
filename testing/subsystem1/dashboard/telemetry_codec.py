"""Decode the provisional Subsystem 1 telemetry wire format."""

from __future__ import annotations

from typing import Any


FRAME_SIZE = 8
FORMAT_VERSION = 1
FLAG_DISTANCE_VALID = 0x01
FLAG_STOPPED = 0x02
FLAG_LINK_UP = 0x04
KNOWN_FLAGS = FLAG_DISTANCE_VALID | FLAG_STOPPED | FLAG_LINK_UP


def decode_frame(
    payload: bytes,
    previous_robot_ms: int | None = None,
    expected_period_ms: int = 1000,
) -> dict[str, Any]:
    """Decode one frame and derive wrap-safe timing diagnostics.

    The expected period is a test setting, not a validated requirement.
    """

    if len(payload) != FRAME_SIZE:
        raise ValueError(f"expected {FRAME_SIZE} bytes, received {len(payload)}")
    if expected_period_ms <= 0:
        raise ValueError("expected_period_ms must be positive")

    version = payload[0]
    flags = payload[1]
    captured_ms = int.from_bytes(payload[2:6], byteorder="little", signed=False)
    encoded_distance_mm = int.from_bytes(
        payload[6:8], byteorder="little", signed=False
    )
    distance_valid = bool(flags & FLAG_DISTANCE_VALID)

    robot_delta_ms: int | None = None
    estimated_missing = 0
    gap = False
    if previous_robot_ms is not None:
        robot_delta_ms = (captured_ms - previous_robot_ms) & 0xFFFFFFFF
        tolerance_ms = max(100, expected_period_ms // 2)
        gap = robot_delta_ms > (expected_period_ms + tolerance_ms)
        if gap:
            estimated_missing = max(
                0, round(robot_delta_ms / expected_period_ms) - 1
            )

    return {
        "version": version,
        "version_supported": version == FORMAT_VERSION,
        "flags": flags,
        "unknown_flags": flags & ~KNOWN_FLAGS,
        "captured_ms": captured_ms,
        "distance_valid": distance_valid,
        "distance_mm": encoded_distance_mm if distance_valid else None,
        "encoded_distance_mm": encoded_distance_mm,
        "stopped": bool(flags & FLAG_STOPPED),
        "link_up": bool(flags & FLAG_LINK_UP),
        "robot_delta_ms": robot_delta_ms,
        "gap": gap,
        "estimated_missing": estimated_missing,
        "raw_hex": payload.hex(),
    }

