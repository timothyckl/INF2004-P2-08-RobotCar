/** @file telemetry_encoder.h
 * @brief Provisional binary telemetry format for isolated codec development.
 */
#ifndef TELEMETRY_ENCODER_H
#define TELEMETRY_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_ENCODER_SIZE 8u
#define TELEMETRY_ENCODER_VERSION 1u
#define TELEMETRY_ENCODER_DISTANCE_VALID 0x01u
#define TELEMETRY_ENCODER_STOPPED        0x02u
#define TELEMETRY_ENCODER_LINK_UP        0x04u

/*
 * Provisional one-byte wire values. Subsystem owners may keep different
 * internal enums and map them to these values after team approval. Unknown
 * is always zero so a cleared field never reports a valid operating state.
 */
typedef uint8_t telemetry_encoder_controller_state_t;

#define TELEMETRY_ENCODER_CTRL_UNKNOWN   0u
#define TELEMETRY_ENCODER_CTRL_INIT      1u
#define TELEMETRY_ENCODER_CTRL_READY     2u
#define TELEMETRY_ENCODER_CTRL_RUNNING   3u
#define TELEMETRY_ENCODER_CTRL_STOPPED   4u
#define TELEMETRY_ENCODER_CTRL_COMPLETE  5u
#define TELEMETRY_ENCODER_CTRL_RECOVERY  6u
#define TELEMETRY_ENCODER_CTRL_FAULT     7u

#define TELEMETRY_ENCODER_CTRL_STOP_LATCHED       0x01u
#define TELEMETRY_ENCODER_CTRL_WHEELS_STATIONARY  0x02u
#define TELEMETRY_ENCODER_CTRL_MOVING             0x04u
#define TELEMETRY_ENCODER_CTRL_FAULT_ACTIVE       0x08u

typedef uint8_t telemetry_encoder_line_state_t;

#define TELEMETRY_ENCODER_LINE_UNKNOWN       0u
#define TELEMETRY_ENCODER_LINE_VALID         1u
#define TELEMETRY_ENCODER_LINE_LOST          2u
#define TELEMETRY_ENCODER_LINE_REACQUIRING   3u
#define TELEMETRY_ENCODER_LINE_AMBIGUOUS     4u
#define TELEMETRY_ENCODER_LINE_SENSOR_FAULT  5u

/* Navigation meanings remain subject to the barcode command specification. */
typedef uint8_t telemetry_encoder_navigation_t;

#define TELEMETRY_ENCODER_NAV_NONE      0u
#define TELEMETRY_ENCODER_NAV_LEFT      1u
#define TELEMETRY_ENCODER_NAV_RIGHT     2u
#define TELEMETRY_ENCODER_NAV_STRAIGHT  3u
#define TELEMETRY_ENCODER_NAV_U_TURN    4u
#define TELEMETRY_ENCODER_NAV_STOP      5u

typedef uint8_t telemetry_encoder_terrain_state_t;

#define TELEMETRY_ENCODER_TERRAIN_UNKNOWN        0u
#define TELEMETRY_ENCODER_TERRAIN_LEVEL          1u
#define TELEMETRY_ENCODER_TERRAIN_ASCENDING      2u
#define TELEMETRY_ENCODER_TERRAIN_PEAK           3u
#define TELEMETRY_ENCODER_TERRAIN_DESCENDING     4u
#define TELEMETRY_ENCODER_TERRAIN_HUMP_COMPLETE  5u
#define TELEMETRY_ENCODER_TERRAIN_FAULT          6u

typedef uint8_t telemetry_encoder_bypass_side_t;

#define TELEMETRY_ENCODER_BYPASS_NONE   0u
#define TELEMETRY_ENCODER_BYPASS_LEFT   1u
#define TELEMETRY_ENCODER_BYPASS_RIGHT  2u

typedef uint8_t telemetry_encoder_bypass_outcome_t;

#define TELEMETRY_ENCODER_BYPASS_NO_RESULT      0u
#define TELEMETRY_ENCODER_BYPASS_PLAN_READY     1u
#define TELEMETRY_ENCODER_BYPASS_NO_SAFE_ROUTE  2u
#define TELEMETRY_ENCODER_BYPASS_CANCELLED      3u
#define TELEMETRY_ENCODER_BYPASS_EXPIRED        4u
#define TELEMETRY_ENCODER_BYPASS_COMPLETED      5u
#define TELEMETRY_ENCODER_BYPASS_FAILED         6u

typedef uint8_t telemetry_encoder_obstacle_failure_t;

#define TELEMETRY_ENCODER_OBS_NO_FAILURE        0u
#define TELEMETRY_ENCODER_OBS_INVALID_DISTANCE  1u
#define TELEMETRY_ENCODER_OBS_ECHO_TIMEOUT      2u
#define TELEMETRY_ENCODER_OBS_ECHO_STUCK        3u
#define TELEMETRY_ENCODER_OBS_STOP_TIMEOUT      4u
#define TELEMETRY_ENCODER_OBS_SCAN_TIMEOUT      5u
#define TELEMETRY_ENCODER_OBS_NO_SAFE_ROUTE     6u
#define TELEMETRY_ENCODER_OBS_CANCELLED         7u
#define TELEMETRY_ENCODER_OBS_DELIVERY_FAILURE  8u
#define TELEMETRY_ENCODER_OBS_LINE_RETURN_FAIL  9u

typedef struct
{
    uint32_t captured_ms;
    uint16_t distance_mm;
    bool     distance_valid;
    bool     stopped;
    bool     link_up;
} telemetry_encoder_snapshot_t;

/**
 * @brief Encode version, flags, uint32 time and uint16 distance, little-endian.
 * @param p_snapshot Immutable, consistent caller-owned snapshot.
 * @param p_output Separate writable buffer; must not overlap the snapshot.
 * @param capacity Writable bytes available at p_output.
 * @return Eight on success; zero on invalid arguments or insufficient space.
 * @note Failure leaves output unchanged; invalid distance encodes as zero.
 * No allocation, locks, network calls or structure overlays are used.
 */
size_t telemetry_encoder_encode (
    const telemetry_encoder_snapshot_t * p_snapshot,
    uint8_t * p_output, size_t capacity);

#endif /* TELEMETRY_ENCODER_H */

/*** End of file. ***/

