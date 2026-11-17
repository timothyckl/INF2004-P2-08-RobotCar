/**
 * @file scan_telemetry.h
 * @brief Decoded obstacle status for a later S1 telemetry mapping.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Source: subsystem 5 decision 7 and PostTri1 Version B. This interface
 * contains no RTOS, driver or wire-format types. S1 chooses its own compact
 * representation after the team agrees the field contract. S5 updates its
 * protected snapshot every 10 ms; this getter briefly copies it with a
 * non-blocking lock and performs no sensor, motor or network operation.
 */
#ifndef SCAN_TELEMETRY_H
#define SCAN_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Interpret forward distance only when FORWARD_VALID is set. */
#define SCAN_TELEMETRY_SAFE          (0u)
#define SCAN_TELEMETRY_UNSAFE        (1u)
#define SCAN_TELEMETRY_INVALID       (2u)
#define SCAN_TELEMETRY_STALE         (3u)

/* Scan result; S1 maps these decoded values to its eventual wire contract. */
#define SCAN_TELEMETRY_NONE          (0u)
#define SCAN_TELEMETRY_PLAN          (1u)
#define SCAN_TELEMETRY_CLEAR         (2u)
#define SCAN_TELEMETRY_NO_ROUTE      (3u)
#define SCAN_TELEMETRY_FAULT         (4u)
#define SCAN_TELEMETRY_CANCELLED     (5u)

#define SCAN_TELEMETRY_SIDE_NONE     (0u)
#define SCAN_TELEMETRY_SIDE_LEFT     (1u)
#define SCAN_TELEMETRY_SIDE_RIGHT    (2u)

#define SCAN_TELEMETRY_FORWARD_VALID (0x01u)
#define SCAN_TELEMETRY_PROFILE_SEEN  (0x02u)
#define SCAN_TELEMETRY_SHAPE_ESTIMATE (0x04u)
#define SCAN_TELEMETRY_CLEARANCE_EST (0x08u)
#define SCAN_TELEMETRY_SCANNING      (0x10u)
#define SCAN_TELEMETRY_REQUEST_PENDING (0x20u)

/**
 * One internally consistent decoded copy. Distances are whole millimetres.
 * Width and centre describe only the span of observed rays, never a proven
 * physical obstacle boundary. Clearance is the configured route target,
 * not measured free space. Check flags before using optional fields. A result
 * is repeated in snapshots; request_id lets S1 avoid treating it as a new
 * event each 10 ms. Detailed scan samples remain local to S5.
 */
typedef struct
{
    uint32_t captured_ms;           /* snapshot time, controller clock */
    uint32_t request_id;            /* nonzero while scanning or a result */
    uint16_t forward_mm;            /* 20..4000 when FORWARD_VALID */
    uint16_t forward_age_ms;        /* latest valid attempt's age */
    uint16_t obstacle_forward_mm;   /* range at 0 degrees */
    uint16_t nearest_observed_mm;   /* nearest ray classified as obstacle */
    uint16_t observed_width_mm;     /* estimated span, not full width */
    int16_t  centre_offset_mm;      /* left positive, sensor-relative */
    uint16_t estimated_min_clearance_mm; /* model target, not measured */
    uint8_t  forward_verdict;       /* SCAN_TELEMETRY_* verdict */
    uint8_t  scan_result;           /* SCAN_TELEMETRY_* result */
    uint8_t  bypass_side;           /* NONE, LEFT or RIGHT */
    uint8_t  flags;                 /* SCAN_TELEMETRY_* flags */
} scan_telemetry_t;

/**
 * @brief Copy S5's latest decoded status without waiting for sensor input.
 *
 * The internal status lock uses TMO_POL. False means S5 is unavailable or
 * busy; the caller's output is left unchanged. No network call occurs here.
 */
bool scan_telemetry_get (scan_telemetry_t * p_out);

#endif /* SCAN_TELEMETRY_H */

/* End of file. */
