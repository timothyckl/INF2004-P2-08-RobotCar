/**
 * @file scan_profile.h
 * @brief Observed obstacle geometry from one refined ultrasonic scan.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Source: subsystem 5 decisions 4 and 5 in docs/decisions.md (FR6).
 * These are ray-based estimates, not measured physical dimensions or proof
 * that a vehicle can pass. Missing edges remain explicitly unknown.
 */
#ifndef SCAN_PROFILE_H
#define SCAN_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#define SCAN_PROFILE_ANGLES (25u)  /* -60 to +60 degrees, 5-degree bins */
#define SCAN_PROFILE_MAX_RANGE_MM (4000u)  /* HC-SR04 nominal limit */

/** Result of validating and classifying a refined scan. */
typedef enum
{
    SCAN_PROFILE_INVALID = 0,
    SCAN_PROFILE_CLEAR,
    SCAN_PROFILE_OBSERVED
} scan_profile_result_t;

/** Provisional depth separates obstacle echoes from a beam beyond it. */
typedef struct
{
    uint16_t depth_mm;
    uint16_t range_max_mm;
} scan_profile_config_t;

/**
 * Lateral offsets are relative to the sensor's forward axis: left positive.
 * The observed span is a ray-based estimate, not the obstacle's full
 * physical width. Values are meaningful only with their corresponding flags.
 */
typedef struct
{
    uint16_t forward_mm;
    uint16_t nearest_observed_mm;
    uint16_t left_extent_mm;
    uint16_t right_extent_mm;
    uint16_t observed_width_mm;
    int16_t  centre_offset_mm;
    bool     b_left_edge;
    bool     b_right_edge;
    bool     b_shape_estimated;
} scan_profile_t;

/**
 * @brief Validate the coarse bins and estimate both refined obstacle edges.
 *
 * Coarse bins (multiples of 15 degrees) must be present. Fine bins are only
 * required around the first blocked-to-open transition on each side. Any
 * second observed obstacle beyond an open gap invalidates that side.
 * @return INVALID, CLEAR or OBSERVED; output is zeroed on invalid input.
 */
scan_profile_result_t scan_profile_make (
    uint16_t const * p_distances,
    scan_profile_config_t const * p_config,
    scan_profile_t * p_profile);

#endif /* SCAN_PROFILE_H */

/* End of file. */
