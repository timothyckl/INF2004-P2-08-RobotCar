/**
 * @file bypass_plan.h
 * @brief Bypass route from one coarse or refined profile (pure logic).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Input: the nine coarse medians, index 0 = -60 degrees (right) to index 8
 * = +60 degrees (left), forward at index 4 (decision 4: positive = left),
 * or the 5-degree profile. For each side the edge is the last beam that
 * still hits the obstacle before a beam passes it (range beyond the
 * forward distance plus the assumed depth). A side with no passing beam
 * within 60 degrees is unobserved and cannot be used (decision 5).
 *
 * The route turns 45 degrees, never 90 (S2 decision 17): reverse if
 * needed, turn 45 degrees to the bypass side, drive diagonally until the
 * axle is the sideways offset across and turn straight. The controller
 * then passes the obstacle by measurement (S2 decision 19): it steps
 * forward, looks sideways at look_deg and stops stepping once a reading
 * exceeds clear_mm, then drives extra_mm, turns 45 degrees back, searches
 * diagonally for the original line and aligns. A look beyond 60 degrees
 * misses only once the far end is within cot(60 + beam allowance) of the
 * sideways gap ahead of the sensor; extra_mm puts the turn back where it
 * clears a far end there. Stepping past pass_max_mm means the obstacle is
 * deeper than depth_max_mm. Each length is the shortest that keeps the
 * whole body (from the measured extents) clearance + margin from the
 * obstacle,
 * including the corners swept by each in-place turn. The sideways offset
 * plus the margin must stay within search_max_mm. Left is used whenever
 * it has a route; right only when it does not (S2 decision 16).
 *
 * Simplifications: a front scan cannot see the obstacle's depth, so a
 * configured depth is assumed; the obstacle is the rectangle behind its
 * nearest observed point, out to the observed edge.
 */
#ifndef BYPASS_PLAN_H
#define BYPASS_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#include "scan_profile.h"

#define BYPASS_PLAN_ANGLES      (9u)
#define BYPASS_PLAN_FINE_ANGLES SCAN_PROFILE_ANGLES
#define BYPASS_PLAN_SEGMENTS    (4u)
#define BYPASS_PLAN_LOOK_DEG    (60)    /* side look, the servo's limit */

/* Upper bound on each geometry setting, for the overflow analysis. */
#define BYPASS_PLAN_GEOMETRY_MAX_MM (1000u)

#define BYPASS_PLAN_TURN        (1u)    /* value: mdeg, positive = left */
#define BYPASS_PLAN_MOVE        (2u)    /* value: mm, positive = forward */

#define BYPASS_PLAN_LEFT        (1u)
#define BYPASS_PLAN_RIGHT       (2u)

/** Outcome of one planning attempt. */
typedef enum
{
    BYPASS_PLAN_OK = 0,
    BYPASS_PLAN_NO_OBSTACLE,    /* nothing within range straight ahead */
    BYPASS_PLAN_NO_ROUTE,       /* both sides blocked or unobserved */
    BYPASS_PLAN_INVALID         /* bad input or configuration */
} bypass_plan_result_t;

/** One motion step for the Vehicle Controller to execute. */
typedef struct
{
    uint8_t kind;
    int32_t value;
} bypass_plan_segment_t;

/** Geometry and margins; the caller supplies provisional values. */
typedef struct
{
    uint16_t left_side_mm;      /* centre line to the left-most point */
    uint16_t right_side_mm;     /* centre line to the right-most point */
    uint16_t front_mm;          /* axle to the front-most point */
    uint16_t rear_mm;           /* axle to the rear-most point */
    uint16_t sensor_mm;         /* axle to the ultrasonic sensor face */
    uint16_t clearance_mm;      /* physical clearance, agreed 50 mm */
    uint16_t margin_mm;         /* sensing and motion error allowance */
    uint16_t depth_mm;          /* separates obstacle rays from beyond */
    uint16_t back_max_mm;       /* longest reverse before the first turn */
    uint16_t search_max_mm;     /* sideways reacquisition bound, 500 mm */
    uint16_t range_max_mm;      /* sensor range limit, 4000 mm */
    uint16_t beam_deg;          /* edge beam allowance, 5-degree steps */
    uint16_t depth_max_mm;      /* deepest obstacle passed, >= depth_mm */
} bypass_plan_config_t;

/** A route the Vehicle Controller can execute segment by segment. */
typedef struct
{
    bypass_plan_result_t  result;
    uint8_t               side;
    uint8_t               count;
    bypass_plan_segment_t segments[BYPASS_PLAN_SEGMENTS];
    int32_t               approach_mdeg;  /* turn back towards the line */
    uint16_t              search_mm;      /* forward search bound */
    int32_t               align_mdeg;     /* turn once the line is found */
    uint16_t              obstacle_mm;    /* forward distance */
    uint16_t              lateral_mm;     /* sideways offset of the axle */
    int16_t               look_deg;       /* side look; positive = left */
    uint16_t              clear_mm;       /* a look beyond this has passed */
    uint16_t              extra_mm;       /* forward after the first pass */
    uint16_t              pass_max_mm;    /* most forward stepping */
    uint16_t              estimated_min_clearance_mm; /* configured target,
                                                        not physical proof */
    scan_profile_t    profile;        /* observed rays, if fine scan */
} bypass_plan_t;

/**
 * @brief Plan a bypass from one complete coarse profile.
 *
 * The beam allowance is not applied: at 15-degree steps an allowance of
 * up to 15 degrees never moves an edge.
 *
 * @param p_distances Nine medians in millimetres, -60 to +60 degrees.
 * @param p_config Geometry and margins.
 * @param p_plan Receives the plan; result says whether it is usable.
 * @return true only for BYPASS_PLAN_OK.
 */
bool bypass_plan_make (uint16_t const * p_distances,
                       bypass_plan_config_t const * p_config,
                       bypass_plan_t * p_plan);

/**
 * Plan from a 5-degree profile. Coarse positions (multiples of 15 degrees)
 * must be present; only selected fine positions need readings. A route needs
 * both fine readings at its first observed obstacle edge. The returned
 * profile contains ray-based estimates, with explicit unknown-edge flags.
 */
bool bypass_plan_make_fine (uint16_t const * p_distances,
                            bypass_plan_config_t const * p_config,
                            bypass_plan_t * p_plan);

#endif /* BYPASS_PLAN_H */

/* End of file. */
