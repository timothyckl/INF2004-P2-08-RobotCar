/**
 * @file bypass_plan.h
 * @brief Bypass route from one coarse or refined profile (pure logic).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Input: the nine coarse medians, index 0 = -60 degrees (right) to index 8
 * = +60 degrees (left), forward at index 4 (decision 4: positive = left).
 * For each side the edge is the last beam that still hits the obstacle
 * before a beam passes it (range beyond the forward distance plus the
 * assumed depth). A side with no passing beam within 60 degrees is
 * unobserved and cannot be used (decision 5).
 *
 * The route steps sideways by the edge's lateral offset plus half the
 * vehicle width, the agreed 50 mm clearance and a planning margin, drives
 * past the obstacle, turns back towards the original line and searches for
 * it within search_max_mm, then aligns with the original direction. The
 * shorter sideways step wins; left wins a tie.
 *
 * Simplifications, to be reviewed against decision 5: a front scan cannot
 * see the obstacle's depth, so a configured depth is assumed; the swept
 * footprint is approximated by the rectangle the sideways step leaves.
 */
#ifndef BYPASS_PLAN_H
#define BYPASS_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#define BYPASS_PLAN_ANGLES      (9u)
#define BYPASS_PLAN_FINE_ANGLES (25u)
#define BYPASS_PLAN_SEGMENTS    (4u)

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
    uint16_t half_width_mm;     /* vehicle centre line to widest point */
    uint16_t clearance_mm;      /* physical clearance, agreed 50 mm */
    uint16_t margin_mm;         /* sensing and motion error allowance */
    uint16_t depth_mm;          /* assumed obstacle depth */
    uint16_t search_max_mm;     /* reacquisition bound, agreed 500 mm */
    uint16_t range_max_mm;      /* sensor range limit, 4000 mm */
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
    uint16_t              lateral_mm;     /* sideways step */
} bypass_plan_t;

/**
 * @brief Plan a bypass from one complete coarse profile.
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
 * both fine readings at its first observed obstacle edge.
 */
bool bypass_plan_make_fine (uint16_t const * p_distances,
                            bypass_plan_config_t const * p_config,
                            bypass_plan_t * p_plan);

#endif /* BYPASS_PLAN_H */

/* End of file. */
