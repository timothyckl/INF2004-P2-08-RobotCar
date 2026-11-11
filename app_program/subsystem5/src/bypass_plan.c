/**
 * @file bypass_plan.c
 * @brief Bypass route from one coarse or refined profile (see bypass_plan.h).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Source: obstacle-scanning decision 5 in docs/decisions.md.
 */
#include "../include/bypass_plan.h"

#include <stddef.h>

#define BYPASS_PLAN_FORWARD     (4u)    /* index of 0 degrees */

/* sin(15, 30, 45, 60 degrees) in thousandths, for the coarse steps. */
static uint32_t const g_plan_sin_permille[4] = {259u, 500u, 707u, 866u};
static int32_t const  g_plan_quarter_mdeg    = 90000;

/** Sideways step for one side, or false if that side is unusable. */
static bool bypass_plan_side (uint16_t const * p_distances,
                              bypass_plan_config_t const * p_config,
                              bool b_left, uint32_t * p_lateral_mm);

/** Sideways step past an edge: the vehicle side facing the obstacle. */
static uint32_t bypass_plan_lateral (bypass_plan_config_t const * p_config,
                                     uint32_t edge_mm, bool b_left);

/** Fill the four segments and the search/align turns for one side. */
static void bypass_plan_fill (bypass_plan_config_t const * p_config,
                              uint16_t obstacle_mm, bool b_left,
                              uint32_t lateral_mm, bypass_plan_t * p_plan);

/** Choose left when valid, else right, after the profile is analysed. */
static void bypass_plan_choose (bypass_plan_config_t const * p_config,
                                uint16_t obstacle_mm, bool b_left,
                                uint32_t left_mm, bool b_right,
                                uint32_t right_mm, bypass_plan_t * p_plan);

bool
bypass_plan_make (uint16_t const * p_distances,
                  bypass_plan_config_t const * p_config,
                  bypass_plan_t * p_plan)
{
    bool b_inputs_valid = (NULL != p_distances) && (NULL != p_config)
                          && (NULL != p_plan);

    if (NULL != p_plan)
    {
        bypass_plan_t const cleared = {0};

        *p_plan        = cleared;
        p_plan->result = BYPASS_PLAN_INVALID;
    }

    if (b_inputs_valid)
    {
        b_inputs_valid = (0u < p_config->search_max_mm)
                         && (0u < p_config->range_max_mm);

        for (uint32_t index = 0u;
             b_inputs_valid && (index < BYPASS_PLAN_ANGLES); index++)
        {
            b_inputs_valid = (0u < p_distances[index]);
        }

    }

    if (b_inputs_valid)
    {
        uint16_t const obstacle_mm = p_distances[BYPASS_PLAN_FORWARD];
        uint32_t       left_mm     = 0u;
        uint32_t       right_mm    = 0u;

        p_plan->obstacle_mm = obstacle_mm;

        if (obstacle_mm >= p_config->range_max_mm)
        {
            p_plan->result = BYPASS_PLAN_NO_OBSTACLE;
        }
        else
        {
            bool const b_left  = bypass_plan_side(p_distances, p_config,
                                                  true, &left_mm);
            bool const b_right = bypass_plan_side(p_distances, p_config,
                                                  false, &right_mm);

            bypass_plan_choose(p_config, obstacle_mm, b_left, left_mm,
                               b_right, right_mm, p_plan);

        }

    }

    return ((NULL != p_plan) && (BYPASS_PLAN_OK == p_plan->result));
}

bool
bypass_plan_make_fine (uint16_t const * p_distances,
                       bypass_plan_config_t const * p_config,
                       bypass_plan_t * p_plan)
{
    bool const b_valid = (NULL != p_distances) && (NULL != p_config)
                         && (NULL != p_plan);

    if (NULL != p_plan)
    {
        bypass_plan_t const cleared = {0};

        *p_plan = cleared;
        p_plan->result = BYPASS_PLAN_INVALID;
    }

    if (b_valid)
    {
        scan_profile_config_t const profile_config =
        {
            p_config->depth_mm, p_config->range_max_mm
        };
        scan_profile_result_t const profile_result =
            scan_profile_make(p_distances, &profile_config,
                                  &p_plan->profile);
        uint16_t const obstacle_mm = p_plan->profile.forward_mm;
        uint32_t left_mm = 0u;
        uint32_t right_mm = 0u;

        p_plan->obstacle_mm = obstacle_mm;

        if ((SCAN_PROFILE_CLEAR == profile_result)
            && (0u < p_config->search_max_mm))
        {
            p_plan->result = BYPASS_PLAN_NO_OBSTACLE;
        }
        else if ((SCAN_PROFILE_OBSERVED == profile_result)
                 && (0u < p_config->search_max_mm))
        {
            left_mm = bypass_plan_lateral(p_config,
                                          p_plan->profile.left_extent_mm,
                                          true);
            right_mm = bypass_plan_lateral(p_config,
                                           p_plan->profile.right_extent_mm,
                                           false);

            bool const b_left = p_plan->profile.b_left_edge
                && ((left_mm + p_config->margin_mm)
                    <= p_config->search_max_mm);
            bool const b_right = p_plan->profile.b_right_edge
                && ((right_mm + p_config->margin_mm)
                    <= p_config->search_max_mm);

            bypass_plan_choose(p_config, obstacle_mm, b_left, left_mm,
                               b_right, right_mm, p_plan);
        }
    }

    return ((NULL != p_plan) && (BYPASS_PLAN_OK == p_plan->result));
}

static void
bypass_plan_choose (bypass_plan_config_t const * p_config,
                    uint16_t obstacle_mm, bool b_left, uint32_t left_mm,
                    bool b_right, uint32_t right_mm, bypass_plan_t * p_plan)
{
    /* Left whenever valid (S2 decision 16): the barcode sensor on the
       left then faces away from the obstacle. */
    if (b_left)
    {
        bypass_plan_fill(p_config, obstacle_mm, true, left_mm, p_plan);
    }
    else if (b_right)
    {
        bypass_plan_fill(p_config, obstacle_mm, false, right_mm, p_plan);
    }
    else
    {
        p_plan->result = BYPASS_PLAN_NO_ROUTE;
    }
}

static bool
bypass_plan_side (uint16_t const * p_distances,
                  bypass_plan_config_t const * p_config,
                  bool b_left, uint32_t * p_lateral_mm)
{
    uint32_t const pass_mm =
        (uint32_t) p_distances[BYPASS_PLAN_FORWARD]
        + (uint32_t) p_config->depth_mm;
    uint32_t edge_mm  = 0u;
    bool     b_passed = false;

    for (uint32_t step = 0u; (step < 4u) && (!b_passed); step++)
    {
        uint32_t const index = b_left ? (BYPASS_PLAN_FORWARD + 1u + step)
                                      : (BYPASS_PLAN_FORWARD - 1u - step);
        uint32_t const range = p_distances[index];

        if (range > pass_mm)
        {
            b_passed = true;
        }
        else
        {
            /* Still on the obstacle: its lateral extent reaches here. */
            uint32_t const lateral = (range * g_plan_sin_permille[step])
                                     / 1000u;

            edge_mm = (lateral > edge_mm) ? lateral : edge_mm;
        }

    }

    uint32_t const lateral_mm = bypass_plan_lateral(p_config, edge_mm,
                                                    b_left);

    *p_lateral_mm = lateral_mm;

    /* The search back to the line must fit inside its agreed bound. */
    return (b_passed
            && ((lateral_mm + p_config->margin_mm)
                <= p_config->search_max_mm));
}

static uint32_t
bypass_plan_lateral (bypass_plan_config_t const * p_config, uint32_t edge_mm,
                     bool b_left)
{
    /* Going left, the obstacle passes on the vehicle's right. */
    uint32_t const side_mm = b_left ? p_config->right_side_mm
                                    : p_config->left_side_mm;

    return (edge_mm + side_mm + p_config->clearance_mm
            + p_config->margin_mm);
}

static void
bypass_plan_fill (bypass_plan_config_t const * p_config,
                  uint16_t obstacle_mm, bool b_left, uint32_t lateral_mm,
                  bypass_plan_t * p_plan)
{
    int32_t const  turn       = b_left ? g_plan_quarter_mdeg
                                       : -g_plan_quarter_mdeg;
    uint32_t const forward_mm = (uint32_t) obstacle_mm
                                + p_config->depth_mm
                                + p_config->clearance_mm
                                + p_config->margin_mm;

    /* The casts are safe: lateral_mm + margin_mm is at most search_max_mm
       (checked in bypass_plan_side), and forward_mm is a sum of four
       uint16_t values, far below INT32_MAX. */
    p_plan->result               = BYPASS_PLAN_OK;
    p_plan->side                 = b_left ? BYPASS_PLAN_LEFT
                                          : BYPASS_PLAN_RIGHT;
    p_plan->count                = BYPASS_PLAN_SEGMENTS;
    p_plan->segments[0].kind     = BYPASS_PLAN_TURN;
    p_plan->segments[0].value    = turn;
    p_plan->segments[1].kind     = BYPASS_PLAN_MOVE;
    p_plan->segments[1].value    = (int32_t) lateral_mm;
    p_plan->segments[2].kind     = BYPASS_PLAN_TURN;
    p_plan->segments[2].value    = -turn;
    p_plan->segments[3].kind     = BYPASS_PLAN_MOVE;
    p_plan->segments[3].value    = (int32_t) forward_mm;
    p_plan->approach_mdeg        = -turn;
    p_plan->search_mm            = (uint16_t) (lateral_mm
                                               + p_config->margin_mm);
    p_plan->align_mdeg           = turn;
    p_plan->lateral_mm           = (uint16_t) lateral_mm;
    p_plan->estimated_min_clearance_mm = p_config->clearance_mm;
}

/* End of file. */
