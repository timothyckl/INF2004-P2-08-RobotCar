/**
 * @file bypass_plan.c
 * @brief Bypass route from one coarse or refined profile (see bypass_plan.h).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Source: obstacle-scanning decision 5 in docs/decisions.md, revised by
 * S2 decisions 16 and 17.
 *
 * Route geometry works in a mirrored frame: origin at the axle where the
 * car stopped, y forward, x towards the bypass side. The obstacle is the
 * rectangle x <= edge, face <= y <= end (end = face + assumed depth); the
 * car body is the rectangle -near..far sideways and -rear..front along its
 * heading, where near is the vehicle side facing the obstacle.
 */
#include "../include/bypass_plan.h"

#include <stddef.h>

#define BYPASS_PLAN_FORWARD     (4u)    /* index of 0 degrees */

/* Turn checks sample the body every 5 degrees and the obstacle outline
   every 20 mm, so they add half of each step to the clearance: 10 mm for
   the outline, and reach x sin(2.5 deg) = reach x 44/1000 for the angle. */
#define BYPASS_PLAN_HEADINGS    (10)    /* 0 to 45 degrees */
#define BYPASS_PLAN_OUTLINE_MM  (20)
#define BYPASS_PLAN_FACE_MM     (100)   /* face sampled inwards this far */
#define BYPASS_PLAN_SEARCH_STEP (10)    /* route lengths tried in 10 mm */
#define BYPASS_PLAN_FORWARD_MAX (1000)  /* longest extra forward tried */

/* sin(15, 30, 45, 60 degrees) in thousandths, for the coarse steps. */
static uint32_t const g_plan_sin_permille[4] = {259u, 500u, 707u, 866u};

/* sin and cos of 0 to 45 degrees in 5-degree steps, in thousandths. */
static int32_t const g_plan_sin_45[BYPASS_PLAN_HEADINGS] =
{
    0, 87, 174, 259, 342, 423, 500, 574, 643, 707
};
static int32_t const g_plan_cos_45[BYPASS_PLAN_HEADINGS] =
{
    1000, 996, 985, 966, 940, 906, 866, 819, 766, 707
};

static int32_t const g_plan_eighth_mdeg = 45000;

/* sqrt(2) and sqrt(1/2) in thousandths. sqrt(2) is rounded up so the
   diagonal reaches the sideways offset and a bound is never short. */
static int32_t const g_plan_root2_up_permille = 1415;
static int32_t const g_plan_root_half_permille = 707;

/** Obstacle and body in the mirrored frame (see the file comment). */
typedef struct
{
    int32_t near_mm;
    int32_t far_mm;
    int32_t front_mm;
    int32_t rear_mm;
    int32_t check_mm;       /* clearance, margin, sampling allowance */
    int32_t edge_mm;
    int32_t face_mm;
    int32_t end_mm;
} bypass_plan_geom_t;

/** One side's route: reverse, sideways offset, extra forward travel. */
typedef struct
{
    int32_t back_mm;
    int32_t side_mm;
    int32_t forward_mm;
} bypass_plan_route_t;

/** Edge of one side from the coarse medians, or false if unobserved. */
static bool bypass_plan_edge (uint16_t const * p_distances,
                              bypass_plan_config_t const * p_config,
                              bool b_left, uint32_t * p_edge_mm);

/** Shortest route past an edge on one side, or false if none fits. */
static bool bypass_plan_route (bypass_plan_config_t const * p_config,
                               uint32_t edge_mm, uint32_t face_mm,
                               bool b_left, bypass_plan_route_t * p_route);

/** True if a 45-degree in-place turn at the axle clears the obstacle. */
static bool bypass_plan_turn_clear (bypass_plan_geom_t const * p_geom,
                                    int32_t axle_x, int32_t axle_y,
                                    int32_t sign);

/** True if one obstacle point is clear of the body at one heading. */
static bool bypass_plan_point_clear (bypass_plan_geom_t const * p_geom,
                                     int32_t dx, int32_t dy,
                                     int32_t sin_pm, int32_t cos_pm);

/** True if the geometry fits the overflow analysis. */
static bool bypass_plan_config_ok (bypass_plan_config_t const * p_config);

/** Fill the segments and the search/align turns for one side. */
static void bypass_plan_fill (bypass_plan_config_t const * p_config,
                              bool b_left, bypass_plan_route_t const * p_route,
                              bypass_plan_t * p_plan);

/** Left when it has a route, else right (S2 decision 16). */
static void bypass_plan_choose (bypass_plan_config_t const * p_config,
                                uint32_t face_mm, bool b_left,
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
        b_inputs_valid = bypass_plan_config_ok(p_config);

        for (uint32_t index = 0u;
             b_inputs_valid && (index < BYPASS_PLAN_ANGLES); index++)
        {
            b_inputs_valid = (0u < p_distances[index])
                             && (p_distances[index]
                                 <= p_config->range_max_mm);
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
            bool const b_left  = bypass_plan_edge(p_distances, p_config,
                                                  true, &left_mm);
            bool const b_right = bypass_plan_edge(p_distances, p_config,
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

    if (b_valid && bypass_plan_config_ok(p_config))
    {
        scan_profile_config_t const profile_config =
        {
            p_config->depth_mm, p_config->range_max_mm, p_config->beam_deg
        };
        scan_profile_result_t const profile_result =
            scan_profile_make(p_distances, &profile_config,
                                  &p_plan->profile);

        p_plan->obstacle_mm = p_plan->profile.forward_mm;

        if (SCAN_PROFILE_CLEAR == profile_result)
        {
            p_plan->result = BYPASS_PLAN_NO_OBSTACLE;
        }
        else if (SCAN_PROFILE_OBSERVED == profile_result)
        {
            /* The nearest observed point stands in for the near face. */
            bypass_plan_choose(p_config,
                               p_plan->profile.nearest_observed_mm,
                               p_plan->profile.b_left_edge,
                               p_plan->profile.left_extent_mm,
                               p_plan->profile.b_right_edge,
                               p_plan->profile.right_extent_mm, p_plan);
        }
        else
        {
            /* INVALID: the result stays BYPASS_PLAN_INVALID. */
        }
    }

    return ((NULL != p_plan) && (BYPASS_PLAN_OK == p_plan->result));
}

static bool
bypass_plan_config_ok (bypass_plan_config_t const * p_config)
{
    uint16_t const geometry[] =
    {
        p_config->left_side_mm, p_config->right_side_mm,
        p_config->front_mm, p_config->rear_mm, p_config->sensor_mm,
        p_config->clearance_mm, p_config->margin_mm, p_config->depth_mm,
        p_config->back_max_mm, p_config->search_max_mm
    };
    bool b_ok = (0u < p_config->search_max_mm)
                && (0u < p_config->range_max_mm)
                && (p_config->range_max_mm <= SCAN_PROFILE_MAX_RANGE_MM);

    /* Every coordinate then stays below 16000 mm, so squared distances
       and the thousandths products stay far below INT32_MAX. */
    for (uint32_t index = 0u;
         b_ok && (index < (sizeof(geometry) / sizeof(geometry[0])));
         index++)
    {
        b_ok = (geometry[index] <= BYPASS_PLAN_GEOMETRY_MAX_MM);
    }

    return (b_ok);
}

static void
bypass_plan_choose (bypass_plan_config_t const * p_config, uint32_t face_mm,
                    bool b_left, uint32_t left_mm, bool b_right,
                    uint32_t right_mm, bypass_plan_t * p_plan)
{
    bypass_plan_route_t route = {0, 0, 0};

    /* Left whenever it has a route (S2 decision 16): the barcode sensor
       on the left then faces away from the obstacle. */
    if (b_left
        && bypass_plan_route(p_config, left_mm, face_mm, true, &route))
    {
        bypass_plan_fill(p_config, true, &route, p_plan);
    }
    else if (b_right
             && bypass_plan_route(p_config, right_mm, face_mm, false,
                                  &route))
    {
        bypass_plan_fill(p_config, false, &route, p_plan);
    }
    else
    {
        p_plan->result = BYPASS_PLAN_NO_ROUTE;
    }
}

static bool
bypass_plan_edge (uint16_t const * p_distances,
                  bypass_plan_config_t const * p_config,
                  bool b_left, uint32_t * p_edge_mm)
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

    *p_edge_mm = edge_mm;

    return (b_passed);
}

/*
 * The route (S2 decision 17): reverse back_mm, turn 45 degrees to the
 * bypass side, drive diagonally until the axle is side_mm across, turn
 * straight, drive forward_mm, then (in the controller) turn 45 degrees
 * back and search diagonally for the line. Each length is the shortest,
 * in 10 mm steps, that keeps every part of the body clearance + margin
 * from the obstacle: the diagonals and the straight by perpendicular
 * distance, each in-place turn by the swept-body check.
 */
static bool
bypass_plan_route (bypass_plan_config_t const * p_config, uint32_t edge_mm,
                   uint32_t face_mm, bool b_left,
                   bypass_plan_route_t * p_route)
{
    int32_t const side_mm = b_left ? (int32_t)p_config->right_side_mm
                                   : (int32_t)p_config->left_side_mm;
    int32_t const reach_mm = (int32_t)p_config->front_mm
        + (int32_t)((p_config->left_side_mm > p_config->right_side_mm)
                    ? p_config->left_side_mm : p_config->right_side_mm);
    int32_t const clear_mm = (int32_t)p_config->clearance_mm
                             + (int32_t)p_config->margin_mm;
    int32_t const face0_mm = (int32_t)p_config->sensor_mm + (int32_t)face_mm;
    int32_t const pass_mm = side_mm + clear_mm;  /* body side to obstacle */
    int32_t const diagonal_mm =
        ((pass_mm * g_plan_root2_up_permille) + 999) / 1000;
    int32_t const side_max_mm = (int32_t)p_config->search_max_mm
                                - (int32_t)p_config->margin_mm;
    bypass_plan_geom_t geom;
    bool b_found = false;

    geom.near_mm  = side_mm;
    geom.far_mm   = b_left ? (int32_t)p_config->left_side_mm
                           : (int32_t)p_config->right_side_mm;
    geom.front_mm = (int32_t)p_config->front_mm;
    geom.rear_mm  = (int32_t)p_config->rear_mm;
    geom.check_mm = clear_mm + (BYPASS_PLAN_OUTLINE_MM / 2)
                    + (((reach_mm * 44) + 999) / 1000);
    geom.edge_mm  = (int32_t)edge_mm;

    /* Reverse until the first turn and the outbound diagonal clear the
       near corner; the car came along this path, so it is free. */
    for (int32_t back = 0; (!b_found)
         && (back <= (int32_t)p_config->back_max_mm);
         back += BYPASS_PLAN_SEARCH_STEP)
    {
        geom.face_mm = face0_mm + back;
        geom.end_mm  = geom.face_mm + (int32_t)p_config->depth_mm;
        b_found = (((geom.face_mm - geom.edge_mm)
                    * g_plan_root_half_permille) >= (pass_mm * 1000))
                  && bypass_plan_turn_clear(&geom, 0, 0, 1);
        p_route->back_mm = back;
    }

    /* Sideways: past the edge, and far enough that the turn straight
       clears the obstacle; within the agreed sideways bound (option A). */
    if (b_found)
    {
        b_found = false;

        for (int32_t side = geom.edge_mm + pass_mm;
             (!b_found) && (side <= side_max_mm);
             side += BYPASS_PLAN_SEARCH_STEP)
        {
            b_found = bypass_plan_turn_clear(&geom, side, side, 1);
            p_route->side_mm = side;
        }
    }

    /* Forward: until the return diagonal passes the far corner and the
       turn onto it clears the obstacle. */
    if (b_found)
    {
        int32_t const side = p_route->side_mm;
        int32_t const least = diagonal_mm + geom.edge_mm + geom.end_mm
                              - (2 * side);
        int32_t const first = (least > 0) ? least : 0;

        b_found = false;

        for (int32_t forward = first; (!b_found)
             && (forward <= (first + BYPASS_PLAN_FORWARD_MAX));
             forward += BYPASS_PLAN_SEARCH_STEP)
        {
            b_found = bypass_plan_turn_clear(&geom, side, side + forward,
                                             -1);
            p_route->forward_mm = forward;
        }
    }

    return (b_found);
}

/* sign +1: headings 0 to +45 degrees (towards the bypass side), the
   first turn and the turn straight; -1: 0 to -45, the turn back. */
static bool
bypass_plan_turn_clear (bypass_plan_geom_t const * p_geom, int32_t axle_x,
                        int32_t axle_y, int32_t sign)
{
    bool b_clear = true;

    for (uint32_t step = 0u;
         b_clear && (step < (uint32_t)BYPASS_PLAN_HEADINGS); step++)
    {
        int32_t const s = sign * g_plan_sin_45[step];
        int32_t const c = g_plan_cos_45[step];

        /* The side facing the bypass, from the near face to the far. */
        for (int32_t y = p_geom->face_mm; b_clear && (y <= p_geom->end_mm);
             y += BYPASS_PLAN_OUTLINE_MM)
        {
            b_clear = bypass_plan_point_clear(p_geom,
                                              p_geom->edge_mm - axle_x,
                                              y - axle_y, s, c);
        }

        b_clear = b_clear
                  && bypass_plan_point_clear(p_geom,
                                             p_geom->edge_mm - axle_x,
                                             p_geom->end_mm - axle_y, s, c);

        /* The near and far faces, inwards from that side. */
        for (int32_t x = p_geom->edge_mm - BYPASS_PLAN_OUTLINE_MM;
             b_clear && (x >= (p_geom->edge_mm - BYPASS_PLAN_FACE_MM));
             x -= BYPASS_PLAN_OUTLINE_MM)
        {
            b_clear = bypass_plan_point_clear(p_geom, x - axle_x,
                                              p_geom->face_mm - axle_y, s, c)
                      && bypass_plan_point_clear(p_geom, x - axle_x,
                                                 p_geom->end_mm - axle_y,
                                                 s, c);
        }
    }

    return (b_clear);
}

static bool
bypass_plan_point_clear (bypass_plan_geom_t const * p_geom, int32_t dx,
                         int32_t dy, int32_t sin_pm, int32_t cos_pm)
{
    /* The point in body coordinates: u across (+ = bypass side), v along
       the heading, rotated by the heading whose sine is sin_pm. */
    int32_t const u = ((dx * cos_pm) - (dy * sin_pm)) / 1000;
    int32_t const v = ((dx * sin_pm) + (dy * cos_pm)) / 1000;
    int32_t du = 0;
    int32_t dv = 0;

    if (u < -p_geom->near_mm)
    {
        du = -p_geom->near_mm - u;
    }
    else if (u > p_geom->far_mm)
    {
        du = u - p_geom->far_mm;
    }
    else
    {
        /* Within the body's width. */
    }

    if (v < -p_geom->rear_mm)
    {
        dv = -p_geom->rear_mm - v;
    }
    else if (v > p_geom->front_mm)
    {
        dv = v - p_geom->front_mm;
    }
    else
    {
        /* Within the body's length. */
    }

    return (((du * du) + (dv * dv)) >= (p_geom->check_mm * p_geom->check_mm));
}

static void
bypass_plan_fill (bypass_plan_config_t const * p_config, bool b_left,
                  bypass_plan_route_t const * p_route, bypass_plan_t * p_plan)
{
    int32_t const turn = b_left ? g_plan_eighth_mdeg : -g_plan_eighth_mdeg;
    int32_t const side = p_route->side_mm;

    /* side + margin is at most search_max_mm <= 1000 (bypass_plan_route,
       bypass_plan_config_ok), so the search fits uint16_t; the moves are
       at most a few thousand millimetres. */
    p_plan->result            = BYPASS_PLAN_OK;
    p_plan->side              = b_left ? BYPASS_PLAN_LEFT : BYPASS_PLAN_RIGHT;
    p_plan->count             = BYPASS_PLAN_SEGMENTS;
    p_plan->segments[0].kind  = BYPASS_PLAN_MOVE;
    p_plan->segments[0].value = -p_route->back_mm;
    p_plan->segments[1].kind  = BYPASS_PLAN_TURN;
    p_plan->segments[1].value = turn;
    p_plan->segments[2].kind  = BYPASS_PLAN_MOVE;
    p_plan->segments[2].value = ((side * g_plan_root2_up_permille) + 999)
                                / 1000;
    p_plan->segments[3].kind  = BYPASS_PLAN_TURN;
    p_plan->segments[3].value = -turn;
    p_plan->segments[4].kind  = BYPASS_PLAN_MOVE;
    p_plan->segments[4].value = p_route->forward_mm;
    p_plan->approach_mdeg     = -turn;
    p_plan->search_mm         = (uint16_t)
        ((((side + (int32_t)p_config->margin_mm) * g_plan_root2_up_permille)
          + 999) / 1000);
    p_plan->align_mdeg        = turn;
    p_plan->lateral_mm        = (uint16_t)side;
    p_plan->estimated_min_clearance_mm = p_config->clearance_mm;
}

/* End of file. */
