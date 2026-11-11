/**
 * @file test_planning.c
 * @brief Forward monitoring (decision 3) and bypass planning (decision 5).
 *
 * Profiles and geometry are synthetic fixtures, not robot measurements.
 * Profile index 0 is -60 degrees (right), index 8 is +60 degrees (left).
 */
#include "../include/test_planning.h"
#include "../include/test_runner.h"
#include "../../../../app_program/subsystem5/include/bypass_plan.h"
#include "../../../../app_program/subsystem5/include/forward_monitor.h"

#include <stddef.h>

/* Geometry: 82 mm left and 63 mm right of the centre line, 132 mm from
   the axle to the front, 56 mm to the rear, sensor 81 mm ahead; 50 mm
   clearance, synthetic 20 mm margin, 200 mm depth, 150 mm reverse limit,
   500 mm sideways search bound, 4000 mm range, 15 degree beam
   allowance. Expected lengths marked
   golden were checked by an independent float sweep of the body along
   the route (S2 decision 17), not derived by hand. */
static bypass_plan_config_t const g_planning_config =
{
    82u, 63u, 132u, 56u, 81u, 50u, 20u, 200u, 150u, 500u, 4000u, 15u
};

/* Obstacle-facing side + clearance + margin, for a left bypass. */
#define TEST_PLANNING_PASS_LEFT  (63u + 50u + 20u)

/** S5-U25: unsafe, invalid and stale readings request a stop. */
static bool test_planning_monitor (test_interface_api_t const * p_api);

/** Forward invalid attempts fault only on the third consecutive sample. */
static bool test_planning_monitor_fault (test_interface_api_t const * p_api);

/** S5-U36: left wins whenever valid; right only when left is not. */
static bool test_planning_shorter (test_interface_api_t const * p_api);

/** S5-U37: blocked or unobserved sides give no usable plan. */
static bool test_planning_no_route (test_interface_api_t const * p_api);

/** S5-U53: plan segments, search bound and alignment. */
static bool test_planning_segments (test_interface_api_t const * p_api);

/** S5-U54: nothing in range ahead is not an obstacle. */
static bool test_planning_clear (test_interface_api_t const * p_api);

/** S5-U60: refined edges; left preferred, right if left is invalid. */
static bool test_planning_fine_edges (test_interface_api_t const * p_api);

/** S5-U61: incomplete or inconsistent edge readings yield no route. */
static bool test_planning_fine_reject (test_interface_api_t const * p_api);

/** S5-U62: observed rays give an explicitly estimated shape and clearance. */
static bool test_planning_profile (test_interface_api_t const * p_api);

/** S5-U63: missing one edge leaves shape fields unknown. */
static bool test_planning_partial_profile (test_interface_api_t const * p_api);

/** S5-U64: a close obstacle is reversed from before the first turn. */
static bool test_planning_reverse (test_interface_api_t const * p_api);

/** S5-U65: the turn straight can push the sideways offset out. */
static bool test_planning_turn_clear (test_interface_api_t const * p_api);

/** S5-U66: a right route mirrors a left one with the sides swapped. */
static bool test_planning_mirror (test_interface_api_t const * p_api);

/** S5-U67: edges are pulled in to the beam allowance inside the miss. */
static bool test_planning_beam (test_interface_api_t const * p_api);

void
test_planning_run (test_interface_api_t const * p_api)
{
    static test_runner_case_t const cases[] =
    {
        {"S5-U25", "unsafe, invalid or stale data stops",
         test_planning_monitor},
        {"S5-U55", "third invalid forward attempt faults",
         test_planning_monitor_fault},
        {"S5-U36", "left whenever valid, else right",
         test_planning_shorter},
        {"S5-U37", "blocked or unobserved sides give no plan",
         test_planning_no_route},
        {"S5-U53", "plan segments, search bound and align",
         test_planning_segments},
        {"S5-U54", "nothing ahead in range is no obstacle",
         test_planning_clear},
        {"S5-U60", "fine edges: left preferred, else right",
         test_planning_fine_edges},
        {"S5-U61", "missing or inconsistent edge has no route",
         test_planning_fine_reject},
        {"S5-U62", "observed span and plan clearance estimate",
         test_planning_profile},
        {"S5-U63", "missing edge keeps shape unknown",
         test_planning_partial_profile},
        {"S5-U64", "close obstacle: reverse before turning",
         test_planning_reverse},
        {"S5-U65", "turn straight widens the offset",
         test_planning_turn_clear},
        {"S5-U66", "right route mirrors left",
         test_planning_mirror},
        {"S5-U67", "beam allowance pulls edges in",
         test_planning_beam}
    };
    uint32_t const count = (uint32_t) (sizeof(cases) / sizeof(cases[0]));

    test_runner_cases(cases, count, p_api);
}

static bool
test_planning_monitor_fault (test_interface_api_t const * p_api)
{
    forward_monitor_t monitor;

    (void) p_api;
    forward_monitor_reset(&monitor);
    forward_monitor_sample(&monitor, false, 0u, 100u);
    forward_monitor_sample(&monitor, false, 0u, 200u);

    bool const b_below = (2u == monitor.invalid_count)
                         && (!forward_monitor_faulted(&monitor));

    forward_monitor_sample(&monitor, true, 300u, 300u);

    bool const b_recovered = (0u == monitor.invalid_count)
                             && (!forward_monitor_faulted(&monitor));

    forward_monitor_sample(&monitor, false, 0u, 400u);
    forward_monitor_sample(&monitor, false, 0u, 500u);
    forward_monitor_sample(&monitor, false, 0u, 600u);

    return (b_below && b_recovered && (3u == monitor.invalid_count)
            && forward_monitor_faulted(&monitor));
}

static bool
test_planning_monitor (test_interface_api_t const * p_api)
{
    forward_monitor_config_t const config = {250u, 150000u};
    forward_monitor_t              monitor;

    (void) p_api;
    forward_monitor_reset(&monitor);

    bool const b_stale_start = (FORWARD_MONITOR_STALE
                                == forward_monitor_check(&monitor, &config,
                                                         0u));

    forward_monitor_sample(&monitor, true, 600u, 1000u);

    bool const b_safe = (FORWARD_MONITOR_SAFE
                         == forward_monitor_check(&monitor, &config,
                                                  151000u));
    bool const b_stale = (FORWARD_MONITOR_STALE
                          == forward_monitor_check(&monitor, &config,
                                                   151001u));

    forward_monitor_sample(&monitor, true, 249u, 200000u);

    bool const b_unsafe = (FORWARD_MONITOR_UNSAFE
                           == forward_monitor_check(&monitor, &config,
                                                    200000u));

    forward_monitor_sample(&monitor, true, 250u, 210000u);

    bool const b_boundary = (FORWARD_MONITOR_SAFE
                             == forward_monitor_check(&monitor, &config,
                                                      210000u));

    forward_monitor_sample(&monitor, false, 3000u, 220000u);

    bool const b_invalid = (FORWARD_MONITOR_INVALID
                            == forward_monitor_check(&monitor, &config,
                                                     220000u));

    /* Freshness across the 32-bit wrap. */
    forward_monitor_sample(&monitor, true, 900u, UINT32_MAX - 1000u);

    bool const b_wrap = (FORWARD_MONITOR_SAFE
                         == forward_monitor_check(&monitor, &config, 5000u));

    return (b_stale_start && b_safe && b_stale && b_unsafe && b_boundary
            && b_invalid && b_wrap);
}

static bool
test_planning_shorter (test_interface_api_t const * p_api)
{
    /* Obstacle 400 mm ahead, wider to the left: left (420 x 0.5 = 210 mm
       edge) is longer than right (106 mm) but has a route, so it wins. */
    uint16_t const wide_left[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 420u, 3000u, 3000u
    };
    /* Left edge 450 x 0.707 = 318 mm: no left route within 500 mm. */
    uint16_t const too_wide_left[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 420u, 410u, 400u, 410u, 420u, 450u, 3000u
    };
    /* The left side never passes within 60 degrees: right is used. */
    uint16_t const left_wall[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 420u, 450u, 460u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_left  = bypass_plan_make(wide_left, &g_planning_config,
                                          &plan)
                         && (BYPASS_PLAN_LEFT == plan.side)
                         && (plan.lateral_mm
                             >= (210u + TEST_PLANNING_PASS_LEFT));
    bool const b_right = bypass_plan_make(left_wall, &g_planning_config,
                                          &plan)
                         && (BYPASS_PLAN_RIGHT == plan.side);
    bool const b_bound = bypass_plan_make(too_wide_left, &g_planning_config,
                                          &plan)
                         && (BYPASS_PLAN_RIGHT == plan.side);

    return (b_left && b_right && b_bound);
}

static bool
test_planning_no_route (test_interface_api_t const * p_api)
{
    /* Wall: no beam passes on either side within 60 degrees. */
    uint16_t const wall[BYPASS_PLAN_ANGLES] =
    {
        460u, 450u, 430u, 410u, 400u, 410u, 430u, 450u, 460u
    };
    /* Edges observed, but too wide to return within 500 mm. */
    uint16_t const wide[BYPASS_PLAN_ANGLES] =
    {
        3000u, 590u, 560u, 540u, 520u, 540u, 560u, 590u, 3000u
    };
    uint16_t       missing[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_wall = (!bypass_plan_make(wall, &g_planning_config, &plan))
                        && (BYPASS_PLAN_NO_ROUTE == plan.result)
                        && (0u == plan.count);
    bool const b_wide = (!bypass_plan_make(wide, &g_planning_config, &plan))
                        && (BYPASS_PLAN_NO_ROUTE == plan.result);

    missing[6] = 0u;    /* an invalid (unobserved) sample */

    bool const b_invalid =
        (!bypass_plan_make(missing, &g_planning_config, &plan))
        && (BYPASS_PLAN_INVALID == plan.result)
        && (!bypass_plan_make(NULL, &g_planning_config, &plan))
        && (!bypass_plan_make(wall, NULL, &plan));

    return (b_wall && b_wide && b_invalid);
}

static bool
test_planning_segments (test_interface_api_t const * p_api)
{
    /* Left edge at +15 degrees and 610 mm: 610 x 0.259 = 157 mm. Far
       enough that nothing reverses or widens: offset 157 + 133 = 290 mm,
       diagonal 290 x 1.415 = 411 mm, search (290 + 20) x 1.415 = 439 mm.
       Forward: at least the return diagonal's 189 + 157 + (81 + 610 +
       200) - 2 x 290 = 657 mm; the turn onto it makes it 707 (golden). */
    uint16_t const profile[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 610u, 600u, 610u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_made = bypass_plan_make(profile, &g_planning_config, &plan);

    return (b_made && (BYPASS_PLAN_LEFT == plan.side) && (5u == plan.count)
            && (BYPASS_PLAN_MOVE == plan.segments[0].kind)
            && (0 == plan.segments[0].value)
            && (BYPASS_PLAN_TURN == plan.segments[1].kind)
            && (45000 == plan.segments[1].value)
            && (BYPASS_PLAN_MOVE == plan.segments[2].kind)
            && (411 == plan.segments[2].value)
            && (BYPASS_PLAN_TURN == plan.segments[3].kind)
            && (-45000 == plan.segments[3].value)
            && (BYPASS_PLAN_MOVE == plan.segments[4].kind)
            && (707 == plan.segments[4].value)
            && (-45000 == plan.approach_mdeg)
            && (45000 == plan.align_mdeg)
            && (290u == plan.lateral_mm)
            && (439u == plan.search_mm));
}

static bool
test_planning_clear (test_interface_api_t const * p_api)
{
    uint16_t const open[BYPASS_PLAN_ANGLES] =
    {
        4000u, 4000u, 4000u, 4000u, 4000u, 4000u, 4000u, 4000u, 4000u
    };
    bypass_plan_t plan;

    (void) p_api;

    return ((!bypass_plan_make(open, &g_planning_config, &plan))
            && (BYPASS_PLAN_NO_OBSTACLE == plan.result));
}

static bool
test_planning_fine_edges (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    profile[7] = 3000u;  /* -25 degrees: right beam passes the edge. */
    profile[8] = 410u;   /* -20 degrees: right beam hits the edge. */
    profile[16] = 410u;  /* +20 degrees: left beam hits the edge. */
    profile[17] = 410u;  /* +25 degrees: left edge reaches farther. */

    /* Left edge 106 mm, right 71 mm (S5-U67): left is longer but has a
       route, so it wins, sized by the right side. */
    bool const b_left = bypass_plan_make_fine(profile, &g_planning_config,
                                               &plan)
                        && (BYPASS_PLAN_LEFT == plan.side)
                        && (plan.lateral_mm
                            >= (106u + TEST_PLANNING_PASS_LEFT));

    profile[17] = 0u;     /* Left edge reading missing: right is used. */

    bool const b_right = bypass_plan_make_fine(profile, &g_planning_config,
                                                &plan)
                         && (BYPASS_PLAN_RIGHT == plan.side)
                         && (plan.lateral_mm >= (71u + 82u + 50u + 20u));

    return (b_left && b_right);
}

static bool
test_planning_fine_reject (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    bool const b_missing = (!bypass_plan_make_fine(profile,
                                                    &g_planning_config,
                                                    &plan))
                           && (BYPASS_PLAN_NO_ROUTE == plan.result);

    profile[7] = 410u;   /* A far blocked ray after a nearer open ray. */
    profile[8] = 3000u;

    bool const b_inconsistent = (!bypass_plan_make_fine(profile,
                                                         &g_planning_config,
                                                         &plan))
                                && (BYPASS_PLAN_NO_ROUTE == plan.result);

    profile[7] = 3000u;
    profile[8] = 410u;
    profile[3] = 410u;   /* Another obstacle beyond the first right gap. */

    bool const b_second_obstacle = (!bypass_plan_make_fine(
                                      profile, &g_planning_config, &plan))
                                   && (BYPASS_PLAN_NO_ROUTE == plan.result);

    profile[12] = 0u;    /* Missing coarse forward measurement is invalid. */

    bool const b_invalid = (!bypass_plan_make_fine(profile,
                                                    &g_planning_config,
                                                    &plan))
                           && (BYPASS_PLAN_INVALID == plan.result);

    return (b_missing && b_inconsistent && b_second_obstacle && b_invalid);
}

static bool
test_planning_profile (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    profile[7] = 3000u;
    profile[8] = 410u;
    profile[16] = 410u;
    profile[17] = 410u;

    bool const b_plan = bypass_plan_make_fine(profile, &g_planning_config,
                                               &plan);

    return (b_plan && (BYPASS_PLAN_LEFT == plan.side)
            && (50u == plan.estimated_min_clearance_mm)
            && (400u == plan.profile.forward_mm)
            && (400u == plan.profile.nearest_observed_mm)
            && plan.profile.b_left_edge && plan.profile.b_right_edge
            && plan.profile.b_shape_estimated
            && (106u == plan.profile.left_extent_mm)
            && (71u == plan.profile.right_extent_mm)
            && (177u == plan.profile.observed_width_mm)
            && (17 == plan.profile.centre_offset_mm));
}

static bool
test_planning_partial_profile (test_interface_api_t const * p_api)
{
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    profile[7] = 3000u;
    profile[8] = 410u;

    bool const b_plan = bypass_plan_make_fine(profile, &g_planning_config,
                                               &plan);
    bool const b_unknown_shape = (BYPASS_PLAN_RIGHT == plan.side)
                                 && plan.profile.b_right_edge
                                 && (!plan.profile.b_left_edge)
                                 && (!plan.profile.b_shape_estimated)
                                 && (0u == plan.profile.observed_width_mm)
                                 && (0 == plan.profile.centre_offset_mm);

    bypass_plan_config_t invalid_config = g_planning_config;

    invalid_config.range_max_mm = 4001u;

    bool const b_range_guard = (!bypass_plan_make_fine(profile,
                                                       &invalid_config, &plan))
                               && (BYPASS_PLAN_INVALID == plan.result);

    invalid_config = g_planning_config;
    invalid_config.front_mm = BYPASS_PLAN_GEOMETRY_MAX_MM + 1u;

    bool const b_geometry_guard = (!bypass_plan_make_fine(profile,
                                                          &invalid_config,
                                                          &plan))
                                  && (BYPASS_PLAN_INVALID == plan.result);

    return (b_plan && b_unknown_shape && b_range_guard && b_geometry_guard);
}

static bool
test_planning_reverse (test_interface_api_t const * p_api)
{
    /* Stopped 245 mm away, left edge 300 x 0.707 = 212 mm. The outbound
       diagonal needs (81 + 245 + back - 212) x 0.707 >= 133 mm, so back
       >= 75 mm: 80 in 10 mm steps. A longer reverse than allowed is
       refused, and the right side is used. */
    uint16_t profile[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 3000u, 245u, 250u, 260u, 300u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    bool const b_reverse = bypass_plan_make(profile, &g_planning_config,
                                            &plan)
                           && (BYPASS_PLAN_LEFT == plan.side)
                           && (-80 == plan.segments[0].value)
                           && (45000 == plan.segments[1].value);

    bypass_plan_config_t short_config = g_planning_config;

    short_config.back_max_mm = 70u;   /* left cannot reverse far enough */

    bool const b_limit = bypass_plan_make(profile, &short_config, &plan)
                         && (BYPASS_PLAN_RIGHT == plan.side);

    /* 30 mm away with a 0 mm edge: the diagonal needs only 80 mm, but the
       front would sweep the face in the first turn until 130 (golden). */
    uint16_t const close[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 3000u, 30u, 3000u, 3000u, 3000u, 3000u
    };
    bool const b_first_turn = bypass_plan_make(close, &g_planning_config,
                                               &plan)
                              && (BYPASS_PLAN_LEFT == plan.side)
                              && (-130 == plan.segments[0].value);

    return (b_reverse && b_limit && b_first_turn);
}

static bool
test_planning_turn_clear (test_interface_api_t const * p_api)
{
    /* Stopped 245 mm away, left edge 260 x 0.5 = 130 mm: the diagonal
       alone needs 130 + 133 = 263 mm across, but the turn straight there
       would sweep the near corner, so the offset grows to 303 (golden). */
    uint16_t const profile[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 3000u, 245u, 250u, 260u, 3000u, 3000u
    };
    bypass_plan_t plan;

    (void) p_api;

    return (bypass_plan_make(profile, &g_planning_config, &plan)
            && (BYPASS_PLAN_LEFT == plan.side)
            && (0 == plan.segments[0].value)
            && (303u == plan.lateral_mm)
            && (429 == plan.segments[2].value)
            && ((plan.lateral_mm + g_planning_config.margin_mm)
                <= g_planning_config.search_max_mm));
}

static bool
test_planning_mirror (test_interface_api_t const * p_api)
{
    /* The S5-U36 left wall goes right, past the wider left side; its
       mirror image, with the sides swapped, goes left by the same route
       with every turn reversed. */
    uint16_t const right_only[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 420u, 450u, 460u
    };
    uint16_t mirrored[BYPASS_PLAN_ANGLES] = {0u};
    bypass_plan_config_t swapped = g_planning_config;
    bypass_plan_t right;
    bypass_plan_t left;
    bool b_same = true;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        mirrored[index] = right_only[(BYPASS_PLAN_ANGLES - 1u) - index];
    }

    swapped.left_side_mm  = g_planning_config.right_side_mm;
    swapped.right_side_mm = g_planning_config.left_side_mm;

    b_same = bypass_plan_make(right_only, &g_planning_config, &right)
             && bypass_plan_make(mirrored, &swapped, &left)
             && (BYPASS_PLAN_RIGHT == right.side)
             && (BYPASS_PLAN_LEFT == left.side)
             && (right.lateral_mm == left.lateral_mm)
             && (right.search_mm == left.search_mm)
             && (right.approach_mdeg == -left.approach_mdeg)
             && (right.align_mdeg == -left.align_mdeg);

    for (uint32_t index = 0u; b_same && (index < BYPASS_PLAN_SEGMENTS);
         index++)
    {
        int32_t const r = right.segments[index].value;
        int32_t const l = left.segments[index].value;

        b_same = (right.segments[index].kind == left.segments[index].kind)
                 && ((BYPASS_PLAN_TURN == right.segments[index].kind)
                     ? (r == -l) : (r == l));
    }

    return (b_same);
}

static bool
test_planning_beam (test_interface_api_t const * p_api)
{
    /* Left: hits at +15, +20, +25 (410 mm), first miss +30. Right: hits
       at -15, -20, first miss -25. Without an allowance each edge is its
       outermost hit: 410 x sin 25 = 173, 410 x sin 20 = 140 mm. With 15
       degrees, nothing is placed beyond +15 or -10 degrees: 410 x sin 15
       = 106, 410 x sin 10 = 71 mm. An allowance past the miss puts the
       edge on the axis; a bad allowance is invalid. */
    uint16_t profile[BYPASS_PLAN_FINE_ANGLES] = {0u};
    uint16_t const coarse[BYPASS_PLAN_ANGLES] =
    {
        3000u, 3000u, 3000u, 410u, 400u, 410u, 3000u, 3000u, 3000u
    };
    bypass_plan_config_t config = g_planning_config;
    bypass_plan_t plan;

    (void) p_api;

    for (uint32_t index = 0u; index < BYPASS_PLAN_ANGLES; index++)
    {
        profile[index * 3u] = coarse[index];
    }

    profile[7] = 3000u;
    profile[8] = 410u;
    profile[16] = 410u;
    profile[17] = 410u;

    config.beam_deg = 0u;

    bool const b_none = bypass_plan_make_fine(profile, &config, &plan)
                        && (173u == plan.profile.left_extent_mm)
                        && (140u == plan.profile.right_extent_mm);

    config.beam_deg = 15u;

    bool const b_beam = bypass_plan_make_fine(profile, &config, &plan)
                        && (106u == plan.profile.left_extent_mm)
                        && (71u == plan.profile.right_extent_mm);

    config.beam_deg = 30u;

    bool const b_wide = bypass_plan_make_fine(profile, &config, &plan)
                        && (0u == plan.profile.left_extent_mm)
                        && (0u == plan.profile.right_extent_mm);

    config.beam_deg = 12u;

    bool const b_invalid = (!bypass_plan_make_fine(profile, &config, &plan))
                           && (BYPASS_PLAN_INVALID == plan.result);

    return (b_none && b_beam && b_wide && b_invalid);
}

/* End of file. */
