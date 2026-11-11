/**
 * @file test_bypass_control.c
 * @brief Test the pure bypass and line reacquisition (bypass_control.c).
 */
#include "test_bypass_control.h"

#if APP_TEST

#include <stdint.h>

#include "bypass_control.h"
#include "motion_api.h"
#include "subsystem2.h"

#define TEST_BYPASS_LIMIT_MS    5000u
#define TEST_BYPASS_FOUND_MS    100u
#define TEST_BYPASS_TOL_MDEG    15000
#define TEST_BYPASS_AHEAD_MM    60u
#define TEST_BYPASS_QUARTER     90000

static void test_bypass_setup(bypass_control_t * p_bypass,
                              bypass_control_plan_t * p_plan,
                              bypass_control_input_t * p_in);
static bool test_bypass_act(bypass_control_output_t const * p_out,
                            bypass_control_action_t action, int32_t value);
static void test_bypass_done(bypass_control_t * p_bypass,
                             bypass_control_input_t * p_in,
                             bypass_control_output_t * p_out,
                             uint16_t * p_id);
static void test_bypass_send(bypass_control_t * p_bypass,
                             bypass_control_output_t const * p_out,
                             uint16_t * p_id);
static bool test_bypass_to_search(bypass_control_t * p_bypass,
                                  bypass_control_input_t * p_in,
                                  uint16_t * p_id);

/* The plan is driven in order, skipping empty segments; a bad plan fails. */
bool
test_bypass_control_segments (void)
{
    bypass_control_t        bypass;
    bypass_control_plan_t   plan;
    bypass_control_input_t  in;
    bypass_control_output_t out;
    uint16_t                id = 10u;
    bool                    b_passed;

    test_bypass_setup(&bypass, &plan, &in);
    b_passed = !bypass_control_active(&bypass)
               && (S2_MODE_NONE == bypass_control_mode(&bypass));

    plan.seg[2].value = 0;          /* an empty segment is skipped */
    bypass_control_begin(&bypass, &plan, &in, &out);
    b_passed = b_passed && bypass_control_active(&bypass)
               && (S2_MODE_BYPASS == bypass_control_mode(&bypass))
               && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                  TEST_BYPASS_QUARTER);
    test_bypass_send(&bypass, &out, &id);

    /* A second begin while active changes nothing. */
    bypass_control_begin(&bypass, &plan, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action);

    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, 282);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, 548);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           -TEST_BYPASS_QUARTER);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_SEARCH, 0)
               && (S2_MODE_REACQUIRE == bypass_control_mode(&bypass));

    /* Out-of-range values and unknown kinds fail before any motion. */
    test_bypass_setup(&bypass, &plan, &in);
    plan.seg[1].value = (int32_t)MOTION_DISTANCE_MAX_MM + 1;
    bypass_control_begin(&bypass, &plan, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAILED == out.outcome)
               && (BYPASS_CONTROL_FAIL_PLAN == out.fail)
               && (BYPASS_CONTROL_ACT_NONE == out.action)
               && !bypass_control_active(&bypass);

    test_bypass_setup(&bypass, &plan, &in);
    plan.seg[0].kind = 0u;
    bypass_control_begin(&bypass, &plan, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_PLAN == out.fail);

    test_bypass_setup(&bypass, &plan, &in);
    plan.count = BYPASS_CONTROL_SEGMENTS + 1u;
    bypass_control_begin(&bypass, &plan, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_PLAN == out.fail);

    return (b_passed);
}

/* Only this command's DONE advances; its failure or a send failure fails. */
bool
test_bypass_control_motion (void)
{
    static uint8_t const failures[] =
    {
        MOTION_EVT_REJECTED, MOTION_EVT_FAULT, MOTION_EVT_STOPPED
    };
    bypass_control_t        bypass;
    bypass_control_plan_t   plan;
    bypass_control_input_t  in;
    bypass_control_output_t out;
    uint16_t                id = 20u;
    bool                    b_passed = true;

    test_bypass_setup(&bypass, &plan, &in);
    bypass_control_begin(&bypass, &plan, &in, &out);
    test_bypass_send(&bypass, &out, &id);

    /* Another command's DONE, or a stale status, is not this one's. */
    in.last_id = (uint16_t)(id - 1u);
    in.last_result = MOTION_EVT_DONE;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action)
               && (BYPASS_CONTROL_NONE == out.outcome);

    in.last_id = id;
    in.b_status_ok = false;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action);
    in.b_status_ok = true;

    for (uint8_t i = 0u; i < (uint8_t)sizeof(failures); i++)
    {
        test_bypass_setup(&bypass, &plan, &in);
        bypass_control_begin(&bypass, &plan, &in, &out);
        test_bypass_send(&bypass, &out, &id);
        in.last_id = id;
        in.last_result = failures[i];
        bypass_control_step(&bypass, &in, &out);
        b_passed = b_passed && (BYPASS_CONTROL_FAILED == out.outcome)
                   && (BYPASS_CONTROL_FAIL_MOTION == out.fail)
                   && !bypass_control_active(&bypass);
    }

    test_bypass_setup(&bypass, &plan, &in);
    bypass_control_begin(&bypass, &plan, &in, &out);
    bypass_control_sent(&bypass, false, 0u, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_MOTION == out.fail)
               && !bypass_control_active(&bypass);

    /* Abort ends the bypass silently; a later step does nothing. */
    test_bypass_setup(&bypass, &plan, &in);
    bypass_control_begin(&bypass, &plan, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    bypass_control_abort(&bypass);
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && !bypass_control_active(&bypass)
               && (BYPASS_CONTROL_ACT_NONE == out.action)
               && (BYPASS_CONTROL_NONE == out.outcome);

    return (b_passed);
}

/* A forward hazard is ignored in turns and fails forward travel. */
bool
test_bypass_control_hazard (void)
{
    bypass_control_t        bypass;
    bypass_control_plan_t   plan;
    bypass_control_input_t  in;
    bypass_control_output_t out;
    uint16_t                id = 30u;
    bool                    b_passed;

    /* The first turn starts within the hazard distance. */
    test_bypass_setup(&bypass, &plan, &in);
    in.b_hazard = true;
    bypass_control_begin(&bypass, &plan, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    bypass_control_step(&bypass, &in, &out);
    b_passed = (BYPASS_CONTROL_NONE == out.outcome)
               && bypass_control_active(&bypass);

    /* Still a hazard as the turn ends: the forward MOVE is requested ... */
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, 282);
    test_bypass_send(&bypass, &out, &id);

    /* ... and stopped on the next cycle. */
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAILED == out.outcome)
               && (BYPASS_CONTROL_FAIL_HAZARD == out.fail);

    /* A hazard during the search fails it too. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.b_hazard = true;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_HAZARD == out.fail);

    /* Ignored in the align turn, but it must not resume into a hazard. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS;
    in.travel_mm += (int32_t)TEST_BYPASS_AHEAD_MM;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           TEST_BYPASS_QUARTER);
    test_bypass_send(&bypass, &out, &id);
    in.b_hazard = true;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && bypass_control_active(&bypass);
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_HAZARD == out.fail);

    return (b_passed);
}

/* Stable line, step onto it, align and check the heading; bounded. */
bool
test_bypass_control_reacquire (void)
{
    bypass_control_t        bypass;
    bypass_control_plan_t   plan;
    bypass_control_input_t  in;
    bypass_control_output_t out;
    uint16_t                id = 40u;
    bool                    b_passed;

    test_bypass_setup(&bypass, &plan, &in);
    b_passed = test_bypass_to_search(&bypass, &in, &id);

    /* A flicker shorter than found_ms is not the line. */
    in.now_ms += 10u;
    in.travel_mm += 100;
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS - 10u;
    in.b_on_line = false;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action);

    /* Seen again, confirmed 100 ms and 10 mm later: the axle has
       60 - 10 = 50 mm left to the point the sensors first saw. */
    in.now_ms += 10u;
    in.travel_mm += 10;
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS - 1u;
    in.travel_mm += 10;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action);
    in.now_ms += 1u;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, 50)
               && (S2_MODE_REACQUIRE == bypass_control_mode(&bypass));
    test_bypass_send(&bypass, &out, &id);

    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           TEST_BYPASS_QUARTER);
    test_bypass_send(&bypass, &out, &id);

    /* Heading 14 degrees off the start (wrapping past 180) is in range. */
    in.heading_mdeg = -166000;
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && (BYPASS_CONTROL_DONE == out.outcome)
               && !bypass_control_active(&bypass);

    /* 16 degrees off fails. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS;
    in.travel_mm += (int32_t)TEST_BYPASS_AHEAD_MM + 5;   /* overshot */
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, -5);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 180000 - 16000;
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_HEADING == out.fail);

    /* Past the plan's search distance. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.travel_mm += (int32_t)plan.search_mm;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_NONE == out.outcome);
    in.travel_mm += 1;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_SEARCH == out.fail);

    /* Past the search time. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.now_ms += TEST_BYPASS_LIMIT_MS + 1u;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_SEARCH == out.fail);

    return (b_passed);
}

/* The plan from the stage 2 run (S5 test log, 2026-11-10), facing
   heading 180 degrees so the check must wrap. */
static void
test_bypass_setup (bypass_control_t * p_bypass, bypass_control_plan_t * p_plan,
                   bypass_control_input_t * p_in)
{
    bypass_control_cfg_t const cfg =
    {
        TEST_BYPASS_LIMIT_MS, TEST_BYPASS_FOUND_MS, TEST_BYPASS_TOL_MDEG,
        TEST_BYPASS_AHEAD_MM
    };
    bypass_control_plan_t const plan =
    {
        {
            {BYPASS_CONTROL_TURN, TEST_BYPASS_QUARTER},
            {BYPASS_CONTROL_MOVE, 282},
            {BYPASS_CONTROL_TURN, -TEST_BYPASS_QUARTER},
            {BYPASS_CONTROL_MOVE, 548}
        },
        4u, -TEST_BYPASS_QUARTER, TEST_BYPASS_QUARTER, 312u
    };
    bypass_control_input_t const in =
    {
        UINT32_MAX - 1000u, 1000, 180000, 0u, MOTION_EVT_NONE, true, false,
        false
    };

    (void)bypass_control_init(p_bypass, &cfg);
    *p_plan = plan;
    *p_in = in;
}

static bool
test_bypass_act (bypass_control_output_t const * p_out,
                 bypass_control_action_t action, int32_t value)
{
    return ((action == p_out->action) && (value == p_out->value)
            && (BYPASS_CONTROL_NONE == p_out->outcome));
}

/* Report the outstanding command DONE and step once. */
static void
test_bypass_done (bypass_control_t * p_bypass, bypass_control_input_t * p_in,
                  bypass_control_output_t * p_out, uint16_t * p_id)
{
    p_in->now_ms += 10u;
    p_in->last_id = *p_id;
    p_in->last_result = MOTION_EVT_DONE;
    bypass_control_step(p_bypass, p_in, p_out);
}

static void
test_bypass_send (bypass_control_t * p_bypass,
                  bypass_control_output_t const * p_out, uint16_t * p_id)
{
    bypass_control_output_t sent;

    if (BYPASS_CONTROL_ACT_NONE != p_out->action)
    {
        *p_id = (uint16_t)(*p_id + 1u);
        bypass_control_sent(p_bypass, true, *p_id, &sent);
    }
}

/* Begin and drive every segment; true once searching. Travel is unchanged,
   so the search is measured from the setup's travel. */
static bool
test_bypass_to_search (bypass_control_t * p_bypass,
                       bypass_control_input_t * p_in, uint16_t * p_id)
{
    bypass_control_plan_t   plan;
    bypass_control_output_t out;
    bypass_control_input_t  start;

    test_bypass_setup(p_bypass, &plan, &start);
    *p_in = start;
    bypass_control_begin(p_bypass, &plan, p_in, &out);

    for (uint8_t i = 0u; (i < (plan.count + 1u))
                         && (BYPASS_CONTROL_ACT_SEARCH != out.action); i++)
    {
        test_bypass_send(p_bypass, &out, p_id);
        test_bypass_done(p_bypass, p_in, &out, p_id);
    }

    test_bypass_send(p_bypass, &out, p_id);

    return (BYPASS_CONTROL_ACT_SEARCH == out.action);
}

#endif /* APP_TEST */

/* End of file. */
