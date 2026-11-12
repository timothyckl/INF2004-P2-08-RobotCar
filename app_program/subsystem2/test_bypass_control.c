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
#define TEST_BYPASS_STEP_MM     50u
#define TEST_BYPASS_WAIT_MS     3000u

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
static bool test_bypass_to_look(bypass_control_t * p_bypass,
                                bypass_control_input_t * p_in,
                                bypass_control_output_t * p_out,
                                uint16_t * p_id);
static void test_bypass_answer(bypass_control_t * p_bypass,
                               bypass_control_input_t * p_in,
                               bypass_control_output_t * p_out,
                               uint16_t id, uint16_t mm);

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
    test_bypass_send(&bypass, &out, &id);

    /* Braking ignores it too; still, with no step left, spin to align. */
    in.b_hazard = true;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_NONE == out.outcome);
    in.b_hazard = false;
    in.b_still = true;
    in.heading_mdeg = 90000;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_SPIN, 1);
    test_bypass_send(&bypass, &out, &id);

    /* Ignored while spinning; on the line, stop; still, the hazard ends
       it rather than resuming into it. */
    in.b_hazard = true;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && bypass_control_active(&bypass);
    in.heading_mdeg = 178000;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_HALT, 0);
    test_bypass_send(&bypass, &out, &id);
    bypass_control_step(&bypass, &in, &out);
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

    /* Seen again and confirmed 100 ms later: brake (S2 decision 20). */
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
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_HALT, 0)
               && (S2_MODE_REACQUIRE == bypass_control_mode(&bypass));
    test_bypass_send(&bypass, &out, &id);

    /* It rolls 15 mm more while braking; its own STOPPED result is not a
       failure. Once still the axle has 60 - 10 - 15 = 35 mm left to the
       point the sensors first saw. */
    in.now_ms += 50u;
    in.travel_mm += 15;
    in.last_id = id;
    in.last_result = MOTION_EVT_STOPPED;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action)
               && (BYPASS_CONTROL_NONE == out.outcome);
    in.b_still = true;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, 35);
    test_bypass_send(&bypass, &out, &id);

    /* Step done, facing 90 degrees from the start: spin left (S2
       decision 21). */
    in.heading_mdeg = 90000;
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_SPIN, 1);
    test_bypass_send(&bypass, &out, &id);

    /* Black before 90 - 2 x 15 = 60 degrees is still the crossing. */
    in.heading_mdeg = 149000;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action);

    /* A sensor on the line 76 degrees in: stop; once still, a heading 14
       degrees off the start (wrapping past 180) is in range. */
    in.heading_mdeg = -166000;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_HALT, 0);
    test_bypass_send(&bypass, &out, &id);
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_DONE == out.outcome)
               && !bypass_control_active(&bypass);

    /* With the robot's 45 degree align, no line within 45 + 125 = 170
       degrees: the line is lost. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    bypass.plan.align_mdeg = 45000;
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS;
    in.travel_mm += (int32_t)TEST_BYPASS_AHEAD_MM;
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    in.b_still = true;
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    in.b_on_line = false;
    in.heading_mdeg = -10000;                   /* 170 degrees turned */
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_NONE == out.outcome);
    in.heading_mdeg = -9999;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_SEARCH == out.fail);

    /* The line sets the heading (S2 decision 22): spinning from 0 to
       black at 20 degrees ends 160 degrees off the start, the wrong way. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS;
    in.travel_mm += (int32_t)TEST_BYPASS_AHEAD_MM + 5;   /* overshot */
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    in.b_still = true;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, -5);
    test_bypass_send(&bypass, &out, &id);
    bypass.plan.align_mdeg = 45000;
    in.heading_mdeg = 0;
    test_bypass_done(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 20000;
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_HEADING == out.fail);

    /* A line curving 60 degrees away is followed, not failed. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS;
    in.travel_mm += (int32_t)TEST_BYPASS_AHEAD_MM + 5;   /* overshot */
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    in.b_still = true;
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    bypass.plan.align_mdeg = 45000;
    in.heading_mdeg = 90000;
    test_bypass_done(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 120000;                   /* 60 degrees off */
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_DONE == out.outcome);

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

    /* Found, but the wheels never come to rest within 3 s. */
    test_bypass_setup(&bypass, &plan, &in);
    b_passed = b_passed && test_bypass_to_search(&bypass, &in, &id);
    in.b_on_line = true;
    bypass_control_step(&bypass, &in, &out);
    in.now_ms += TEST_BYPASS_FOUND_MS;
    bypass_control_step(&bypass, &in, &out);
    test_bypass_send(&bypass, &out, &id);
    in.now_ms += TEST_BYPASS_WAIT_MS;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_NONE == out.outcome);
    in.now_ms += 1u;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_MOTION == out.fail);

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
        TEST_BYPASS_AHEAD_MM, TEST_BYPASS_STEP_MM, TEST_BYPASS_WAIT_MS
    };
    bypass_control_plan_t const plan =
    {
        {
            {BYPASS_CONTROL_TURN, TEST_BYPASS_QUARTER},
            {BYPASS_CONTROL_MOVE, 282},
            {BYPASS_CONTROL_TURN, -TEST_BYPASS_QUARTER},
            {BYPASS_CONTROL_MOVE, 548}
        },
        4u, -TEST_BYPASS_QUARTER, TEST_BYPASS_QUARTER, 312u, 0, 0, 0u, 0u,
        0u, 0u
    };
    bypass_control_input_t const in =
    {
        UINT32_MAX - 1000u, 1000, 180000, 0u, MOTION_EVT_NONE, true, false,
        false, false, 0u, 0u, false, false
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

/* Give a plan a pass (look -60, passed beyond 300 mm, then 200 mm,
   stepping at most 120 mm), begin it and drive every segment; true once
   the first look is requested. */
static bool
test_bypass_to_look (bypass_control_t * p_bypass,
                     bypass_control_input_t * p_in,
                     bypass_control_output_t * p_out, uint16_t * p_id)
{
    bypass_control_plan_t plan;

    test_bypass_setup(p_bypass, &plan, p_in);
    plan.look_deg = -60;
    plan.clear_mm = 300u;
    plan.extra_mm = 200u;
    plan.pass_max_mm = 120u;
    bypass_control_begin(p_bypass, &plan, p_in, p_out);

    for (uint8_t i = 0u; (i < plan.count)
                         && (BYPASS_CONTROL_ACT_LOOK != p_out->action); i++)
    {
        test_bypass_send(p_bypass, p_out, p_id);
        test_bypass_done(p_bypass, p_in, p_out, p_id);
    }

    return (test_bypass_act(p_out, BYPASS_CONTROL_ACT_LOOK, -60)
            && (S2_MODE_BYPASS == bypass_control_mode(p_bypass)));
}

/* Answer look id with a side median and step once. */
static void
test_bypass_answer (bypass_control_t * p_bypass, bypass_control_input_t * p_in,
                    bypass_control_output_t * p_out, uint16_t id,
                    uint16_t mm)
{
    p_in->now_ms += 10u;
    p_in->look_id = id;
    p_in->look_mm = mm;
    p_in->b_look_done = true;
    bypass_control_step(p_bypass, p_in, p_out);
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

/* Look, step while the side reads the obstacle, then drive the extra
   distance once a look has passed it and continue (S2 decision 19). */
bool
test_bypass_control_pass (void)
{
    bypass_control_t        bypass;
    bypass_control_input_t  in;
    bypass_control_output_t out;
    uint16_t                id = 40u;
    bool                    b_passed;

    b_passed = test_bypass_to_look(&bypass, &in, &out, &id);
    bypass.plan.pass_max_mm = 200u;     /* room for a third step */
    test_bypass_send(&bypass, &out, &id);

    /* No answer yet, a motion DONE, or another look's answer: wait. */
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action);
    test_bypass_answer(&bypass, &in, &out, (uint16_t)(id - 1u), 900u);
    b_passed = b_passed && (BYPASS_CONTROL_ACT_NONE == out.action);

    /* 250 and 300 mm are the obstacle (not beyond 300): step 50 each. */
    test_bypass_answer(&bypass, &in, &out, id, 250u);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE,
                                           (int32_t)TEST_BYPASS_STEP_MM);
    in.b_look_done = false;
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_LOOK, -60);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_answer(&bypass, &in, &out, id, 300u);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE,
                                           (int32_t)TEST_BYPASS_STEP_MM);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);

    /* 301 mm is beyond 300 but within 100 mm of the 250 mm side reading
       (S2 decision 22): step again. */
    test_bypass_answer(&bypass, &in, &out, id, 301u);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE,
                                           (int32_t)TEST_BYPASS_STEP_MM);
    in.b_look_done = false;
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);

    /* 350 mm has passed: the extra 200 mm, then the approach turn. */
    test_bypass_answer(&bypass, &in, &out, id, 350u);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE,
                                           200);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           -TEST_BYPASS_QUARTER);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_done(&bypass, &in, &out, &id);

    return (b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_SEARCH, 0)
            && (S2_MODE_REACQUIRE == bypass_control_mode(&bypass)));
}

/* A faulted, unsent or unanswered look, or no pass within the stepping
   limit, fails; a hazard fails a step but not a look. */
bool
test_bypass_control_pass_fail (void)
{
    bypass_control_t        bypass;
    bypass_control_input_t  in;
    bypass_control_output_t out;
    bypass_control_output_t sent;
    uint16_t                id = 60u;
    bool                    b_passed;

    /* A faulted look. */
    b_passed = test_bypass_to_look(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);
    in.b_look_fault = true;
    in.look_id = id;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAILED == out.outcome)
               && (BYPASS_CONTROL_FAIL_LOOK == out.fail);

    /* A look that could not be sent. */
    b_passed = b_passed && test_bypass_to_look(&bypass, &in, &out, &id);
    bypass_control_sent(&bypass, false, 0u, &sent);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_LOOK == sent.fail);

    /* No answer within 3 s; a hazard while looking is ignored. */
    b_passed = b_passed && test_bypass_to_look(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);
    in.b_hazard = true;
    in.now_ms += TEST_BYPASS_WAIT_MS;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_NONE == out.outcome);
    in.now_ms += 1u;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_LOOK == out.fail);

    /* A hazard during a step. */
    b_passed = b_passed && test_bypass_to_look(&bypass, &in, &out, &id);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_answer(&bypass, &in, &out, id, 250u);
    test_bypass_send(&bypass, &out, &id);
    in.b_hazard = true;
    bypass_control_step(&bypass, &in, &out);
    b_passed = b_passed && (BYPASS_CONTROL_FAIL_HAZARD == out.fail);

    /* Still beside the obstacle after 100 mm: a third step would pass
       the 120 mm limit. */
    b_passed = b_passed && test_bypass_to_look(&bypass, &in, &out, &id);

    for (uint8_t i = 0u; i < 2u; i++)
    {
        test_bypass_send(&bypass, &out, &id);
        test_bypass_answer(&bypass, &in, &out, id, 250u);
        in.b_look_done = false;
        test_bypass_send(&bypass, &out, &id);
        test_bypass_done(&bypass, &in, &out, &id);
    }

    test_bypass_send(&bypass, &out, &id);
    test_bypass_answer(&bypass, &in, &out, id, 250u);

    return (b_passed && (BYPASS_CONTROL_FAILED == out.outcome)
            && (BYPASS_CONTROL_FAIL_DEPTH == out.fail)
            && !bypass_control_active(&bypass));
}

/* Turn to look (S2 decision 22): each look follows a 20 degree turn
   towards the obstacle; the turn back, and the approach turn, are to the
   start heading (180 degrees here, at the wrap), so drift is corrected;
   a miss before pass_min_mm of stepping is not a pass. */
bool
test_bypass_control_look_turn (void)
{
    bypass_control_t        bypass;
    bypass_control_input_t  in;
    bypass_control_output_t out;
    bypass_control_plan_t   plan;
    uint16_t                id = 80u;
    bool                    b_passed = true;

    test_bypass_setup(&bypass, &plan, &in);
    plan.look_deg = -60;
    plan.look_turn_mdeg = -20000;
    plan.clear_mm = 300u;
    plan.extra_mm = 200u;
    plan.pass_min_mm = 50u;
    plan.pass_max_mm = 120u;
    bypass_control_begin(&bypass, &plan, &in, &out);

    for (uint8_t i = 0u; (i < plan.count)
                         && (BYPASS_CONTROL_ACT_TURN != out.action
                             || (-20000 != out.value)); i++)
    {
        test_bypass_send(&bypass, &out, &id);
        test_bypass_done(&bypass, &in, &out, &id);
    }

    /* Turn in (overshooting to -21.5 degrees), look, turn back by 21.5:
       4000 mm before 50 mm of stepping is the obstacle still ahead, so
       step. */
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           -20000);
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 158500;    /* -21.5 from 180 */
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_LOOK, -60);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_answer(&bypass, &in, &out, id, 4000u);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           21500);
    in.b_look_done = false;
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 180000;
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE,
                                           (int32_t)TEST_BYPASS_STEP_MM);
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 178000;   /* the step curved 2 degrees */
    test_bypass_done(&bypass, &in, &out, &id);

    /* After 50 mm, 301 mm has passed: turn back by the full 22 degrees,
       then the extra 200 mm. */
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           -20000);
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 158000;
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_LOOK, -60);
    test_bypass_send(&bypass, &out, &id);
    test_bypass_answer(&bypass, &in, &out, id, 301u);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           22000);
    in.b_look_done = false;
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 180000;
    test_bypass_done(&bypass, &in, &out, &id);
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_MOVE, 200);
    test_bypass_send(&bypass, &out, &id);
    in.heading_mdeg = 179000;   /* the extra move curved too */
    test_bypass_done(&bypass, &in, &out, &id);

    /* The approach turn is to -90 degrees from the start: -89. */
    b_passed = b_passed && test_bypass_act(&out, BYPASS_CONTROL_ACT_TURN,
                                           -89000);

    /* An oversized turn is an invalid plan. */
    plan.look_turn_mdeg = (int32_t)MOTION_ANGLE_MAX_MDEG + 1;
    bypass_control_abort(&bypass);
    bypass_control_begin(&bypass, &plan, &in, &out);

    return (b_passed && (BYPASS_CONTROL_FAIL_PLAN == out.fail));
}

#endif /* APP_TEST */

/* End of file. */
