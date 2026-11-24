/**
 * @file test_nav_control.c
 * @brief Test the barcode-command junction policy (nav_control.c).
 */
#include "test_nav_control.h"

#if APP_TEST

#include <stdint.h>

#include "motion_api.h"
#include "nav_control.h"

#define TEST_NAV_AHEAD_MM   133u
#define TEST_NAV_PENDING_MS 20000u
#define TEST_NAV_PHASE_MS   8000u
#define TEST_NAV_ACCEPT     30000
#define TEST_NAV_OVER       45000

static nav_control_cfg_t const g_cfg =
{
    TEST_NAV_AHEAD_MM, TEST_NAV_PENDING_MS, TEST_NAV_PHASE_MS,
    TEST_NAV_ACCEPT, TEST_NAV_OVER
};

static void test_nav_setup(nav_control_t * p_nav, nav_control_input_t * p_in);
static bool test_nav_out(nav_control_output_t const * p_out,
                         nav_control_action_t action, int32_t value,
                         nav_control_outcome_t outcome);
static bool test_nav_to_spin(nav_control_t * p_nav,
                             nav_control_input_t * p_in, uint8_t command,
                             int32_t spin);
static bool test_nav_spin_to(nav_control_t * p_nav,
                             nav_control_input_t * p_in, int32_t step_mdeg,
                             int32_t stop_mdeg);

/* S2-U74: a command is held until a junction; STRAIGHT is consumed there;
   an unanswered command expires; bad commands are refused. */
bool
test_nav_control_hold (void)
{
    nav_control_t nav;
    nav_control_input_t in;
    nav_control_output_t out;
    nav_control_cfg_t bad = g_cfg;
    bool b_passed;

    test_nav_setup(&nav, &in);
    bad.accept_mdeg = 0;
    b_passed = (!nav_control_init(&nav, &bad))
               && nav_control_init(&nav, &g_cfg)
               && (!nav_control_stage(&nav, NAV_CONTROL_CMD_NONE, 0u))
               && (!nav_control_stage(&nav, 5u, 0u))
               && nav_control_stage(&nav, NAV_CONTROL_CMD_STRAIGHT, 0u)
               && (!nav_control_manoeuvring(&nav));

    in.now_ms = 1000u;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_NONE, 0,
                                        NAV_CONTROL_OUT_NONE);
    in.b_junction = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_NONE, 0,
                                        NAV_CONTROL_OUT_STRAIGHT)
               && (NAV_CONTROL_IDLE == nav.state);

    /* A newer command replaces a held one; expiry drops it. */
    in.b_junction = false;
    b_passed = b_passed && nav_control_stage(&nav, NAV_CONTROL_CMD_LEFT, 0u)
               && nav_control_stage(&nav, NAV_CONTROL_CMD_RIGHT, 10u)
               && (NAV_CONTROL_CMD_RIGHT == nav.command);
    in.now_ms = TEST_NAV_PENDING_MS + 10u;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_NONE == out.outcome);
    in.now_ms++;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_EXPIRED == out.outcome)
               && (NAV_CONTROL_IDLE == nav.state);

    /* Nothing held: a junction does nothing. */
    in.b_junction = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_NONE, 0,
                                        NAV_CONTROL_OUT_NONE);

    /* Expiry across the timer wrap. */
    b_passed = b_passed
               && nav_control_stage(&nav, NAV_CONTROL_CMD_LEFT,
                                    UINT32_MAX - 100u);
    in.b_junction = false;
    in.now_ms = 1000u;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_PENDING == nav.state);

    return (b_passed);
}

/* S2-U75: LEFT and RIGHT stop, step the axle onto the junction less the
   travel since it was seen, spin and stop on the line; black is ignored
   until the turn is nearly done. */
bool
test_nav_control_turn (void)
{
    nav_control_t nav;
    nav_control_input_t in;
    nav_control_output_t out;
    bool b_passed;

    test_nav_setup(&nav, &in);
    b_passed = test_nav_to_spin(&nav, &in, NAV_CONTROL_CMD_LEFT, 1);

    /* The crossing just left is seen early and ignored. */
    in.b_on_line = true;
    in.heading_mdeg = 5000;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_ACT_NONE == out.action)
               && (NAV_CONTROL_SPIN == nav.state);
    in.b_on_line = false;
    b_passed = b_passed && test_nav_spin_to(&nav, &in, 5000, 80000);
    in.b_on_line = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_STOP, 0,
                                        NAV_CONTROL_OUT_NONE)
               && nav_control_manoeuvring(&nav);
    in.b_still = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_DONE == out.outcome)
               && (!nav_control_manoeuvring(&nav));

    /* RIGHT spins the other way and counts a falling heading. */
    test_nav_setup(&nav, &in);
    b_passed = b_passed
               && test_nav_to_spin(&nav, &in, NAV_CONTROL_CMD_RIGHT, -1)
               && test_nav_spin_to(&nav, &in, -5000, -70000);
    in.b_on_line = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_ACT_STOP == out.action);

    /* Rolled past the step: no MOVE, straight to the spin. */
    test_nav_setup(&nav, &in);
    in.junction_travel_mm = 0;
    in.travel_mm = (int32_t)TEST_NAV_AHEAD_MM + 5;
    b_passed = b_passed && nav_control_stage(&nav, NAV_CONTROL_CMD_LEFT, 0u);
    in.b_junction = true;
    nav_control_step(&nav, &in, &out);
    in.b_junction = false;
    in.b_still = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_SPIN, 1,
                                        NAV_CONTROL_OUT_NONE);

    return (b_passed);
}

/* S2-U76: a U-turn counts past 180 degrees across the heading wrap and
   ignores a side branch on the way. */
bool
test_nav_control_uturn (void)
{
    nav_control_t nav;
    nav_control_input_t in;
    nav_control_output_t out;
    bool b_passed;

    test_nav_setup(&nav, &in);
    in.heading_mdeg = 170000;
    b_passed = test_nav_to_spin(&nav, &in, NAV_CONTROL_CMD_UTURN, 1);

    /* A branch at 90 degrees is not the way back. */
    b_passed = b_passed && test_nav_spin_to(&nav, &in, 10000, -100000);
    in.b_on_line = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_SPIN == nav.state);
    in.b_on_line = false;

    /* 170 + 160 = 330 = -30 degrees: 160 turned, within 30 of 180. */
    b_passed = b_passed && test_nav_spin_to(&nav, &in, 10000, -30000)
               && (160000 == nav.turned_mdeg);
    in.b_on_line = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_ACT_STOP == out.action);

    return (b_passed);
}

/* S2-U77: overshoot, a rejected step and a slow stop each fail; abort
   drops everything. */
bool
test_nav_control_fail (void)
{
    nav_control_t nav;
    nav_control_input_t in;
    nav_control_output_t out;
    bool b_passed;

    /* No line within 90 + 45 degrees. */
    test_nav_setup(&nav, &in);
    b_passed = test_nav_to_spin(&nav, &in, NAV_CONTROL_CMD_LEFT, 1)
               && test_nav_spin_to(&nav, &in, 5000, 135000);
    in.heading_mdeg += 5000;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_FAILED == out.outcome)
               && (NAV_CONTROL_IDLE == nav.state);

    /* The step is rejected. */
    test_nav_setup(&nav, &in);
    b_passed = b_passed && nav_control_stage(&nav, NAV_CONTROL_CMD_LEFT, 0u);
    in.b_junction = true;
    nav_control_step(&nav, &in, &out);
    in.b_junction = false;
    in.b_still = true;
    nav_control_step(&nav, &in, &out);
    nav_control_sent(&nav, 7u);
    in.last_id = 7u;
    in.last_result = MOTION_EVT_REJECTED;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_FAILED == out.outcome);

    /* Not still within the phase limit. */
    test_nav_setup(&nav, &in);
    b_passed = b_passed && nav_control_stage(&nav, NAV_CONTROL_CMD_RIGHT, 0u);
    in.b_junction = true;
    nav_control_step(&nav, &in, &out);
    in.b_junction = false;
    in.now_ms = TEST_NAV_PHASE_MS + 1u;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_FAILED == out.outcome);

    /* Abort mid-manoeuvre; staging is refused while manoeuvring. */
    test_nav_setup(&nav, &in);
    b_passed = b_passed && nav_control_stage(&nav, NAV_CONTROL_CMD_LEFT, 0u);
    in.b_junction = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && nav_control_manoeuvring(&nav)
               && (!nav_control_stage(&nav, NAV_CONTROL_CMD_RIGHT, 0u));
    nav_control_abort(&nav);
    b_passed = b_passed && (!nav_control_manoeuvring(&nav))
               && (NAV_CONTROL_IDLE == nav.state);
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_NONE, 0,
                                        NAV_CONTROL_OUT_NONE)
               && (!nav_control_init(NULL, &g_cfg))
               && (!nav_control_manoeuvring(NULL));

    return (b_passed);
}

/* S2-U79 (post-demo test, 2026-11-25): a step onto the junction that
   never reports, and a stop on the new line that never settles, each
   fail at the phase limit rather than hold the car; null inputs are
   ignored. */
bool
test_nav_control_phase_limits (void)
{
    nav_control_t nav;
    nav_control_input_t in;
    nav_control_output_t out;
    bool b_passed;

    /* ADVANCE: the step is sent but no result ever arrives. */
    test_nav_setup(&nav, &in);
    b_passed = nav_control_stage(&nav, NAV_CONTROL_CMD_LEFT, 0u);
    in.b_junction = true;
    nav_control_step(&nav, &in, &out);
    in.b_junction = false;
    in.b_still = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_ACT_MOVE == out.action);
    nav_control_sent(&nav, 9u);
    in.b_still = false;
    in.now_ms = TEST_NAV_PHASE_MS;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_NONE == out.outcome)
               && (NAV_CONTROL_ADVANCE == nav.state);
    in.now_ms++;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_FAILED == out.outcome)
               && (NAV_CONTROL_IDLE == nav.state);

    /* SETTLE: on the new line, but the wheels never stop. */
    test_nav_setup(&nav, &in);
    b_passed = b_passed
               && test_nav_to_spin(&nav, &in, NAV_CONTROL_CMD_LEFT, 1)
               && test_nav_spin_to(&nav, &in, 5000, 80000);
    in.b_on_line = true;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_SETTLE == nav.state);
    in.now_ms += TEST_NAV_PHASE_MS + 1u;
    nav_control_step(&nav, &in, &out);
    b_passed = b_passed && (NAV_CONTROL_OUT_FAILED == out.outcome)
               && (NAV_CONTROL_IDLE == nav.state);

    /* Null arguments change nothing and do not crash. */
    nav_control_step(NULL, &in, &out);
    nav_control_step(&nav, NULL, &out);
    nav_control_step(&nav, &in, NULL);
    b_passed = b_passed && (NAV_CONTROL_IDLE == nav.state)
               && (!nav_control_stage(NULL, NAV_CONTROL_CMD_LEFT, 0u));

    return (b_passed);
}

static void
test_nav_setup (nav_control_t * p_nav, nav_control_input_t * p_in)
{
    nav_control_input_t const zero = {0};

    (void)nav_control_init(p_nav, &g_cfg);
    *p_in = zero;
    p_in->b_status_ok = true;
    p_in->travel_mm = 1000;
    p_in->junction_travel_mm = 990;     /* 10 mm rolled since black */
}

static bool
test_nav_out (nav_control_output_t const * p_out,
              nav_control_action_t action, int32_t value,
              nav_control_outcome_t outcome)
{
    return ((action == p_out->action) && (value == p_out->value)
            && (outcome == p_out->outcome));
}

/* Stage, reach a junction, stop, step (133 - 10 mm) and start the spin. */
static bool
test_nav_to_spin (nav_control_t * p_nav, nav_control_input_t * p_in,
                  uint8_t command, int32_t spin)
{
    nav_control_output_t out;
    bool b_passed = nav_control_stage(p_nav, command, p_in->now_ms);

    p_in->b_junction = true;
    nav_control_step(p_nav, p_in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_STOP, 0,
                                        NAV_CONTROL_OUT_STARTED);
    p_in->b_junction = false;
    nav_control_step(p_nav, p_in, &out);
    b_passed = b_passed && (NAV_CONTROL_ACT_NONE == out.action);
    p_in->b_still = true;
    nav_control_step(p_nav, p_in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_MOVE,
                                        (int32_t)TEST_NAV_AHEAD_MM - 10,
                                        NAV_CONTROL_OUT_NONE);
    nav_control_sent(p_nav, 3u);
    p_in->b_still = false;
    p_in->last_id = 3u;
    p_in->last_result = MOTION_EVT_DONE;
    nav_control_step(p_nav, p_in, &out);
    b_passed = b_passed && test_nav_out(&out, NAV_CONTROL_ACT_SPIN, spin,
                                        NAV_CONTROL_OUT_NONE);

    return (b_passed);
}

/* Turn in steps until the heading reaches stop_mdeg, with no line seen. */
static bool
test_nav_spin_to (nav_control_t * p_nav, nav_control_input_t * p_in,
                  int32_t step_mdeg, int32_t stop_mdeg)
{
    nav_control_output_t out;
    bool b_passed = true;
    uint16_t guard = 0u;

    p_in->b_on_line = false;
    while ((p_in->heading_mdeg != stop_mdeg) && (guard < 200u) && b_passed)
    {
        p_in->heading_mdeg += step_mdeg;
        if (p_in->heading_mdeg > 180000)
        {
            p_in->heading_mdeg -= 360000;
        }
        else if (p_in->heading_mdeg <= -180000)
        {
            p_in->heading_mdeg += 360000;
        }
        p_in->now_ms += 10u;
        nav_control_step(p_nav, p_in, &out);
        b_passed = (NAV_CONTROL_SPIN == p_nav->state);
        guard++;
    }

    return (b_passed && (guard < 200u));
}

#endif /* APP_TEST */

/* End of file. */
