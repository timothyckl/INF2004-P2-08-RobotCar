/**
 * @file test_scanning.c
 * @brief Initial coarse angle and settling checks using synthetic calibration.
 * Source: docs/decisions.md, subsystem 5 section 4.
 */
#include "../include/test_scanning.h"

#include "../include/test_fixture.h"
#include "../include/test_runner.h"
#include "../include/test_subject.h"

static uint32_t const g_test_scanning_request = 7u;

/** Verify the first non-forward command uses the coarse start angle. */
static bool test_scanning_first_angle (test_interface_api_t const * p_api);

/** Verify that triggering waits for the configured settling interval. */
static bool test_scanning_settling (test_interface_api_t const * p_api);

/** Verify that withdrawn authorisation prevents scan activity. */
static bool test_scanning_withdraw (test_interface_api_t const * p_api);

/** Verify that a rejected servo command prevents scan entry. */
static bool test_scanning_servo_rejected (
    test_interface_api_t const * p_api);

/** Verify that a rejected trigger command aborts the active scan. */
static bool test_scanning_trigger_rejected (
    test_interface_api_t const * p_api);

/** Supply the matching inputs needed to authorise a scan. */
static void test_scanning_authorise (test_interface_api_t const * p_api);

/** Count trigger attempts recorded at or after an observation index. */
static uint32_t test_scanning_triggers (uint32_t first);

/** Report whether any matching output attempt was rejected. */
static bool test_scanning_rejected (test_interface_output_kind_t kind);

/** Report whether rejection is followed by a forward servo attempt. */
static bool test_scanning_forward_after (uint32_t first);

void
test_scanning_run (test_interface_api_t const * p_api)
{
    test_runner_case_t const cases[] =
    {
        {"S5-U15", "coarse scan starts at minus sixty",
         test_scanning_first_angle},
        {"S5-U16", "no trigger before calibrated settling",
         test_scanning_settling},
        {"S5-U17", "withdrawn permission prevents scanning",
         test_scanning_withdraw},
        {"S5-U51", "rejected servo command prevents scanning",
         test_scanning_servo_rejected},
        {"S5-U52", "rejected trigger command aborts scanning",
         test_scanning_trigger_rejected}
    };
    uint32_t const count = sizeof(cases) / sizeof(cases[0u]);

    test_runner_cases(cases, count, p_api);
}

static bool
test_scanning_first_angle (test_interface_api_t const * p_api)
{
    bool b_found = false;
    bool b_valid = true;

    test_subject_start(p_api, 0u);
    test_scanning_authorise(p_api);
    test_fixture_advance(1000u, p_api);

    for (uint32_t index = 0u; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * const p_output =
            &test_fixture_record(index)->output;

        if ((TEST_INTERFACE_SERVO == p_output->kind)
            && (0 != p_output->angle_deg))
        {
            b_valid = b_valid && (b_found || (-60 == p_output->angle_deg));
            b_found = true;
        }

    }

    return (b_found && b_valid && (!test_fixture_overflowed()));
}

static bool
test_scanning_settling (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);

    uint32_t const first = test_fixture_size();

    test_scanning_authorise(p_api);
    test_fixture_advance(999u, p_api);

    bool const b_waited = (0u == test_scanning_triggers(first));

    test_fixture_advance(1u, p_api);

    bool const b_sampled = (1u == test_scanning_triggers(first));

    return (b_waited && b_sampled && (!test_fixture_overflowed()));
}

static bool
test_scanning_withdraw (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       g_test_scanning_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       g_test_scanning_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       g_test_scanning_request, false);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       g_test_scanning_request, true);
    test_fixture_advance(1000u, p_api);

    return ((test_subject_forward_only())
            && (0u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (test_subject_no_plan()));
}

static bool
test_scanning_servo_rejected (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_reject_output(TEST_INTERFACE_SERVO, 1u);
    test_scanning_authorise(p_api);
    test_fixture_advance(2000u, p_api);

    return (test_scanning_rejected(TEST_INTERFACE_SERVO)
            && (1u == test_fixture_count(TEST_INTERFACE_SERVO))
            && (0u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (test_subject_no_plan())
            && (!test_fixture_overflowed()));
}

static bool
test_scanning_trigger_rejected (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_reject_output(TEST_INTERFACE_TRIGGER, 1u);
    test_scanning_authorise(p_api);

    uint32_t const first = test_fixture_size();

    test_fixture_advance(1000u, p_api);
    test_fixture_advance(1000u, p_api);

    return (test_scanning_rejected(TEST_INTERFACE_TRIGGER)
            && (1u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (test_scanning_forward_after(first))
            && (test_subject_no_plan())
            && (!test_fixture_overflowed()));
}

static void
test_scanning_authorise (test_interface_api_t const * p_api)
{
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       g_test_scanning_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       g_test_scanning_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       g_test_scanning_request, true);
}

static uint32_t
test_scanning_triggers (uint32_t first)
{
    uint32_t count = 0u;

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * const p_output =
            &test_fixture_record(index)->output;

        if ((TEST_INTERFACE_TRIGGER == p_output->kind) && (p_output->b_value))
        {
            count++;
        }

    }

    return (count);
}

static bool
test_scanning_rejected (test_interface_output_kind_t kind)
{
    bool b_was_rejected = false;

    for (uint32_t index = 0u; index < test_fixture_size(); index++)
    {
        test_fixture_record_t const * const p_record =
            test_fixture_record(index);

        if ((kind == p_record->output.kind) && (!p_record->b_accepted))
        {
            b_was_rejected = true;
        }

    }

    return (b_was_rejected);
}

static bool
test_scanning_forward_after (uint32_t first)
{
    bool b_rejection_seen = false;
    bool b_forward_seen   = false;

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_fixture_record_t const * const p_record =
            test_fixture_record(index);

        bool const b_trigger_rejected =
            (TEST_INTERFACE_TRIGGER == p_record->output.kind)
            && (!p_record->b_accepted);
        bool const b_forward_attempt  =
            (TEST_INTERFACE_SERVO == p_record->output.kind)
            && (0 == p_record->output.angle_deg)
            && (p_record->output.b_value);

        if (b_trigger_rejected)
        {
            b_rejection_seen = true;
        }
        else if (b_rejection_seen && b_forward_attempt)
        {
            b_forward_seen = true;
        }
        else
        {
            /* No relevant state change for this observation. */
        }

    }

    return (b_forward_seen);
}

/* End of file. */
