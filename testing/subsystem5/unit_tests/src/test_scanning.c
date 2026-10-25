/**
 * @file test_scanning.c
 * @brief Initial coarse angle and settling checks using synthetic calibration.
 * Source: docs/decisions.md, subsystem 5 section 4.
 */
#include "../include/test_scanning.h"

#include "../include/test_fixture.h"
#include "../include/test_runner.h"
#include "../include/test_subject.h"

#define TEST_SCANNING_REQUEST 7u

static bool test_scanning_first_angle (test_interface_api_t const * p_api);
static bool test_scanning_settling (test_interface_api_t const * p_api);
static bool test_scanning_withdraw (test_interface_api_t const * p_api);
static void test_scanning_authorise (test_interface_api_t const * p_api);
static uint32_t test_scanning_triggers (uint32_t first);

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
         test_scanning_withdraw}
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
        test_interface_output_t const * p_output =
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
    uint32_t first = test_fixture_size();

    test_scanning_authorise(p_api);
    test_fixture_advance(999u, p_api);
    bool b_waited = (0u == test_scanning_triggers(first));

    test_fixture_advance(1u, p_api);
    bool b_sampled = (1u == test_scanning_triggers(first));

    return (b_waited && b_sampled && (!test_fixture_overflowed()));
}

static bool
test_scanning_withdraw (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       TEST_SCANNING_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       TEST_SCANNING_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       TEST_SCANNING_REQUEST, false);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       TEST_SCANNING_REQUEST, true);
    test_fixture_advance(1000u, p_api);

    return ((test_subject_forward_only()) && (test_subject_no_plan()));
}

static void
test_scanning_authorise (test_interface_api_t const * p_api)
{
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       TEST_SCANNING_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       TEST_SCANNING_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       TEST_SCANNING_REQUEST, true);
}

static uint32_t
test_scanning_triggers (uint32_t first)
{
    uint32_t count = 0u;

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * p_output =
            &test_fixture_record(index)->output;

        if ((TEST_INTERFACE_TRIGGER == p_output->kind) && (p_output->b_value))
        {
            count++;
        }
    }

    return (count);
}

/* End of file. */
