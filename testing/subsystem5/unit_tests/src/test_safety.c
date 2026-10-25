/**
 * @file test_safety.c
 * @brief No scan may bypass the controller's request-matched stop gate.
 * Sources: Design Review section 6.5; decisions.md, subsystem 5 section 6.
 */
#include "../include/test_safety.h"

#include "../include/test_fixture.h"
#include "../include/test_runner.h"
#include "../include/test_subject.h"

#define TEST_SAFETY_REQUEST 7u
#define TEST_SAFETY_OTHER 8u
#define TEST_SAFETY_SETTLING_US 1000u

static bool test_safety_no_stop (test_interface_api_t const * p_api);
static bool test_safety_wrong_stop (test_interface_api_t const * p_api);
static bool test_safety_no_permission (test_interface_api_t const * p_api);
static bool test_safety_matching (test_interface_api_t const * p_api);
static bool test_safety_cancel (test_interface_api_t const * p_api);
static bool test_safety_new_stop (test_interface_api_t const * p_api);
static void test_safety_start_scan (test_interface_api_t const * p_api);
static bool test_safety_no_scan_after (uint32_t first);

void
test_safety_run (test_interface_api_t const * p_api)
{
    test_runner_case_t const cases[] =
    {
        {"S5-U09", "permission without stop cannot scan", test_safety_no_stop},
        {"S5-U10", "wrong request stop cannot scan", test_safety_wrong_stop},
        {"S5-U11", "stop without permission cannot scan",
         test_safety_no_permission},
        {"S5-U12", "matched permission and stop allow scan",
         test_safety_matching},
        {"S5-U13", "cancel invalidates scan and acknowledges",
         test_safety_cancel},
        {"S5-U14", "new STOP aborts an authorised scan", test_safety_new_stop}
    };
    uint32_t const count = sizeof(cases) / sizeof(cases[0u]);

    test_runner_cases(cases, count, p_api);
}

static bool
test_safety_no_stop (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_advance(TEST_SAFETY_SETTLING_US, p_api);

    return ((test_subject_forward_only()) && (test_subject_no_plan()));
}

static bool
test_safety_wrong_stop (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       TEST_SAFETY_OTHER, true);
    test_fixture_advance(TEST_SAFETY_SETTLING_US, p_api);

    return ((test_subject_forward_only()) && (test_subject_no_plan()));
}

static bool
test_safety_no_permission (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_advance(TEST_SAFETY_SETTLING_US, p_api);

    return ((test_subject_forward_only()) && (test_subject_no_plan()));
}

static bool
test_safety_matching (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_safety_start_scan(p_api);

    return ((!test_subject_forward_only()) && (test_subject_no_plan())
            && (!test_fixture_overflowed()));
}

static bool
test_safety_cancel (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_safety_start_scan(p_api);
    bool b_started = !test_subject_forward_only();
    uint32_t first = test_fixture_size();

    test_fixture_send(p_api, TEST_INTERFACE_CANCEL, TEST_SAFETY_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
    test_fixture_advance(1000u, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, false);
    test_fixture_advance(1000u, p_api);

    bool b_acknowledged = false;

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * p_output =
            &test_fixture_record(index)->output;

        if ((TEST_INTERFACE_CANCEL_ACK == p_output->kind)
            && (TEST_SAFETY_REQUEST == p_output->request_id))
        {
            b_acknowledged = true;
        }
    }

    return (b_started && b_acknowledged && (test_safety_no_scan_after(first)));
}

static bool
test_safety_new_stop (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_safety_start_scan(p_api);
    bool b_started = !test_subject_forward_only();
    uint32_t first = test_fixture_size();

    test_fixture_send(p_api, TEST_INTERFACE_STOP, 0u, true);
    test_fixture_advance(2000u, p_api);

    return (b_started && (test_safety_no_scan_after(first)));
}

static void
test_safety_start_scan (test_interface_api_t const * p_api)
{
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       TEST_SAFETY_REQUEST, true);
    test_fixture_advance(TEST_SAFETY_SETTLING_US, p_api);
}

static bool
test_safety_no_scan_after (uint32_t first)
{
    bool b_safe = !test_fixture_overflowed();

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * p_output =
            &test_fixture_record(index)->output;
        bool b_sweeping = (TEST_INTERFACE_SERVO == p_output->kind)
                         && (0 != p_output->angle_deg);

        if (b_sweeping || (TEST_INTERFACE_PLAN == p_output->kind))
        {
            b_safe = false;
        }
    }

    return (b_safe);
}

/* End of file. */
