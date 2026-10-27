/**
 * @file test_safety.c
 * @brief No scan may bypass the controller's request-matched stop gate.
 * Sources: Design Review section 6.5; decisions.md, subsystem 5 section 6.
 */
#include "../include/test_safety.h"

#include "../include/test_fixture.h"
#include "../include/test_runner.h"
#include "../include/test_subject.h"

static uint32_t const g_test_safety_request     = 7u;
static uint32_t const g_test_safety_other       = 8u;
static uint32_t const g_test_safety_settling_us = 1000u;

/** Verify that authorisation alone cannot start scanning. */
static bool test_safety_no_stop (test_interface_api_t const * p_api);

/** Verify that stop confirmation for another request is ignored. */
static bool test_safety_wrong_stop (test_interface_api_t const * p_api);

/** Verify that stop confirmation alone cannot start scanning. */
static bool test_safety_no_permission (test_interface_api_t const * p_api);

/** Verify that matching safety inputs permit scanning. */
static bool test_safety_matching (test_interface_api_t const * p_api);

/** Verify that cancellation invalidates active scan work. */
static bool test_safety_cancel (test_interface_api_t const * p_api);

/** Verify that a new stop event invalidates active scan work. */
static bool test_safety_new_stop (test_interface_api_t const * p_api);

/** Supply the matching inputs needed to start a scan. */
static void test_safety_start_scan (test_interface_api_t const * p_api);

/** Report whether no scanning output occurs after an observation index. */
static bool test_safety_no_scan_after (uint32_t first);

/** Report whether the current trace contains no scan activity. */
static bool test_safety_is_inactive (void);

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
                       g_test_safety_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       g_test_safety_request, true);
    test_fixture_advance(g_test_safety_settling_us, p_api);

    return (test_safety_is_inactive());
}

static bool
test_safety_wrong_stop (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       g_test_safety_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       g_test_safety_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       g_test_safety_other, true);
    test_fixture_advance(g_test_safety_settling_us, p_api);

    return (test_safety_is_inactive());
}

static bool
test_safety_no_permission (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       g_test_safety_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       g_test_safety_request, true);
    test_fixture_advance(g_test_safety_settling_us, p_api);

    return (test_safety_is_inactive());
}

static bool
test_safety_matching (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_safety_start_scan(p_api);

    return ((!test_subject_forward_only()) && (test_subject_no_plan())
            && (1u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (!test_fixture_overflowed()));
}

static bool
test_safety_cancel (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_safety_start_scan(p_api);

    bool const     b_started = !test_subject_forward_only();
    uint32_t const first     = test_fixture_size();

    test_fixture_send(p_api, TEST_INTERFACE_CANCEL,
                       g_test_safety_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
    test_fixture_advance(1000u, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, false);
    test_fixture_advance(1000u, p_api);

    bool b_acknowledged = false;

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * const p_output =
            &test_fixture_record(index)->output;

        if ((TEST_INTERFACE_CANCEL_ACK == p_output->kind)
            && (g_test_safety_request == p_output->request_id))
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

    bool const     b_started = !test_subject_forward_only();
    uint32_t const first     = test_fixture_size();

    test_fixture_send(p_api, TEST_INTERFACE_STOP, 0u, true);
    test_fixture_advance(2000u, p_api);

    return (b_started && (test_safety_no_scan_after(first)));
}

static void
test_safety_start_scan (test_interface_api_t const * p_api)
{
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                       g_test_safety_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                       g_test_safety_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                       g_test_safety_request, true);
    test_fixture_advance(g_test_safety_settling_us, p_api);
}

static bool
test_safety_no_scan_after (uint32_t first)
{
    bool b_safe = !test_fixture_overflowed();

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * const p_output =
            &test_fixture_record(index)->output;

        bool const b_sweeping   =
            (TEST_INTERFACE_SERVO == p_output->kind)
            && (0 != p_output->angle_deg);
        bool const b_triggering =
            (TEST_INTERFACE_TRIGGER == p_output->kind)
            && (p_output->b_value);

        if (b_sweeping || b_triggering
            || (TEST_INTERFACE_PLAN == p_output->kind))
        {
            b_safe = false;
        }

    }

    return (b_safe);
}

static bool
test_safety_is_inactive (void)
{
    return ((test_subject_forward_only())
            && (0u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (test_subject_no_plan()));
}

/* End of file. */
