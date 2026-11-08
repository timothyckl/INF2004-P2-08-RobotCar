/**
 * @file test_capture.c
 * @brief Echo capture, sampling and coarse-profile cases (decisions 3, 4).
 *
 * Echo widths are synthetic: width_us = distance_mm * 100 / 17, the
 * production conversion inverted. They are not measurements.
 */
#include "../include/test_capture.h"
#include "../include/test_fixture.h"
#include "../include/test_runner.h"
#include "../include/test_subject.h"

#include <stddef.h>

static uint32_t const g_capture_request = 9u;
static uint32_t const g_capture_rise_us = 100u;

/** S5-U19: no trigger interval below 65 ms. */
static bool test_capture_spacing (test_interface_api_t const * p_api);

/** S5-U20: only completion within 30 ms supplies a sample. */
static bool test_capture_deadline (test_interface_api_t const * p_api);

/** S5-U21: malformed and stuck-high ECHO. */
static bool test_capture_malformed (test_interface_api_t const * p_api);

/** S5-U22: an old completion cannot complete a new capture. */
static bool test_capture_stale (test_interface_api_t const * p_api);

/** S5-U23: the published angle distance is the median of three. */
static bool test_capture_median (test_interface_api_t const * p_api);

/** S5-U24: the third consecutive invalid attempt faults. */
static bool test_capture_retry (test_interface_api_t const * p_api);

/** S5-U26: a complete coarse scan covers -60..60 in 15 degree steps. */
static bool test_capture_coarse (test_interface_api_t const * p_api);

/** A fresh unsafe forward reading cannot release a completed profile. */
static bool test_capture_clearance (test_interface_api_t const * p_api);

/** S5-U28: a scan that runs past 15 s faults. */
static bool test_capture_limit (test_interface_api_t const * p_api);

/** Grant request, authorisation and stop confirmation. */
static void test_capture_authorise (test_interface_api_t const * p_api);

/** Advance in 1 ms steps until a new trigger appears; false if none. */
static bool test_capture_await (test_interface_api_t const * p_api,
                                uint32_t limit_ms);

/** Deliver one echo whose fall is fall_us after the trigger. */
static void test_capture_echo (test_interface_api_t const * p_api,
                               uint32_t fall_us);

/** Echo width in microseconds for a whole number of millimetres. */
static uint32_t test_capture_width (uint32_t distance_mm);

/** Count accepted outputs of one kind from an index onwards. */
static uint32_t test_capture_count (test_interface_output_kind_t kind,
                                    uint32_t first);

/** Distance of the first DISTANCE output, or 0 if there is none. */
static uint16_t test_capture_first_distance (void);

void
test_capture_run (test_interface_api_t const * p_api)
{
    static test_runner_case_t const cases[] =
    {
        {"S5-U19", "trigger spacing at least 65 ms", test_capture_spacing},
        {"S5-U20", "completion after 30 ms is no sample",
         test_capture_deadline},
        {"S5-U21", "malformed capture and stuck ECHO",
         test_capture_malformed},
        {"S5-U22", "old completion cannot complete new capture",
         test_capture_stale},
        {"S5-U23", "angle distance is the median of three",
         test_capture_median},
        {"S5-U24", "third consecutive invalid attempt faults",
         test_capture_retry},
        {"S5-U26", "coarse scan angles and three samples",
         test_capture_coarse},
        {"S5-U56", "plan waits for safe forward clearance",
         test_capture_clearance},
        {"S5-U28", "scan past 15 s faults", test_capture_limit}
    };
    uint32_t const count = (uint32_t) (sizeof(cases) / sizeof(cases[0]));

    test_runner_cases(cases, count, p_api);
}

static bool
test_capture_spacing (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);

    bool const b_first = test_capture_await(p_api, 5u);

    test_capture_echo(p_api, g_capture_rise_us + test_capture_width(340u));
    test_fixture_advance(64000u - g_capture_rise_us
                         - test_capture_width(340u), p_api);

    bool const b_spaced = (1u == test_fixture_count(TEST_INTERFACE_TRIGGER));

    test_fixture_advance(1000u, p_api);

    return (b_first && b_spaced
            && (2u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (!test_fixture_overflowed()));
}

static bool
test_capture_deadline (test_interface_api_t const * p_api)
{
    uint32_t const width = test_capture_width(340u);   /* 2000 us */

    /* In time: three completions at 29 ms make one median sample. */
    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);

    for (uint32_t sample = 0u; sample < 3u; sample++)
    {
        (void) test_capture_await(p_api, 70u);
        test_fixture_advance(29000u - width, p_api);
        test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
        test_fixture_advance(width, p_api);
        test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, false);
    }

    bool const b_in_time = (1u == test_fixture_count(TEST_INTERFACE_DISTANCE));

    /* Late: the same width completing at 31 ms is an invalid attempt. The
       fall is dispatched before the tick at that time, as the task drains
       queued edges before it ticks, so the edge's own age decides. */
    test_interface_input_t const late_fall = {TEST_INTERFACE_ECHO, 0u, false};

    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);

    for (uint32_t sample = 0u; sample < 3u; sample++)
    {
        (void) test_capture_await(p_api, 70u);
        test_fixture_advance(29000u - width, p_api);
        test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
        (void) test_fixture_schedule(width + 2000u, &late_fall);
        test_fixture_advance(width + 2000u, p_api);
    }

    return (b_in_time
            && (0u == test_fixture_count(TEST_INTERFACE_DISTANCE))
            && (1u == test_fixture_count(TEST_INTERFACE_FAULT))
            && test_subject_no_plan());
}

static bool
test_capture_malformed (test_interface_api_t const * p_api)
{
    /* Unsolicited edges before the trigger, then a fall without rise. */
    test_subject_start(p_api, 0u);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, false);
    test_capture_authorise(p_api);
    (void) test_capture_await(p_api, 5u);
    test_fixture_advance(500u, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, false);
    test_fixture_advance(1000u, p_api);

    bool const b_no_sample =
        (0u == test_fixture_count(TEST_INTERFACE_DISTANCE))
        && (0u == test_fixture_count(TEST_INTERFACE_FAULT));

    /* Stuck high: no retrigger while high, then a fault. */
    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);
    (void) test_capture_await(p_api, 5u);
    test_fixture_advance(g_capture_rise_us, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
    test_fixture_advance(120000u, p_api);

    bool const b_held = (1u == test_fixture_count(TEST_INTERFACE_TRIGGER))
                        && (0u == test_fixture_count(TEST_INTERFACE_FAULT));

    test_fixture_advance(20000u, p_api);

    return (b_no_sample && b_held
            && (1u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (1u == test_fixture_count(TEST_INTERFACE_FAULT))
            && test_subject_no_plan());
}

static bool
test_capture_stale (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);
    (void) test_capture_await(p_api, 5u);
    test_fixture_advance(g_capture_rise_us, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
    test_fixture_send(p_api, TEST_INTERFACE_CANCEL, g_capture_request, true);

    /* The old capture's fall arrives after the cancel, before re-arming. */
    test_fixture_advance(1000u, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, false);
    test_capture_authorise(p_api);

    uint32_t const first = test_fixture_size();

    (void) test_capture_await(p_api, 70u);
    test_fixture_advance(5000u, p_api);

    bool const b_no_sample = (0u == test_capture_count(TEST_INTERFACE_DISTANCE,
                                                       first));

    /* Three genuine captures still complete the angle. */
    test_capture_echo(p_api, g_capture_rise_us + test_capture_width(340u));

    for (uint32_t sample = 1u; sample < 3u; sample++)
    {
        (void) test_capture_await(p_api, 70u);
        test_capture_echo(p_api,
                          g_capture_rise_us + test_capture_width(340u));
    }

    return (b_no_sample
            && (1u == test_capture_count(TEST_INTERFACE_DISTANCE, first))
            && (1u == test_fixture_count(TEST_INTERFACE_CANCEL_ACK)));
}

static bool
test_capture_median (test_interface_api_t const * p_api)
{
    uint32_t const mm[3] = {170u, 850u, 340u};

    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);

    for (uint32_t sample = 0u; sample < 3u; sample++)
    {
        (void) test_capture_await(p_api, 70u);
        test_capture_echo(p_api,
                          g_capture_rise_us + test_capture_width(mm[sample]));
    }

    return ((1u == test_fixture_count(TEST_INTERFACE_DISTANCE))
            && (340u == test_capture_first_distance()));
}

static bool
test_capture_retry (test_interface_api_t const * p_api)
{
    /* Two invalid attempts, then valid ones: no fault. */
    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);
    (void) test_capture_await(p_api, 5u);
    (void) test_capture_await(p_api, 70u);
    (void) test_capture_await(p_api, 70u);

    for (uint32_t sample = 0u; sample < 3u; sample++)
    {
        test_capture_echo(p_api,
                          g_capture_rise_us + test_capture_width(340u));
        (void) test_capture_await(p_api, 70u);
    }

    bool const b_recovered =
        (0u == test_fixture_count(TEST_INTERFACE_FAULT))
        && (1u <= test_fixture_count(TEST_INTERFACE_DISTANCE));

    /* Three consecutive invalid attempts: fault, no profile. */
    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);
    (void) test_capture_await(p_api, 5u);
    (void) test_capture_await(p_api, 70u);
    (void) test_capture_await(p_api, 70u);
    test_fixture_advance(40000u, p_api);

    return (b_recovered
            && (1u == test_fixture_count(TEST_INTERFACE_FAULT))
            && (0u == test_fixture_count(TEST_INTERFACE_DISTANCE))
            && test_subject_no_plan());
}

static bool
test_capture_coarse (test_interface_api_t const * p_api)
{
    bool    b_angles = true;
    int16_t expected = -60;

    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);

    for (uint32_t sample = 0u; sample < 27u; sample++)
    {
        (void) test_capture_await(p_api, 70u);
        test_capture_echo(p_api,
                          g_capture_rise_us + test_capture_width(850u));
    }

    bool const b_held = test_subject_no_plan();
    bool const b_forward_trigger = test_capture_await(p_api, 70u);

    test_capture_echo(p_api,
                      g_capture_rise_us + test_capture_width(850u));

    for (uint32_t index = 0u; index < test_fixture_size(); index++)
    {
        test_interface_output_t const * const p_output =
            &test_fixture_record(index)->output;

        if (TEST_INTERFACE_DISTANCE == p_output->kind)
        {
            b_angles = b_angles && (expected == p_output->angle_deg)
                       && (850u == p_output->distance_mm);
            expected = (int16_t) (expected + 15);
        }

    }

    test_fixture_record_t const * const p_last =
        test_fixture_record(test_fixture_size() - 1u);

    return (b_angles && (75 == expected) && b_held && b_forward_trigger
            && (28u == test_fixture_count(TEST_INTERFACE_TRIGGER))
            && (1u == test_fixture_count(TEST_INTERFACE_PLAN))
            && (0u == test_fixture_count(TEST_INTERFACE_FAULT))
            && (NULL != p_last)
            && (TEST_INTERFACE_SERVO == p_last->output.kind)
            && (0 == p_last->output.angle_deg)
            && (!test_fixture_overflowed()));
}

static bool
test_capture_clearance (test_interface_api_t const * p_api)
{
    test_subject_start(p_api, 0u);
    test_capture_authorise(p_api);

    for (uint32_t sample = 0u; sample < 27u; sample++)
    {
        (void) test_capture_await(p_api, 70u);
        test_capture_echo(p_api,
                          g_capture_rise_us + test_capture_width(850u));
    }

    bool const b_held = test_subject_no_plan();

    (void) test_capture_await(p_api, 70u);
    test_capture_echo(p_api,
                      g_capture_rise_us + test_capture_width(170u));

    return (b_held && test_subject_no_plan()
            && (1u == test_fixture_count(TEST_INTERFACE_FAULT))
            && (!test_fixture_overflowed()));
}

static bool
test_capture_limit (test_interface_api_t const * p_api)
{
    /* 2 s settling per angle cannot finish nine angles inside 15 s. */
    test_interface_config_t const slow = {2000000u, 200u};
    uint32_t elapsed_ms = 0u;

    test_fixture_reset(0u);
    p_api->p_reset_fixture(test_fixture_ports(), &slow);
    test_capture_authorise(p_api);

    while ((0u == test_fixture_count(TEST_INTERFACE_FAULT))
           && (elapsed_ms < 16000u))
    {
        if (test_capture_await(p_api, 70u))
        {
            test_capture_echo(p_api,
                              g_capture_rise_us + test_capture_width(850u));
        }

        elapsed_ms += 70u;
    }

    return ((1u == test_fixture_count(TEST_INTERFACE_FAULT))
            && test_subject_no_plan()
            && (test_fixture_count(TEST_INTERFACE_DISTANCE) < 9u));
}

static void
test_capture_authorise (test_interface_api_t const * p_api)
{
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_REQUEST,
                      g_capture_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_SCAN_AUTHORISE,
                      g_capture_request, true);
    test_fixture_send(p_api, TEST_INTERFACE_STOP_CONFIRM,
                      g_capture_request, true);
}

static bool
test_capture_await (test_interface_api_t const * p_api, uint32_t limit_ms)
{
    uint32_t const before = test_fixture_count(TEST_INTERFACE_TRIGGER);
    uint32_t       waited = 0u;

    while ((before == test_fixture_count(TEST_INTERFACE_TRIGGER))
           && (waited < limit_ms))
    {
        test_fixture_advance(1000u, p_api);
        waited++;
    }

    return (before != test_fixture_count(TEST_INTERFACE_TRIGGER));
}

static void
test_capture_echo (test_interface_api_t const * p_api, uint32_t fall_us)
{
    test_fixture_advance(g_capture_rise_us, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, true);
    test_fixture_advance(fall_us - g_capture_rise_us, p_api);
    test_fixture_send(p_api, TEST_INTERFACE_ECHO, 0u, false);
}

static uint32_t
test_capture_width (uint32_t distance_mm)
{
    return ((distance_mm * 100u) / 17u);
}

static uint32_t
test_capture_count (test_interface_output_kind_t kind, uint32_t first)
{
    uint32_t count = 0u;

    for (uint32_t index = first; index < test_fixture_size(); index++)
    {
        test_fixture_record_t const * const p_record =
            test_fixture_record(index);

        if ((kind == p_record->output.kind) && (p_record->b_accepted))
        {
            count++;
        }

    }

    return (count);
}

static uint16_t
test_capture_first_distance (void)
{
    uint16_t distance_mm = 0u;
    bool     b_found     = false;

    for (uint32_t index = 0u; (index < test_fixture_size()) && (!b_found);
         index++)
    {
        test_interface_output_t const * const p_output =
            &test_fixture_record(index)->output;

        if (TEST_INTERFACE_DISTANCE == p_output->kind)
        {
            distance_mm = p_output->distance_mm;
            b_found     = true;
        }

    }

    return (distance_mm);
}

/* End of file. */
