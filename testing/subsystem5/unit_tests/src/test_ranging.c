/**
 * @file test_ranging.c
 * @brief Independent pulse fixtures from distance = width_us * 0.17 mm.
 * Sources: HC-SR04 datasheet and docs/decisions.md, subsystem 5 section 3.
 */
#include "../include/test_ranging.h"

#include "../include/test_runner.h"
#include <stddef.h>

#define TEST_RANGING_FIXTURES 3u

static uint16_t const g_test_ranging_sentinel = 1234u;

/** Verify exact integral pulse-to-distance conversions. */
static bool test_ranging_exact (test_interface_api_t const * p_api);

/** Verify the minimum supported pulse boundary. */
static bool test_ranging_minimum (test_interface_api_t const * p_api);

/** Verify the maximum supported pulse boundary. */
static bool test_ranging_maximum (test_interface_api_t const * p_api);

/** Verify that an invalid pulse preserves the previous output. */
static bool test_ranging_unchanged (test_interface_api_t const * p_api);

/** Verify that a null output pointer is rejected. */
static bool test_ranging_null (test_interface_api_t const * p_api);

/** Verify that oversized pulses cannot become valid through overflow. */
static bool test_ranging_overflow (test_interface_api_t const * p_api);

void
test_ranging_run (test_interface_api_t const * p_api)
{
    test_runner_case_t const cases[] =
    {
        {"S5-U03", "known integral pulse conversions", test_ranging_exact},
        {"S5-U04", "minimum physical range boundary", test_ranging_minimum},
        {"S5-U05", "maximum physical range boundary", test_ranging_maximum},
        {"S5-U06", "invalid conversion preserves output",
         test_ranging_unchanged},
        {"S5-U07", "null conversion output is rejected", test_ranging_null},
        {"S5-U08", "oversized pulses cannot overflow valid",
         test_ranging_overflow}
    };
    uint32_t const count = sizeof(cases) / sizeof(cases[0u]);

    test_runner_cases(cases, count, p_api);
}

static bool
test_ranging_exact (test_interface_api_t const * p_api)
{
    uint32_t const widths_us[TEST_RANGING_FIXTURES] = {1000u, 2000u, 5000u};
    uint16_t const distances[TEST_RANGING_FIXTURES] = {170u, 340u, 850u};
    bool           b_passed                       = true;

    for (uint32_t index = 0u; index < TEST_RANGING_FIXTURES; index++)
    {
        uint16_t   distance_mm = 0u;
        bool const b_valid     =
            p_api->p_pulse_mm(widths_us[index], &distance_mm);

        b_passed = b_valid && (distances[index] == distance_mm) && b_passed;
    }

    return (b_passed);
}

static bool
test_ranging_minimum (test_interface_api_t const * p_api)
{
    uint16_t   distance_mm = g_test_ranging_sentinel;
    bool const b_below     = p_api->p_pulse_mm(117u, &distance_mm);
    bool const b_above     = p_api->p_pulse_mm(118u, &distance_mm);

    /* At integer microsecond resolution, exactly 20 mm is not representable. */
    return ((!b_below) && b_above && (distance_mm >= 20u)
            && (distance_mm <= 21u));
}

static bool
test_ranging_maximum (test_interface_api_t const * p_api)
{
    uint16_t   distance_mm      = 0u;
    bool const b_maximum        = p_api->p_pulse_mm(23530u, &distance_mm);
    bool const b_distance_valid = (4000u == distance_mm);
    bool const b_above          =
        p_api->p_pulse_mm(23531u, &distance_mm);

    return (b_maximum && b_distance_valid && (!b_above));
}

static bool
test_ranging_unchanged (test_interface_api_t const * p_api)
{
    uint16_t   distance_mm = g_test_ranging_sentinel;
    bool const b_valid     = p_api->p_pulse_mm(0u, &distance_mm);

    return ((!b_valid) && (g_test_ranging_sentinel == distance_mm));
}

static bool
test_ranging_null (test_interface_api_t const * p_api)
{
    return (!p_api->p_pulse_mm(2000u, NULL));
}

static bool
test_ranging_overflow (test_interface_api_t const * p_api)
{
    uint16_t   distance_mm = g_test_ranging_sentinel;
    bool const b_late      = p_api->p_pulse_mm(30000u, &distance_mm);
    bool const b_maximum   = p_api->p_pulse_mm(UINT32_MAX, &distance_mm);

    return ((!b_late) && (!b_maximum)
            && (g_test_ranging_sentinel == distance_mm));
}

/* End of file. */
