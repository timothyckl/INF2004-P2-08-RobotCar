/**
 * @file test_runner.c
 * @brief Pico W reporting; absent subsystem logic never produces a PASS.
 */
#include "../include/test_runner.h"

#include <limits.h>
#include <stddef.h>
#include <tm/tmonitor.h>

#if UINT_MAX != UINT32_MAX
#error "The Pico W console counter format requires a 32-bit UINT."
#endif

static uint32_t g_passed;
static uint32_t g_failed;
static uint32_t g_blocked;

void
test_runner_begin (void)
{
    g_passed  = 0u;
    g_failed  = 0u;
    g_blocked = 0u;
}

void
test_runner_check (char const * p_id, bool b_passed,
                   char const * p_description)
{
    char const * p_status = "FAIL";

    if (b_passed)
    {
        g_passed++;
        p_status = "PASS";
    }
    else
    {
        g_failed++;
    }

    /* T-Monitor reads ASCII format bytes without modifying the literal. */
    tm_printf((UB const *) "[TEST] %s %s  %s\n",
              p_id, p_status, p_description);
}

void
test_runner_cases (test_runner_case_t const * p_cases, uint32_t count,
                   test_interface_api_t const * p_api)
{
    for (uint32_t index = 0u; index < count; index++)
    {
        if (NULL == p_api)
        {
            g_blocked++;

            /* T-Monitor reads ASCII format bytes without mutation. */
            tm_printf((UB const *) "[TEST] %s BLOCKED  %s: "
                      "subsystem logic not implemented\n",
                      p_cases[index].p_id, p_cases[index].p_description);
        }
        else
        {
            bool b_passed = p_cases[index].p_run(p_api);

            test_runner_check(p_cases[index].p_id, b_passed,
                              p_cases[index].p_description);
        }
    }
}

void
test_runner_finish (void)
{
    /* The format is read-only. The checked 32-bit UINT preserves counts. */
    tm_printf((UB const *) "[TEST] subsystem5 SUMMARY %u passed, "
              "%u failed, %u blocked\n",
              (UINT) g_passed, (UINT) g_failed, (UINT) g_blocked);
}

/* End of file. */
