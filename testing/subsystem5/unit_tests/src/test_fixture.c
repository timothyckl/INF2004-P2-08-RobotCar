/**
 * @file test_fixture.c
 * @brief Fake dependencies only, with no subsystem behaviour.
 */
#include "../include/test_fixture.h"

#include <stddef.h>

/** Relative delays avoid ambiguous comparisons across clock wraparound. */
typedef struct
{
    uint32_t               delay_us;
    test_interface_input_t input;
} test_fixture_pending_t;

static uint32_t                  g_time_us;
static uint32_t                  g_record_count;
static uint32_t                  g_pending_count;
static bool                      g_b_accept_results;
static bool                      g_b_overflow;
static test_fixture_record_t    g_records[TEST_FIXTURE_CAPACITY];
static test_fixture_pending_t   g_pending[TEST_FIXTURE_CAPACITY];

static uint32_t test_fixture_now_us (void);
static bool test_fixture_emit (test_interface_output_t const * p_output);
static void test_fixture_tick (test_interface_api_t const * p_api);
static void test_fixture_elapse (uint32_t elapsed_us);

void
test_fixture_reset (uint32_t start_us)
{
    g_time_us          = start_us;
    g_record_count     = 0u;
    g_pending_count    = 0u;
    g_b_accept_results = true;
    g_b_overflow       = false;
}

test_interface_port_t const *
test_fixture_ports (void)
{
    static test_interface_port_t const ports =
    {
        test_fixture_now_us,
        test_fixture_emit
    };

    return (&ports);
}

bool
test_fixture_schedule (uint32_t delay_us,
                        test_interface_input_t const * p_input)
{
    bool b_scheduled = false;

    if ((NULL != p_input) && (g_pending_count < TEST_FIXTURE_CAPACITY))
    {
        uint32_t position = g_pending_count;

        while ((position > 0u)
               && (g_pending[position - 1u].delay_us > delay_us))
        {
            g_pending[position] = g_pending[position - 1u];
            position--;
        }

        g_pending[position].delay_us = delay_us;
        g_pending[position].input    = *p_input;
        g_pending_count++;
        b_scheduled = true;
    }

    return (b_scheduled);
}

void
test_fixture_advance (uint32_t elapsed_us,
                       test_interface_api_t const * p_api)
{
    uint32_t remaining_us = elapsed_us;

    if (NULL != p_api)
    {
        while ((g_pending_count > 0u)
               && (g_pending[0u].delay_us <= remaining_us))
        {
            uint32_t               delay_us = g_pending[0u].delay_us;
            test_interface_input_t input    = g_pending[0u].input;

            test_fixture_elapse(delay_us);
            remaining_us -= delay_us;
            g_pending_count--;

            for (uint32_t index = 0u; index < g_pending_count; index++)
            {
                g_pending[index] = g_pending[index + 1u];
            }

            p_api->p_dispatch(&input);
            test_fixture_tick(p_api);
        }

        test_fixture_elapse(remaining_us);
        test_fixture_tick(p_api);
    }
}

void
test_fixture_send (test_interface_api_t const * p_api,
                    test_interface_input_kind_t kind,
                    uint32_t request_id, bool b_value)
{
    test_interface_input_t input = {kind, request_id, b_value};

    p_api->p_dispatch(&input);
}

void
test_fixture_result_queue (bool b_accept)
{
    g_b_accept_results = b_accept;
}

uint32_t
test_fixture_count (test_interface_output_kind_t kind)
{
    uint32_t count = 0u;

    for (uint32_t index = 0u; index < g_record_count; index++)
    {
        if (kind == g_records[index].output.kind)
        {
            count++;
        }
    }

    return (count);
}

test_fixture_record_t const *
test_fixture_record (uint32_t index)
{
    test_fixture_record_t const * p_record = NULL;

    if (index < g_record_count)
    {
        p_record = &g_records[index];
    }

    return (p_record);
}

uint32_t
test_fixture_size (void)
{
    return (g_record_count);
}

bool
test_fixture_overflowed (void)
{
    return (g_b_overflow);
}

static uint32_t
test_fixture_now_us (void)
{
    return (g_time_us);
}

static bool
test_fixture_emit (test_interface_output_t const * p_output)
{
    bool b_result = (TEST_INTERFACE_DISTANCE == p_output->kind)
                    || (TEST_INTERFACE_PLAN == p_output->kind);
    bool b_accept = (!b_result) || g_b_accept_results;

    if (g_record_count < TEST_FIXTURE_CAPACITY)
    {
        g_records[g_record_count].output     = *p_output;
        g_records[g_record_count].time_us    = g_time_us;
        g_records[g_record_count].b_accepted = b_accept;
        g_record_count++;
    }
    else
    {
        g_b_overflow = true;
        b_accept    = false;
    }

    return (b_accept);
}

static void
test_fixture_tick (test_interface_api_t const * p_api)
{
    test_interface_input_t input = {TEST_INTERFACE_TICK, 0u, false};

    p_api->p_dispatch(&input);
}

static void
test_fixture_elapse (uint32_t elapsed_us)
{
    /* Unsigned addition deliberately models the native timer wrapping. */
    g_time_us += elapsed_us;

    for (uint32_t index = 0u; index < g_pending_count; index++)
    {
        g_pending[index].delay_us -= elapsed_us;
    }
}

/* End of file. */
