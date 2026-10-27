/**
 * @file scan_test_adapter.c
 * @brief Translation between test fixtures and production scan logic.
 */
#include "../include/scan_test_adapter.h"

#if APP_TEST
#include "../include/range_measurement.h"
#include "../include/scan_engine.h"

#include <stddef.h>

static scan_engine_t        g_scan_engine;
static test_interface_port_t g_test_port = {NULL, NULL};

/** Reset the production scan engine with controlled test dependencies. */
static void scan_test_reset (test_interface_port_t const * p_port,
                             test_interface_config_t const * p_config);

/** Convert an echo duration through the production ranging module. */
static bool scan_test_convert_echo (uint32_t echo_us,
                                    uint16_t * p_distance_mm);

/** Translate and dispatch one controlled input to the scan engine. */
static void scan_test_dispatch (test_interface_input_t const * p_input);

/** Read the controlled test clock when one is available. */
static uint32_t scan_test_time_now_us (void);

/** Translate and publish one production output to the test fixture. */
static bool scan_test_publish (scan_engine_output_t const * p_output);

/** Translate a test input kind to its production equivalent. */
static bool scan_test_map_input (test_interface_input_kind_t input_kind,
                                 scan_engine_event_kind_t * p_event_kind);

/** Translate a production output kind to its test equivalent. */
static bool scan_test_map_output (scan_engine_output_kind_t output_kind,
                                  test_interface_output_kind_t * p_test_kind);

test_interface_api_t const *
scan_test_adapter_get (void)
{
    static test_interface_api_t const test_api =
    {
        scan_test_reset,
        scan_test_convert_echo,
        scan_test_dispatch
    };

    return (&test_api);
}

static void
scan_test_reset (test_interface_port_t const * p_port,
                 test_interface_config_t const * p_config)
{
    scan_engine_port_t const scan_port =
    {
        scan_test_time_now_us,
        scan_test_publish
    };
    scan_engine_config_t scan_config = {0u, 0u};

    g_test_port.p_now_us = NULL;
    g_test_port.p_emit   = NULL;

    if (NULL != p_port)
    {
        g_test_port = *p_port;
    }

    if (NULL != p_config)
    {
        scan_config.settling_us = p_config->settling_us;
        scan_config.hazard_mm   = p_config->hazard_mm;
    }

    scan_engine_reset(&g_scan_engine, &scan_port, &scan_config);
}

static bool
scan_test_convert_echo (uint32_t echo_us,
                        uint16_t * p_distance_mm)
{
    return (range_measurement_from_echo(echo_us, p_distance_mm));
}

static void
scan_test_dispatch (test_interface_input_t const * p_input)
{
    scan_engine_event_t event = {SCAN_ENGINE_TICK, 0u, false};
    bool b_is_mapped = false;

    if (NULL != p_input)
    {
        b_is_mapped      = scan_test_map_input(p_input->kind, &event.kind);
        event.request_id = p_input->request_id;
        event.b_asserted = p_input->b_value;
    }

    if (b_is_mapped)
    {
        scan_engine_process(&g_scan_engine, &event);
    }

}

static uint32_t
scan_test_time_now_us (void)
{
    uint32_t time_us = 0u;

    if (NULL != g_test_port.p_now_us)
    {
        time_us = g_test_port.p_now_us();
    }

    return (time_us);
}

static bool
scan_test_publish (scan_engine_output_t const * p_output)
{
    bool b_is_published = false;

    if ((NULL != p_output) && (NULL != g_test_port.p_emit))
    {
        test_interface_output_t output =
        {
            TEST_INTERFACE_FAULT,
            p_output->request_id,
            p_output->angle_deg,
            p_output->distance_mm,
            p_output->b_asserted
        };
        bool const b_is_mapped =
            scan_test_map_output(p_output->kind, &output.kind);

        if (b_is_mapped)
        {
            b_is_published = g_test_port.p_emit(&output);
        }

    }

    return (b_is_published);
}

static bool
scan_test_map_input (test_interface_input_kind_t input_kind,
                     scan_engine_event_kind_t * p_event_kind)
{
    bool b_is_mapped = true;

    if (NULL == p_event_kind)
    {
        b_is_mapped = false;
    }
    else
    {
        switch (input_kind)
        {
            case TEST_INTERFACE_TICK:
                *p_event_kind = SCAN_ENGINE_TICK;
            break;

            case TEST_INTERFACE_ECHO:
                *p_event_kind = SCAN_ENGINE_ECHO_EDGE;
            break;

            case TEST_INTERFACE_SCAN_REQUEST:
                *p_event_kind = SCAN_ENGINE_REQUEST;
            break;

            case TEST_INTERFACE_SCAN_AUTHORISE:
                *p_event_kind = SCAN_ENGINE_AUTHORISE;
            break;

            case TEST_INTERFACE_STOP_CONFIRM:
                *p_event_kind = SCAN_ENGINE_STOP_CONFIRM;
            break;

            case TEST_INTERFACE_CANCEL:
                *p_event_kind = SCAN_ENGINE_CANCEL;
            break;

            case TEST_INTERFACE_STOP:
                *p_event_kind = SCAN_ENGINE_STOP;
            break;

            case TEST_INTERFACE_MOVEMENT:
                *p_event_kind = SCAN_ENGINE_MOVEMENT;
            break;

            case TEST_INTERFACE_RESET:
                *p_event_kind = SCAN_ENGINE_RESET;
            break;

            default:
                b_is_mapped = false;
            break;
        }

    }

    return (b_is_mapped);
}

static bool
scan_test_map_output (scan_engine_output_kind_t output_kind,
                      test_interface_output_kind_t * p_test_kind)
{
    bool b_is_mapped = true;

    if (NULL == p_test_kind)
    {
        b_is_mapped = false;
    }
    else
    {
        switch (output_kind)
        {
            case SCAN_ENGINE_TRIGGER:
                *p_test_kind = TEST_INTERFACE_TRIGGER;
            break;

            case SCAN_ENGINE_SERVO:
                *p_test_kind = TEST_INTERFACE_SERVO;
            break;

            case SCAN_ENGINE_DISTANCE:
                *p_test_kind = TEST_INTERFACE_DISTANCE;
            break;

            case SCAN_ENGINE_HAZARD:
                *p_test_kind = TEST_INTERFACE_HAZARD;
            break;

            case SCAN_ENGINE_FAULT:
                *p_test_kind = TEST_INTERFACE_FAULT;
            break;

            case SCAN_ENGINE_CANCEL_ACK:
                *p_test_kind = TEST_INTERFACE_CANCEL_ACK;
            break;

            case SCAN_ENGINE_READY:
                *p_test_kind = TEST_INTERFACE_READY;
            break;

            case SCAN_ENGINE_PLAN:
                *p_test_kind = TEST_INTERFACE_PLAN;
            break;

            default:
                b_is_mapped = false;
            break;
        }

    }

    return (b_is_mapped);
}
#endif

/* End of file. */
