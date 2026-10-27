/**
 * @file scan_engine.c
 * @brief Safety-gated obstacle scan sequencing.
 */
#include "../include/scan_engine.h"

#include <stddef.h>

#define SCAN_FIRST_ANGLE_DEG    (-60)
#define SCAN_FORWARD_ANGLE_DEG      0

static void scan_engine_accept_request(scan_engine_t * p_engine,
                                       scan_engine_event_t const * p_event);
static void scan_engine_set_authority(scan_engine_t * p_engine,
                                      scan_engine_event_t const * p_event);
static void scan_engine_confirm_stop(scan_engine_t * p_engine,
                                     scan_engine_event_t const * p_event);
static void scan_engine_cancel(scan_engine_t * p_engine,
                               scan_engine_event_t const * p_event);
static void scan_engine_abort(scan_engine_t * p_engine);
static void scan_engine_try_start(scan_engine_t * p_engine);
static void scan_engine_tick(scan_engine_t * p_engine);
static void scan_engine_publish(scan_engine_t const * p_engine,
                                scan_engine_output_kind_t kind,
                                int16_t angle_deg,
                                bool b_asserted);

void
scan_engine_reset (scan_engine_t * p_engine,
                   scan_engine_port_t const * p_port,
                   scan_engine_config_t const * p_config)
{
    if (NULL != p_engine)
    {
        p_engine->port.p_time_now_us = NULL;
        p_engine->port.p_publish = NULL;
        p_engine->config.settling_us = 0u;
        p_engine->config.hazard_mm = 0u;
        p_engine->request_id = 0u;
        p_engine->settling_started_us = 0u;
        p_engine->b_has_request = false;
        p_engine->b_is_authorised = false;
        p_engine->b_is_stopped = false;
        p_engine->b_is_scanning = false;
        p_engine->b_trigger_started = false;

        if (NULL != p_port)
        {
            p_engine->port = *p_port;
        }

        if (NULL != p_config)
        {
            p_engine->config = *p_config;
        }
    }
}

void
scan_engine_process (scan_engine_t * p_engine,
                     scan_engine_event_t const * p_event)
{
    if ((NULL != p_engine) && (NULL != p_event))
    {
        switch (p_event->kind)
        {
            case SCAN_ENGINE_TICK:
                scan_engine_tick(p_engine);
            break;

            case SCAN_ENGINE_ECHO_EDGE:
                /* Echo capture is added after the safety gate is verified. */
            break;

            case SCAN_ENGINE_REQUEST:
                scan_engine_accept_request(p_engine, p_event);
            break;

            case SCAN_ENGINE_AUTHORISE:
                scan_engine_set_authority(p_engine, p_event);
            break;

            case SCAN_ENGINE_STOP_CONFIRM:
                scan_engine_confirm_stop(p_engine, p_event);
            break;

            case SCAN_ENGINE_CANCEL:
                scan_engine_cancel(p_engine, p_event);
            break;

            case SCAN_ENGINE_STOP:
            case SCAN_ENGINE_MOVEMENT:
                if (p_event->b_asserted)
                {
                    scan_engine_abort(p_engine);
                }
            break;

            case SCAN_ENGINE_RESET:
                scan_engine_abort(p_engine);
            break;

            default:
                scan_engine_abort(p_engine);
            break;
        }

        scan_engine_try_start(p_engine);
    }
}

static void
scan_engine_accept_request (scan_engine_t * p_engine,
                            scan_engine_event_t const * p_event)
{
    if (p_event->b_asserted && (!p_engine->b_has_request))
    {
        p_engine->request_id = p_event->request_id;
        p_engine->b_has_request = true;
        p_engine->b_is_authorised = false;
        p_engine->b_is_stopped = false;
    }
}

static void
scan_engine_set_authority (scan_engine_t * p_engine,
                           scan_engine_event_t const * p_event)
{
    bool const b_is_current = p_engine->b_has_request
                              && (p_engine->request_id
                                  == p_event->request_id);

    if (b_is_current)
    {
        p_engine->b_is_authorised = p_event->b_asserted;

        if ((!p_event->b_asserted) && p_engine->b_is_scanning)
        {
            scan_engine_abort(p_engine);
        }
    }
}

static void
scan_engine_confirm_stop (scan_engine_t * p_engine,
                          scan_engine_event_t const * p_event)
{
    bool const b_is_current = p_engine->b_has_request
                              && (p_engine->request_id
                                  == p_event->request_id);

    if (b_is_current)
    {
        p_engine->b_is_stopped = p_event->b_asserted;

        if ((!p_event->b_asserted) && p_engine->b_is_scanning)
        {
            scan_engine_abort(p_engine);
        }
    }
}

static void
scan_engine_cancel (scan_engine_t * p_engine,
                    scan_engine_event_t const * p_event)
{
    bool const b_is_current = p_engine->b_has_request
                              && (p_engine->request_id
                                  == p_event->request_id);

    if (b_is_current && p_event->b_asserted)
    {
        scan_engine_publish(p_engine, SCAN_ENGINE_CANCEL_ACK,
                            SCAN_FORWARD_ANGLE_DEG, true);
        scan_engine_abort(p_engine);
    }
}

static void
scan_engine_abort (scan_engine_t * p_engine)
{
    bool const b_was_scanning = p_engine->b_is_scanning;

    p_engine->b_has_request = false;
    p_engine->b_is_authorised = false;
    p_engine->b_is_stopped = false;
    p_engine->b_is_scanning = false;
    p_engine->b_trigger_started = false;

    if (b_was_scanning)
    {
        scan_engine_publish(p_engine, SCAN_ENGINE_SERVO,
                            SCAN_FORWARD_ANGLE_DEG, true);
    }
}

static void
scan_engine_try_start (scan_engine_t * p_engine)
{
    bool const b_can_start = p_engine->b_has_request
                             && p_engine->b_is_authorised
                             && p_engine->b_is_stopped
                             && (!p_engine->b_is_scanning)
                             && (NULL != p_engine->port.p_time_now_us);

    if (b_can_start)
    {
        p_engine->b_is_scanning = true;
        p_engine->b_trigger_started = false;
        p_engine->settling_started_us = p_engine->port.p_time_now_us();
        scan_engine_publish(p_engine, SCAN_ENGINE_SERVO,
                            SCAN_FIRST_ANGLE_DEG, true);
    }
}

static void
scan_engine_tick (scan_engine_t * p_engine)
{
    bool const b_clock_ready = NULL != p_engine->port.p_time_now_us;

    if (p_engine->b_is_scanning
        && (!p_engine->b_trigger_started)
        && b_clock_ready)
    {
        uint32_t const now_us = p_engine->port.p_time_now_us();
        uint32_t const elapsed_us = now_us - p_engine->settling_started_us;

        if (p_engine->config.settling_us <= elapsed_us)
        {
            p_engine->b_trigger_started = true;
            scan_engine_publish(p_engine, SCAN_ENGINE_TRIGGER,
                                SCAN_FIRST_ANGLE_DEG, true);
        }
    }
}

static void
scan_engine_publish (scan_engine_t const * p_engine,
                     scan_engine_output_kind_t kind,
                     int16_t angle_deg,
                     bool b_asserted)
{
    if (NULL != p_engine->port.p_publish)
    {
        scan_engine_output_t const output =
        {
            kind,
            p_engine->request_id,
            angle_deg,
            0u,
            b_asserted
        };

        (void)p_engine->port.p_publish(&output);
    }
}

/* End of file. */
