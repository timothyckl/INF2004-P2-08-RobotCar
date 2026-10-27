/**
 * @file scan_engine.c
 * @brief Safety-gated obstacle scan sequencing.
 *
 * Source: obstacle-scanning decisions 4 and 6 in docs/decisions.md.
 */
#include "../include/scan_engine.h"

#include <stddef.h>

static int16_t const g_scan_first_angle_deg   = -60;
static int16_t const g_scan_forward_angle_deg = 0;

/** Accept a new request only while the scanner is idle. */
static void scan_engine_accept_request (scan_engine_t * p_engine,
                                        scan_engine_event_t const * p_event);

/** Apply authorisation only to the active request. */
static void scan_engine_set_authority (scan_engine_t * p_engine,
                                       scan_engine_event_t const * p_event);

/** Apply stop confirmation only to the active request. */
static void scan_engine_confirm_stop (scan_engine_t * p_engine,
                                      scan_engine_event_t const * p_event);

/** Acknowledge and invalidate cancellation of the active request. */
static void scan_engine_cancel (scan_engine_t * p_engine,
                                scan_engine_event_t const * p_event);

/** Invalidate active work and request a best-effort forward position. */
static void scan_engine_abort (scan_engine_t * p_engine);

/** Start scanning only when all safety gates and ports are ready. */
static void scan_engine_try_start (scan_engine_t * p_engine);

/** Advance time-dependent scan work without blocking. */
static void scan_engine_tick (scan_engine_t * p_engine);

/** Request one trigger and abort the scan if the command is rejected. */
static void scan_engine_try_trigger (scan_engine_t * p_engine);

/** Publish one output and report whether the receiver accepted it. */
static bool scan_engine_publish (scan_engine_t const * p_engine,
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
        p_engine->port.p_time_now_us  = NULL;
        p_engine->port.p_publish      = NULL;
        p_engine->config.settling_us  = 0u;
        p_engine->config.hazard_mm    = 0u;
        p_engine->request_id          = 0u;
        p_engine->settling_started_us = 0u;
        p_engine->b_has_request       = false;
        p_engine->b_is_authorised     = false;
        p_engine->b_is_stopped        = false;
        p_engine->b_is_scanning       = false;
        p_engine->b_trigger_started   = false;

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
                /* TODO: Add echo capture for planned cases S5-U18 to S5-U22. */
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
    if ((p_event->b_asserted) && (!p_engine->b_has_request))
    {
        p_engine->request_id      = p_event->request_id;
        p_engine->b_has_request   = true;
        p_engine->b_is_authorised = false;
        p_engine->b_is_stopped    = false;
    }

}

static void
scan_engine_set_authority (scan_engine_t * p_engine,
                           scan_engine_event_t const * p_event)
{
    bool const b_is_current = (p_engine->b_has_request)
                              && (p_engine->request_id
                                  == p_event->request_id);

    if (b_is_current)
    {
        p_engine->b_is_authorised = p_event->b_asserted;

        if ((!p_event->b_asserted) && (p_engine->b_is_scanning))
        {
            scan_engine_abort(p_engine);
        }

    }

}

static void
scan_engine_confirm_stop (scan_engine_t * p_engine,
                          scan_engine_event_t const * p_event)
{
    bool const b_is_current = (p_engine->b_has_request)
                              && (p_engine->request_id
                                  == p_event->request_id);

    if (b_is_current)
    {
        p_engine->b_is_stopped = p_event->b_asserted;

        if ((!p_event->b_asserted) && (p_engine->b_is_scanning))
        {
            scan_engine_abort(p_engine);
        }

    }

}

static void
scan_engine_cancel (scan_engine_t * p_engine,
                    scan_engine_event_t const * p_event)
{
    bool const b_is_current = (p_engine->b_has_request)
                              && (p_engine->request_id
                                  == p_event->request_id);

    if (b_is_current && (p_event->b_asserted))
    {
        scan_engine_publish(p_engine, SCAN_ENGINE_CANCEL_ACK,
                            g_scan_forward_angle_deg, true);
        scan_engine_abort(p_engine);
    }

}

static void
scan_engine_abort (scan_engine_t * p_engine)
{
    bool const b_was_scanning = p_engine->b_is_scanning;

    p_engine->b_has_request     = false;
    p_engine->b_is_authorised   = false;
    p_engine->b_is_stopped      = false;
    p_engine->b_is_scanning     = false;
    p_engine->b_trigger_started = false;

    if (b_was_scanning)
    {
        /* Active work remains invalid if this return command is rejected. */
        scan_engine_publish(p_engine, SCAN_ENGINE_SERVO,
                            g_scan_forward_angle_deg, true);
    }

}

static void
scan_engine_try_start (scan_engine_t * p_engine)
{
    bool const b_can_start = (p_engine->b_has_request)
                             && (p_engine->b_is_authorised)
                             && (p_engine->b_is_stopped)
                             && (!p_engine->b_is_scanning)
                             && (NULL != p_engine->port.p_time_now_us)
                             && (NULL != p_engine->port.p_publish);

    if (b_can_start)
    {
        bool const b_servo_accepted = scan_engine_publish(
            p_engine, SCAN_ENGINE_SERVO, g_scan_first_angle_deg, true);

        if (b_servo_accepted)
        {
            p_engine->b_is_scanning       = true;
            p_engine->b_trigger_started   = false;
            p_engine->settling_started_us =
                p_engine->port.p_time_now_us();
        }
        else
        {
            scan_engine_abort(p_engine);
        }

    }

}

static void
scan_engine_tick (scan_engine_t * p_engine)
{
    bool const b_clock_ready = (NULL != p_engine->port.p_time_now_us);

    if ((p_engine->b_is_scanning)
        && (!p_engine->b_trigger_started)
        && b_clock_ready)
    {
        uint32_t const now_us = p_engine->port.p_time_now_us();

        /* Unsigned subtraction handles one documented clock wrap. */
        uint32_t const elapsed_us = now_us - p_engine->settling_started_us;

        if (p_engine->config.settling_us <= elapsed_us)
        {
            scan_engine_try_trigger(p_engine);
        }

    }

}

static void
scan_engine_try_trigger (scan_engine_t * p_engine)
{
    bool const b_trigger_accepted = scan_engine_publish(
        p_engine, SCAN_ENGINE_TRIGGER, g_scan_first_angle_deg, true);

    if (b_trigger_accepted)
    {
        p_engine->b_trigger_started = true;
    }
    else
    {
        scan_engine_abort(p_engine);
    }

}

static bool
scan_engine_publish (scan_engine_t const * p_engine,
                     scan_engine_output_kind_t kind,
                     int16_t angle_deg,
                     bool b_asserted)
{
    bool b_is_published = false;

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

        b_is_published = p_engine->port.p_publish(&output);
    }

    return (b_is_published);
}

/* End of file. */
