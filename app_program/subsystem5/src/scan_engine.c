/**
 * @file scan_engine.c
 * @brief Safety-gated obstacle scan sequencing.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Source: obstacle-scanning decisions 3, 4, 6 and 8 in docs/decisions.md.
 * Coarse angles are refined at 5 degrees around observed obstacle edges.
 */
#include "../include/scan_engine.h"

#include "../include/range_measurement.h"

#include <stddef.h>

static int16_t const  g_scan_first_angle_deg   = -60;
static int16_t const  g_scan_forward_angle_deg = 0;
static int16_t const  g_scan_step_deg          = 15;
static uint32_t const g_scan_retrigger_us      = 65000u;
static uint32_t const g_scan_deadline_us       = 30000u;
static uint32_t const g_scan_stuck_us          = 130000u;
static uint32_t const g_scan_limit_us          = 15000000u;
static uint32_t const g_scan_request_limit_us  = 2000000u;
static uint8_t const  g_scan_invalid_limit     = 3u;

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

/** Record one ECHO edge belonging to the open capture. */
static void scan_engine_echo (scan_engine_t * p_engine,
                              scan_engine_event_t const * p_event);

/** Close the open capture with a valid distance or an invalid attempt. */
static void scan_engine_complete (scan_engine_t * p_engine, bool b_valid,
                                  uint16_t distance_mm);

/** Publish the angle's median, then move on or finish the profile. */
static void scan_engine_next_angle (scan_engine_t * p_engine);

/** Choose five-degree samples inside each observed coarse edge interval. */
static void scan_engine_prepare_fine (scan_engine_t * p_engine);

/** Report a fault and invalidate the scan. */
static void scan_engine_fail (scan_engine_t * p_engine);

/** Current coarse angle in degrees. */
static int16_t scan_engine_angle (scan_engine_t const * p_engine);

/** Median of three samples. */
static uint16_t scan_engine_median (uint16_t const * p_samples);

/** Publish one output and report whether the receiver accepted it. */
static bool scan_engine_publish (scan_engine_t const * p_engine,
                                 scan_engine_output_kind_t kind,
                                 int16_t angle_deg,
                                 uint16_t distance_mm,
                                 bool b_asserted);

void
scan_engine_reset (scan_engine_t * p_engine,
                   scan_engine_port_t const * p_port,
                   scan_engine_config_t const * p_config)
{
    if (NULL != p_engine)
    {
        scan_engine_t const cleared = {0};

        *p_engine = cleared;

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
                scan_engine_echo(p_engine, p_event);
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
    if ((p_event->b_asserted) && (0u != p_event->request_id)
        && (!p_engine->b_has_request))
    {
        p_engine->request_id      = p_event->request_id;
        p_engine->request_started_us =
            (NULL != p_engine->port.p_time_now_us)
            ? p_engine->port.p_time_now_us() : 0u;
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
        (void) scan_engine_publish(p_engine, SCAN_ENGINE_CANCEL_ACK,
                                   g_scan_forward_angle_deg, 0u, true);
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
    p_engine->b_awaiting_clearance = false;
    p_engine->b_fine_scanning   = false;
    p_engine->b_trigger_started = false;
    p_engine->b_capture_open    = false;
    p_engine->sample_count      = 0u;
    p_engine->invalid_count     = 0u;

    if (b_was_scanning)
    {
        /* Active work remains invalid if this return command is rejected. */
        (void) scan_engine_publish(p_engine, SCAN_ENGINE_SERVO,
                                   g_scan_forward_angle_deg, 0u, true);
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
        uint32_t const now_us = p_engine->port.p_time_now_us();

        if ((now_us - p_engine->request_started_us)
            > g_scan_request_limit_us)
        {
            scan_engine_fail(p_engine);
            return;
        }

        bool const b_servo_accepted = scan_engine_publish(
            p_engine, SCAN_ENGINE_SERVO, g_scan_first_angle_deg, 0u, true);

        if (b_servo_accepted)
        {
            uint32_t const started_us = p_engine->port.p_time_now_us();

            p_engine->b_is_scanning       = true;
            p_engine->b_awaiting_clearance = false;
            p_engine->b_fine_scanning     = false;
            p_engine->b_trigger_started   = false;
            p_engine->b_has_trigger       = false;
            p_engine->b_capture_open      = false;
            p_engine->angle_index         = 0u;
            p_engine->fine_count          = 0u;
            p_engine->fine_index          = 0u;
            p_engine->sample_count        = 0u;
            p_engine->invalid_count       = 0u;
            p_engine->settling_started_us = started_us;
            p_engine->scan_started_us     = started_us;
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

    if ((p_engine->b_is_scanning) && b_clock_ready)
    {
        uint32_t const now_us = p_engine->port.p_time_now_us();

        /* Unsigned subtraction handles one documented clock wrap. */
        uint32_t const settled_us = now_us - p_engine->settling_started_us;
        uint32_t const since_us   = now_us - p_engine->trigger_us;

        if ((now_us - p_engine->scan_started_us) > g_scan_limit_us)
        {
            scan_engine_fail(p_engine);
        }
        else if (p_engine->b_capture_open)
        {
            if (since_us >= g_scan_deadline_us)
            {
                /* No completion within 30 ms: not a measurement. */
                scan_engine_complete(p_engine, false, 0u);
            }

        }
        else if (p_engine->b_echo_high && p_engine->b_has_trigger)
        {
            /* Never retrigger while ECHO is high; stuck high faults. */
            if (since_us >= g_scan_stuck_us)
            {
                scan_engine_fail(p_engine);
            }

        }
        else if ((p_engine->config.settling_us <= settled_us)
                 && ((!p_engine->b_has_trigger)
                     || (since_us >= g_scan_retrigger_us)))
        {
            scan_engine_try_trigger(p_engine);
        }
        else
        {
            /* Settling, or waiting out the 65 ms trigger spacing. */
        }

    }
    else if (p_engine->b_has_request && b_clock_ready
             && ((p_engine->port.p_time_now_us()
                  - p_engine->request_started_us)
                 > g_scan_request_limit_us))
    {
        scan_engine_fail(p_engine);
    }
    else
    {
        /* No timed work is active. */
    }

}

static void
scan_engine_try_trigger (scan_engine_t * p_engine)
{
    bool const b_trigger_accepted = scan_engine_publish(
        p_engine, SCAN_ENGINE_TRIGGER, scan_engine_angle(p_engine), 0u,
        true);

    if (b_trigger_accepted)
    {
        p_engine->b_trigger_started = true;
        p_engine->b_has_trigger     = true;
        p_engine->b_capture_open    = true;
        p_engine->b_rise_seen       = false;
        p_engine->trigger_us        = p_engine->port.p_time_now_us();
    }
    else
    {
        scan_engine_abort(p_engine);
    }

}

static void
scan_engine_echo (scan_engine_t * p_engine,
                  scan_engine_event_t const * p_event)
{
    /* Edges carry their own capture time. One from before the trigger
       wraps to a huge age and is rejected with any other late edge. */
    uint32_t const age_us = p_event->time_us - p_engine->trigger_us;

    if (p_event->b_asserted)
    {
        p_engine->b_echo_high = true;

        if ((p_engine->b_capture_open) && (!p_engine->b_rise_seen)
            && (age_us < g_scan_deadline_us))
        {
            p_engine->b_rise_seen = true;
            p_engine->rise_us     = p_event->time_us;
        }

    }
    else
    {
        p_engine->b_echo_high = false;

        if ((p_engine->b_capture_open) && (p_engine->b_rise_seen))
        {
            uint16_t distance_mm = 0u;
            bool const b_in_time = (age_us <= g_scan_deadline_us);
            bool const b_valid   = b_in_time
                && range_measurement_from_echo(
                       p_event->time_us - p_engine->rise_us, &distance_mm);

            scan_engine_complete(p_engine, b_valid, distance_mm);
        }

        /* A fall without its rise is malformed and is ignored. */
    }

}

static void
scan_engine_complete (scan_engine_t * p_engine, bool b_valid,
                      uint16_t distance_mm)
{
    p_engine->b_capture_open = false;

    if (b_valid)
    {
        p_engine->invalid_count = 0u;

        if (p_engine->b_awaiting_clearance)
        {
            if (distance_mm >= p_engine->config.hazard_mm)
            {
                if (!scan_engine_publish(p_engine, SCAN_ENGINE_PLAN,
                                         g_scan_forward_angle_deg,
                                         distance_mm, true))
                {
                    scan_engine_fail(p_engine);
                }
                else
                {
                    scan_engine_abort(p_engine);
                }
            }
            else
            {
                scan_engine_fail(p_engine);
            }
        }
        else
        {
            p_engine->samples[p_engine->sample_count] = distance_mm;
            p_engine->sample_count++;

            if (SCAN_ENGINE_SAMPLES <= p_engine->sample_count)
            {
                scan_engine_next_angle(p_engine);
            }
        }

    }
    else
    {
        p_engine->invalid_count++;

        if (g_scan_invalid_limit <= p_engine->invalid_count)
        {
            scan_engine_fail(p_engine);
        }

    }

}

static void
scan_engine_next_angle (scan_engine_t * p_engine)
{
    uint16_t const distance_mm = scan_engine_median(p_engine->samples);

    (void) scan_engine_publish(p_engine, SCAN_ENGINE_DISTANCE,
                               scan_engine_angle(p_engine), distance_mm, true);

    p_engine->sample_count = 0u;

    if (p_engine->b_fine_scanning)
    {
        p_engine->fine_index++;

        if (p_engine->fine_index >= p_engine->fine_count)
        {
            p_engine->b_awaiting_clearance = true;
        }
    }
    else
    {
        p_engine->coarse_mm[p_engine->angle_index] = distance_mm;
        p_engine->angle_index++;

        if (p_engine->angle_index >= SCAN_ENGINE_ANGLES)
        {
            scan_engine_prepare_fine(p_engine);
            p_engine->b_fine_scanning = (p_engine->fine_count > 0u);
            p_engine->b_awaiting_clearance = !p_engine->b_fine_scanning;
        }
    }

    /* Return forward, settle and measure fresh clearance before a plan. */
    if (scan_engine_publish(p_engine, SCAN_ENGINE_SERVO,
                            scan_engine_angle(p_engine), 0u, true))
    {
        p_engine->settling_started_us = p_engine->port.p_time_now_us();
    }
    else
    {
        scan_engine_fail(p_engine);
    }

}

static void
scan_engine_prepare_fine (scan_engine_t * p_engine)
{
    uint32_t const pass_mm =
        (uint32_t)p_engine->coarse_mm[SCAN_ENGINE_ANGLES / 2u]
        + (uint32_t)p_engine->config.depth_mm;

    p_engine->fine_count = 0u;
    p_engine->fine_index = 0u;

    for (uint8_t index = 0u; index < (SCAN_ENGINE_ANGLES - 1u); index++)
    {
        bool const b_first_blocked = (p_engine->coarse_mm[index] <= pass_mm);
        bool const b_next_blocked =
            (p_engine->coarse_mm[index + 1u] <= pass_mm);

        if (b_first_blocked != b_next_blocked)
        {
            int16_t const base_deg = (int16_t)(g_scan_first_angle_deg
                                     + ((int16_t)index * g_scan_step_deg));

            p_engine->fine_angles[p_engine->fine_count] =
                (int16_t)(base_deg + 5);
            p_engine->fine_count++;
            p_engine->fine_angles[p_engine->fine_count] =
                (int16_t)(base_deg + 10);
            p_engine->fine_count++;
        }
    }
}

static void
scan_engine_fail (scan_engine_t * p_engine)
{
    (void) scan_engine_publish(p_engine, SCAN_ENGINE_FAULT,
                               scan_engine_angle(p_engine), 0u, true);
    scan_engine_abort(p_engine);
}

static int16_t
scan_engine_angle (scan_engine_t const * p_engine)
{
    /* The cast is safe: the index is below SCAN_ENGINE_ANGLES. */
    int16_t angle_deg = g_scan_forward_angle_deg;

    if (!p_engine->b_awaiting_clearance)
    {
        if (p_engine->b_fine_scanning)
        {
            angle_deg = p_engine->fine_angles[p_engine->fine_index];
        }
        else
        {
            angle_deg = (int16_t) (g_scan_first_angle_deg
                        + ((int16_t) p_engine->angle_index
                           * g_scan_step_deg));
        }
    }

    return (angle_deg);
}

static uint16_t
scan_engine_median (uint16_t const * p_samples)
{
    uint16_t const a = p_samples[0];
    uint16_t const b = p_samples[1];
    uint16_t const c = p_samples[2];
    uint16_t       median = c;

    if (((a <= b) && (b <= c)) || ((c <= b) && (b <= a)))
    {
        median = b;
    }
    else if (((b <= a) && (a <= c)) || ((c <= a) && (a <= b)))
    {
        median = a;
    }
    else
    {
        /* c is the middle value. */
    }

    return (median);
}

static bool
scan_engine_publish (scan_engine_t const * p_engine,
                     scan_engine_output_kind_t kind,
                     int16_t angle_deg,
                     uint16_t distance_mm,
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
            distance_mm,
            b_asserted
        };

        b_is_published = p_engine->port.p_publish(&output);
    }

    return (b_is_published);
}

/* End of file. */
