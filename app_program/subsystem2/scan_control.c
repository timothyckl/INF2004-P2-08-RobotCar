/**
 * @file scan_control.c
 * @brief Implement the pure Scan & Plan handshake (see scan_control.h).
 */
#include "scan_control.h"

#include "subsystem2.h"

static void scan_control_clear(scan_control_output_t * p_out);
static void scan_control_push(scan_control_output_t * p_out, uint8_t kind,
                              bool b_asserted);
static void scan_control_finish(scan_control_t * p_scan,
                                scan_control_output_t * p_out,
                                scan_control_outcome_t outcome);

bool
scan_control_init (scan_control_t * p_scan, uint32_t limit_ms)
{
    bool const b_valid = (NULL != p_scan) && (0u != limit_ms);

    if (b_valid)
    {
        p_scan->limit_ms = limit_ms;
        p_scan->request_id = 0u;
        p_scan->since_ms = 0u;
        p_scan->state = SCAN_CONTROL_IDLE;
    }

    return (b_valid);
}

bool
scan_control_active (scan_control_t const * p_scan)
{
    return ((NULL != p_scan) && (SCAN_CONTROL_IDLE != p_scan->state));
}

void
scan_control_begin (scan_control_t * p_scan, uint32_t now_ms,
                    scan_control_output_t * p_out)
{
    if (NULL == p_out)
    {
        return;
    }

    scan_control_clear(p_out);

    if ((NULL == p_scan) || scan_control_active(p_scan))
    {
        return;
    }

    p_scan->request_id++;

    if (0u == p_scan->request_id)
    {
        p_scan->request_id = 1u;    /* request IDs are never 0 */
    }

    p_scan->since_ms = now_ms;
    p_scan->state = SCAN_CONTROL_STOPPING;
    p_out->b_stop_motion = true;
    scan_control_push(p_out, S2_SCAN_REQUEST, true);
}

void
scan_control_step (scan_control_t * p_scan, bool b_still, bool b_result_ok,
                   uint32_t result_id, uint8_t result, uint32_t now_ms,
                   scan_control_output_t * p_out)
{
    if (NULL == p_out)
    {
        return;
    }

    scan_control_clear(p_out);

    if (!scan_control_active(p_scan))
    {
        return;
    }

    if ((SCAN_CONTROL_STOPPING == p_scan->state) && b_still)
    {
        scan_control_push(p_out, S2_SCAN_STOP_CONFIRM, true);
        scan_control_push(p_out, S2_SCAN_AUTHORISE, true);
        p_scan->state = SCAN_CONTROL_SCANNING;
    }

    if ((SCAN_CONTROL_SCANNING == p_scan->state) && b_result_ok
        && (result_id == p_scan->request_id)
        && (SCAN_CONTROL_RESULT_NONE != result))
    {
        if (SCAN_CONTROL_RESULT_PLAN == result)
        {
            scan_control_finish(p_scan, p_out, SCAN_CONTROL_PLAN);
        }
        else if (SCAN_CONTROL_RESULT_CLEAR == result)
        {
            scan_control_finish(p_scan, p_out, SCAN_CONTROL_CLEAR);
        }
        else
        {
            scan_control_finish(p_scan, p_out, SCAN_CONTROL_FAILED);
        }
    }
    else if ((now_ms - p_scan->since_ms) > p_scan->limit_ms)
    {
        scan_control_push(p_out, S2_SCAN_CANCEL, true);
        scan_control_finish(p_scan, p_out, SCAN_CONTROL_TIMEOUT);
    }
    else
    {
        /* Waiting for stillness or for subsystem 5's result. */
    }
}

void
scan_control_abort (scan_control_t * p_scan, scan_control_output_t * p_out)
{
    if (NULL == p_out)
    {
        return;
    }

    scan_control_clear(p_out);

    if (scan_control_active(p_scan))
    {
        scan_control_push(p_out, S2_SCAN_STOP, true);
        scan_control_finish(p_scan, p_out, SCAN_CONTROL_FAILED);
    }
}

static void
scan_control_clear (scan_control_output_t * p_out)
{
    p_out->n_msg = 0u;
    p_out->b_stop_motion = false;
    p_out->outcome = SCAN_CONTROL_NONE;
}

static void
scan_control_push (scan_control_output_t * p_out, uint8_t kind,
                   bool b_asserted)
{
    if (p_out->n_msg < SCAN_CONTROL_MSGS_MAX)
    {
        p_out->msg[p_out->n_msg].kind = kind;
        p_out->msg[p_out->n_msg].b_asserted = b_asserted;
        p_out->n_msg++;
    }
}

static void
scan_control_finish (scan_control_t * p_scan, scan_control_output_t * p_out,
                     scan_control_outcome_t outcome)
{
    p_scan->state = SCAN_CONTROL_IDLE;
    p_out->outcome = outcome;
}

/* End of file. */
