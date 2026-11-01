/**
 * @file s2_bridge.c
 * @brief Subsystem 2 adapters for subsystem 1's service_bridge, and the
 *        remote_q record builder.
 *
 * Runs in the WiFi service task: no call here waits. Every function reads
 * an object ID once into a local and fails (returns false) while it is 0,
 * so calls before subsystem2_init() are harmless.
 */
#include "subsystem2.h"

static volatile uint32_t g_stop_ms    = 0u;  /* written by the WiFi task */
static volatile uint32_t g_stop_count = 0u;
static uint32_t          g_next_request = 1u; /* WiFi task only */

bool
controller_raise_remote_stop (void * p_context, uint32_t received_ms)
{
    ID const flg = controller_flags;

    (void)p_context;

    if (flg <= 0)
    {
        return (false);
    }

    /* Store the metadata first: setting the flag publishes it. */
    g_stop_ms    = received_ms;
    g_stop_count = g_stop_count + 1u;

    return (tk_set_flg(flg, CTRL_FLG_REMOTE_STOP) >= E_OK);
}

bool
controller_send_remote_command (void * p_context, uint8_t command,
                                uint32_t received_ms)
{
    ID const        q = remote_q;
    s2_remote_cmd_t msg;

    (void)p_context;

    if ((q <= 0) || (!s2_remote_make(command, g_next_request, received_ms,
                                     &msg)))
    {
        return (false);
    }

    if (tk_snd_mbf(q, &msg, (INT)sizeof(msg), TMO_POL) < E_OK)
    {
        return (false);     /* full: dropped, never retried later */
    }

    g_next_request++;

    if (0u == g_next_request)
    {
        g_next_request = 1u;    /* 0 is never a valid request ID */
    }

    return (true);
}

bool
s2_remote_stop_info (uint32_t * p_received_ms, uint32_t * p_count)
{
    if ((NULL == p_received_ms) || (NULL == p_count))
    {
        return (false);
    }

    *p_received_ms = g_stop_ms;
    *p_count       = g_stop_count;

    return (true);
}

bool
s2_remote_make (uint8_t command, uint32_t request_id, uint32_t received_ms,
                s2_remote_cmd_t * p_msg)
{
    if ((NULL == p_msg) || (0u == request_id)
        || ((S2_REMOTE_START != command) && (S2_REMOTE_RESET != command)))
    {
        return (false);
    }

    p_msg->request_id  = request_id;
    p_msg->received_ms = received_ms;
    p_msg->command     = command;
    p_msg->reserved[0] = 0u;
    p_msg->reserved[1] = 0u;
    p_msg->reserved[2] = 0u;

    return (true);
}

/* End of file. */
