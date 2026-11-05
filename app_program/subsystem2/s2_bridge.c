/**
 * @file s2_bridge.c
 * @brief Subsystem 2 adapters for subsystem 1's service_bridge, and the
 *        remote_q record builder.
 *
 * Runs in the WiFi service task: no call here waits. Every function reads
 * an object ID once into a local and fails (returns false) while it is 0,
 * so calls before subsystem2_init() are harmless.
 */
#include "s2_bridge.h"

#include "task_controller.h"

#define S2_LOCAL_ID_BASE    0x80000001u /* local IDs: top bit set */

static volatile uint32_t g_stop_ms    = 0u;  /* written by the WiFi task */
static volatile uint32_t g_stop_count = 0u;
static uint32_t          g_next_request = 1u; /* WiFi task only */
static uint32_t          g_next_local   = S2_LOCAL_ID_BASE; /* one task */

bool
controller_raise_remote_stop (void * p_context, uint32_t received_ms)
{
    (void)p_context;

    return (s2_bridge_raise_stop(controller_flags, received_ms));
}

bool
controller_send_remote_command (void * p_context, uint8_t command,
                                uint32_t received_ms)
{
    (void)p_context;

    if (!s2_bridge_send_command(remote_q, command, g_next_request,
                                received_ms))
    {
        return (false);
    }

    g_next_request++;

    if (0u == g_next_request)
    {
        g_next_request = 1u;    /* 0 is never a valid request ID */
    }

    return (true);
}

/* Store the metadata first: setting the flag publishes it. */
bool
s2_bridge_raise_stop (ID flg, uint32_t received_ms)
{
    if (flg <= 0)
    {
        return (false);
    }

    g_stop_ms    = received_ms;
    g_stop_count = g_stop_count + 1u;

    return (tk_set_flg(flg, CTRL_FLG_REMOTE_STOP) >= E_OK);
}

/* A full queue drops the command: it is never retried later. */
bool
s2_bridge_send_command (ID q, uint8_t command, uint32_t request_id,
                        uint32_t received_ms)
{
    s2_remote_cmd_t msg;

    return ((q > 0) && s2_remote_make(command, request_id, received_ms, &msg)
            && (tk_snd_mbf(q, &msg, (INT)sizeof(msg), TMO_POL) >= E_OK));
}

bool
s2_local_stop (void)
{
    ID const flg = controller_flags;

    return ((flg > 0) && (tk_set_flg(flg, CTRL_FLG_LOCAL_STOP) >= E_OK));
}

bool
s2_local_command (uint8_t command)
{
    SYSTIM tim;
    uint32_t const t = (tk_get_tim(&tim) >= E_OK) ? (uint32_t)tim.lo : 0u;

    if (!s2_bridge_send_command(remote_q, command, g_next_local, t))
    {
        return (false);
    }

    g_next_local++;

    if (g_next_local < S2_LOCAL_ID_BASE)
    {
        g_next_local = S2_LOCAL_ID_BASE;    /* stay in the local range */
    }

    return (true);
}

bool
s2_line_make (uint8_t position, uint32_t captured_ms,
              s2_line_report_t * p_report)
{
    if ((NULL == p_report) || (position > S2_LINE_STOP))
    {
        return (false);
    }

    p_report->captured_ms = captured_ms;
    p_report->position    = position;
    p_report->reserved[0] = 0u;
    p_report->reserved[1] = 0u;
    p_report->reserved[2] = 0u;

    return (true);
}

bool
s2_line_submit (uint8_t position, uint32_t captured_ms)
{
    s2_line_report_t report;
    ID const q = line_q;

    return ((q > 0) && s2_line_make(position, captured_ms, &report)
            && (tk_snd_mbf(q, &report, (INT)sizeof(report), TMO_POL)
                >= E_OK));
}

bool
s2_line_start (void)
{
    return (s2_local_command(S2_REMOTE_START));
}

bool
s2_line_stop (void)
{
    return (s2_local_stop());
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
