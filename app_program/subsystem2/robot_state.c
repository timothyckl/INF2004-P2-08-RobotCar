/**
 * @file robot_state.c
 * @brief robot_state under state_mutex (see robot_state.h, subsystem2.h).
 *
 * state_mutex uses priority inheritance and is held only for one struct
 * copy, so a reader (including the higher-priority WiFi task) can never
 * hold up the controller for long. The WiFi task's reader never waits.
 */
#include "robot_state.h"

#define S2_STATE_LOCK_TMO_MS    10      /* task readers, not the WiFi task */
#define S2_DISTANCE_MAX_MM      65535   /* uint16_t telemetry field */

static ID               g_state_mtx = 0;
static s2_robot_state_t g_state;        /* guarded by g_state_mtx */

ER
s2_robot_state_create (void)
{
    T_CMTX         cmtx = {0};
    s2_robot_state_t const boot = {0u, 0, 0u, S2_SYS_INITIALISING,
                                   S2_FAULT_NONE, S2_RS_STOP_LATCHED,
                                   0u, S2_RESULT_NONE, 0u, S2_MODE_NONE, 0u};

    g_state      = boot;
    cmtx.mtxatr  = TA_INHERIT;

    ID const mtx = tk_cre_mtx(&cmtx);

    if (mtx < E_OK)
    {
        return (mtx);
    }

    g_state_mtx = mtx;      /* published only once it exists */

    return (E_OK);
}

void
s2_robot_state_publish (s2_robot_state_t const * p_state)
{
    if ((NULL != p_state) && (g_state_mtx > 0)
        && (tk_loc_mtx(g_state_mtx, S2_STATE_LOCK_TMO_MS) >= E_OK))
    {
        g_state = *p_state;
        (void)tk_unl_mtx(g_state_mtx);
    }
}

ER
s2_get_robot_state (s2_robot_state_t * p_state)
{
    ID const mtx = g_state_mtx;

    if (NULL == p_state)
    {
        return (E_PAR);
    }

    if (mtx <= 0)
    {
        return (E_OBJ);
    }

    ER const er = tk_loc_mtx(mtx, S2_STATE_LOCK_TMO_MS);

    if (er < E_OK)
    {
        return (er);
    }

    *p_state = g_state;
    (void)tk_unl_mtx(mtx);

    return (E_OK);
}

/* WiFi task: never waits; a busy lock skips one telemetry publication. */
bool
s2_read_telemetry (s2_telemetry_t * p_out)
{
    ID const         mtx = g_state_mtx;
    s2_robot_state_t st;

    if ((NULL == p_out) || (mtx <= 0) || (tk_loc_mtx(mtx, TMO_POL) < E_OK))
    {
        return (false);
    }

    st = g_state;
    (void)tk_unl_mtx(mtx);
    s2_telemetry_from_state(&st, p_out);

    return (true);
}

void
s2_telemetry_from_state (s2_robot_state_t const * p_state,
                         s2_telemetry_t * p_out)
{
    if ((NULL == p_state) || (NULL == p_out))
    {
        return;
    }

    int32_t const d = p_state->run_distance_mm;
    bool const    b_known = (0u != (p_state->flags & S2_RS_DISTANCE_VALID));

    p_out->captured_ms      = p_state->captured_ms;
    p_out->b_distance_valid = b_known && (d >= 0)
                              && (d <= S2_DISTANCE_MAX_MM);
    p_out->distance_mm      = (!b_known || (d < 0)) ? 0u
                              : ((d > S2_DISTANCE_MAX_MM)
                                 ? (uint16_t)S2_DISTANCE_MAX_MM
                                 : (uint16_t)d);
    p_out->b_stopped        = (0u != (p_state->flags & S2_RS_STOPPED));
}

/* End of file. */
