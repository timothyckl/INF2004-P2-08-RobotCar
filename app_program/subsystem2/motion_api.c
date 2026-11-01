/**
 * @file motion_api.c
 * @brief Subsystem 2 Motion Control API (see motion_api.h).
 *
 * Builds commands and sends them to motion_q with TMO_POL. Status reads
 * (motion_get_status, motion_stop_confirmed) are in task_motion.c, next
 * to the data they copy.
 */
#include "motion_api.h"

#include "motion_cmd.h"
#include "motion_config.h"

static uint16_t g_next_id = 1u;     /* Vehicle Controller task only */
static uint16_t g_last_id = 0u;

static ER motion_send (uint8_t cmd, int32_t value);

ER
motion_move_forward (uint16_t distance_mm)
{
    return (motion_send(S2_CMD_MOVE, (int32_t)distance_mm));
}

ER
motion_move_backward (uint16_t distance_mm)
{
    return (motion_send(S2_CMD_MOVE, -(int32_t)distance_mm));
}

ER
motion_turn_left (uint32_t angle_mdeg)
{
    return ((angle_mdeg > MOTION_ANGLE_MAX_MDEG) ? E_PAR
            : motion_send(S2_CMD_TURN, (int32_t)angle_mdeg));
}

ER
motion_turn_right (uint32_t angle_mdeg)
{
    return ((angle_mdeg > MOTION_ANGLE_MAX_MDEG) ? E_PAR
            : motion_send(S2_CMD_TURN, -(int32_t)angle_mdeg));
}

ER
motion_stop (void)
{
    return (motion_send(S2_CMD_STOP, 0));
}

ER
motion_reset (void)
{
    return (motion_send(S2_CMD_RESET, 0));
}

ER
motion_new_run (void)
{
    return (motion_send(S2_CMD_NEW_RUN, 0));
}

uint16_t
motion_last_command_id (void)
{
    return (g_last_id);
}

/**
 * @brief Build and range-check one command.
 *
 * MOVE: 1..MOTION_DISTANCE_MAX_MM either way, at MC_MOVE_SPEED_MM_S.
 * TURN: 1..MOTION_ANGLE_MAX_MDEG either way, at MC_TURN_SPEED_MM_S.
 */
bool
motion_api_make (uint8_t cmd, int32_t value, uint16_t cmd_id,
                 s2_motion_cmd_t * p_cmd)
{
    s2_motion_cmd_t c = {0};
    int32_t const   mag = (value < 0) ? -value : value;

    if ((NULL == p_cmd) || (INT32_MIN == value))
    {
        return (false);
    }

    c.cmd    = cmd;
    c.cmd_id = cmd_id;

    if (S2_CMD_MOVE == cmd)
    {
        if ((0 == mag) || (mag > (int32_t)MOTION_DISTANCE_MAX_MM))
        {
            return (false);
        }

        c.u.move.distance_mm = (int16_t)value;
        c.u.move.speed_mm_s  = (int16_t)MC_MOVE_SPEED_MM_S;
    }
    else if (S2_CMD_TURN == cmd)
    {
        if ((0 == mag) || (mag > (int32_t)MOTION_ANGLE_MAX_MDEG))
        {
            return (false);
        }

        c.u.turn.angle_mdeg = value;
        c.u.turn.speed_mm_s = (int16_t)MC_TURN_SPEED_MM_S;
    }
    else if ((S2_CMD_STOP != cmd) && (S2_CMD_RESET != cmd)
             && (S2_CMD_NEW_RUN != cmd))
    {
        return (false);     /* SPEED has no API function yet */
    }
    else
    {
        /* No payload. */
    }

    *p_cmd = c;

    return (true);
}

static ER
motion_send (uint8_t cmd, int32_t value)
{
    s2_motion_cmd_t c;
    SYSTIM          tim;

    if (!motion_api_make(cmd, value, g_next_id, &c))
    {
        return (E_PAR);
    }

    if (motion_q <= 0)
    {
        return (E_OBJ);
    }

    c.time_ms = (tk_get_tim(&tim) >= E_OK) ? (uint32_t)tim.lo : 0u;

    ER const er = tk_snd_mbf(motion_q, &c, (INT)sizeof(c), TMO_POL);

    if (er < E_OK)
    {
        return (er);
    }

    g_last_id = g_next_id;
    g_next_id = (uint16_t)(g_next_id + 1u);

    return (E_OK);
}

/* End of file. */
