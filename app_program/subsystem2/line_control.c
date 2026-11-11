/**
 * @file line_control.c
 * @brief Implement pure two-sensor line following and bounded recovery.
 */
#include "line_control.h"

#include "subsystem2.h"

static line_control_action_t line_control_follow_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_lost_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_correction_action(uint8_t position);
static line_control_action_t line_control_curve_action(uint8_t position);
static line_control_action_t line_control_direction_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static bool line_control_has_recent_direction(
    line_control_t const * p_control, uint32_t now_ms);

bool
line_control_init (line_control_t * p_control,
                   line_control_cfg_t const * p_cfg,
                   uint32_t now_ms)
{
    bool b_valid = false;

    if ((NULL != p_control) && (NULL != p_cfg))
    {
        b_valid = (0u != p_cfg->curve_entry_ms)
                  && (0u != p_cfg->lost_limit_ms)
                  && (0u != p_cfg->direction_memory_ms)
                  && (0u != p_cfg->direction_change_ms);

        p_control->cfg = *p_cfg;
        p_control->mode = b_valid ? LINE_CONTROL_MODE_FOLLOW
                                  : LINE_CONTROL_MODE_INVALID;
        p_control->mode_since_ms = now_ms;
        p_control->correction_since_ms = now_ms;
        p_control->last_direction_ms = now_ms;
        p_control->pending_since_ms = now_ms;
        p_control->last_correction = S2_LINE_CENTER;
        p_control->pending_correction = S2_LINE_CENTER;
        p_control->b_correction_active = false;
        p_control->b_pending_correction = false;
        p_control->b_valid = b_valid;
    }

    return (b_valid);
}

line_control_action_t
line_control_step (line_control_t * p_control, uint8_t position,
                   uint32_t now_ms)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if ((NULL != p_control) && p_control->b_valid)
    {
        switch (p_control->mode)
        {
            case LINE_CONTROL_MODE_FOLLOW:
                action = line_control_follow_step(p_control, position, now_ms);
            break;

            case LINE_CONTROL_MODE_LOST_RECOVERY:
                action = line_control_lost_step(p_control, position, now_ms);
            break;

            case LINE_CONTROL_MODE_INVALID:
            default:
                p_control->b_valid = false;
                action = LINE_CONTROL_ACTION_STOP;
            break;
        }
    }

    return (action);
}

line_control_mode_t
line_control_mode (line_control_t const * p_control)
{
    line_control_mode_t mode = LINE_CONTROL_MODE_INVALID;

    if ((NULL != p_control) && p_control->b_valid)
    {
        mode = p_control->mode;
    }

    return (mode);
}

static line_control_action_t
line_control_follow_step (line_control_t * p_control, uint8_t position,
                          uint32_t now_ms)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_CENTER == position)
    {
        p_control->b_correction_active = false;
        p_control->b_pending_correction = false;
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else if ((S2_LINE_LEFT == position) || (S2_LINE_RIGHT == position))
    {
        action = line_control_direction_step(p_control, position, now_ms);
    }
    else if (S2_LINE_LOST == position)
    {
        if (line_control_has_recent_direction(p_control, now_ms))
        {
            p_control->mode = LINE_CONTROL_MODE_LOST_RECOVERY;
            p_control->mode_since_ms = now_ms;
            action = line_control_curve_action(p_control->last_correction);
        }
        else
        {
            action = LINE_CONTROL_ACTION_STOP;
        }
    }
    else
    {
        action = LINE_CONTROL_ACTION_STOP;
    }

    return (action);
}

static line_control_action_t
line_control_direction_step (line_control_t * p_control, uint8_t position,
                             uint32_t now_ms)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STRAIGHT;
    bool const b_previous_direction =
        (S2_LINE_LEFT == p_control->last_correction)
        || (S2_LINE_RIGHT == p_control->last_correction);
    bool const b_direction_change = b_previous_direction
                                    && (position
                                        != p_control->last_correction);
    bool b_direction_ready = !b_direction_change;

    if (b_direction_change)
    {
        if ((!p_control->b_pending_correction)
            || (position != p_control->pending_correction))
        {
            p_control->pending_correction = position;
            p_control->pending_since_ms = now_ms;
            p_control->b_pending_correction = true;
        }

        b_direction_ready = ((now_ms - p_control->pending_since_ms)
                             >= p_control->cfg.direction_change_ms);
    }

    if (b_direction_ready)
    {
        if ((!p_control->b_correction_active)
            || (position != p_control->last_correction))
        {
            p_control->correction_since_ms = now_ms;
            p_control->b_correction_active = true;
        }

        p_control->last_correction = position;
        p_control->last_direction_ms = now_ms;
        p_control->b_pending_correction = false;

        if ((now_ms - p_control->correction_since_ms)
            >= p_control->cfg.curve_entry_ms)
        {
            action = line_control_curve_action(position);
        }
        else
        {
            action = line_control_correction_action(position);
        }
    }
    else
    {
        p_control->b_correction_active = false;
    }

    return (action);
}

static line_control_action_t
line_control_lost_step (line_control_t * p_control, uint8_t position,
                        uint32_t now_ms)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_LOST != position)
    {
        p_control->mode = LINE_CONTROL_MODE_FOLLOW;
        p_control->mode_since_ms = now_ms;
        action = line_control_follow_step(p_control, position, now_ms);
    }
    else if ((now_ms - p_control->mode_since_ms)
             < p_control->cfg.lost_limit_ms)
    {
        action = line_control_curve_action(p_control->last_correction);
    }
    else
    {
        action = LINE_CONTROL_ACTION_STOP;
    }

    return (action);
}

static line_control_action_t
line_control_correction_action (uint8_t position)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_LEFT == position)
    {
        action = LINE_CONTROL_ACTION_CORRECT_LEFT;
    }
    else if (S2_LINE_RIGHT == position)
    {
        action = LINE_CONTROL_ACTION_CORRECT_RIGHT;
    }
    else
    {
        /* Only a single-black observation can request a correction. */
    }

    return (action);
}

static line_control_action_t
line_control_curve_action (uint8_t position)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_LEFT == position)
    {
        action = LINE_CONTROL_ACTION_CURVE_LEFT;
    }
    else if (S2_LINE_RIGHT == position)
    {
        action = LINE_CONTROL_ACTION_CURVE_RIGHT;
    }
    else
    {
        /* Recovery direction must come from a single-black observation. */
    }

    return (action);
}

static bool
line_control_has_recent_direction (line_control_t const * p_control,
                                   uint32_t now_ms)
{
    bool const b_direction = (S2_LINE_LEFT == p_control->last_correction)
                             || (S2_LINE_RIGHT
                                 == p_control->last_correction);

    return (b_direction
            && ((now_ms - p_control->last_direction_ms)
                <= p_control->cfg.direction_memory_ms));
}

/* End of file. */
