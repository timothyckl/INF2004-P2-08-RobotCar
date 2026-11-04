/**
 * @file line_control.c
 * @brief Implement pure line-following and bounded recovery policy.
 */
#include "line_control.h"

#include "subsystem2.h"

static line_control_action_t line_control_follow_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_crossing_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_search_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_correction_action(uint8_t position);
static line_control_action_t line_control_pivot_action(uint8_t position);

bool
line_control_init (line_control_t * p_control,
                   line_control_cfg_t const * p_cfg,
                   uint32_t now_ms)
{
    bool b_valid = false;

    if ((NULL != p_control) && (NULL != p_cfg))
    {
        b_valid = (0u != p_cfg->crossing_limit_ms)
                  && (0u != p_cfg->correction_limit_ms)
                  && (0u != p_cfg->search_limit_ms);

        p_control->cfg = *p_cfg;
        p_control->mode = b_valid ? LINE_CONTROL_MODE_FOLLOW
                                  : LINE_CONTROL_MODE_INVALID;
        p_control->mode_since_ms = now_ms;
        p_control->correction_since_ms = now_ms;
        p_control->last_correction = S2_LINE_CENTER;
        p_control->b_correction_active = false;
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

            case LINE_CONTROL_MODE_CROSSING:
                action = line_control_crossing_step(p_control, position,
                                                    now_ms);
            break;

            case LINE_CONTROL_MODE_SEARCH:
                action = line_control_search_step(p_control, position, now_ms);
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
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else if ((S2_LINE_LEFT == position) || (S2_LINE_RIGHT == position))
    {
        if ((!p_control->b_correction_active)
            || (position != p_control->last_correction))
        {
            p_control->correction_since_ms = now_ms;
            p_control->last_correction = position;
            p_control->b_correction_active = true;
        }

        if ((now_ms - p_control->correction_since_ms)
            >= p_control->cfg.correction_limit_ms)
        {
            p_control->mode = LINE_CONTROL_MODE_SEARCH;
            p_control->mode_since_ms = now_ms;
            action = line_control_pivot_action(position);
        }
        else
        {
            action = line_control_correction_action(position);
        }
    }
    else if (S2_LINE_BOTH_BLACK == position)
    {
        p_control->mode = LINE_CONTROL_MODE_CROSSING;
        p_control->mode_since_ms = now_ms;
        p_control->b_correction_active = false;
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else
    {
        action = LINE_CONTROL_ACTION_STOP;
    }

    return (action);
}

static line_control_action_t
line_control_crossing_step (line_control_t * p_control, uint8_t position,
                            uint32_t now_ms)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_BOTH_BLACK != position)
    {
        p_control->mode = LINE_CONTROL_MODE_FOLLOW;
        p_control->mode_since_ms = now_ms;
        action = line_control_follow_step(p_control, position, now_ms);
    }
    else if ((now_ms - p_control->mode_since_ms)
             < p_control->cfg.crossing_limit_ms)
    {
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else
    {
        action = LINE_CONTROL_ACTION_STOP;
    }

    return (action);
}

static line_control_action_t
line_control_search_step (line_control_t * p_control, uint8_t position,
                          uint32_t now_ms)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_CENTER == position)
    {
        p_control->mode = LINE_CONTROL_MODE_FOLLOW;
        p_control->mode_since_ms = now_ms;
        p_control->b_correction_active = false;
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else if (S2_LINE_BOTH_BLACK == position)
    {
        p_control->mode = LINE_CONTROL_MODE_CROSSING;
        p_control->mode_since_ms = now_ms;
        p_control->b_correction_active = false;
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else if ((S2_LINE_LEFT == position) || (S2_LINE_RIGHT == position))
    {
        if (position != p_control->last_correction)
        {
            p_control->mode = LINE_CONTROL_MODE_FOLLOW;
            p_control->mode_since_ms = now_ms;
            p_control->b_correction_active = false;
            action = line_control_follow_step(p_control, position, now_ms);
        }
        else if ((now_ms - p_control->mode_since_ms)
                 < p_control->cfg.search_limit_ms)
        {
            action = line_control_pivot_action(position);
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
line_control_pivot_action (uint8_t position)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_LEFT == position)
    {
        action = LINE_CONTROL_ACTION_PIVOT_LEFT;
    }
    else if (S2_LINE_RIGHT == position)
    {
        action = LINE_CONTROL_ACTION_PIVOT_RIGHT;
    }
    else
    {
        /* A search direction must come from the last correction. */
    }

    return (action);
}

/* End of file. */
