/**
 * @file line_control.c
 * @brief Implement two-sensor line following and bounded junction handling.
 */
#include "line_control.h"

#include "subsystem2.h"

static line_control_action_t line_control_follow_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_confirm_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_crossing_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_exit_step(
    line_control_t * p_control, uint8_t position, uint32_t now_ms);
static line_control_action_t line_control_direction_action(uint8_t position,
                                                           bool b_curve);
static uint32_t line_control_center_hold_ms(
    line_control_t const * p_control);

bool
line_control_init (line_control_t * p_control,
                   line_control_cfg_t const * p_cfg,
                   uint32_t now_ms)
{
    bool b_valid = false;

    if ((NULL != p_control) && (NULL != p_cfg))
    {
        b_valid = (0u != p_cfg->curve_entry_ms)
                  && (0u != p_cfg->left_center_hold_ms)
                  && (0u != p_cfg->right_center_hold_ms)
                  && (0u != p_cfg->junction_confirm_ms)
                  && (0u != p_cfg->crossing_limit_ms)
                  && (0u != p_cfg->exit_min_ms)
                  && (0u != p_cfg->exit_center_ms)
                  && (0u != p_cfg->exit_side_ms)
                  && (p_cfg->exit_min_ms < p_cfg->exit_limit_ms)
                  && (p_cfg->exit_center_ms
                      <= (p_cfg->exit_limit_ms - p_cfg->exit_min_ms))
                  && (p_cfg->exit_side_ms
                      <= (p_cfg->exit_limit_ms - p_cfg->exit_min_ms));

        p_control->cfg = *p_cfg;
        p_control->mode = b_valid ? LINE_CONTROL_MODE_FOLLOW
                                  : LINE_CONTROL_MODE_INVALID;
        p_control->held_action = LINE_CONTROL_ACTION_STRAIGHT;
        p_control->mode_since_ms = now_ms;
        p_control->correction_since_ms = now_ms;
        p_control->curve_seen_ms = now_ms;
        p_control->center_since_ms = now_ms;
        p_control->side_since_ms = now_ms;
        p_control->last_correction = S2_LINE_CENTER;
        p_control->exit_side = S2_LINE_CENTER;
        p_control->b_correction_active = false;
        p_control->b_curve_active = false;
        p_control->b_center_candidate = false;
        p_control->b_side_candidate = false;
        p_control->b_valid = b_valid;
    }

    return (b_valid);
}

line_control_action_t
line_control_step (line_control_t * p_control, uint8_t position,
                   uint32_t now_ms)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if ((NULL != p_control) && p_control->b_valid
        && (position < S2_LINE_STOP))
    {
        switch (p_control->mode)
        {
            case LINE_CONTROL_MODE_FOLLOW:
                action = line_control_follow_step(p_control, position, now_ms);
            break;

            case LINE_CONTROL_MODE_JUNCTION_CONFIRM:
                action = line_control_confirm_step(p_control, position,
                                                   now_ms);
            break;

            case LINE_CONTROL_MODE_CROSSING:
                action = line_control_crossing_step(p_control, position,
                                                    now_ms);
            break;

            case LINE_CONTROL_MODE_EXIT:
                action = line_control_exit_step(p_control, position, now_ms);
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
        if (p_control->b_curve_active
            && ((now_ms - p_control->curve_seen_ms)
                < line_control_center_hold_ms(p_control)))
        {
            action = line_control_direction_action(
                p_control->last_correction, true);
        }
        else
        {
            p_control->b_correction_active = false;
            p_control->b_curve_active = false;
            action = LINE_CONTROL_ACTION_STRAIGHT;
        }
    }
    else if ((S2_LINE_LEFT == position) || (S2_LINE_RIGHT == position))
    {
        if ((!p_control->b_correction_active)
            || (position != p_control->last_correction))
        {
            p_control->correction_since_ms = now_ms;
            p_control->last_correction = position;
            p_control->b_correction_active = true;
            p_control->b_curve_active = false;
        }

        if ((now_ms - p_control->correction_since_ms)
            >= p_control->cfg.curve_entry_ms)
        {
            p_control->curve_seen_ms = now_ms;
            p_control->b_curve_active = true;
            action = line_control_direction_action(position, true);
        }
        else
        {
            action = line_control_direction_action(position, false);
        }
    }
    else if (S2_LINE_BOTH_BLACK == position)
    {
        p_control->mode = LINE_CONTROL_MODE_JUNCTION_CONFIRM;
        p_control->mode_since_ms = now_ms;
        p_control->b_center_candidate = false;
        p_control->b_side_candidate = false;
        action = p_control->held_action;
    }
    else
    {
        /* The caller rejects STOP and values outside the report protocol. */
    }

    p_control->held_action = action;

    return (action);
}

static line_control_action_t
line_control_confirm_step (line_control_t * p_control, uint8_t position,
                           uint32_t now_ms)
{
    line_control_action_t action;

    if (S2_LINE_BOTH_BLACK != position)
    {
        p_control->mode = LINE_CONTROL_MODE_FOLLOW;
        p_control->mode_since_ms = now_ms;
        action = line_control_follow_step(p_control, position, now_ms);
    }
    else if ((now_ms - p_control->mode_since_ms)
             >= p_control->cfg.junction_confirm_ms)
    {
        p_control->mode = LINE_CONTROL_MODE_CROSSING;
        p_control->mode_since_ms = now_ms;
        p_control->b_correction_active = false;
        p_control->b_curve_active = false;
        p_control->held_action = LINE_CONTROL_ACTION_STRAIGHT;
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else
    {
        action = p_control->held_action;
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
        p_control->mode = LINE_CONTROL_MODE_EXIT;
        p_control->mode_since_ms = now_ms;
        p_control->b_center_candidate = false;
        p_control->b_side_candidate = false;
        action = LINE_CONTROL_ACTION_STRAIGHT;
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
line_control_exit_step (line_control_t * p_control, uint8_t position,
                        uint32_t now_ms)
{
    uint32_t const elapsed_ms = now_ms - p_control->mode_since_ms;
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (elapsed_ms >= p_control->cfg.exit_limit_ms)
    {
        action = LINE_CONTROL_ACTION_STOP;
    }
    else if (elapsed_ms < p_control->cfg.exit_min_ms)
    {
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else if (S2_LINE_CENTER == position)
    {
        p_control->b_side_candidate = false;

        if (!p_control->b_center_candidate)
        {
            p_control->center_since_ms = now_ms;
            p_control->b_center_candidate = true;
        }

        if ((now_ms - p_control->center_since_ms)
            >= p_control->cfg.exit_center_ms)
        {
            p_control->mode = LINE_CONTROL_MODE_FOLLOW;
            p_control->mode_since_ms = now_ms;
            p_control->b_correction_active = false;
            p_control->b_curve_active = false;
            p_control->b_center_candidate = false;
        }

        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else if (S2_LINE_LEFT == position)
    {
        p_control->b_center_candidate = false;
        if ((!p_control->b_side_candidate)
            || (S2_LINE_LEFT != p_control->exit_side))
        {
            p_control->side_since_ms = now_ms;
            p_control->exit_side = S2_LINE_LEFT;
            p_control->b_side_candidate = true;
        }

        if ((now_ms - p_control->side_since_ms)
            >= p_control->cfg.exit_side_ms)
        {
            p_control->mode = LINE_CONTROL_MODE_FOLLOW;
            p_control->mode_since_ms = now_ms;
            p_control->b_side_candidate = false;
            action = line_control_follow_step(p_control, position, now_ms);
        }
        else
        {
            action = LINE_CONTROL_ACTION_STRAIGHT;
        }
    }
    else if (S2_LINE_RIGHT == position)
    {
        p_control->b_center_candidate = false;
        if ((!p_control->b_side_candidate)
            || (S2_LINE_RIGHT != p_control->exit_side))
        {
            p_control->side_since_ms = now_ms;
            p_control->exit_side = S2_LINE_RIGHT;
            p_control->b_side_candidate = true;
        }

        if ((now_ms - p_control->side_since_ms)
            >= p_control->cfg.exit_side_ms)
        {
            p_control->mode = LINE_CONTROL_MODE_FOLLOW;
            p_control->mode_since_ms = now_ms;
            p_control->b_side_candidate = false;
            action = line_control_follow_step(p_control, position, now_ms);
        }
        else
        {
            action = LINE_CONTROL_ACTION_STRAIGHT;
        }
    }
    else if (S2_LINE_BOTH_BLACK == position)
    {
        p_control->b_center_candidate = false;
        p_control->b_side_candidate = false;
        action = LINE_CONTROL_ACTION_STRAIGHT;
    }
    else
    {
        /* The caller rejects STOP and values outside the report protocol. */
    }

    return (action);
}

static line_control_action_t
line_control_direction_action (uint8_t position, bool b_curve)
{
    line_control_action_t action = LINE_CONTROL_ACTION_STOP;

    if (S2_LINE_LEFT == position)
    {
        action = b_curve ? LINE_CONTROL_ACTION_CURVE_LEFT
                         : LINE_CONTROL_ACTION_CORRECT_LEFT;
    }
    else if (S2_LINE_RIGHT == position)
    {
        action = b_curve ? LINE_CONTROL_ACTION_CURVE_RIGHT
                         : LINE_CONTROL_ACTION_CORRECT_RIGHT;
    }
    else
    {
        /* A steering direction must come from a single-black observation. */
    }

    return (action);
}

static uint32_t
line_control_center_hold_ms (line_control_t const * p_control)
{
    uint32_t hold_ms = 0u;

    if (S2_LINE_LEFT == p_control->last_correction)
    {
        hold_ms = p_control->cfg.left_center_hold_ms;
    }
    else if (S2_LINE_RIGHT == p_control->last_correction)
    {
        hold_ms = p_control->cfg.right_center_hold_ms;
    }
    else
    {
        /* No direction means no curve hold. */
    }

    return (hold_ms);
}

/* End of file. */
