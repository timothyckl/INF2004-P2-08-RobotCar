/**
 * @file line_control.h
 * @brief Pure two-sensor line-following and junction policy.
 */
#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LINE_CONTROL_MODE_INVALID = 0,
    LINE_CONTROL_MODE_FOLLOW,
    LINE_CONTROL_MODE_JUNCTION_CONFIRM,
    LINE_CONTROL_MODE_CROSSING,
    LINE_CONTROL_MODE_EXIT
} line_control_mode_t;

typedef enum
{
    LINE_CONTROL_ACTION_STOP = 0,
    LINE_CONTROL_ACTION_STRAIGHT,
    LINE_CONTROL_ACTION_CORRECT_LEFT,
    LINE_CONTROL_ACTION_CORRECT_RIGHT,
    LINE_CONTROL_ACTION_CURVE_LEFT,
    LINE_CONTROL_ACTION_CURVE_RIGHT
} line_control_action_t;

typedef struct
{
    uint32_t curve_entry_ms;
    uint32_t left_center_hold_ms;
    uint32_t right_center_hold_ms;
    uint32_t junction_confirm_ms;
    uint32_t crossing_limit_ms;
    uint32_t exit_min_ms;
    uint32_t exit_center_ms;
    uint32_t exit_side_ms;
    uint32_t exit_limit_ms;
} line_control_cfg_t;

typedef struct
{
    line_control_cfg_t    cfg;
    line_control_mode_t   mode;
    line_control_action_t held_action;
    uint32_t              mode_since_ms;
    uint32_t              correction_since_ms;
    uint32_t              curve_seen_ms;
    uint32_t              center_since_ms;
    uint32_t              side_since_ms;
    uint8_t               last_correction;
    uint8_t               exit_side;
    bool                  b_correction_active;
    bool                  b_curve_active;
    bool                  b_center_candidate;
    bool                  b_side_candidate;
    bool                  b_valid;
} line_control_t;

/** Initialise a policy. Every time limit must be nonzero and ordered. */
bool line_control_init(line_control_t * p_control,
                       line_control_cfg_t const * p_cfg,
                       uint32_t now_ms);

/** Convert the latest S2_LINE_* observation into a controller action. */
line_control_action_t line_control_step(line_control_t * p_control,
                                        uint8_t position,
                                        uint32_t now_ms);

/** Return INVALID for a null or uninitialised policy. */
line_control_mode_t line_control_mode(line_control_t const * p_control);

#endif /* LINE_CONTROL_H */

/* End of file. */
