/**
 * @file line_control.h
 * @brief Pure two-sensor line-following and bounded recovery policy.
 */
#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LINE_CONTROL_MODE_INVALID = 0,
    LINE_CONTROL_MODE_FOLLOW,
    LINE_CONTROL_MODE_LOST_RECOVERY
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
    uint32_t lost_limit_ms;
    uint32_t direction_memory_ms;
    uint32_t direction_change_ms;
} line_control_cfg_t;

typedef struct
{
    line_control_cfg_t  cfg;
    line_control_mode_t mode;
    uint32_t            mode_since_ms;
    uint32_t            correction_since_ms;
    uint32_t            last_direction_ms;
    uint32_t            pending_since_ms;
    uint8_t             last_correction;
    uint8_t             pending_correction;
    bool                b_correction_active;
    bool                b_pending_correction;
    bool                b_valid;
} line_control_t;

/** Initialise a policy. Every time limit must be nonzero. */
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
