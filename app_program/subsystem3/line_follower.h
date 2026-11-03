/**
 * @file line_follower.h
 * @brief Pure two-sensor line-position and PD-control logic.
 *
 * The module does not access GPIO, encoders, RTOS objects, or motor PWM. Speed
 * targets use a normalized 0 to 1000 range for a future Vehicle Controller
 * adapter. They are recommendations and do not authorise movement.
 */

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_FOLLOWER_GAIN_SCALE       (256U)

typedef uint8_t line_follower_position_t;

#define LINE_FOLLOWER_POSITION_CENTRED     (0U)
#define LINE_FOLLOWER_POSITION_ROBOT_LEFT  (1U)
#define LINE_FOLLOWER_POSITION_ROBOT_RIGHT (2U)
#define LINE_FOLLOWER_POSITION_UNKNOWN     (3U)

typedef uint8_t line_follower_state_t;

#define LINE_FOLLOWER_STATE_FOLLOWING    (0U)
#define LINE_FOLLOWER_STATE_JUNCTION     (1U)
#define LINE_FOLLOWER_STATE_RECOVERING   (2U)
#define LINE_FOLLOWER_STATE_SENSOR_FAULT (3U)

typedef uint8_t line_follower_event_t;

#define LINE_FOLLOWER_EVENT_NONE         (0U)
#define LINE_FOLLOWER_EVENT_LINE_LOST    (1U)
#define LINE_FOLLOWER_EVENT_JUNCTION     (2U)
#define LINE_FOLLOWER_EVENT_REACQUIRED   (4U)
#define LINE_FOLLOWER_EVENT_SENSOR_FAULT (8U)

typedef int8_t line_follower_direction_t;

#define LINE_FOLLOWER_DIRECTION_LEFT  (-1)
#define LINE_FOLLOWER_DIRECTION_RIGHT (1)

typedef struct
{
    uint16_t minimum_speed;
    uint16_t base_speed;
    uint16_t maximum_speed;
    uint16_t maximum_correction;
    uint16_t proportional_gain_q8;
    uint16_t derivative_gain_q8;
    uint16_t junction_minimum_mm;
    uint8_t  decision_samples;
    uint8_t  junction_samples;
    uint8_t  reacquire_samples;
} line_follower_config_t;

typedef struct
{
    bool     b_left_black;
    bool     b_right_black;
    uint16_t travel_delta_mm;
} line_follower_sample_t;

typedef struct
{
    line_follower_position_t position;
    line_follower_state_t    state;
    line_follower_event_t    events;
    int8_t                   error;
    int16_t                  correction;
    uint16_t                 left_target;
    uint16_t                 right_target;
    bool                     b_line_valid;
} line_follower_output_t;

typedef struct
{
    line_follower_config_t    config;
    line_follower_state_t     state;
    int8_t                    current_error;
    int8_t                    previous_error;
    int8_t                    candidate_error;
    line_follower_direction_t recovery_direction;
    uint16_t                  junction_distance_mm;
    uint8_t                   candidate_count;
    uint8_t                   junction_count;
    uint8_t                   junction_clear_count;
    uint8_t                   reacquire_count;
    line_follower_event_t     pending_events;
    bool                      b_junction_reported;
    bool                      b_recovery_edge_seen;
    bool                      b_initialized;
} line_follower_t;

/** Initialize one independent line-following controller. */
bool line_follower_init(line_follower_t * p_follower,
                        line_follower_config_t const * p_config);

/** Process simultaneous left/right inputs and an encoder-distance increment. */
bool line_follower_step(line_follower_t * p_follower,
                        line_follower_sample_t const * p_sample,
                        line_follower_output_t * p_output);

/** Enter bounded-search output mode after controller-authorised line loss. */
bool line_follower_start_recovery(line_follower_t * p_follower,
                                  line_follower_direction_t direction);

#endif /* LINE_FOLLOWER_H */

/* End of file. */
