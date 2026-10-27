/**
 * @file terrain_logic.h
 * @brief Deterministic Subsystem 4 terrain-processing interface.
 *
 * This interface accepts decoded acceleration and optional odometry inputs.
 * It has no RTOS, I2C, motor-control, or networking dependency.
 */
#ifndef TERRAIN_LOGIC_H
#define TERRAIN_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

/* Set to one only when terrain_logic.c provides the complete binding. */
#define S4_TERRAIN_LOGIC_AVAILABLE 0u

typedef uint8_t s4_terrain_state_t;

#define S4_TERRAIN_STATE_CALIBRATING ((s4_terrain_state_t) 0u)
#define S4_TERRAIN_STATE_MONITORING  ((s4_terrain_state_t) 1u)
#define S4_TERRAIN_STATE_ASCENDING   ((s4_terrain_state_t) 2u)
#define S4_TERRAIN_STATE_DESCENDING  ((s4_terrain_state_t) 3u)
#define S4_TERRAIN_STATE_FAULT       ((s4_terrain_state_t) 4u)

typedef uint8_t s4_terrain_fault_t;

#define S4_TERRAIN_FAULT_NONE        ((s4_terrain_fault_t) 0u)
#define S4_TERRAIN_FAULT_CALIBRATION ((s4_terrain_fault_t) 1u)
#define S4_TERRAIN_FAULT_SENSOR      ((s4_terrain_fault_t) 2u)
#define S4_TERRAIN_FAULT_STALE       ((s4_terrain_fault_t) 3u)

#define S4_TERRAIN_EVENT_READY         (UINT8_C(1) << 0u)
#define S4_TERRAIN_EVENT_HUMP_COMPLETE (UINT8_C(1) << 1u)
#define S4_TERRAIN_EVENT_IMPACT        (UINT8_C(1) << 2u)
#define S4_TERRAIN_EVENT_FAULT         (UINT8_C(1) << 3u)

typedef struct
{
    int16_t  acceleration_x_mg;
    int16_t  acceleration_y_mg;
    int16_t  acceleration_z_mg;
    uint32_t capture_time_us;
    bool     b_valid;
} s4_terrain_sample_t;

typedef struct
{
    uint16_t calibration_samples;
    uint16_t calibration_span_mg;
    uint16_t ascent_threshold_mg;
    uint16_t level_threshold_mg;
    uint16_t impact_threshold_mg;
    uint16_t max_sample_gap_ms;
    uint8_t  confirmation_samples;
    uint8_t  invalid_limit;
} s4_terrain_config_t;

typedef struct
{
    s4_terrain_state_t state;
    s4_terrain_fault_t fault;
    uint8_t            events;
    int16_t            forward_tilt_mg;
    uint16_t           current_height_mm;
    uint16_t           maximum_height_mm;
    bool               b_current_height_valid;
    bool               b_maximum_height_valid;
} s4_terrain_result_t;

typedef struct
{
    s4_terrain_config_t config;
    s4_terrain_result_t result;
    int64_t             calibration_sum_x;
    int64_t             calibration_sum_y;
    int64_t             calibration_sum_z;
    int16_t             calibration_min_x;
    int16_t             calibration_max_x;
    int16_t             calibration_min_y;
    int16_t             calibration_max_y;
    int16_t             calibration_min_z;
    int16_t             calibration_max_z;
    int16_t             offset_x_mg;
    int16_t             offset_y_mg;
    int16_t             offset_z_mg;
    int32_t             filtered_x_mg;
    uint32_t            previous_time_us;
    uint32_t            calibration_count;
    uint16_t            candidate_height_mm;
    uint8_t             confirmation_count;
    uint8_t             invalid_count;
    bool                b_have_previous_time;
    bool                b_candidate_height_valid;
} s4_terrain_context_t;

void s4_terrain_init(s4_terrain_context_t * p_context,
                     s4_terrain_config_t const * p_config);
void s4_terrain_process(s4_terrain_context_t * p_context,
                        s4_terrain_sample_t const * p_sample,
                        uint16_t distance_delta_mm,
                        bool b_distance_valid);
void s4_terrain_request_recovery(s4_terrain_context_t * p_context);
void s4_terrain_start_new_run(s4_terrain_context_t * p_context);
s4_terrain_result_t const *
s4_terrain_get_result(s4_terrain_context_t const * p_context);

#endif /* TERRAIN_LOGIC_H */

/* End of file. */
