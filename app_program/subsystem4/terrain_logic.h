/**
 * @file terrain_logic.h
 * @brief Deterministic Subsystem 4 terrain-processing interface.
 *
 * This interface accepts decoded acceleration in the vehicle's axes and a
 * flag saying whether the forward axis may be read as pitch. It has no
 * RTOS, I2C, motor-control, or networking dependency.
 *
 * Hump height comes from tilt and the vehicle's contact geometry
 * (decision 8): the car stands on its left wheel, right wheel and front
 * caster, and the tilt fixes the plane through them. With the lowest
 * contact on the floor, the height of the highest one is the hump height.
 */
#ifndef TERRAIN_LOGIC_H
#define TERRAIN_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#define S4_TERRAIN_LOGIC_AVAILABLE 1u

typedef uint8_t s4_terrain_state_t;

#define S4_TERRAIN_STATE_CALIBRATING ((s4_terrain_state_t) 0u)
#define S4_TERRAIN_STATE_MONITORING  ((s4_terrain_state_t) 1u)
#define S4_TERRAIN_STATE_ON_HUMP     ((s4_terrain_state_t) 2u)
#define S4_TERRAIN_STATE_FAULT       ((s4_terrain_state_t) 4u)

typedef uint8_t s4_terrain_fault_t;

#define S4_TERRAIN_FAULT_NONE        ((s4_terrain_fault_t) 0u)
#define S4_TERRAIN_FAULT_CALIBRATION ((s4_terrain_fault_t) 1u)
#define S4_TERRAIN_FAULT_SENSOR      ((s4_terrain_fault_t) 2u)
#define S4_TERRAIN_FAULT_STALE       ((s4_terrain_fault_t) 3u)

/* Which contact was highest at a hump's peak. */
typedef uint8_t s4_hump_contact_t;

#define S4_HUMP_CONTACT_NONE         ((s4_hump_contact_t) 0u)
#define S4_HUMP_CONTACT_LEFT         ((s4_hump_contact_t) 1u)
#define S4_HUMP_CONTACT_RIGHT        ((s4_hump_contact_t) 2u)
#define S4_HUMP_CONTACT_BOTH_WHEELS  ((s4_hump_contact_t) 3u)
#define S4_HUMP_CONTACT_CASTER       ((s4_hump_contact_t) 4u)

/* Longest tilt moving average, in samples (S4 decision 10). */
#define S4_AVERAGE_MAX_SAMPLES       16u

#define S4_TERRAIN_EVENT_READY         (UINT8_C(1) << 0u)
#define S4_TERRAIN_EVENT_HUMP_COMPLETE (UINT8_C(1) << 1u)
#define S4_TERRAIN_EVENT_IMPACT        (UINT8_C(1) << 2u)
#define S4_TERRAIN_EVENT_FAULT         (UINT8_C(1) << 3u)

/* Acceleration in the vehicle's axes: x forward, y left, z up. */
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
    uint16_t impact_threshold_mg;
    uint16_t max_sample_gap_ms;
    uint16_t max_hump_ms;
    uint16_t track_mm;          /* left to right wheel contact */
    uint16_t wheelbase_mm;      /* wheel axle to caster contact */
    uint16_t rise_um;           /* height that starts a hump */
    uint16_t level_um;          /* height that ends it */
    uint8_t  confirmation_samples;
    uint8_t  invalid_limit;
    uint8_t  average_samples;   /* tilt moving average, 1 to
                                   S4_AVERAGE_MAX_SAMPLES */
} s4_terrain_config_t;

typedef struct
{
    s4_terrain_state_t state;
    s4_terrain_fault_t fault;
    uint8_t            events;
    s4_hump_contact_t  hump_contact;      /* of the last completed hump */
    int16_t            forward_tilt_mg;
    int16_t            lateral_tilt_mg;
    uint16_t           current_height_mm; /* last completed hump's peak */
    uint16_t           maximum_height_mm; /* highest peak this run */
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
    int32_t             average_x_mg[S4_AVERAGE_MAX_SAMPLES];
    int32_t             average_y_mg[S4_AVERAGE_MAX_SAMPLES];
    int32_t             average_sum_x_mg;
    int32_t             average_sum_y_mg;
    int32_t             filtered_x_mg;  /* moving average of forward */
    int32_t             filtered_y_mg;  /* moving average of left */
    uint8_t             average_index;  /* next slot to write */
    uint8_t             average_count;  /* samples held, up to the length */
    uint32_t            previous_time_us;
    uint32_t            calibration_count;
    uint32_t            hump_start_us;
    int32_t             peak_height_um;
    s4_hump_contact_t   peak_contact;
    uint8_t             confirmation_count;
    uint8_t             invalid_count;
    bool                b_have_previous_time;
    bool                b_impact_active;
    bool                b_wait_level;
} s4_terrain_context_t;

void s4_terrain_init(s4_terrain_context_t * p_context,
                     s4_terrain_config_t const * p_config);
void s4_terrain_process(s4_terrain_context_t * p_context,
                        s4_terrain_sample_t const * p_sample,
                        bool b_pitch_valid);
void s4_terrain_request_recovery(s4_terrain_context_t * p_context);
void s4_terrain_start_new_run(s4_terrain_context_t * p_context);
s4_terrain_result_t const *
s4_terrain_get_result(s4_terrain_context_t const * p_context);

#endif /* TERRAIN_LOGIC_H */

/* End of file. */
