/**
 * @file line_sensor.h
 * @brief Active-object adapter for IR sampling and line-following logic.
 *
 * The functions expose perception and normalized steering recommendations to
 * the Vehicle Controller. They do not authorise motion or write motor PWM.
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "line_follower.h"
#include "../app.h"

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_TASK_PRIORITY (16)

typedef struct
{
    line_follower_output_t control;
    uint32_t               sample_count;
    line_follower_event_t  latched_events;
    bool                   b_left_raw;
    bool                   b_right_raw;
    bool                   b_left_black;
    bool                   b_right_black;
    bool                   b_sensor_fault;
} line_sensor_snapshot_t;

/** Initialize synchronization and default, unvalidated calibration values. */
ER line_sensor_init(void);

/** Start the sole task that owns and samples GP4 and GP16. */
ER line_sensor_start(void);

/** Copy the latest sample and control recommendation under a mutex. */
ER line_sensor_get_snapshot(line_sensor_snapshot_t * p_snapshot);

/** Atomically return and clear events for the single Vehicle Controller. */
ER line_sensor_take_events(line_follower_event_t * p_events);

/** Set the observed digital level that represents black for each sensor. */
ER line_sensor_set_calibration(bool b_left_black_level,
                               bool b_right_black_level);

/** Add incremental encoder travel for junction confirmation. */
ER line_sensor_add_travel_mm(uint16_t travel_delta_mm);

/** Request controller-authorised line recovery in the last known direction. */
ER line_sensor_start_recovery(line_follower_direction_t direction);

#endif /* LINE_SENSOR_H */

/* End of file. */
