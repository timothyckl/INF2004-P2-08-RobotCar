/**
 * @file s4_motion_class.h
 * @brief Motion classification and encoder-estimated yaw rate (pure).
 *
 * Observational only: nothing here commands motion or changes hump, impact
 * or lift detection. One call per IMU sample classifies the car as
 * stationary, cruising, accelerating, braking, turning, climbing or
 * descending a hump, or impact, from Motion Control's wheel speeds (IF-17)
 * and the terrain logic's state. The LSM303DLHC has no gyroscope, so the
 * yaw rate is estimated from the wheel encoders, (right - left) / track,
 * positive to the left as Motion Control's heading; it is not an IMU
 * angular-rate measurement.
 *
 * Priority, highest first: unknown (no fresh Motion Control snapshot),
 * impact, climbing or descending (while on a hump), turning, stationary,
 * accelerating or braking, cruising. A new state is taken once the same
 * candidate holds for confirm_samples samples; unknown and impact are
 * taken at once, and impact is held for confirm_samples samples.
 */
#ifndef S4_MOTION_CLASS_H
#define S4_MOTION_CLASS_H

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t s4_motion_class_t;

#define S4_MOTION_UNKNOWN      ((s4_motion_class_t) 0u)
#define S4_MOTION_STATIONARY   ((s4_motion_class_t) 1u)
#define S4_MOTION_CRUISING     ((s4_motion_class_t) 2u)
#define S4_MOTION_ACCELERATING ((s4_motion_class_t) 3u)
#define S4_MOTION_BRAKING      ((s4_motion_class_t) 4u)
#define S4_MOTION_TURNING      ((s4_motion_class_t) 5u)
#define S4_MOTION_CLIMBING     ((s4_motion_class_t) 6u)
#define S4_MOTION_DESCENDING   ((s4_motion_class_t) 7u)
#define S4_MOTION_IMPACT       ((s4_motion_class_t) 8u)
#define S4_MOTION_CLASS_COUNT  9u

/*
 * Mean-speed change gate, shared by the pitch gate of the IMU task and the
 * classifier (S4 decision 8): a change of the mean wheel speed by more
 * than change_mm_s between two samples starts a window of hold_samples
 * more samples in which forward acceleration reads as pitch.
 */
typedef struct
{
    int32_t speed_mm_s;     /* mean wheel speed at the previous sample */
    uint8_t hold;           /* samples left in the window */
    int8_t  direction;      /* +1 speeding up, -1 slowing, 0 steady */
    bool    b_have_previous;
} s4_speed_gate_t;

typedef struct
{
    uint16_t track_mm;          /* left to right wheel contact */
    uint16_t still_mm_s;        /* both wheels at or below: stationary */
    uint16_t turn_ddeg_s;       /* |yaw| at or above: turning, 0.1 deg/s */
    uint16_t max_age_ms;        /* older snapshot: unknown */
    uint32_t hump_band_um;      /* below the peak by more: descending */
    uint8_t  confirm_samples;   /* debounce, at least 1 */
} s4_motion_class_config_t;

typedef struct
{
    int16_t  left_mm_s;         /* Motion Control snapshot (IF-17) */
    int16_t  right_mm_s;
    uint32_t age_ms;            /* of that snapshot */
    bool     b_motion_valid;    /* the snapshot was read */
    int8_t   speed_change;      /* s4_speed_gate_t.direction */
    bool     b_terrain_valid;   /* terrain monitoring or on a hump */
    bool     b_on_hump;
    int32_t  height_um;         /* uncalibrated height, this sample */
    int32_t  peak_um;           /* peak of the hump in progress */
    bool     b_impact;          /* impact event this sample */
} s4_motion_input_t;

typedef struct
{
    s4_motion_class_t state;
    s4_motion_class_t candidate;
    uint8_t           candidate_count;
    uint8_t           impact_hold;
    int16_t           yaw_ddeg_s;   /* encoder-estimated, 0.1 deg/s */
} s4_motion_classifier_t;

/** Forget the previous speed (a failed snapshot read); the window stays. */
void s4_speed_gate_reset(s4_speed_gate_t * p_gate);

/**
 * One sample of the mean wheel speed. Returns true when the speed is
 * steady, so pitch may be read as tilt: false on the first sample after a
 * reset and while a change window is open.
 */
bool s4_speed_gate_step(s4_speed_gate_t * p_gate, int32_t speed_mm_s,
                        int32_t change_mm_s, uint8_t hold_samples);

/** Encoder-estimated yaw rate in 0.1 deg/s, left positive, saturated. */
int16_t s4_motion_yaw_ddeg_s(int16_t left_mm_s, int16_t right_mm_s,
                             uint16_t track_mm);

void s4_motion_class_init(s4_motion_classifier_t * p_class);

/** One sample; returns the (debounced) state, also in p_class->state. */
s4_motion_class_t s4_motion_class_step(
    s4_motion_classifier_t * p_class,
    s4_motion_class_config_t const * p_config,
    s4_motion_input_t const * p_input);

/** Printable name of a state ("?" if out of range). */
char const * s4_motion_class_name(s4_motion_class_t state);

#endif /* S4_MOTION_CLASS_H */

/* End of file. */
