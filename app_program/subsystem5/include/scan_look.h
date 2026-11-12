/**
 * @file scan_look.h
 * @brief One stationary sideways look during a bypass (pure logic).
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Source: S2 decision 19 in docs/decisions.md. While the controller
 * passes an obstacle it stops and asks for one reading at the plan's look
 * angle. The look waits until the wheels are confirmed still, turns the
 * servo, waits the settling time, takes the median of three valid
 * readings, turns the servo forward, waits again and ends only after one
 * fresh valid forward reading, so the controller never moves on a stale
 * forward verdict. Three consecutive invalid readings, movement, lost
 * authority or the time limit fault the look and turn the servo forward.
 * The caller keeps triggering readings; this module only says which ones
 * count. It touches no kernel object.
 */
#ifndef SCAN_LOOK_H
#define SCAN_LOOK_H

#include <stdbool.h>
#include <stdint.h>

#define SCAN_LOOK_SAMPLES       (3u)
#define SCAN_LOOK_ANGLE_MAX     (45)    /* servo limit, drv_servo.h */

typedef enum
{
    SCAN_LOOK_IDLE = 0,
    SCAN_LOOK_STILL,        /* waiting for the wheels to be still */
    SCAN_LOOK_OUT,          /* servo turning to the look angle */
    SCAN_LOOK_SAMPLE,       /* collecting the look's readings */
    SCAN_LOOK_BACK,         /* servo turning forward */
    SCAN_LOOK_FRESH         /* waiting for one fresh forward reading */
} scan_look_state_t;

typedef enum
{
    SCAN_LOOK_NONE = 0,
    SCAN_LOOK_DONE,
    SCAN_LOOK_FAULT
} scan_look_result_t;

typedef struct
{
    uint32_t settling_us;       /* servo settling, as for a scan */
    uint32_t still_limit_us;    /* longest wait for the wheels */
    uint32_t limit_us;          /* longest whole look */
} scan_look_cfg_t;

/** Results of one call; cleared by every call. */
typedef struct
{
    bool               b_servo;     /* turn the servo to servo_deg */
    int16_t            servo_deg;
    bool               b_resume;    /* restart forward monitoring now */
    scan_look_result_t result;      /* set once, when the look ends */
    uint16_t           distance_mm; /* the look's median when DONE */
} scan_look_output_t;

typedef struct
{
    scan_look_cfg_t   cfg;
    scan_look_state_t state;
    uint32_t          id;
    uint32_t          started_us;
    uint32_t          since_us;
    uint16_t          samples[SCAN_LOOK_SAMPLES];
    uint16_t          median_mm;
    int16_t           angle_deg;
    uint8_t           count;
    uint8_t           invalid;
} scan_look_t;

/** Initialise idle. All times must be nonzero. */
bool scan_look_init (scan_look_t * p_look, scan_look_cfg_t const * p_cfg);

/** True while a look is in progress. */
bool scan_look_active (scan_look_t const * p_look);

/** True while a look's or the fresh forward reading is wanted. */
bool scan_look_capturing (scan_look_t const * p_look);

/** Start a look; an angle beyond the servo's limit faults at once. */
void scan_look_start (scan_look_t * p_look, uint32_t id, int16_t angle_deg,
                      uint32_t now_us, scan_look_output_t * p_out);

/** Advance the look's timing; b_still is motion_stop_confirmed(). */
void scan_look_tick (scan_look_t * p_look, uint32_t now_us, bool b_still,
                     bool b_authority, scan_look_output_t * p_out);

/** One reading's outcome; ignored unless scan_look_capturing(). */
void scan_look_sample (scan_look_t * p_look, bool b_valid,
                       uint16_t distance_mm, uint32_t now_us,
                       scan_look_output_t * p_out);

#endif /* SCAN_LOOK_H */

/* End of file. */
