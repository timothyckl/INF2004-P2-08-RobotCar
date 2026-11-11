/**
 * @file bypass_control.h
 * @brief Pure bypass and line reacquisition for the Vehicle Controller.
 *
 * Drives subsystem 5's plan (IF-13) after a PLAN scan result: each plan
 * segment as one MOVE or TURN, then the approach turn back towards the
 * original line, then a forward search. The search succeeds once a line
 * sensor has been on black for found_ms; the car then moves so its axle
 * reaches the point where the sensors first saw the line, turns by the
 * align angle and checks that the heading is within heading_tol_mdeg of
 * the heading at the start (S5 decision 9).
 *
 * A rejected, failed or stopped command, a forward hazard while driving
 * forward, or an exhausted search bound (search_limit_ms, or the plan's
 * search_mm) fails the bypass. In-place turns ignore the forward hazard:
 * they do not close on the obstacle, and the first one starts within the
 * hazard distance. A hazard when the align turn ends fails it too, as
 * following would resume into it. The caller sends the commands and
 * reports each send
 * with bypass_control_sent(); this module touches no kernel object.
 */
#ifndef BYPASS_CONTROL_H
#define BYPASS_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define BYPASS_CONTROL_SEGMENTS     5u

/* Segment kinds: subsystem 5's codes (bypass_plan.h BYPASS_PLAN_*). */
#define BYPASS_CONTROL_TURN         1u  /* value: mdeg, positive = left */
#define BYPASS_CONTROL_MOVE         2u  /* value: mm, positive = forward */

typedef enum
{
    BYPASS_CONTROL_IDLE = 0,
    BYPASS_CONTROL_SEGMENT,     /* a plan segment or the approach turn */
    BYPASS_CONTROL_SEARCH,      /* forward towards the line */
    BYPASS_CONTROL_STEP,        /* axle onto the point the line was seen */
    BYPASS_CONTROL_ALIGN        /* turn back to the original direction */
} bypass_control_state_t;

typedef enum
{
    BYPASS_CONTROL_ACT_NONE = 0,
    BYPASS_CONTROL_ACT_MOVE,    /* value: signed mm */
    BYPASS_CONTROL_ACT_TURN,    /* value: signed mdeg, positive = left */
    BYPASS_CONTROL_ACT_SEARCH   /* hold the search speed on both wheels */
} bypass_control_action_t;

typedef enum
{
    BYPASS_CONTROL_NONE = 0,
    BYPASS_CONTROL_DONE,        /* back on the line: resume following */
    BYPASS_CONTROL_FAILED       /* fail says why; the run must stop */
} bypass_control_outcome_t;

typedef enum
{
    BYPASS_CONTROL_FAIL_NONE = 0,
    BYPASS_CONTROL_FAIL_PLAN,       /* segment kind or value out of range */
    BYPASS_CONTROL_FAIL_MOTION,     /* not sent, rejected, fault, stopped */
    BYPASS_CONTROL_FAIL_HAZARD,     /* forward hazard while going forward */
    BYPASS_CONTROL_FAIL_SEARCH,     /* line not found within the bounds */
    BYPASS_CONTROL_FAIL_HEADING     /* aligned, but off the original */
} bypass_control_fail_t;

typedef struct
{
    uint8_t kind;               /* BYPASS_CONTROL_TURN or _MOVE */
    int32_t value;
} bypass_control_segment_t;

/** Subsystem 5's plan, copied when the scan ends (results expire). */
typedef struct
{
    bypass_control_segment_t seg[BYPASS_CONTROL_SEGMENTS];
    uint8_t                  count;
    int32_t                  approach_mdeg;
    int32_t                  align_mdeg;
    uint16_t                 search_mm;
} bypass_control_plan_t;

typedef struct
{
    uint32_t search_limit_ms;   /* agreed 5 s */
    uint32_t found_ms;          /* agreed 100 ms of stable line */
    int32_t  heading_tol_mdeg;  /* agreed 15 degrees */
    uint16_t sensor_ahead_mm;   /* line sensors ahead of the axle */
} bypass_control_cfg_t;

/** One control cycle's view of the vehicle. */
typedef struct
{
    uint32_t now_ms;
    int32_t  travel_mm;         /* motion_status_t.travel_mm */
    int32_t  heading_mdeg;      /* motion_status_t.heading_mdeg */
    uint16_t last_id;           /* motion_status_t.last_id */
    uint8_t  last_result;       /* motion_status_t.last_result */
    bool     b_status_ok;       /* the motion fields above are current */
    bool     b_on_line;         /* fresh report, a sensor on black */
    bool     b_hazard;          /* forward hazard, or no fresh verdict */
} bypass_control_input_t;

/** Results of one call; cleared by every call. */
typedef struct
{
    bypass_control_action_t  action;
    int32_t                  value;
    bypass_control_outcome_t outcome;
    bypass_control_fail_t    fail;
} bypass_control_output_t;

typedef struct
{
    bypass_control_cfg_t   cfg;
    bypass_control_plan_t  plan;
    uint32_t               since_ms;    /* search start */
    uint32_t               seen_ms;     /* line first seen */
    int32_t                start_mm;    /* travel at search start */
    int32_t                seen_mm;     /* travel when first seen */
    int32_t                heading_ref_mdeg;
    uint16_t               cmd_id;      /* last command sent */
    uint8_t                next;        /* next segment; count = approach */
    bypass_control_state_t state;
    bool                   b_have_id;
    bool                   b_forward;   /* last command drives forward */
    bool                   b_seen;
} bypass_control_t;

/** Initialise idle. Limits must be nonzero, the tolerance positive. */
bool bypass_control_init(bypass_control_t * p_bypass,
                         bypass_control_cfg_t const * p_cfg);

/** True while a bypass is in progress. */
bool bypass_control_active(bypass_control_t const * p_bypass);

/** S2_MODE_BYPASS, S2_MODE_REACQUIRE, or S2_MODE_NONE when idle. */
uint8_t bypass_control_mode(bypass_control_t const * p_bypass);

/**
 * Start a bypass: validate and copy the plan, record the heading and
 * request the first command. An invalid plan fails at once (FAIL_PLAN).
 * Ignored while a bypass is active.
 */
void bypass_control_begin(bypass_control_t * p_bypass,
                          bypass_control_plan_t const * p_plan,
                          bypass_control_input_t const * p_in,
                          bypass_control_output_t * p_out);

/**
 * Report the send of the action just requested. b_sent false fails the
 * bypass (FAIL_MOTION); cmd_id is the command's motion_q ID.
 */
void bypass_control_sent(bypass_control_t * p_bypass, bool b_sent,
                         uint16_t cmd_id, bypass_control_output_t * p_out);

/** Advance an active bypass by one control cycle. */
void bypass_control_step(bypass_control_t * p_bypass,
                         bypass_control_input_t const * p_in,
                         bypass_control_output_t * p_out);

/** End an active bypass because of a stop or fault elsewhere. */
void bypass_control_abort(bypass_control_t * p_bypass);

#endif /* BYPASS_CONTROL_H */

/* End of file. */
