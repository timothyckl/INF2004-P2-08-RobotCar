/**
 * @file bypass_control.h
 * @brief Pure bypass and line reacquisition for the Vehicle Controller.
 *
 * Drives subsystem 5's plan (IF-13) after a PLAN scan result: each plan
 * segment as one MOVE or TURN; then, when the plan has a look angle, the
 * pass (S2 decision 19): turn look_turn_mdeg towards the obstacle, look
 * sideways, turn back to the start heading (S2 decision 22), and until a
 * look reads beyond clear_mm, and BYPASS_CONTROL_PASS_JUMP_MM beyond the
 * nearest side reading so far, after at least pass_min_mm of stepping, MOVE
 * step_mm and look again, failing past pass_max_mm; once passed, MOVE
 * extra_mm. Then the approach turn, to approach_mdeg from the start
 * heading, back towards the original line, then a forward search. The
 * search succeeds once a line sensor has been on black for found_ms; the
 * car then stops (S2 decision 20) and moves so its axle reaches the point
 * where the sensors first saw the line. It then spins towards the
 * original direction until a sensor sees the line (S2 decision 21),
 * accepted only once it has turned the align angle less twice
 * heading_tol_mdeg, and failing (FAIL_SEARCH) past
 * the align angle plus 125 degrees; it stops and checks only that it
 * has not taken the line the wrong way: within
 * BYPASS_CONTROL_WRONG_WAY_MDEG of the heading at the start (S5 decision
 * 9, widened by S2 decision 22). The line itself sets the heading, as it
 * may curve beyond the obstacle.
 *
 * A rejected, failed or stopped command, a forward hazard while driving
 * forward, a faulted or unanswered look, an obstacle deeper than the
 * pass allows, or an exhausted search bound (search_limit_ms, or the plan's
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

#define BYPASS_CONTROL_SEGMENTS     4u

/* A passing look must also read this much beyond the nearest side reading
   while beside the obstacle (UNMEASURED; S5's clear_mm margin). The scan
   overstates widths, so clear_mm alone can sit just above the side: 287
   mm against 269 passed a box still beside the car (2026-11-12). */
#define BYPASS_CONTROL_PASS_JUMP_MM 100u

/* Once aligned, the car must be within this of the start heading: wider
   would be the line taken the wrong way (S2 decision 22). Not a guess at
   the line's direction, which can curve beyond the obstacle: 15 degrees
   failed cars already on a curving line (2026-11-12). */
#define BYPASS_CONTROL_WRONG_WAY_MDEG   90000

/* Segment kinds: subsystem 5's codes (bypass_plan.h BYPASS_PLAN_*). */
#define BYPASS_CONTROL_TURN         1u  /* value: mdeg, positive = left */
#define BYPASS_CONTROL_MOVE         2u  /* value: mm, positive = forward */

typedef enum
{
    BYPASS_CONTROL_IDLE = 0,
    BYPASS_CONTROL_SEGMENT,     /* a plan segment or the approach turn */
    BYPASS_CONTROL_PASS,        /* stepping and looking past the obstacle */
    BYPASS_CONTROL_SEARCH,      /* forward towards the line */
    BYPASS_CONTROL_HALT,        /* line found: braking to a standstill */
    BYPASS_CONTROL_STEP,        /* axle onto the point the line was seen */
    BYPASS_CONTROL_ALIGN        /* spin towards the original direction
                                   until a sensor sees the line */
} bypass_control_state_t;

/* One look cycle while passing (S2 decision 22). */
typedef enum
{
    BYPASS_PASS_TURN_IN = 0,    /* turning towards the obstacle */
    BYPASS_PASS_LOOK,           /* waiting for the look's answer */
    BYPASS_PASS_TURN_OUT,       /* turning back straight */
    BYPASS_PASS_MOVE            /* stepping, or driving extra_mm */
} bypass_pass_phase_t;

typedef enum
{
    BYPASS_CONTROL_ACT_NONE = 0,
    BYPASS_CONTROL_ACT_MOVE,    /* value: signed mm */
    BYPASS_CONTROL_ACT_TURN,    /* value: signed mdeg, positive = left */
    BYPASS_CONTROL_ACT_SEARCH,  /* hold the search speed on both wheels */
    BYPASS_CONTROL_ACT_LOOK,    /* value: look angle, degrees, + = left */
    BYPASS_CONTROL_ACT_HALT,    /* motion STOP command (not the STOP
                                   latch); its STOPPED result is success */
    BYPASS_CONTROL_ACT_SPIN     /* value: +1 spin left, -1 spin right */
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
    BYPASS_CONTROL_FAIL_MOTION,     /* not sent, rejected, fault, stopped,
                                       or not still within wait_limit_ms */
    BYPASS_CONTROL_FAIL_HAZARD,     /* forward hazard while going forward */
    BYPASS_CONTROL_FAIL_SEARCH,     /* line not found within the bounds */
    BYPASS_CONTROL_FAIL_HEADING,    /* aligned, but the wrong way */
    BYPASS_CONTROL_FAIL_LOOK,       /* look faulted or not answered */
    BYPASS_CONTROL_FAIL_DEPTH       /* not passed within pass_max_mm */
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
    int16_t                  look_deg;      /* 0: no pass, drive on */
    int32_t                  look_turn_mdeg; /* car turn around each look */
    uint16_t                 clear_mm;      /* a look beyond has passed */
    uint16_t                 extra_mm;      /* forward once passed */
    uint16_t                 pass_min_mm;   /* stepping before a pass */
    uint16_t                 pass_max_mm;   /* most stepping */
} bypass_control_plan_t;

typedef struct
{
    uint32_t search_limit_ms;   /* agreed 5 s */
    uint32_t found_ms;          /* agreed 100 ms of stable line */
    int32_t  heading_tol_mdeg;  /* agreed 15 degrees: the spin turns at
                                   least the align angle less twice this */
    uint16_t sensor_ahead_mm;   /* line sensors ahead of the axle */
    uint16_t step_mm;           /* forward step between looks */
    uint32_t wait_limit_ms;     /* longest wait for a look or a halt */
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
    bool     b_still;           /* motion_stop_confirmed() */
    uint16_t look_id;           /* the look the answer below belongs to */
    uint16_t look_mm;           /* side median when b_look_done */
    bool     b_look_done;
    bool     b_look_fault;
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
    int32_t                align_start_mdeg;  /* heading at spin start */
    uint32_t               wait_since_ms;  /* look or halt start */
    int32_t                stepped_mm;  /* forward stepped while passing */
    uint16_t               look_mm;     /* answer of the current look */
    uint16_t               side_mm;     /* nearest side reading; 0: none */
    bypass_pass_phase_t    phase;
    uint16_t               cmd_id;      /* last motion command sent */
    uint16_t               look_cmd_id; /* last look sent */
    uint8_t                next;        /* next segment; count = approach */
    bypass_control_state_t state;
    bool                   b_have_id;
    bool                   b_have_look;
    bool                   b_looking;   /* waiting for a look's answer */
    bool                   b_extra;     /* driving extra_mm, passed */
    bool                   b_passed;    /* the pass is complete */
    bool                   b_aligned;   /* spin stopped on the line */
    bool                   b_forward;   /* last command drives forward */
    bool                   b_seen;
} bypass_control_t;

/** Initialise idle. Limits and the step must be nonzero, the tolerance
    positive. */
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
 * bypass (FAIL_MOTION, or FAIL_LOOK for a look); cmd_id is the command's
 * motion_q ID, or the look's scan_q request ID.
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
