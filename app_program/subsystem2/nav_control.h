/**
 * @file nav_control.h
 * @brief Pure barcode-command policy: act at the next junction.
 *
 * S2 decision 26. A decoded barcode command is held until line following
 * confirms the next junction, however long that takes (S2 decision 36: a
 * time limit expired before the junction on the course twice). A newer
 * command replaces it; abort (STOP, not running) drops it. A scan and
 * bypass do not: the policy is simply not stepped while they run, and
 * line following does not confirm junctions off the line. STRAIGHT then lets the normal crossing
 * proceed. LEFT, RIGHT and UTURN stop the car, step forward so the axle
 * is on the junction (decision 20: the step subtracts the travel since the
 * junction was seen), then spin in place towards the turn until a sensor
 * sees the line (decision 21: black is ignored until the turn is nearly
 * done, and an overshoot fails). Line following then takes over.
 *
 * The caller sends each requested action and reports a MOVE's ID with
 * nav_control_sent(); this module touches no kernel object and no motor.
 */
#ifndef NAV_CONTROL_H
#define NAV_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    NAV_CONTROL_IDLE = 0,
    NAV_CONTROL_PENDING,    /* command held, following the line */
    NAV_CONTROL_HALT,       /* stopping at the junction */
    NAV_CONTROL_ADVANCE,    /* stepping the axle onto the junction */
    NAV_CONTROL_SPIN,       /* spinning until a sensor sees the line */
    NAV_CONTROL_SETTLE      /* stopping on the new line */
} nav_control_state_t;

typedef enum
{
    NAV_CONTROL_ACT_NONE = 0,
    NAV_CONTROL_ACT_STOP,   /* motion STOP command (not the safety latch) */
    NAV_CONTROL_ACT_MOVE,   /* value: forward mm */
    NAV_CONTROL_ACT_SPIN    /* value: +1 spin left, -1 spin right */
} nav_control_action_t;

typedef enum
{
    NAV_CONTROL_OUT_NONE = 0,
    NAV_CONTROL_OUT_STARTED,    /* junction reached; manoeuvre begins */
    NAV_CONTROL_OUT_STRAIGHT,   /* STRAIGHT consumed at the junction */
    NAV_CONTROL_OUT_DONE,       /* on the new line and still */
    NAV_CONTROL_OUT_FAILED      /* the caller must latch STOP */
} nav_control_outcome_t;

/* Commands: the S2_NAV_* values of subsystem2.h. */
#define NAV_CONTROL_CMD_NONE        0u
#define NAV_CONTROL_CMD_LEFT        1u
#define NAV_CONTROL_CMD_RIGHT       2u
#define NAV_CONTROL_CMD_STRAIGHT    3u
#define NAV_CONTROL_CMD_UTURN       4u

typedef struct
{
    uint16_t sensor_ahead_mm;   /* front line sensors ahead of the axle */
    uint32_t phase_limit_ms;    /* bound on each stop, step and spin */
    int32_t  accept_mdeg;       /* black accepted this short of the turn */
    int32_t  over_mdeg;         /* fail this far past the turn */
} nav_control_cfg_t;

typedef struct
{
    uint32_t now_ms;
    int32_t  travel_mm;         /* motion_status_t.travel_mm */
    int32_t  heading_mdeg;      /* motion_status_t.heading_mdeg */
    int32_t  junction_travel_mm;/* travel when both sensors went black */
    uint16_t last_id;
    uint8_t  last_result;       /* MOTION_EVT_* */
    bool     b_status_ok;
    bool     b_still;           /* motion_stop_confirmed() */
    bool     b_on_line;         /* fresh report with a sensor on black */
    bool     b_junction;        /* line following confirmed a junction */
} nav_control_input_t;

typedef struct
{
    nav_control_action_t  action;
    int32_t               value;
    nav_control_outcome_t outcome;
} nav_control_output_t;

typedef struct
{
    nav_control_cfg_t   cfg;
    nav_control_state_t state;
    uint32_t            staged_ms;      /* when the held command arrived */
    uint32_t            phase_ms;
    int32_t             spin_last_mdeg;
    int32_t             turned_mdeg;    /* towards the turn, accumulated */
    uint16_t            wait_id;
    uint8_t             command;        /* NAV_CONTROL_CMD_* */
    bool                b_waiting;      /* a MOVE result is awaited */
    bool                b_valid;
} nav_control_t;

bool nav_control_init(nav_control_t * p_nav, nav_control_cfg_t const * p_cfg);

/** Hold a command for the next junction; a newer one replaces a held one.
 * Refused while a manoeuvre runs, and for NONE or unknown commands.
 * now_ms is kept in staged_ms for the caller's log only; it never expires.
 */
bool nav_control_stage(nav_control_t * p_nav, uint8_t command,
                       uint32_t now_ms);

/** Advance once per controller period. */
void nav_control_step(nav_control_t * p_nav,
                      nav_control_input_t const * p_in,
                      nav_control_output_t * p_out);

/** Report the command ID of a MOVE that was sent. If any action cannot be
 * sent, the caller aborts the policy and latches STOP instead.
 */
void nav_control_sent(nav_control_t * p_nav, uint16_t cmd_id);

/** Drop any held command or manoeuvre (STOP, scan, not running). */
void nav_control_abort(nav_control_t * p_nav);

/** True from the junction stop until the car is still on the new line. */
bool nav_control_manoeuvring(nav_control_t const * p_nav);

#endif /* NAV_CONTROL_H */

/* End of file. */
