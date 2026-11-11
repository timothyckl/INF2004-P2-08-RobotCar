/**
 * @file motion_logic.h
 * @brief Subsystem 2 Motion Control state machine (pure logic).
 *
 * Implements the Motion Control statechart (design review section 6.2):
 * Init, Idle, Speed, Move, Turn, Stopping, Fault. No hardware or kernel
 * calls: the task wrapper (task_motion.c) supplies encoder counts, the
 * STOP level and a microsecond clock, then applies the duties and sends
 * the events. Unit tests drive it with a synthetic wheel model.
 *
 * Wheels are "left" and "right" with forward-positive counts and duties;
 * the wrapper maps them to motor and encoder channels and polarity.
 *
 * Straight line: Move adds a second PID on the left/right distance
 * difference. Its output (mm/s) is subtracted from the left wheel's speed
 * target and added to the right's. The per-wheel speed PIDs match speeds
 * but cannot remove a distance already lost (spin-up, a bump, slip); the
 * difference loop does, and its integral term cancels a steady
 * left/right imbalance the speed loops leave behind.
 *
 * Output ramp: each wheel's |duty| may grow by at most duty_step_max per
 * step; a sign change goes through 0 first; a decrease, STOP and Fault
 * are immediate (motion_ramp). It is applied here, before the stall
 * check, so stall timing starts only once the duty really reaches the
 * stall level. The ramp is odd-symmetric, so the wrapper's motor invert
 * (a sign flip) after it gives the same result as ramping after the
 * invert.
 *
 * Rules:
 * - Latched STOP and S2_CMD_STOP win: motion states enter Stopping at
 *   once with zero duty. Motion commands are rejected while STOP is set.
 * - A new motion command replaces the current one only while STOP is
 *   clear. A replaced command gets no DONE.
 * - Stopping brakes until both wheels are still for still_ms, then
 *   reports DONE (target reached) or STOPPED (stop), and returns to Idle.
 *   No confirmation within stop_limit_ms is Fault(STOP_TIMEOUT).
 *   motion_is_stopped() is the stop-confirmation query.
 * - Stall, encoder fault and Move/Turn timeout enter Fault: zero duty,
 *   FAULT event, b_raise_fault. Only RESET leaves Fault, and only with
 *   both wheels still and valid encoder data.
 */
#ifndef MOTION_LOGIC_H
#define MOTION_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_api.h"
#include "motion_cmd.h"
#include "odometry.h"
#include "pid.h"

#define MOTION_WINDOW_MAX   8u      /* speed window limit, periods */
#define MOTION_EVENTS_MAX   4u      /* events one call can produce */
#define MOTION_STALL_LEFT   0x01u
#define MOTION_STALL_RIGHT  0x02u
#define MOTION_TURN_MAX_MDEG ((int32_t)MOTION_ANGLE_MAX_MDEG)

typedef enum
{
    MOTION_INIT = 0,
    MOTION_IDLE,
    MOTION_SPEED,
    MOTION_MOVE,
    MOTION_TURN,
    MOTION_STOPPING,
    MOTION_FAULT
} motion_state_t;

/** Tuning; values come from motion_config.h (or tests). */
typedef struct
{
    odom_cfg_t odom;
    pid_cfg_t  pid;             /* both wheels, out range = duty range */
    pid_cfg_t  straight_pid;    /* mm difference in, mm/s correction out */
    int32_t    speed_max_mm_s;
    uint32_t   still_mm_s;      /* |speed| at or below = still */
    uint32_t   still_ms;
    uint32_t   stop_limit_ms;
    int32_t    stall_duty;      /* |duty| at or above with no motion */
    uint32_t   stall_ms;
    uint32_t   timeout_factor;
    uint32_t   timeout_extra_ms;
    uint32_t   move_tol_mm;
    uint32_t   turn_tol_mdeg;
    uint32_t   enc_errors_max;  /* new decode errors allowed per step */
    uint32_t   enc_invalid_max; /* consecutive invalid inputs allowed */
    uint32_t   speed_window;    /* 1..MOTION_WINDOW_MAX steps */
    int32_t    duty_step_max;   /* largest |duty| increase per step, > 0 */
} motion_cfg_t;

/** Inputs sampled by the wrapper each period. */
typedef struct
{
    uint32_t now_us;            /* free-running 1 MHz clock */
    uint32_t count_l;           /* forward-positive, free-running */
    uint32_t count_r;
    uint32_t errors_l;          /* cumulative decode errors */
    uint32_t errors_r;
    bool     b_counts_valid;    /* false if the snapshot failed */
    bool     b_stop;            /* latched STOP level */
} motion_input_t;

typedef struct
{
    uint8_t  type;              /* MOTION_EVT_* */
    uint8_t  reason;            /* MOTION_FAULT_*, MOTION_REJ_*, or 0 */
    uint16_t cmd_id;
} motion_event_t;

/** Results of one call; clear with motion_output_clear() per period. */
typedef struct
{
    int32_t        duty_l;
    int32_t        duty_r;
    int32_t        fault_duty_l;  /* exact values immediately before Fault */
    int32_t        fault_duty_r;
    int32_t        fault_speed_l;
    int32_t        fault_speed_r;
    motion_event_t events[MOTION_EVENTS_MAX];
    uint8_t        n_events;
    uint8_t        fault_state;
    uint8_t        fault_reason;
    uint8_t        stalled_wheels; /* MOTION_STALL_* mask */
    bool           b_raise_fault;   /* raise the controller FAULT bit */
} motion_output_t;

/** Controller state; private, use only through motion_*(). */
typedef struct
{
    motion_cfg_t   cfg;
    motion_state_t state;
    uint8_t        fault;
    pid_ctrl_t     pid_l;
    pid_ctrl_t     pid_r;
    pid_ctrl_t     pid_diff;    /* straight-line correction */
    int32_t        duty_l;      /* last output, held through a bad sample */
    int32_t        duty_r;
    uint16_t       res_id;      /* latest result for motion_status_t */
    uint8_t        res_type;
    uint8_t        res_reason;

    /* active command */
    uint16_t       cmd_id;
    uint16_t       last_id;
    bool           b_have_id;
    int32_t        tgt_l;       /* speed targets, mm/s */
    int32_t        tgt_r;
    int32_t        goal;        /* Move mm or Turn mdeg, signed */
    int32_t        speed;       /* Move/Turn speed magnitude */
    uint32_t       start_l;     /* counts at command start */
    uint32_t       start_r;
    uint32_t       start_us;
    uint32_t       limit_us;    /* Move/Turn timeout */

    /* stopping */
    uint8_t        pending_type;
    uint16_t       pending_id;
    uint32_t       stop_start_us;

    /* measurement */
    bool           b_have_prev;
    uint32_t       prev_l;
    uint32_t       prev_r;
    uint32_t       prev_err_l;
    uint32_t       prev_err_r;
    uint32_t       win_l[MOTION_WINDOW_MAX + 1u];
    uint32_t       win_r[MOTION_WINDOW_MAX + 1u];
    uint32_t       win_us[MOTION_WINDOW_MAX + 1u];
    uint32_t       win_head;
    uint32_t       win_fill;
    int32_t        speed_l;
    int32_t        speed_r;
    bool           b_still_run;
    uint32_t       still_since_us;
    bool           b_still;
    bool           b_stop_prev;
    bool           b_stall_l;
    bool           b_stall_r;
    uint32_t       stall_since_l;
    uint32_t       stall_since_r;
    uint32_t       invalid_count;
    bool           b_valid;     /* last input was valid */

    /* odometry */
    int32_t        tot_l;       /* signed counts since boot */
    int32_t        tot_r;
    int32_t        run_l0;      /* totals at run start */
    int32_t        run_r0;
    int32_t        x_um;
    int32_t        y_um;
    int32_t        heading_mdeg;
    uint8_t        run_id;
} motion_t;

bool motion_init (motion_t * p_m, motion_cfg_t const * p_cfg);
void motion_output_clear (motion_output_t * p_out);
void motion_start (motion_t * p_m, bool b_hw_ok, motion_output_t * p_out);
void motion_command (motion_t * p_m, s2_motion_cmd_t const * p_cmd,
                     motion_input_t const * p_in, motion_output_t * p_out);
void motion_step (motion_t * p_m, motion_input_t const * p_in,
                  motion_output_t * p_out);
void motion_fault (motion_t * p_m, uint8_t reason, motion_output_t * p_out);
void motion_get_status_logic (motion_t const * p_m,
                              motion_status_t * p_status);
bool motion_is_stopped (motion_t const * p_m);
motion_state_t motion_state (motion_t const * p_m);
int32_t motion_ramp (int32_t prev, int32_t target, int32_t step_max);
uint8_t motion_fault_code (motion_t const * p_m);

#endif /* MOTION_LOGIC_H */

/* End of file. */
