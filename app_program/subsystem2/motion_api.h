/**
 * @file motion_api.h
 * @brief Subsystem 2 Motion Control API (Buddy 2: moveForward and so on).
 *
 * PROVISIONAL until agreed in TEAM.md section 5.
 *
 * Command functions build one motion_q command and send it without
 * blocking. They never touch PWM: only the Motion Control task does.
 * Call them from the Vehicle Controller task only (the one commander, so
 * command IDs stay in order). Each returns:
 *   E_OK    queued; motion_last_command_id() gives its ID
 *   E_PAR   argument out of range, nothing sent
 *   E_OBJ   motion_q not created yet (before subsystem2_init())
 *   E_TMOUT motion_q full, nothing sent
 * A queued command can still be rejected by Motion Control (for example
 * while STOP is latched); the result appears in motion_get_status().
 *
 * Units: mm and millidegrees. Turns are in place. Speeds come from
 * motion_config.h (UNMEASURED).
 *
 * Status (motion_get_status, motion_stop_confirmed) may be read from any
 * task; it is copied under a mutex with priority inheritance.
 */
#ifndef MOTION_API_H
#define MOTION_API_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#define MOTION_DISTANCE_MAX_MM  32767u      /* int16_t command field */
#define MOTION_ANGLE_MAX_MDEG   360000u     /* one full turn */

/* motion_status_t.last_result */
#define MOTION_EVT_NONE         0u
#define MOTION_EVT_READY        1u
#define MOTION_EVT_DONE         2u      /* reached target, wheels still */
#define MOTION_EVT_STOPPED      3u      /* stop confirmed, wheels still */
#define MOTION_EVT_REJECTED     4u      /* last_reason says why */
#define MOTION_EVT_FAULT        5u

/* Fault codes (motion_status_t.fault, and last_reason of a FAULT). */
#define MOTION_FAULT_NONE       0u
#define MOTION_FAULT_INIT       1u      /* driver start-up failed */
#define MOTION_FAULT_ENCODER    2u
#define MOTION_FAULT_STALL      3u
#define MOTION_FAULT_TIMEOUT    4u      /* Move/Turn took too long */
#define MOTION_FAULT_STOP_TIMEOUT 5u    /* not still within 2 s */
#define MOTION_FAULT_EXTERNAL   6u      /* raised by the task wrapper */

/* Reject reasons (last_reason of a REJECTED), above the fault codes. */
#define MOTION_REJ_STOP         0x80u   /* latched STOP is set */
#define MOTION_REJ_STALE        0x81u   /* ID not newer than the last */
#define MOTION_REJ_PARAM        0x82u   /* parameter out of range */
#define MOTION_REJ_STATE        0x83u   /* not allowed in this state */

/* motion_status_t.flags */
#define MOTION_ST_VALID         0x01u   /* encoders healthy */
#define MOTION_ST_STOPPED       0x02u   /* both wheels still >= 100 ms */

/** Motion Control snapshot, updated every control period. */
typedef struct
{
    uint32_t captured_ms;
    int32_t  travel_mm;     /* signed centre travel since boot */
    int16_t  x_mm;          /* pose since NEW_RUN: x forward */
    int16_t  y_mm;          /* y left */
    int32_t  heading_mdeg;  /* left positive, -180000 < h <= 180000 */
    int16_t  left_mm_s;
    int16_t  right_mm_s;
    uint16_t last_id;       /* command of the latest result */
    uint8_t  last_result;   /* MOTION_EVT_* */
    uint8_t  last_reason;   /* MOTION_FAULT_* or MOTION_REJ_* */
    uint8_t  state;         /* motion_state_t */
    uint8_t  fault;         /* MOTION_FAULT_* while in Fault */
    uint8_t  run_id;        /* +1 per NEW_RUN, wraps */
    uint8_t  flags;         /* MOTION_ST_* */
} motion_status_t;

ER       motion_move_forward (uint16_t distance_mm);
ER       motion_move_backward (uint16_t distance_mm);
ER       motion_turn_left (uint32_t angle_mdeg);
ER       motion_turn_right (uint32_t angle_mdeg);
ER       motion_stop (void);
ER       motion_reset (void);       /* leave Fault; wheels must be still */
ER       motion_new_run (void);     /* Idle only: zero pose, run_id + 1 */
uint16_t motion_last_command_id (void);

ER       motion_get_status (motion_status_t * p_status);
bool     motion_stop_confirmed (void);

#endif /* MOTION_API_H */

/* End of file. */
