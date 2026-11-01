/**
 * @file motion_cmd.h
 * @brief motion_q command format: Vehicle Controller to Motion Control.
 *
 * PRIVATE to subsystem 2: both ends are subsystem 2 tasks, so no other
 * subsystem includes this file. PROVISIONAL.
 *
 * Stale commands: Motion Control rejects every command while the latched
 * STOP (S2_SFLG_STOP) is set, and the controller drains motion_q before it
 * clears STOP. The controller is the only sender, so nothing older can
 * arrive afterwards. cmd_id increases by one per command (wrapping) so
 * results in sensor_q can be matched, and Motion Control also rejects an
 * id that is not newer than the last one it handled. STOP is obeyed
 * whatever its id. Build commands with motion_api.c, not by hand.
 */
#ifndef MOTION_CMD_H
#define MOTION_CMD_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#define S2_CMD_SPEED    1u  /* hold per-wheel speeds until replaced */
#define S2_CMD_MOVE     2u  /* drive a signed distance, then stop */
#define S2_CMD_TURN     3u  /* rotate in place by a signed angle */
#define S2_CMD_STOP     4u  /* brake and confirm stop */
#define S2_CMD_RESET    5u  /* leave Fault; needs wheels stopped */
#define S2_CMD_NEW_RUN  6u  /* Idle only: zero the pose, run_id + 1 */

/* Differential speeds let the controller steer while line following
   without a separate steering command. */
typedef struct
{
    int16_t left_mm_s;
    int16_t right_mm_s;
} s2_cmd_speed_t;

typedef struct
{
    int16_t distance_mm;    /* negative = reverse; +/-32.7 m */
    int16_t speed_mm_s;     /* magnitude; sign taken from distance */
} s2_cmd_move_t;

typedef struct
{
    int32_t angle_mdeg;     /* positive = left */
    int16_t speed_mm_s;     /* wheel speed magnitude */
    int16_t reserved;
} s2_cmd_turn_t;

typedef struct
{
    uint32_t time_ms;       /* send time */
    uint16_t cmd_id;
    uint8_t  cmd;           /* S2_CMD_* */
    uint8_t  reserved;
    union
    {
        s2_cmd_speed_t speed;
        s2_cmd_move_t  move;
        s2_cmd_turn_t  turn;
    } u;
} s2_motion_cmd_t;

IMPORT ID motion_q;

/* Build one command (motion_api.c). value is the signed distance (mm)
   for MOVE or signed angle (mdeg, left positive) for TURN, ignored
   otherwise. Returns false, leaving *p_cmd unchanged, if out of range. */
bool motion_api_make (uint8_t cmd, int32_t value, uint16_t cmd_id,
                      s2_motion_cmd_t * p_cmd);

#endif /* MOTION_CMD_H */

/* End of file. */
