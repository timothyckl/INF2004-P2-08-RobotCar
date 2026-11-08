/**
 * @file scan_task.h
 * @brief Obstacle Scan task and its interface to the Vehicle Controller.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 *
 * Inputs from the controller arrive on scan_q (owned by subsystem 2,
 * s2_scan_cmd_t in subsystem2.h): REQUEST, AUTHORISE, STOP_CONFIRM,
 * CANCEL, STOP, MOVEMENT and RESET for one request ID at a time.
 *
 * Outputs to the controller:
 * - CTRL_FLG_HAZARD when the forward verdict turns to a stop while
 *   robot_state reports a moving mission mode (decision 3);
 * - CTRL_FLG_SCAN_FAULT when a scan faults;
 * - scan_task_get(): the forward reading and the latest scan result,
 *   with the bypass plan when one was made.
 * Only the scan task drives the servo (PWM slice 6) and the HC-SR04.
 */
#ifndef SCAN_TASK_H
#define SCAN_TASK_H

#include <stdint.h>

#include "../../app.h"
#include "bypass_plan.h"

#define SCAN_TASK_FLAG_MONITORING   (0x01u)    /* forward readings live */
#define SCAN_TASK_FLAG_SCANNING     (0x02u)
#define SCAN_TASK_FLAG_HAZARD       (0x04u)    /* verdict is not SAFE */
#define SCAN_TASK_FLAG_REQUEST_PENDING (0x08u)

#define SCAN_TASK_RESULT_NONE       (0u)
#define SCAN_TASK_RESULT_PLAN       (1u)       /* plan is usable */
#define SCAN_TASK_RESULT_CLEAR      (2u)       /* nothing in range ahead */
#define SCAN_TASK_RESULT_NO_ROUTE   (3u)
#define SCAN_TASK_RESULT_FAULT      (4u)
#define SCAN_TASK_RESULT_CANCELLED  (5u)

/** Snapshot for the Vehicle Controller and telemetry. */
typedef struct
{
    uint32_t      captured_ms;
    uint32_t      request_id;   /* request the result belongs to */
    uint32_t      result_us;    /* native clock when PLAN or CLEAR was made */
    uint16_t      forward_mm;   /* latest valid forward range, 0 if none */
    uint16_t      forward_age_ms; /* latest attempt's age, saturated */
    uint8_t       verdict;      /* forward_monitor_verdict_t */
    uint8_t       flags;        /* SCAN_TASK_FLAG_* */
    uint8_t       result;       /* SCAN_TASK_RESULT_* for request_id */
    uint8_t       invalid_count;    /* consecutive invalid forward tries */
    uint8_t       reserved[2];
    bypass_plan_t plan;         /* meaningful when result is PLAN */
} scan_task_status_t;

/** Create the status lock and the sampling clock (scan_lifecycle_init). */
ER scan_task_create (void);

/** Start the drivers, the task and its clock (scan_lifecycle_start). */
ER scan_task_start (void);

/** Copy the latest snapshot without waiting; E_TMOUT if busy. */
ER scan_task_get (scan_task_status_t * p_status);

#endif /* SCAN_TASK_H */

/* End of file. */
