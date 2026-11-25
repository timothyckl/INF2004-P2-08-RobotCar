/**
 * @file task_health.h
 * @brief Liveness of the safety-critical tasks (S2 decision 37).
 *
 * Each monitored task calls s2_health_check_in() once per pass of its own
 * loop. Motion Control, which alone writes motor PWM, calls
 * s2_health_guard() every period: a task that has checked in at least once
 * and not since its limit puts Motion Control into Fault (MOTION_FAULT_TASK),
 * so the motors stop at once and stay stopped until RESET, and the
 * hardware watchdog (S2_WATCHDOG=1) is no longer fed.
 *
 * This catches a task that blocks or waits for ever. A task that never
 * yields the CPU also starves Motion Control (priority 15), so only the
 * hardware watchdog catches that.
 *
 * Concurrency: each check-in time has one writer, its own task, and is a
 * single aligned 32-bit store (atomic on the Cortex-M0+); "seen" is set
 * only after the time, and Motion Control reads "seen" first, so it never
 * uses a time that was not written. No lock is needed.
 */
#ifndef TASK_HEALTH_H
#define TASK_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_logic.h"

typedef enum
{
    S2_HEALTH_CONTROLLER = 0,   /* Vehicle Controller, every <= 10 ms */
    S2_HEALTH_LINE,             /* S3 line and barcode loop, every 10 ms */
    S2_HEALTH_SCAN,             /* S5 obstacle scan task, every 10 ms */
    S2_HEALTH_IMU,              /* S4 IMU task, every 20 to 30 ms */
    S2_HEALTH_TASKS
} s2_health_task_t;

/* Stall limits. The guard is a backstop: where a more specific check
   already stops the car, its limit is longer, so that check acts first
   and names the cause.
   - Controller and S5 scan task: 500 ms, the controller's own limit for a
     missing line report (CTRL_LINE_TIMEOUT_MS), 50 periods of these 10 ms
     tasks. Nothing else notices a stalled controller (Motion Control
     would hold its last speeds) or a stalled forward monitor (no hazard
     would be raised while following).
   - S3 line loop: 1000 ms. A stopped loop already latches STOP with cause
     line after 500 ms of missing reports (repeated every 30 ms).
   - S4 IMU task: 1500 ms. It retries a failed sensor every
     S4_RETRY_PERIOD_MS (1000 ms) and checks in on each retry.
   UNMEASURED: no task's longest normal gap has been logged; raise a
   limit, never drop the check, if the car trips on a healthy task. */
#define S2_HEALTH_LIMIT_MS      500u
#define S2_HEALTH_LINE_LIMIT_MS 1000u
#define S2_HEALTH_IMU_LIMIT_MS  1500u

/** One task's check-in record (the evaluator's input). */
typedef struct
{
    uint32_t last_ms;   /* kernel ms clock at the last check-in */
    bool     b_seen;    /* checked in at least once since boot */
} s2_health_record_t;

/** Record that the calling task completed a pass (kernel ms clock). */
void s2_health_check_in (s2_health_task_t task);

/** Copy the check-in records (Motion Control, and tests). */
void s2_health_read (s2_health_record_t p_records[S2_HEALTH_TASKS]);

/**
 * Pure: bit (1 << task) set for each task that has been seen and whose
 * last check-in is more than its limit before now_ms. Wrap-safe (modulo
 * 2^32 differences); a task never seen is not monitored.
 */
uint8_t s2_health_evaluate (
    s2_health_record_t const p_records[S2_HEALTH_TASKS], uint32_t now_ms);

/**
 * Pure: evaluate the records and, if any task has stalled, put Motion
 * Control into Fault (MOTION_FAULT_TASK; the output duties become 0; a
 * Fault already present keeps its first cause). Returns the stalled mask.
 */
uint8_t s2_health_apply (
    s2_health_record_t const p_records[S2_HEALTH_TASKS], uint32_t now_ms,
    motion_t * p_motion, motion_output_t * p_out);

/**
 * Motion Control, each period after motion_step(): s2_health_apply() on
 * the live check-in records. The caller feeds the hardware watchdog only
 * when the returned mask is 0.
 */
uint8_t s2_health_guard (motion_t * p_motion, motion_output_t * p_out,
                         uint32_t now_ms);

/** Printable name of one task, for the stall message. */
char const * s2_health_name (s2_health_task_t task);

#endif /* TASK_HEALTH_H */

/* End of file. */
