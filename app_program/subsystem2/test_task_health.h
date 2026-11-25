/**
 * @file test_task_health.h
 * @brief Tests for the task-liveness evaluator (task_health.c).
 */
#ifndef TEST_TASK_HEALTH_H
#define TEST_TASK_HEALTH_H

#include <stdbool.h>

#if APP_TEST

bool test_task_health_evaluate(void);   /* S2-U81 */

#endif

#endif /* TEST_TASK_HEALTH_H */

/* End of file. */
