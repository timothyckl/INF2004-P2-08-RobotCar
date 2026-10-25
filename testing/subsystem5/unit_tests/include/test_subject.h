/**
 * @file test_subject.h
 * @brief Binding point for real logic and an isolated Pico W test fixture.
 */
#ifndef TEST_SUBJECT_H
#define TEST_SUBJECT_H

#include "../../../../app_program/subsystem5/include/test_interface.h"

/** Return NULL until an adapter to actual subsystem logic is available. */
test_interface_api_t const * test_subject_get (void);

/** Reject incomplete bindings before any test can dereference callbacks. */
bool test_subject_valid (test_interface_api_t const * p_api);

/** Reset dependencies and the actual subject with synthetic parameters. */
void test_subject_start (test_interface_api_t const * p_api, uint32_t time_us);

/** Whether the captured trace contains only forward-facing servo commands. */
bool test_subject_forward_only (void);

/** Whether no bypass plan was emitted. */
bool test_subject_no_plan (void);

#endif /* TEST_SUBJECT_H */

/* End of file. */
