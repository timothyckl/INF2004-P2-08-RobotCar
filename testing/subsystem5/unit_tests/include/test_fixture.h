/**
 * @file test_fixture.h
 * @brief Controlled inputs and output capture for logic tests on the Pico W.
 */
#ifndef TEST_FIXTURE_H
#define TEST_FIXTURE_H

#include "../../../../app_program/subsystem5/include/test_interface.h"

#define TEST_FIXTURE_CAPACITY 128u

/** One attempted output, including failed queue deliveries. */
typedef struct
{
    test_interface_output_t output;
    uint32_t                time_us;
    bool                    b_accepted;
} test_fixture_record_t;

/** Clear time, scheduled inputs, observations and delivery failures. */
void test_fixture_reset (uint32_t start_us);

/** Access the dependency callbacks for the real subsystem under test. */
test_interface_port_t const * test_fixture_ports (void);

/** Schedule a decoded input relative to the current clock. */
bool test_fixture_schedule (uint32_t delay_us,
                             test_interface_input_t const * p_input);

/** Advance time, deliver due inputs in stable order and tick the subject. */
void test_fixture_advance (uint32_t elapsed_us,
                            test_interface_api_t const * p_api);

/** Deliver a decoded input at the current time. */
void test_fixture_send (test_interface_api_t const * p_api,
                         test_interface_input_kind_t kind,
                         uint32_t request_id, bool b_value);

/** Set whether queued distance/plan results can be delivered. */
void test_fixture_result_queue (bool b_accept);

/** Count observations of a particular output kind. */
uint32_t test_fixture_count (test_interface_output_kind_t kind);

/** Inspect an observation; NULL indicates an out-of-range index. */
test_fixture_record_t const * test_fixture_record (uint32_t index);

/** Return the total number of recorded attempts. */
uint32_t test_fixture_size (void);

/** Report capture overflow so a truncated trace cannot pass a test. */
bool test_fixture_overflowed (void);

#endif /* TEST_FIXTURE_H */

/* End of file. */
