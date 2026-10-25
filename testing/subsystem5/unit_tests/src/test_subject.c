/**
 * @file test_subject.c
 * @brief Default absent binding, not a fake implementation of subsystem 5.
 */
#include "../include/test_subject.h"

#include "../include/test_fixture.h"
#include <stddef.h>

#ifndef TEST_SUBJECT_BOUND
test_interface_api_t const *
test_subject_get (void)
{
    return (NULL);
}
#endif

bool
test_subject_valid (test_interface_api_t const * p_api)
{
    bool b_valid = false;

    if (NULL != p_api)
    {
        b_valid = (NULL != p_api->p_reset_fixture)
                  && (NULL != p_api->p_pulse_mm)
                  && (NULL != p_api->p_dispatch);
    }

    return (b_valid);
}

void
test_subject_start (test_interface_api_t const * p_api, uint32_t time_us)
{
    /* These values exercise logic; they are not servo calibration evidence. */
    test_interface_config_t const config = {1000u, 200u};

    test_fixture_reset(time_us);
    p_api->p_reset_fixture(test_fixture_ports(), &config);
}

bool
test_subject_forward_only (void)
{
    bool b_forward = true;

    for (uint32_t index = 0u; index < test_fixture_size(); index++)
    {
        test_fixture_record_t const * p_record = test_fixture_record(index);

        if ((TEST_INTERFACE_SERVO == p_record->output.kind)
            && (0 != p_record->output.angle_deg))
        {
            b_forward = false;
        }
    }

    return (b_forward && (!test_fixture_overflowed()));
}

bool
test_subject_no_plan (void)
{
    return ((0u == test_fixture_count(TEST_INTERFACE_PLAN))
            && (!test_fixture_overflowed()));
}

/* End of file. */
