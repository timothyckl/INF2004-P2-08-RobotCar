/**
 * @file test_line_follower.c
 * @brief Host executable for deterministic line-follower logic tests.
 */

#include "../../../app_program/subsystem3/line_follower.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static line_follower_config_t test_config(void);
static bool test_centred(void);
static bool test_pd_steering(void);
static bool test_junction(void);
static bool test_recovery(void);
static bool test_limits(void);
static bool test_invalid_config(void);
static int test_report(char const * p_name, bool b_passed);

int
main (void)
{
    int failures = 0;

    failures += test_report("centred", test_centred());
    failures += test_report("pd_steering", test_pd_steering());
    failures += test_report("junction", test_junction());
    failures += test_report("recovery", test_recovery());
    failures += test_report("limits", test_limits());
    failures += test_report("invalid_config", test_invalid_config());

    return (failures);
}

static line_follower_config_t
test_config (void)
{
    line_follower_config_t const config =
    {
        .minimum_speed        = 0U,
        .base_speed           = 500U,
        .maximum_speed        = 1000U,
        .maximum_correction   = 300U,
        .proportional_gain_q8 = 25600U,
        .derivative_gain_q8   = 6400U,
        .junction_minimum_mm  = 4U,
        .decision_samples     = 2U,
        .junction_samples     = 3U,
        .reacquire_samples    = 2U
    };

    return (config);
}

static bool
test_centred (void)
{
    line_follower_config_t const config = test_config();
    line_follower_t follower = {0};
    line_follower_sample_t sample = {0};
    line_follower_output_t output = {0};
    bool b_passed = line_follower_init(&follower, &config);

    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (LINE_FOLLOWER_POSITION_CENTRED == output.position) &&
               (config.base_speed == output.left_target) &&
               (config.base_speed == output.right_target);

    return (b_passed);
}

static bool
test_pd_steering (void)
{
    line_follower_config_t const config = test_config();
    line_follower_t follower = {0};
    line_follower_sample_t sample = {0};
    line_follower_output_t output = {0};
    bool b_passed = line_follower_init(&follower, &config);

    sample.b_right_black = true;
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (LINE_FOLLOWER_POSITION_ROBOT_LEFT == output.position) &&
               (125 == output.correction) &&
               (625U == output.left_target) &&
               (375U == output.right_target);

    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed && (100 == output.correction);

    return (b_passed);
}

static bool
test_junction (void)
{
    line_follower_config_t const config = test_config();
    line_follower_t follower = {0};
    line_follower_sample_t sample =
    {
        .b_left_black    = true,
        .b_right_black   = true,
        .travel_delta_mm = 1U
    };
    line_follower_output_t output = {0};
    bool b_passed = line_follower_init(&follower, &config);

    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (LINE_FOLLOWER_EVENT_NONE == output.events);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    sample.travel_delta_mm = 2U;
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (LINE_FOLLOWER_STATE_JUNCTION == output.state) &&
               (0U != (output.events & LINE_FOLLOWER_EVENT_JUNCTION));

    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (LINE_FOLLOWER_EVENT_NONE == output.events);

    return (b_passed);
}

static bool
test_recovery (void)
{
    line_follower_config_t const config = test_config();
    line_follower_t follower = {0};
    line_follower_sample_t sample = {0};
    line_follower_output_t output = {0};
    bool b_passed = line_follower_init(&follower, &config);

    b_passed = b_passed && line_follower_start_recovery(
        &follower, LINE_FOLLOWER_DIRECTION_RIGHT);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (0U != (output.events & LINE_FOLLOWER_EVENT_LINE_LOST)) &&
               (!output.b_line_valid) &&
               (output.left_target > output.right_target);

    sample.b_right_black = true;
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    sample.b_right_black = false;
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (0U != (output.events & LINE_FOLLOWER_EVENT_REACQUIRED)) &&
               output.b_line_valid;

    return (b_passed);
}

static bool
test_limits (void)
{
    line_follower_config_t config = test_config();
    line_follower_t follower = {0};
    line_follower_sample_t sample = {0};
    line_follower_output_t output = {0};
    bool b_passed = false;

    config.base_speed = 900U;
    config.proportional_gain_q8 = UINT16_MAX;
    config.derivative_gain_q8 = UINT16_MAX;
    sample.b_right_black = true;

    b_passed = line_follower_init(&follower, &config);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (config.maximum_speed == output.left_target) &&
               (config.minimum_speed <= output.right_target) &&
               (300 == output.correction);

    return (b_passed);
}

static bool
test_invalid_config (void)
{
    line_follower_config_t config = test_config();
    line_follower_t follower = {0};

    config.maximum_correction = UINT16_MAX;

    return (!line_follower_init(&follower, &config));
}

static int
test_report (char const * p_name, bool b_passed)
{
    int result = 1;

    if (b_passed)
    {
        result = 0;
        printf("PASS %s\n", p_name);
    }
    else
    {
        printf("FAIL %s\n", p_name);
    }

    return (result);
}

/* End of file. */
