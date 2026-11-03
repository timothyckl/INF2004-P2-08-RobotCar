/**
 * @file subsystem3_test.c
 * @brief On-target unit and starter-integration tests for subsystem 3.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

static APP_STATS subsystem3_read_stats(void);
static line_follower_config_t subsystem3_test_config(void);
static bool subsystem3_test_steering(void);
static bool subsystem3_test_junction(void);
static bool subsystem3_test_recovery(void);
static bool subsystem3_test_limits(void);

EXPORT void
subsystem3_test (void)
{
    APP_STATS before = subsystem3_read_stats();
    APP_STATS after = {0};
    line_follower_config_t const config = subsystem3_test_config();
    line_follower_t follower = {0};
    line_follower_sample_t centred_sample = {0};
    line_follower_output_t centred_output = {0};
    line_follower_config_t invalid_config = config;

    TEST_INIT();

    tk_dly_tsk(2000);
    after = subsystem3_read_stats();
    TEST_CHECK("INT-01",
               after.received >= (before.received + 5U),
               "starter samples flow from subsystem 2 to subsystem 3");

    TEST_CHECK("S3-U01",
               (0U < after.received) && (after.min <= after.max) &&
               (0 <= after.min) && (100 >= after.max),
               "starter statistics remain self-consistent");

    TEST_CHECK("S3-U02",
               line_follower_init(&follower, &config) &&
               line_follower_step(&follower,
                                  &centred_sample,
                                  &centred_output) &&
               (LINE_FOLLOWER_POSITION_CENTRED ==
                centred_output.position) &&
               (config.base_speed == centred_output.left_target) &&
               (config.base_speed == centred_output.right_target),
               "both-white centred input produces equal targets");

    TEST_CHECK("S3-U03",
               subsystem3_test_steering(),
               "PD correction steers toward the detected line edge");

    TEST_CHECK("S3-U04",
               subsystem3_test_limits(),
               "large corrections remain inside configured speed limits");

    TEST_CHECK("S3-U05",
               subsystem3_test_junction(),
               "junction requires sensor history and encoder travel");

    TEST_CHECK("S3-U06",
               subsystem3_test_recovery(),
               "recovery reports loss and confirms line reacquisition");

    invalid_config.maximum_correction = UINT16_MAX;
    TEST_CHECK("S3-U07",
               (!line_follower_init(NULL, &config)) &&
               (!line_follower_init(&follower, &invalid_config)),
               "invalid storage and unsafe correction range are rejected");

    TEST_SUMMARY("subsystem3");
}

static APP_STATS
subsystem3_read_stats (void)
{
    APP_STATS statistics = {0};

    tk_loc_mtx(mtx_stats, TMO_FEVR);
    statistics = app_stats;
    tk_unl_mtx(mtx_stats);

    return (statistics);
}

static line_follower_config_t
subsystem3_test_config (void)
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
subsystem3_test_steering (void)
{
    line_follower_config_t const config = subsystem3_test_config();
    line_follower_t follower = {0};
    line_follower_sample_t sample = {0};
    line_follower_output_t output = {0};
    bool b_passed = line_follower_init(&follower, &config);

    sample.b_right_black = true;
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (LINE_FOLLOWER_POSITION_ROBOT_LEFT == output.position) &&
               (output.left_target > output.right_target);

    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed && (100 == output.correction);

    return (b_passed);
}

static bool
subsystem3_test_junction (void)
{
    line_follower_config_t const config = subsystem3_test_config();
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

    return (b_passed);
}

static bool
subsystem3_test_recovery (void)
{
    line_follower_config_t const config = subsystem3_test_config();
    line_follower_t follower = {0};
    line_follower_sample_t sample = {0};
    line_follower_output_t output = {0};
    bool b_passed = line_follower_init(&follower, &config);

    b_passed = b_passed && line_follower_start_recovery(
        &follower, LINE_FOLLOWER_DIRECTION_RIGHT);
    b_passed = b_passed && line_follower_step(&follower, &sample, &output);
    b_passed = b_passed &&
               (0U != (output.events & LINE_FOLLOWER_EVENT_LINE_LOST)) &&
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
subsystem3_test_limits (void)
{
    line_follower_config_t config = subsystem3_test_config();
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

#endif /* APP_TEST */

/* End of file. */
