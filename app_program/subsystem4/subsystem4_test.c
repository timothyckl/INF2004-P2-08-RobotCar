/**
 * @file subsystem4_test.c
 * @brief On-target standalone tests for Subsystem 4 terrain logic.
 */
#include "subsystem4.h"
#include "terrain_logic.h"

#if APP_TEST
#include "../test_report.h"

#define S4_TEST_SAMPLE_PERIOD_US    10000u
#define S4_TEST_TILT_MG             400

static s4_terrain_config_t s4_test_config(void);
static void s4_test_calibrate(s4_terrain_context_t * p_context,
                              uint32_t start_time_us);
static void s4_test_feed(s4_terrain_context_t * p_context,
                         int16_t acceleration_x_mg,
                         uint32_t capture_time_us,
                         uint16_t distance_delta_mm,
                         bool b_distance_valid);
static void s4_test_complete_hump(s4_terrain_context_t * p_context,
                                  uint32_t * p_time_us,
                                  uint16_t distance_delta_mm,
                                  bool b_distance_valid);
static bool s4_test_calibration_ready(void);
static bool s4_test_calibration_fault(void);
static bool s4_test_rejects_spike(void);
static bool s4_test_detects_hump(void);
static bool s4_test_requires_descent(void);
static bool s4_test_invalid_limit(void);
static bool s4_test_invalid_streak_clears(void);
static bool s4_test_time_wrap(void);
static bool s4_test_stale_fault(void);
static bool s4_test_maximum_height(void);
static bool s4_test_recovery_retains_max(void);
static bool s4_test_new_run_clears_max(void);
static bool s4_test_height_needs_distance(void);
static bool s4_test_impact_edge(void);
static bool s4_test_null_safety(void);

void
subsystem4_test (void)
{
    TEST_INIT();

    TEST_CHECK("S4-U03", s4_test_calibration_ready(),
               "stable samples complete calibration");
    TEST_CHECK("S4-U04", s4_test_calibration_fault(),
               "unstable samples cause calibration fault");
    TEST_CHECK("S4-U05", s4_test_rejects_spike(),
               "single tilt spike is not a hump");
    TEST_CHECK("S4-U06", s4_test_detects_hump(),
               "ascent descent level completes one hump");
    TEST_CHECK("S4-U07", s4_test_requires_descent(),
               "ascent returning to level remains incomplete");
    TEST_CHECK("S4-U08", s4_test_invalid_limit(),
               "consecutive invalid samples cause sensor fault");
    TEST_CHECK("S4-U09", s4_test_invalid_streak_clears(),
               "valid sample clears invalid streak");
    TEST_CHECK("S4-U10", s4_test_time_wrap(),
               "timestamp wrap does not cause stale fault");
    TEST_CHECK("S4-U11", s4_test_stale_fault(),
               "excess sample gap causes stale fault");
    TEST_CHECK("S4-U12", s4_test_maximum_height(),
               "larger valid hump becomes run maximum");
    TEST_CHECK("S4-U13", s4_test_recovery_retains_max(),
               "local recovery preserves run maximum");
    TEST_CHECK("S4-U14", s4_test_new_run_clears_max(),
               "new run clears run maximum");
    TEST_CHECK("S4-U15", s4_test_height_needs_distance(),
               "missing distance leaves height invalid");
    TEST_CHECK("S4-U16", s4_test_impact_edge(),
               "impact is reported once per threshold crossing");
    TEST_CHECK("S4-U17", s4_test_null_safety(),
               "null API inputs do not crash");

    TEST_SUMMARY("subsystem4");
}

static s4_terrain_config_t
s4_test_config (void)
{
    s4_terrain_config_t config =
    {
        .calibration_samples = 4u,
        .calibration_span_mg = 10u,
        .ascent_threshold_mg = 100u,
        .level_threshold_mg = 40u,
        .impact_threshold_mg = 700u,
        .max_sample_gap_ms = 100u,
        .confirmation_samples = 2u,
        .invalid_limit = 2u
    };

    return (config);
}

static void
s4_test_calibrate (s4_terrain_context_t * p_context,
                   uint32_t start_time_us)
{
    s4_terrain_config_t config = s4_test_config();
    uint32_t time_us = start_time_us;

    s4_terrain_init(p_context, &config);
    for (uint16_t index = 0u; index < config.calibration_samples; index++)
    {
        s4_test_feed(p_context, 0, time_us, 0u, false);
        time_us += S4_TEST_SAMPLE_PERIOD_US;
    }
}

static void
s4_test_feed (s4_terrain_context_t * p_context,
              int16_t acceleration_x_mg,
              uint32_t capture_time_us,
              uint16_t distance_delta_mm,
              bool b_distance_valid)
{
    s4_terrain_sample_t sample =
    {
        .acceleration_x_mg = acceleration_x_mg,
        .acceleration_y_mg = 0,
        .acceleration_z_mg = 1000,
        .capture_time_us = capture_time_us,
        .b_valid = true
    };

    s4_terrain_process(p_context, &sample, distance_delta_mm,
                       b_distance_valid);
}

static void
s4_test_complete_hump (s4_terrain_context_t * p_context,
                       uint32_t * p_time_us,
                       uint16_t distance_delta_mm,
                       bool b_distance_valid)
{
    for (uint8_t index = 0u; index < 4u; index++)
    {
        *p_time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(p_context, S4_TEST_TILT_MG, *p_time_us,
                     distance_delta_mm, b_distance_valid);
    }

    for (uint8_t index = 0u; index < 2u; index++)
    {
        *p_time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(p_context, -S4_TEST_TILT_MG, *p_time_us,
                     0u, b_distance_valid);
    }

    for (uint8_t index = 0u; index < 2u; index++)
    {
        *p_time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(p_context, 0, *p_time_us, 0u,
                     b_distance_valid);
    }
}

static bool
s4_test_calibration_ready (void)
{
    s4_terrain_context_t context;

    s4_test_calibrate(&context, 0u);

    return ((S4_TERRAIN_STATE_MONITORING == context.result.state) &&
            (S4_TERRAIN_EVENT_READY == context.result.events) &&
            (S4_TERRAIN_FAULT_NONE == context.result.fault));
}

static bool
s4_test_calibration_fault (void)
{
    s4_terrain_context_t context;
    s4_terrain_config_t config = s4_test_config();

    s4_terrain_init(&context, &config);
    s4_test_feed(&context, 0, 0u, 0u, false);
    s4_test_feed(&context, 20, 10000u, 0u, false);
    s4_test_feed(&context, 0, 20000u, 0u, false);
    s4_test_feed(&context, 0, 30000u, 0u, false);

    return ((S4_TERRAIN_STATE_FAULT == context.result.state) &&
            (S4_TERRAIN_FAULT_CALIBRATION == context.result.fault));
}

static bool
s4_test_rejects_spike (void)
{
    s4_terrain_context_t context;

    s4_test_calibrate(&context, 0u);
    s4_test_feed(&context, S4_TEST_TILT_MG, 40000u, 0u, false);
    s4_test_feed(&context, 0, 50000u, 0u, false);

    return (S4_TERRAIN_STATE_MONITORING == context.result.state);
}

static bool
s4_test_detects_hump (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    s4_test_complete_hump(&context, &time_us, 50u, true);

    return ((S4_TERRAIN_STATE_MONITORING == context.result.state) &&
            (0u != (context.result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE)) &&
            context.result.b_current_height_valid);
}

static bool
s4_test_requires_descent (void)
{
    s4_terrain_context_t context;

    s4_test_calibrate(&context, 0u);
    s4_test_feed(&context, S4_TEST_TILT_MG, 40000u, 0u, true);
    s4_test_feed(&context, S4_TEST_TILT_MG, 50000u, 0u, true);
    s4_test_feed(&context, 0, 60000u, 0u, true);
    s4_test_feed(&context, 0, 70000u, 0u, true);

    return ((S4_TERRAIN_STATE_ASCENDING == context.result.state) &&
            (0u == (context.result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE)));
}

static bool
s4_test_invalid_limit (void)
{
    s4_terrain_context_t context;
    s4_terrain_sample_t invalid = {0};

    s4_test_calibrate(&context, 0u);
    s4_terrain_process(&context, &invalid, 0u, false);
    s4_terrain_process(&context, &invalid, 0u, false);

    return ((S4_TERRAIN_STATE_FAULT == context.result.state) &&
            (S4_TERRAIN_FAULT_SENSOR == context.result.fault) &&
            (!context.result.b_current_height_valid));
}

static bool
s4_test_invalid_streak_clears (void)
{
    s4_terrain_context_t context;
    s4_terrain_sample_t invalid = {0};

    s4_test_calibrate(&context, 0u);
    s4_terrain_process(&context, &invalid, 0u, false);
    s4_test_feed(&context, 0, 40000u, 0u, false);
    s4_terrain_process(&context, &invalid, 0u, false);

    return (S4_TERRAIN_STATE_MONITORING == context.result.state);
}

static bool
s4_test_time_wrap (void)
{
    s4_terrain_context_t context;
    uint32_t start_time_us = UINT32_MAX - 20000u;

    s4_test_calibrate(&context, start_time_us);
    s4_test_feed(&context, 0,
                 start_time_us + (4u * S4_TEST_SAMPLE_PERIOD_US),
                 0u, false);

    return (S4_TERRAIN_STATE_MONITORING == context.result.state);
}

static bool
s4_test_stale_fault (void)
{
    s4_terrain_context_t context;

    s4_test_calibrate(&context, 0u);
    s4_test_feed(&context, 0, 200000u, 0u, false);

    return ((S4_TERRAIN_STATE_FAULT == context.result.state) &&
            (S4_TERRAIN_FAULT_STALE == context.result.fault));
}

static bool
s4_test_maximum_height (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    s4_test_complete_hump(&context, &time_us, 50u, true);
    s4_test_complete_hump(&context, &time_us, 100u, true);

    return (context.result.b_maximum_height_valid &&
            (80u == context.result.maximum_height_mm));
}

static bool
s4_test_recovery_retains_max (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    s4_test_complete_hump(&context, &time_us, 50u, true);
    s4_terrain_request_recovery(&context);

    return ((S4_TERRAIN_STATE_CALIBRATING == context.result.state) &&
            context.result.b_maximum_height_valid &&
            (40u == context.result.maximum_height_mm));
}

static bool
s4_test_new_run_clears_max (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    s4_test_complete_hump(&context, &time_us, 50u, true);
    s4_terrain_start_new_run(&context);

    return ((!context.result.b_maximum_height_valid) &&
            (0u == context.result.maximum_height_mm));
}

static bool
s4_test_height_needs_distance (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    s4_test_complete_hump(&context, &time_us, 50u, false);

    return ((0u != (context.result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE)) &&
            (!context.result.b_current_height_valid));
}

static bool
s4_test_impact_edge (void)
{
    s4_terrain_context_t context;
    bool b_first_event;

    s4_test_calibrate(&context, 0u);
    s4_test_feed(&context, 800, 40000u, 0u, false);
    b_first_event = 0u !=
        (context.result.events & S4_TERRAIN_EVENT_IMPACT);
    s4_test_feed(&context, 800, 50000u, 0u, false);

    return (b_first_event &&
            (0u == (context.result.events & S4_TERRAIN_EVENT_IMPACT)));
}

static bool
s4_test_null_safety (void)
{
    s4_terrain_context_t context;
    s4_terrain_config_t config = s4_test_config();
    s4_terrain_sample_t sample = {0};

    s4_terrain_init(NULL, &config);
    s4_terrain_process(NULL, &sample, 0u, false);
    s4_terrain_init(&context, NULL);
    s4_terrain_process(&context, NULL, 0u, false);
    s4_terrain_request_recovery(NULL);
    s4_terrain_start_new_run(NULL);

    return ((NULL == s4_terrain_get_result(NULL)) &&
            (S4_TERRAIN_STATE_FAULT == context.result.state));
}
#endif

/* End of file. */
