/**
 * @file subsystem4_test.c
 * @brief On-target tests for Subsystem 4 terrain logic and decoding.
 *
 * Hump inputs come from the contact geometry (decision 8): a wheel raised
 * h mm rolls the car h/track and pitches it -h/(2 x wheelbase); both
 * wheels raised pitch it -h/wheelbase; the caster raised, +h/wheelbase.
 * With track 119 mm and wheelbase 88 mm, 10 mm is 84 mg roll with -57 mg
 * pitch, or 114 mg pitch.
 */
#include "subsystem4_test_internal.h"

#if APP_TEST
#define S4_TEST_SAMPLE_PERIOD_US    10000u
#define S4_TEST_WHEEL_10MM_ROLL_MG  84
#define S4_TEST_WHEEL_10MM_PITCH_MG (-57)
#define S4_TEST_AXLE_10MM_PITCH_MG  114
#define S4_TEST_HUMP_SAMPLES        8u
#define S4_TEST_LEVEL_SAMPLES       8u

static s4_terrain_config_t s4_test_config(void);
static void s4_test_calibrate(s4_terrain_context_t * p_context,
                              uint32_t start_time_us);
static void s4_test_feed(s4_terrain_context_t * p_context,
                         int16_t forward_mg,
                         int16_t left_mg,
                         uint32_t capture_time_us,
                         bool b_pitch_valid);
static bool s4_test_hump(s4_terrain_context_t * p_context,
                         uint32_t * p_time_us,
                         int16_t forward_mg,
                         int16_t left_mg,
                         bool b_pitch_valid);
static bool s4_test_hump_reads(int16_t forward_mg,
                               int16_t left_mg,
                               s4_hump_contact_t contact);
static bool s4_test_calibration_ready(void);
static bool s4_test_calibration_fault(void);
static bool s4_test_rejects_spike(void);
static bool s4_test_detects_hump(void);
static bool s4_test_invalid_limit(void);
static bool s4_test_invalid_streak_clears(void);
static bool s4_test_time_wrap(void);
static bool s4_test_stale_fault(void);
static bool s4_test_maximum_height(void);
static bool s4_test_recovery_retains_max(void);
static bool s4_test_new_run_clears_max(void);
static bool s4_test_impact_edge(void);
static bool s4_test_null_safety(void);
static bool s4_test_decode_axes(void);
static bool s4_test_decode_signed_limits(void);
static bool s4_test_decode_null_safety(void);
static bool s4_test_hump_time_limit(void);
static bool s4_test_pitch_ignored(void);
static bool s4_test_vibration_averages_out(void);
static bool s4_test_average_keeps_height(void);
static bool s4_test_average_length_checked(void);
#if S4_HARDWARE_TEST
static bool s4_test_hardware_startup(void);
#endif

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
               "rise then level completes one hump");
    TEST_CHECK("S4-U08", s4_test_invalid_limit(),
               "consecutive invalid samples cause sensor fault");
    TEST_CHECK("S4-U09", s4_test_invalid_streak_clears(),
               "valid sample clears invalid streak");
    TEST_CHECK("S4-U10", s4_test_time_wrap(),
               "timestamp wrap does not cause stale fault");
    TEST_CHECK("S4-U11", s4_test_stale_fault(),
               "excess sample gap causes stale fault");
    TEST_CHECK("S4-U12", s4_test_maximum_height(),
               "larger hump becomes run maximum");
    TEST_CHECK("S4-U13", s4_test_recovery_retains_max(),
               "local recovery preserves run maximum");
    TEST_CHECK("S4-U14", s4_test_new_run_clears_max(),
               "new run clears run maximum");
    TEST_CHECK("S4-U16", s4_test_impact_edge(),
               "impact is reported once per threshold crossing");
    TEST_CHECK("S4-U17", s4_test_null_safety(),
               "null API inputs do not crash");
    TEST_CHECK("S4-U18", s4_test_decode_axes(),
               "driver decodes axis order and scale");
    TEST_CHECK("S4-U19", s4_test_decode_signed_limits(),
               "driver decodes signed 12-bit limits");
    TEST_CHECK("S4-U20", s4_test_decode_null_safety(),
               "driver decode rejects null pointers");
    TEST_CHECK("S4-U21", s4_test_hump_time_limit(),
               "hump not ended in time is dropped until level");
    TEST_CHECK("S4-U23", s4_test_hump_reads(S4_TEST_WHEEL_10MM_PITCH_MG,
                                            S4_TEST_WHEEL_10MM_ROLL_MG,
                                            S4_HUMP_CONTACT_LEFT),
               "10 mm under the left wheel reads 10 mm, left wheel");
    TEST_CHECK("S4-U24", s4_test_hump_reads(S4_TEST_WHEEL_10MM_PITCH_MG,
                                            -S4_TEST_WHEEL_10MM_ROLL_MG,
                                            S4_HUMP_CONTACT_RIGHT),
               "10 mm under the right wheel reads 10 mm, right wheel");
    TEST_CHECK("S4-U25", s4_test_hump_reads(-S4_TEST_AXLE_10MM_PITCH_MG, 0,
                                            S4_HUMP_CONTACT_BOTH_WHEELS),
               "10 mm under both wheels reads 10 mm, both wheels");
    TEST_CHECK("S4-U26", s4_test_hump_reads(S4_TEST_AXLE_10MM_PITCH_MG, 0,
                                            S4_HUMP_CONTACT_CASTER),
               "10 mm under the caster reads 10 mm, caster");
    TEST_CHECK("S4-U27", s4_test_pitch_ignored(),
               "pitch while accelerating is not a hump");
    TEST_CHECK("S4-U28", s4_test_vibration_averages_out(),
               "alternating roll averages out, no hump");
    TEST_CHECK("S4-U29", s4_test_average_keeps_height(),
               "held tilt reads its full height through the average");
    TEST_CHECK("S4-U30", s4_test_average_length_checked(),
               "average length 0 or above the maximum is rejected");
#if S4_HARDWARE_TEST
    TEST_CHECK("S4-H01", s4_test_hardware_startup(),
               "live identity configuration and calibration");
#else
    tm_printf((UB const *) "[TEST] S4-H01 BLOCKED  build with "
              "TEST=1 S4_HW_TEST=1 and connected IMU\n");
#endif

    TEST_SUMMARY("subsystem4");
}

static s4_terrain_config_t
s4_test_config (void)
{
    s4_terrain_config_t config =
    {
        .calibration_samples = 4u,
        .calibration_span_mg = 10u,
        .impact_threshold_mg = 700u,
        .max_sample_gap_ms = 100u,
        .max_hump_ms = 1000u,
        .track_mm = 119u,
        .wheelbase_mm = 88u,
        .rise_um = 4000u,
        .level_um = 2000u,
        .confirmation_samples = 2u,
        .invalid_limit = 2u,
        .average_samples = 1u
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
        s4_test_feed(p_context, 0, 0, time_us, true);
        time_us += S4_TEST_SAMPLE_PERIOD_US;
    }
}

static void
s4_test_feed (s4_terrain_context_t * p_context,
              int16_t forward_mg,
              int16_t left_mg,
              uint32_t capture_time_us,
              bool b_pitch_valid)
{
    s4_terrain_sample_t sample =
    {
        .acceleration_x_mg = forward_mg,
        .acceleration_y_mg = left_mg,
        .acceleration_z_mg = 1000,
        .capture_time_us = capture_time_us,
        .b_valid = true
    };

    s4_terrain_process(p_context, &sample, b_pitch_valid);
}

/* Holds the tilt for S4_TEST_HUMP_SAMPLES, then level; true if a hump
 * completed on any sample. */
static bool
s4_test_hump (s4_terrain_context_t * p_context,
              uint32_t * p_time_us,
              int16_t forward_mg,
              int16_t left_mg,
              bool b_pitch_valid)
{
    bool b_complete = false;

    for (uint8_t index = 0u; index < S4_TEST_HUMP_SAMPLES; index++)
    {
        *p_time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(p_context, forward_mg, left_mg, *p_time_us,
                     b_pitch_valid);
        b_complete = b_complete ||
            (0u != (p_context->result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE));
    }
    for (uint8_t index = 0u; index < S4_TEST_LEVEL_SAMPLES; index++)
    {
        *p_time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(p_context, 0, 0, *p_time_us, b_pitch_valid);
        b_complete = b_complete ||
            (0u != (p_context->result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE));
    }

    return (b_complete);
}

static bool
s4_test_hump_reads (int16_t forward_mg,
                    int16_t left_mg,
                    s4_hump_contact_t contact)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;
    bool b_complete;

    s4_test_calibrate(&context, 0u);
    b_complete = s4_test_hump(&context, &time_us, forward_mg, left_mg,
                              true);

    return (b_complete && context.result.b_current_height_valid &&
            (10u == context.result.current_height_mm) &&
            (contact == context.result.hump_contact));
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
    s4_test_feed(&context, 0, 0, 0u, true);
    s4_test_feed(&context, 20, 0, 10000u, true);
    s4_test_feed(&context, 0, 0, 20000u, true);
    s4_test_feed(&context, 0, 0, 30000u, true);

    return ((S4_TERRAIN_STATE_FAULT == context.result.state) &&
            (S4_TERRAIN_FAULT_CALIBRATION == context.result.fault));
}

static bool
s4_test_rejects_spike (void)
{
    s4_terrain_context_t context;
    bool b_complete = false;

    s4_test_calibrate(&context, 0u);
    s4_test_feed(&context, S4_TEST_WHEEL_10MM_PITCH_MG,
                 S4_TEST_WHEEL_10MM_ROLL_MG, 40000u, true);
    for (uint32_t time_us = 50000u; time_us < 150000u;
         time_us += S4_TEST_SAMPLE_PERIOD_US)
    {
        s4_test_feed(&context, 0, 0, time_us, true);
        b_complete = b_complete ||
            (0u != (context.result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE));
    }

    return ((!b_complete) &&
            (S4_TERRAIN_STATE_MONITORING == context.result.state));
}

static bool
s4_test_detects_hump (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;
    bool b_complete;

    s4_test_calibrate(&context, 0u);
    b_complete = s4_test_hump(&context, &time_us,
                              S4_TEST_WHEEL_10MM_PITCH_MG,
                              S4_TEST_WHEEL_10MM_ROLL_MG, true);

    return (b_complete &&
            (S4_TERRAIN_STATE_MONITORING == context.result.state) &&
            context.result.b_current_height_valid);
}

static bool
s4_test_invalid_limit (void)
{
    s4_terrain_context_t context;
    s4_terrain_sample_t invalid = {0};

    s4_test_calibrate(&context, 0u);
    s4_terrain_process(&context, &invalid, true);
    s4_terrain_process(&context, &invalid, true);

    return ((S4_TERRAIN_STATE_FAULT == context.result.state) &&
            (S4_TERRAIN_FAULT_SENSOR == context.result.fault));
}

static bool
s4_test_invalid_streak_clears (void)
{
    s4_terrain_context_t context;
    s4_terrain_sample_t invalid = {0};

    s4_test_calibrate(&context, 0u);
    s4_terrain_process(&context, &invalid, true);
    s4_test_feed(&context, 0, 0, 40000u, true);
    s4_terrain_process(&context, &invalid, true);

    return (S4_TERRAIN_STATE_MONITORING == context.result.state);
}

static bool
s4_test_time_wrap (void)
{
    s4_terrain_context_t context;
    uint32_t start_time_us = UINT32_MAX - 20000u;

    s4_test_calibrate(&context, start_time_us);
    s4_test_feed(&context, 0, 0,
                 start_time_us + (4u * S4_TEST_SAMPLE_PERIOD_US), true);

    return (S4_TERRAIN_STATE_MONITORING == context.result.state);
}

static bool
s4_test_stale_fault (void)
{
    s4_terrain_context_t context;

    s4_test_calibrate(&context, 0u);
    s4_test_feed(&context, 0, 0, 200000u, true);

    return ((S4_TERRAIN_STATE_FAULT == context.result.state) &&
            (S4_TERRAIN_FAULT_STALE == context.result.fault));
}

static bool
s4_test_maximum_height (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    (void) s4_test_hump(&context, &time_us, (S4_TEST_WHEEL_10MM_PITCH_MG / 2),
                        (S4_TEST_WHEEL_10MM_ROLL_MG / 2), true);
    (void) s4_test_hump(&context, &time_us, S4_TEST_WHEEL_10MM_PITCH_MG,
                        S4_TEST_WHEEL_10MM_ROLL_MG, true);

    return (context.result.b_maximum_height_valid &&
            (10u == context.result.maximum_height_mm));
}

static bool
s4_test_recovery_retains_max (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    (void) s4_test_hump(&context, &time_us, S4_TEST_WHEEL_10MM_PITCH_MG,
                        S4_TEST_WHEEL_10MM_ROLL_MG, true);
    s4_terrain_request_recovery(&context);

    return ((S4_TERRAIN_STATE_CALIBRATING == context.result.state) &&
            context.result.b_maximum_height_valid &&
            (10u == context.result.maximum_height_mm));
}

static bool
s4_test_new_run_clears_max (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;

    s4_test_calibrate(&context, 0u);
    (void) s4_test_hump(&context, &time_us, S4_TEST_WHEEL_10MM_PITCH_MG,
                        S4_TEST_WHEEL_10MM_ROLL_MG, true);
    s4_terrain_start_new_run(&context);

    return ((!context.result.b_maximum_height_valid) &&
            (0u == context.result.maximum_height_mm));
}

static bool
s4_test_impact_edge (void)
{
    s4_terrain_context_t context;
    bool b_first_event;

    s4_test_calibrate(&context, 0u);
    s4_test_feed(&context, 800, 0, 40000u, true);
    b_first_event = 0u !=
        (context.result.events & S4_TERRAIN_EVENT_IMPACT);
    s4_test_feed(&context, 800, 0, 50000u, true);

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
    s4_terrain_process(NULL, &sample, true);
    s4_terrain_init(&context, NULL);
    s4_terrain_process(&context, NULL, true);
    s4_terrain_request_recovery(NULL);
    s4_terrain_start_new_run(NULL);

    return ((NULL == s4_terrain_get_result(NULL)) &&
            (S4_TERRAIN_STATE_FAULT == context.result.state));
}

static bool
s4_test_decode_axes (void)
{
    uint8_t const raw_data[6] =
    {
        0x40u, 0x06u, 0x80u, 0xf3u, 0x80u, 0x3eu
    };
    s4_terrain_sample_t sample = {0};
    bool b_decoded = s4_lsm303_decode_acceleration(raw_data, 1234u,
                                                    &sample);

    return (b_decoded && sample.b_valid &&
            (100 == sample.acceleration_x_mg) &&
            (-200 == sample.acceleration_y_mg) &&
            (1000 == sample.acceleration_z_mg) &&
            (1234u == sample.capture_time_us));
}

static bool
s4_test_decode_signed_limits (void)
{
    uint8_t const raw_data[6] =
    {
        0x00u, 0x80u, 0xf0u, 0x7fu, 0x00u, 0x00u
    };
    s4_terrain_sample_t sample = {0};
    bool b_decoded = s4_lsm303_decode_acceleration(raw_data, 0u,
                                                    &sample);

    return (b_decoded && (-2048 == sample.acceleration_x_mg) &&
            (2047 == sample.acceleration_y_mg) &&
            (0 == sample.acceleration_z_mg));
}

static bool
s4_test_decode_null_safety (void)
{
    uint8_t const raw_data[6] = {0u};
    s4_terrain_sample_t sample = {0};

    return ((!s4_lsm303_decode_acceleration(NULL, 0u, &sample)) &&
            (!s4_lsm303_decode_acceleration(raw_data, 0u, NULL)));
}

static bool
s4_test_hump_time_limit (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;
    bool b_complete = false;

    s4_test_calibrate(&context, 0u);

    /* A wheel held raised for longer than max_hump_ms (1 s), then level. */
    for (uint8_t index = 0u; index < 110u; index++)
    {
        time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(&context, S4_TEST_WHEEL_10MM_PITCH_MG,
                     S4_TEST_WHEEL_10MM_ROLL_MG, time_us, true);
        b_complete = b_complete ||
            (0u != (context.result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE));
    }
    for (uint8_t index = 0u; index < S4_TEST_LEVEL_SAMPLES; index++)
    {
        time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(&context, 0, 0, time_us, true);
        b_complete = b_complete ||
            (0u != (context.result.events &
                    S4_TERRAIN_EVENT_HUMP_COMPLETE));
    }

    return ((!b_complete) && (!context.result.b_maximum_height_valid) &&
            (S4_TERRAIN_STATE_MONITORING == context.result.state));
}

static bool
s4_test_pitch_ignored (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;
    bool b_complete;

    s4_test_calibrate(&context, 0u);

    /* Pitch alone (as when accelerating) with pitch marked unreadable. */
    b_complete = s4_test_hump(&context, &time_us,
                              S4_TEST_AXLE_10MM_PITCH_MG, 0, false);

    return ((!b_complete) && (!context.result.b_maximum_height_valid));
}

#if S4_HARDWARE_TEST
static bool
s4_test_hardware_startup (void)
{
    s4_terrain_result_t result = {0};
    bool b_ready = false;

    /* Up to 10 s: the 50 calibration samples take about 1.5 s, but the
     * self-tests running at the same time slow the task, and a moved car
     * restarts calibration. */
    for (uint16_t poll = 0u; (poll < 200u) && (!b_ready); poll++)
    {
        if (E_OK == subsystem4_get_status(&result))
        {
            b_ready = (S4_TERRAIN_STATE_MONITORING == result.state) ||
                      (S4_TERRAIN_STATE_ON_HUMP == result.state);
        }

        if (!b_ready)
        {
            (void) tk_dly_tsk(50u);
        }
    }

    return (b_ready && (S4_TERRAIN_FAULT_NONE == result.fault));
}
#endif

/* Calibrates with a 4-sample tilt average (decision 10). */
static void
s4_test_calibrate_averaged (s4_terrain_context_t * p_context)
{
    s4_terrain_config_t config = s4_test_config();
    uint32_t time_us = 0u;

    config.average_samples = 4u;
    s4_terrain_init(p_context, &config);
    for (uint16_t index = 0u; index < config.calibration_samples; index++)
    {
        s4_test_feed(p_context, 0, 0, time_us, true);
        time_us += S4_TEST_SAMPLE_PERIOD_US;
    }
}

static bool
s4_test_vibration_averages_out (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;
    bool b_started = false;

    s4_test_calibrate_averaged(&context);

    /* Roll of +/-240 mg each sample (about 29 mm one sample at a time)
     * averages to zero over the 4-sample window. */
    for (uint8_t index = 0u; index < 40u; index++)
    {
        int16_t const left_mg = ((index % 2u) == 0u) ? 240 : -240;

        time_us += S4_TEST_SAMPLE_PERIOD_US;
        s4_test_feed(&context, 0, left_mg, time_us, true);
        b_started = b_started ||
            (S4_TERRAIN_STATE_ON_HUMP == context.result.state);
    }

    return ((!b_started) &&
            (S4_TERRAIN_STATE_MONITORING == context.result.state));
}

static bool
s4_test_average_keeps_height (void)
{
    s4_terrain_context_t context;
    uint32_t time_us = 30000u;
    bool b_complete;

    s4_test_calibrate_averaged(&context);
    b_complete = s4_test_hump(&context, &time_us,
                              S4_TEST_WHEEL_10MM_PITCH_MG,
                              S4_TEST_WHEEL_10MM_ROLL_MG, true);

    return (b_complete && context.result.b_current_height_valid &&
            (10u == context.result.current_height_mm) &&
            (S4_HUMP_CONTACT_LEFT == context.result.hump_contact));
}

static bool
s4_test_average_length_checked (void)
{
    s4_terrain_context_t context;
    s4_terrain_config_t config = s4_test_config();
    bool b_zero_rejected;

    config.average_samples = 0u;
    s4_terrain_init(&context, &config);
    b_zero_rejected = (S4_TERRAIN_STATE_FAULT == context.result.state);

    config.average_samples = (uint8_t) (S4_AVERAGE_MAX_SAMPLES + 1u);
    s4_terrain_init(&context, &config);

    return (b_zero_rejected &&
            (S4_TERRAIN_STATE_FAULT == context.result.state));
}

#endif

/* End of file. */
