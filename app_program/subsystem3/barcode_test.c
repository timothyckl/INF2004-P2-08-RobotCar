/**
 * @file barcode_test.c
 * @brief Test-only observation of the side barcode sensor and decoder.
 */

#include "barcode_test.h"

#include "s3_config.h"
#include "s3_ir.h"
#include "s3_ir_gpio.h"
#include "s3_scan.h"

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#ifndef S3_BARCODE_DIAGNOSTIC
#define S3_BARCODE_DIAGNOSTIC 0
#endif

#if APP_TEST && S3_BARCODE_DIAGNOSTIC

#define S3_BAR_TEST_DURATION_MS   UINT32_C(120000)
#define S3_BAR_TEST_HEARTBEAT_MS  UINT32_C(1000)
#define S3_BAR_TEST_POLL_MS       UINT32_C(10)
#define S3_BAR_CAL_WAIT_MS         UINT32_C(5000)
#define S3_BAR_CAL_ONE_SECOND_MS    UINT32_C(1000)
#define S3_BAR_CAL_COUNTDOWN_SECONDS UINT8_C(5)
#define S3_BAR_CAL_SAMPLE_MS         UINT32_C(20)
#define S3_BAR_CAL_SAMPLE_COUNT       UINT16_C(50)

typedef struct
{
    uint16_t minimum;
    uint16_t maximum;
    uint16_t average;
} s3_bar_calibration_stats_t;

static uint32_t s3_barcode_test_now_ms(void);
static char const * s3_barcode_test_colour(bool black);
static bool s3_barcode_test_calibrate(s3_ir_adaptive_config_t * p_config);
static bool s3_barcode_test_sample_surface(
    char const * p_name, s3_bar_calibration_stats_t * p_stats);

/**
 * Log raw transitions separately from line-sensor ADC diagnostics.
 *
 * Transition durations are polling observations. This isolated diagnostic
 * feeds them directly to the production scan logic without starting tasks.
 */
void s3_barcode_test_run(void)
{
    s3_scan_cfg_t const scan_config =
    {
        S3_BAR_RUN_MAX_US,
        S3_BAR_QUIET_US,
        S3_BAR_SENTINEL,
        g_s3_code39_alphabet,
        S3_CODE39_SYMBOL_COUNT
    };
    s3_ir_adaptive_config_t barcode_config = {0U, 0U, 0U, 0U, 0U};
    s3_scan_t scan;
    s3_ir_adaptive_t barcode_state = {0};
    s3_ir_adc_sample_t adc = {0U, 0U, 0U};
    uint32_t start_ms;
    uint32_t last_edge_ms;
    uint32_t last_heartbeat_ms;
    bool previous_black;
    uint16_t decoded_count = 0U;
    uint16_t error_count = 0U;

    ER const init_error = s3_ir_gpio_init();

    if (init_error < E_OK)
    {
        tm_printf((UB *)"[S3 BAR] ERROR ADC2/GP28 initialisation failed\n");
        return;
    }

    tm_printf((UB *)"[S3 BAR CAL] new-sensor calibration; motors disabled\n");
    tm_printf((UB *)"[S3 BAR CAL] keep the sensor at its final mounted "
              "height and under course lighting\n");
    tm_printf((UB *)"[S3 BAR CAL] connect the serial monitor now; "
              "surface prompts begin in 5 seconds\n");
    (void)tk_dly_tsk((RELTIM)S3_BAR_CAL_WAIT_MS);

    if ((!s3_barcode_test_calibrate(&barcode_config))
        || (!s3_ir_adaptive_init(&barcode_state, &barcode_config)))
    {
        tm_printf((UB *)"[S3 BAR CAL] calibration rejected; "
                  "adjust height/alignment and reset to retry\n");
        return;
    }

    tm_printf((UB *)"[S3 BAR CAL] return the sensor to WHITE; "
              "live test starts in 5 seconds\n");
    (void)tk_dly_tsk((RELTIM)S3_BAR_CAL_WAIT_MS);

    if (s3_ir_gpio_read_analog(&adc) < E_OK)
    {
        tm_printf((UB *)"[S3 BAR] ERROR ADC2/GP28 read failed\n");
        return;
    }

    s3_scan_init(&scan);
    start_ms = s3_barcode_test_now_ms();
    last_edge_ms = start_ms;
    last_heartbeat_ms = start_ms;
    previous_black = s3_ir_adaptive_update_unfiltered(&barcode_state,
                                                       adc.barcode);

    tm_printf((UB *)"[S3 BAR] diagnostic start: 120 seconds; "
              "AOUT=ADC2/GP28, DOUT disconnected\n");
    tm_printf((UB *)"[S3 BAR] slide one printed barcode past the "
              "sensor at steady speed\n");
    tm_printf((UB *)"[S3 BAR] adc=%u filtered=%u threshold=%u "
              "state=%s decoded=%u errors=%u\n",
              (UINT)adc.barcode, (UINT)barcode_state.filtered,
              (UINT)barcode_state.threshold,
              (UB *)s3_barcode_test_colour(previous_black),
              (UINT)decoded_count, (UINT)error_count);

    while ((s3_barcode_test_now_ms() - start_ms)
           < S3_BAR_TEST_DURATION_MS)
    {
        uint32_t const now_ms = s3_barcode_test_now_ms();
        uint32_t const now_us = now_ms * UINT32_C(1000);
        s3_scan_result_t result = S3_SCAN_NONE;
        char symbol = '\0';

        if (s3_ir_gpio_read_analog(&adc) < E_OK)
        {
            error_count++;
            tm_printf((UB *)"[S3 BAR] ADC READ ERROR total=%u\n",
                      (UINT)error_count);
            break;
        }

        bool const black = s3_ir_adaptive_update_unfiltered(&barcode_state,
                                                             adc.barcode);

        if (black != previous_black)
        {
            tm_printf((UB *)"[S3 BAR] run=%s duration=%u ms; "
                      "adc=%u filtered=%u threshold=%u state=%s\n",
                      (UB *)s3_barcode_test_colour(previous_black),
                      (UINT)(now_ms - last_edge_ms), (UINT)adc.barcode,
                      (UINT)barcode_state.filtered,
                      (UINT)barcode_state.threshold,
                      (UB *)s3_barcode_test_colour(black));
            previous_black = black;
            last_edge_ms = now_ms;
            result = s3_scan_edge(&scan, &scan_config, black, now_us);
        }

        if (S3_SCAN_ERROR == result)
        {
            error_count++;
            tm_printf((UB *)"[S3 BAR] CAPTURE ERROR total=%u\n",
                      (UINT)error_count);
        }

        result = s3_scan_poll(&scan, &scan_config, now_us, &symbol);

        if (S3_SCAN_DECODED == result)
        {
            decoded_count++;
            tm_printf((UB *)"[S3 BAR] DECODED letter=%c "
                      "total=%u errors=%u\n",
                      (INT)symbol, (UINT)decoded_count,
                      (UINT)error_count);
        }
        else if (S3_SCAN_SINGLE_BAR == result)
        {
            tm_printf((UB *)"[S3 BAR] single black bar; no letter\n");
        }
        else if (S3_SCAN_ERROR == result)
        {
            error_count++;
            tm_printf((UB *)"[S3 BAR] DECODE ERROR total=%u\n",
                      (UINT)error_count);
        }
        else
        {
            /* The scan is idle or waiting for its ending white period. */
        }

        if ((now_ms - last_heartbeat_ms) >= S3_BAR_TEST_HEARTBEAT_MS)
        {
            tm_printf((UB *)"[S3 BAR] adc=%u filtered=%u threshold=%u "
                      "state=%s decoded=%u errors=%u\n",
                      (UINT)adc.barcode, (UINT)barcode_state.filtered,
                      (UINT)barcode_state.threshold,
                      (UB *)s3_barcode_test_colour(black),
                      (UINT)decoded_count,
                      (UINT)error_count);
            last_heartbeat_ms = now_ms;
        }

        (void)tk_dly_tsk((RELTIM)S3_BAR_TEST_POLL_MS);
    }

    tm_printf((UB *)"[S3 BAR] diagnostic complete; reset to rerun\n");
}

static uint32_t s3_barcode_test_now_ms(void)
{
    SYSTIM now = {0};

    return (tk_get_tim(&now) >= E_OK) ? (uint32_t)now.lo : 0U;
}

static char const * s3_barcode_test_colour(bool black)
{
    return black ? "BLACK" : "WHITE";
}

static bool
s3_barcode_test_calibrate (s3_ir_adaptive_config_t * p_config)
{
    s3_bar_calibration_stats_t white = {0U, 0U, 0U};
    s3_bar_calibration_stats_t solid_black = {0U, 0U, 0U};
    s3_bar_calibration_stats_t faded_black = {0U, 0U, 0U};
    uint16_t black_floor;
    uint16_t threshold;
    uint16_t release;
    bool b_valid;

    if (NULL == p_config)
    {
        return false;
    }

    b_valid = s3_barcode_test_sample_surface("WHITE", &white)
              && s3_barcode_test_sample_surface("SOLID BLACK",
                                                &solid_black)
              && s3_barcode_test_sample_surface("FADED BLACK",
                                                &faded_black);

    if (!b_valid)
    {
        return false;
    }

    black_floor = (solid_black.minimum < faded_black.minimum)
                  ? solid_black.minimum : faded_black.minimum;

    if (black_floor <= white.maximum)
    {
        tm_printf((UB *)"[S3 BAR CAL] FAIL ranges overlap: "
                  "white_max=%u black_floor=%u\n",
                  (UINT)white.maximum, (UINT)black_floor);
        return false;
    }

    threshold = white.maximum
                + (uint16_t)((black_floor - white.maximum) / 2U);

    if (threshold <= white.maximum)
    {
        tm_printf((UB *)"[S3 BAR CAL] FAIL separation is too small for "
                  "an integer threshold\n");
        return false;
    }

    release = white.maximum
              + (uint16_t)((threshold - white.maximum) / 2U);

    p_config->initial_white = white.average;
    p_config->minimum_white = white.average;
    p_config->maximum_white = white.average;
    p_config->black_margin = threshold - white.average;
    p_config->white_margin = release - white.average;

    b_valid = (threshold > white.average)
              && (release >= white.average)
              && (p_config->white_margin < p_config->black_margin)
              && (white.maximum
                  <= (S3_IR_ADC_MAX - p_config->black_margin));

    if (!b_valid)
    {
        tm_printf((UB *)"[S3 BAR CAL] FAIL measured values cannot form "
                  "a valid adaptive configuration\n");
        return false;
    }

    tm_printf((UB *)"[S3 BAR CAL] PASS non-overlapping ranges; gap=%u\n",
              (UINT)(black_floor - white.maximum));
    tm_printf((UB *)"[S3 BAR CAL] provisional threshold=%u release=%u\n",
              (UINT)threshold, (UINT)release);
    tm_printf((UB *)"[S3 BAR CAL] fixed runtime config: baseline=%u "
              "black_margin=%u white_margin=%u\n",
              (UINT)p_config->initial_white,
              (UINT)p_config->black_margin,
              (UINT)p_config->white_margin);

    return true;
}

static bool
s3_barcode_test_sample_surface (
    char const * p_name, s3_bar_calibration_stats_t * p_stats)
{
    s3_ir_adc_sample_t sample = {0U, 0U, 0U};
    uint32_t sum = 0U;
    uint16_t sample_index;
    uint8_t seconds_remaining;

    if ((NULL == p_name) || (NULL == p_stats))
    {
        return false;
    }

    tm_printf((UB *)"[S3 BAR CAL] PLACE SENSOR OVER %s NOW\n",
              (UB *)p_name);

    for (seconds_remaining = S3_BAR_CAL_COUNTDOWN_SECONDS;
         seconds_remaining > 0U;
         seconds_remaining--)
    {
        tm_printf((UB *)"[S3 BAR CAL] %s sampling in %u second(s)\n",
                  (UB *)p_name, (UINT)seconds_remaining);
        (void)tk_dly_tsk((RELTIM)S3_BAR_CAL_ONE_SECOND_MS);
    }

    p_stats->minimum = S3_IR_ADC_MAX;
    p_stats->maximum = 0U;
    p_stats->average = 0U;

    for (sample_index = 0U;
         sample_index < S3_BAR_CAL_SAMPLE_COUNT;
         sample_index++)
    {
        if (s3_ir_gpio_read_analog(&sample) < E_OK)
        {
            tm_printf((UB *)"[S3 BAR CAL] ADC read failed while "
                      "sampling %s\n", (UB *)p_name);
            return false;
        }

        if (sample.barcode < p_stats->minimum)
        {
            p_stats->minimum = sample.barcode;
        }

        if (sample.barcode > p_stats->maximum)
        {
            p_stats->maximum = sample.barcode;
        }

        sum += sample.barcode;
        (void)tk_dly_tsk((RELTIM)S3_BAR_CAL_SAMPLE_MS);
    }

    p_stats->average = (uint16_t)(sum / S3_BAR_CAL_SAMPLE_COUNT);
    tm_printf((UB *)"[S3 BAR CAL] %s samples=%u min=%u avg=%u max=%u\n",
              (UB *)p_name, (UINT)S3_BAR_CAL_SAMPLE_COUNT,
              (UINT)p_stats->minimum, (UINT)p_stats->average,
              (UINT)p_stats->maximum);

    return true;
}

#endif /* APP_TEST && S3_BARCODE_DIAGNOSTIC */
