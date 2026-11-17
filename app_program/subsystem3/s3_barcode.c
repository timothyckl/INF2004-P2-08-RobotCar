/**
 * @file s3_barcode.c
 * @brief Side barcode classification and capture; no hardware access.
 */
#include "s3_barcode.h"

#include <stddef.h>

#include "s3_code39.h"

/*
 * PROVISIONAL values from feature/subsystem3-ir-line-barcode (s3_config.h).
 * At the mounted height on 2026-11-11, 50 samples gave white 154..184,
 * solid black 1679..1709 and faded black 1637..1672. The resulting black
 * threshold (baseline + 400) and release level (baseline + 300) decoded
 * the large and three-quarter Z prints in both directions; the smallest
 * print was not reliable. Re-measure on this car before relying on them.
 */
#define S3_BARCODE_INITIAL_WHITE    169U
#define S3_BARCODE_MINIMUM_WHITE    154U
#define S3_BARCODE_MAXIMUM_WHITE    184U
#define S3_BARCODE_BLACK_MARGIN     400U
#define S3_BARCODE_WHITE_MARGIN     300U

/*
 * PROVISIONAL: a 40 mm/s run on 2026-11-11 gave narrow runs of 120..220 ms
 * and wide runs of 420..540 ms. A run longer than RUN_MAX aborts a capture;
 * white for QUIET ends one. Both need S3 hardware validation at the
 * line-follow speed, which is also about 40 mm/s.
 */
#define S3_BARCODE_RUN_MAX_US       800000U
#define S3_BARCODE_QUIET_US         700000U

static s3_scan_cfg_t const g_s3_barcode_scan_cfg =
{
    S3_BARCODE_RUN_MAX_US,
    S3_BARCODE_QUIET_US,
    S3_CODE39_SENTINEL,
    g_s3_code39_alphabet,
    S3_CODE39_SYMBOL_COUNT
};

bool
s3_barcode_init (s3_barcode_t * p_barcode)
{
    static s3_ir_adaptive_config_t const level_cfg =
    {
        S3_BARCODE_INITIAL_WHITE,
        S3_BARCODE_MINIMUM_WHITE,
        S3_BARCODE_MAXIMUM_WHITE,
        S3_BARCODE_BLACK_MARGIN,
        S3_BARCODE_WHITE_MARGIN
    };

    if (NULL == p_barcode)
    {
        return (false);
    }

    s3_scan_init(&p_barcode->scan);

    return (s3_ir_adaptive_init(&p_barcode->level, &level_cfg));
}

s3_scan_result_t
s3_barcode_update (s3_barcode_t * p_barcode, uint16_t adc,
                   uint32_t now_us, char * p_symbol)
{
    s3_scan_result_t result = S3_SCAN_NONE;

    if ((NULL == p_barcode) || (NULL == p_symbol))
    {
        return (result);
    }

    /* An out-of-range or unconfigured sample classifies as white. */
    bool const b_black = s3_ir_adaptive_update_unfiltered(&p_barcode->level,
                                                          adc);

    result = s3_scan_edge(&p_barcode->scan, &g_s3_barcode_scan_cfg,
                          b_black, now_us);
    if (S3_SCAN_NONE == result)
    {
        result = s3_scan_poll(&p_barcode->scan, &g_s3_barcode_scan_cfg,
                              now_us, p_symbol);
    }

    return (result);
}

void
s3_barcode_abort (s3_barcode_t * p_barcode)
{
    if (NULL != p_barcode)
    {
        s3_scan_abort(&p_barcode->scan);
    }
}

bool
s3_barcode_active (s3_barcode_t const * p_barcode)
{
    return ((NULL != p_barcode) && p_barcode->scan.capture.active);
}

/* End of file. */
