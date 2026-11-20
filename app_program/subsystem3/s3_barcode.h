/**
 * @file s3_barcode.h
 * @brief Side barcode sensor: ADC samples in, Code 39 letters out.
 *
 * Pure logic; no ADC, GPIO or kernel calls. The task that owns the side
 * sensor (ADC2/GP28) reads it once per sample period and passes the 12-bit
 * value with a free-running 1 MHz timestamp. A single task must own each
 * s3_barcode_t; it is not safe to share with an ISR or another task.
 */
#ifndef S3_BARCODE_H
#define S3_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "s3_ir.h"
#include "s3_scan.h"

typedef struct
{
    s3_ir_adaptive_t level;     /* black/white classification of ADC2 */
    s3_scan_t        scan;      /* capture in progress */
} s3_barcode_t;

/** Start idle on white with the provisional thresholds.
 * Returns false only if the built-in configuration is invalid.
 */
bool s3_barcode_init(s3_barcode_t * p_barcode);

/** Classify one ADC sample (0..4095), feed any level change, then check
 * whether a capture has finished. Call once per sample period, in time
 * order, less than one timer wrap (about 71 min) apart. *p_symbol is
 * written only when S3_SCAN_DECODED is returned.
 */
s3_scan_result_t s3_barcode_update(s3_barcode_t * p_barcode, uint16_t adc,
                                   uint32_t now_us, char * p_symbol);

/** Abandon any capture in progress, e.g. after an ADC read failure. */
void s3_barcode_abort(s3_barcode_t * p_barcode);

/** True while a capture in progress looks like a barcode: at least three
 * stored runs, the first three each at least 60 ms. A single crossing line,
 * or short blips from a sensor edge, never qualify. It ends when the capture
 * decodes, is rejected or is abandoned; the scan timeouts bound it.
 */
bool s3_barcode_active(s3_barcode_t const * p_barcode);

#endif /* S3_BARCODE_H */

/* End of file. */
