/** @file s3_scan.h Side barcode sensor: edges in, decoded symbols out.
 *
 * Pure logic on top of s3_bar. The owning task feeds timestamped edges of
 * the side barcode sensor (GP16, normalised: true = black) and polls it
 * every sample period. A capture starts at a white-to-black edge and ends
 * after quiet_us of white. Width windows are derived from each capture
 * (midway between its narrowest and widest run), so a steady change in
 * speed does not need new constants. The frame is the s3_bar prototype:
 * sentinel, gap, one symbol, gap, sentinel (29 runs). The read is tried
 * forwards, then reversed. The sentinel and alphabet are supplied by the
 * caller and must match the printed course barcodes.
 */
#ifndef S3_SCAN_H
#define S3_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "s3_bar.h"

typedef enum
{
    S3_SCAN_NONE = 0,       /* nothing finished on this call */
    S3_SCAN_DECODED,        /* *p_symbol holds the payload symbol */
    S3_SCAN_SINGLE_BAR,     /* one bar only: a crossing line, not a code */
    S3_SCAN_ERROR           /* capture or decode failed */
} s3_scan_result_t;

typedef struct
{
    uint32_t            run_max_us;     /* longest single run accepted */
    uint32_t            quiet_us;       /* white this long ends a capture */
    uint16_t            sentinel;       /* 9-element pattern, bit 8 first */
    s3_symbol_t const * p_symbols;
    uint8_t             symbol_count;
} s3_scan_cfg_t;

typedef struct
{
    s3_capture_t capture;
    bool         black;     /* last level fed */
} s3_scan_t;

/** Start idle with the sensor assumed white. */
void s3_scan_init(s3_scan_t * p_scan);
/** Feed one level change. Edges must be in time order. */
s3_scan_result_t s3_scan_edge(s3_scan_t * p_scan, s3_scan_cfg_t const * p_cfg,
                              bool black, uint32_t now_us);
/** Poll once per sample period, with or without edges. */
s3_scan_result_t s3_scan_poll(s3_scan_t * p_scan, s3_scan_cfg_t const * p_cfg,
                              uint32_t now_us, char * p_symbol);
/** Abandon any capture in progress (STOP, reset, lost edges). */
void s3_scan_abort(s3_scan_t * p_scan);

#endif /* S3_SCAN_H */
