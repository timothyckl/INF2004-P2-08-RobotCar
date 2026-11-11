/** @file s3_bar.h Bounded edge capture and configurable barcode foundation. */
#ifndef S3_BAR_H
#define S3_BAR_H

#include <stdbool.h>
#include <stdint.h>

#define S3_BAR_CAPACITY 64U
#define S3_BAR_ELEMENTS 9U
#define S3_BAR_FRAME_RUNS 29U

typedef struct
{
    uint32_t runs[S3_BAR_CAPACITY];
    uint32_t last_us;
    uint8_t  count;
    bool     black;
    bool     active;
    bool     failed;
} s3_capture_t;

typedef struct
{
    uint16_t pattern;
    char     symbol;
} s3_symbol_t;

/** Begin at a verified white-to-black start boundary, excluding quiet zone.
 * A single task must own this state. An ISR must queue edges to that task.
 */
void s3_bar_begin(s3_capture_t * p_capture, uint32_t now_us);
/** Repeated levels are ignored. Time deltas use modulo uint32 arithmetic.
 * Calls must be ordered and less than one timer wrap apart.
 * timeout_us is an unvalidated caller-selected maximum run duration.
 */
bool s3_bar_edge(s3_capture_t * p_capture, bool black, uint32_t now_us,
                 uint32_t timeout_us);
/** Poll even when there are no edges. Timeout aborts, never decodes. */
bool s3_bar_expire(s3_capture_t * p_capture, uint32_t now_us,
                   uint32_t timeout_us);
/** Finish immediately after final black-to-white edge, before quiet zone. */
bool s3_bar_finish(s3_capture_t * p_capture);
/** Prototype format: sentinel, gap, ONE symbol, gap, sentinel.
 * Each symbol has nine alternating black/white runs, starting black.
 * Pattern bit 8 is first run; 1 means wide. No built-in alphabet is assumed.
 * Durations between narrow_max_us and wide_min_us are rejected.
 * Caller must verify framing/quiet zones and the actual printed symbology.
 */
bool s3_bar_decode(s3_capture_t const * p_capture,
                   uint32_t narrow_max_us, uint32_t wide_min_us,
                   uint32_t wide_max_us, uint16_t sentinel,
                   s3_symbol_t const * p_symbols, uint8_t symbol_count,
                   char * p_symbol);

#endif /* S3_BAR_H */

