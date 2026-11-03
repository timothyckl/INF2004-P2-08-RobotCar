/**
 * @file barcode.h
 * @brief Barcode capture (run lengths) and Code 39 decoding.
 *
 * This module is pure logic. It makes no hardware or kernel calls. Widths
 * may be in any unit, such as samples, microseconds or encoder ticks.
 * Decoding uses only the ratio of wide to narrow elements, so the speed
 * may differ from one barcode to the next.
 *
 * NOTE: The course is assumed to use Code 39 "*X*", with X = A (Left),
 * B (Right), C (Straight) or D (U-turn), as in the example table of the
 * Project Description. This is not yet confirmed. Record the answer in
 * docs/decisions.md.
 */
#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

/* "*X*" is three 9-element characters plus two inter-character gaps. */
#define BARCODE_MAX_ELEMENTS    (29u)

/* A wide element must be at least 3/2 of the widest narrow element. */
#define BARCODE_WIDE_RATIO_NUM  (3u)
#define BARCODE_WIDE_RATIO_DEN  (2u)

/* A white run this many times the narrowest element ends a capture. */
#define BARCODE_QUIET_FACTOR    (7u)

/** Navigation commands. The values fit in 3 bits. */
typedef enum
{
    BARCODE_CMD_NONE     = 0,
    BARCODE_CMD_LEFT     = 1,
    BARCODE_CMD_RIGHT    = 2,
    BARCODE_CMD_STRAIGHT = 3,
    BARCODE_CMD_UTURN    = 4
} barcode_cmd_t;

/** Outcome of one barcode_feed() call. */
typedef enum
{
    BARCODE_NONE,           /**< No capture finished in this call.     */
    BARCODE_DECODED,        /**< A valid barcode; *p_cmd is set.       */
    BARCODE_SINGLE_BAR,     /**< One bar only: a crossing or junction. */
    BARCODE_ERROR           /**< Bars were seen but not decoded.       */
} barcode_result_t;

/** State of one capture in progress. */
typedef struct
{
    uint16_t widths[BARCODE_MAX_ELEMENTS];  /**< Stored element widths.   */
    uint8_t  count;                         /**< Number of stored widths. */
    uint16_t run;                           /**< Current run width.       */
    uint16_t narrowest;                     /**< Narrowest stored width.  */
    bool     b_active;                      /**< A capture is running.    */
    bool     b_run_black;                   /**< Level of the current run. */
    bool     b_overflow;                    /**< Too many elements seen.  */
} barcode_t;

/**
 * @brief Abandon any capture and wait for the next black bar.
 * @param p_capture Capture state. NULL is ignored.
 */
void barcode_init (barcode_t * p_capture);

/**
 * @brief Feed one sample into the capture.
 *
 * A capture starts on black and ends after a quiet white run.
 *
 * @param p_capture Capture state.
 * @param b_black   True if the sample is a bar.
 * @param width     How long the sample lasted. Zero is ignored.
 * @param p_cmd     Receives the command when BARCODE_DECODED is returned.
 * @return The outcome of this sample.
 */
barcode_result_t barcode_feed (barcode_t * p_capture, bool b_black,
                               uint16_t width, barcode_cmd_t * p_cmd);

/**
 * @brief Decode stored widths, black first, trying both directions.
 * @param p_widths Element widths.
 * @param count    Number of widths; must be BARCODE_MAX_ELEMENTS.
 * @param p_cmd    Receives the command on success; unchanged otherwise.
 * @return True if a valid command was decoded.
 */
bool barcode_decode (uint16_t const p_widths[], uint8_t count,
                     barcode_cmd_t * p_cmd);

#endif /* BARCODE_H */

/* End of file. */

