/**
 * @file line_track.h
 * @brief Line position, line loss and sensor-fault logic for three
 *        digital IR sensors.
 *
 * This module is pure logic. It makes no hardware or kernel calls, so
 * every rule can be tested on the Pico W with synthetic samples.
 *
 * NOTE: The sample counts below are starting values. They must be tuned
 * on the track (assumption A5, Design Review section 7).
 */
#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdbool.h>
#include <stdint.h>

/* One IR sample uses these bits. A set bit means the sensor sees black. */
#define LINE_TRACK_BIT_LEFT       (0x04u)
#define LINE_TRACK_BIT_CENTRE     (0x02u)
#define LINE_TRACK_BIT_RIGHT      (0x01u)
#define LINE_TRACK_BITS_ALL       (0x07u)

/* A negative position means the line is left of the centre sensor. */
#define LINE_TRACK_POS_MIN        (-2)
#define LINE_TRACK_POS_MAX        (2)

/* Consecutive samples needed before a state change is accepted. */
#define LINE_TRACK_LOST_SAMPLES   (5u)
#define LINE_TRACK_FOUND_SAMPLES  (2u)
#define LINE_TRACK_FAULT_SAMPLES  (20u)

/* Event bits returned by line_track_update(). */
#define LINE_TRACK_EVENT_LOST     (0x01u)
#define LINE_TRACK_EVENT_FOUND    (0x02u)
#define LINE_TRACK_EVENT_FAULT    (0x04u)

/** Tracking state. LINE_TRACK_FAULT stays until line_track_init(). */
typedef enum
{
    LINE_TRACK_FOLLOWING = 0,
    LINE_TRACK_LOST      = 1,
    LINE_TRACK_FAULT     = 2
} line_track_mode_t;

/** History needed to debounce one sensor array. */
typedef struct
{
    line_track_mode_t mode;           /**< Current tracking state.       */
    int8_t            position;       /**< Last valid position. While the
                                           line is lost, its sign shows
                                           the side it was last seen.    */
    bool              b_valid;        /**< Position is from this sample. */
    uint8_t           lost_count;     /**< Consecutive empty samples.    */
    uint8_t           found_count;    /**< Consecutive line samples.     */
    uint8_t           invalid_count;  /**< Consecutive invalid samples.  */
} line_track_t;

/**
 * @brief Reset to FOLLOWING, centred, with no history.
 * @param p_track State to reset. NULL is ignored.
 */
void line_track_init (line_track_t * p_track);

/**
 * @brief Process one IR sample.
 * @param p_track State to update. NULL is ignored.
 * @param bits    Sample built from the LINE_TRACK_BIT_* values.
 * @return The LINE_TRACK_EVENT_* bits raised by this sample.
 */
uint8_t line_track_update (line_track_t * p_track, uint8_t bits);

#endif /* LINE_TRACK_H */

/* End of file. */

