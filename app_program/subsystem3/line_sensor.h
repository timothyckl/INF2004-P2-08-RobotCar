/**
 * @file line_sensor.h
 * @brief Line Sensor task: line position, line events and barcode
 *        commands for the Vehicle Controller (IF-06).
 *
 * NOTE: IF-06 is proposed by subsystem 3 and not yet agreed. Agree the
 * event bits and the status layout with Dalton (S2) and Hong Yih (S1)
 * before relying on them.
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>
#include <tk/tkernel.h>

/* The sampling period equals the kernel tick, the fastest possible. */
#define LINE_SENSOR_PERIOD_MS         (10u)

/* Event bits returned by line_sensor_events_wait(). */
#define LINE_SENSOR_EVENT_LOST        (0x0001u) /* The line is gone.       */
#define LINE_SENSOR_EVENT_FOUND       (0x0002u) /* It was found again.     */
#define LINE_SENSOR_EVENT_JUNCTION    (0x0004u) /* One full bar crossed.   */
#define LINE_SENSOR_EVENT_BARCODE     (0x0008u) /* A new command is ready. */
#define LINE_SENSOR_EVENT_BARCODE_ERR (0x0010u) /* Bars were not decoded.  */
#define LINE_SENSOR_EVENT_FAULT       (0x0020u) /* Readings are invalid.   */
#define LINE_SENSOR_EVENT_ALL         (0x003Fu)

/* Layout of the 16-bit status word:
     bits  0-2   IR sensors L, C, R (1 = black)
     bits  3-5   line position + LINE_SENSOR_STATUS_POS_OFFSET (0 to 4)
     bit   6     1 if the position comes from the current sample
     bits  7-8   line_track_mode_t (line_track.h)
     bits  9-11  last decoded barcode_cmd_t (barcode.h)
     bits 12-15  reserved, always 0 */
#define LINE_SENSOR_STATUS_IR_SHIFT     (0u)
#define LINE_SENSOR_STATUS_IR_MASK      (0x7u)
#define LINE_SENSOR_STATUS_POS_SHIFT    (3u)
#define LINE_SENSOR_STATUS_POS_MASK     (0x7u)
#define LINE_SENSOR_STATUS_POS_OFFSET   (2)
#define LINE_SENSOR_STATUS_VALID_SHIFT  (6u)
#define LINE_SENSOR_STATUS_MODE_SHIFT   (7u)
#define LINE_SENSOR_STATUS_MODE_MASK    (0x3u)
#define LINE_SENSOR_STATUS_CMD_SHIFT    (9u)
#define LINE_SENSOR_STATUS_CMD_MASK     (0x7u)

/**
 * @brief Create the event flag and the sampling clock.
 *
 * Called by subsystem3_init() only.
 *
 * @return E_OK, or the kernel error.
 */
ER line_sensor_init (void);

/**
 * @brief Start the line sensor task, then its sampling clock.
 *
 * Called by subsystem3_start() only.
 *
 * @param priority Task priority from subsystem 3's band.
 * @return E_OK, or the kernel error.
 */
ER line_sensor_start (PRI priority);

/**
 * @brief Read the latest status word.
 *
 * One task writes the word as a single aligned halfword, so any task may
 * call this function without a lock.
 *
 * @return The status word described above.
 */
uint16_t line_sensor_status_get (void);

/**
 * @brief Read the number of samples taken since start-up.
 *
 * The count wraps after 2^32 samples. Compare two readings by unsigned
 * subtraction to check that sensing is still running.
 *
 * @return The sample count.
 */
uint32_t line_sensor_samples_get (void);

/**
 * @brief Wait for line sensor events.
 *
 * Only one task, the Vehicle Controller, may call this function. Only the
 * bits returned are cleared, so an event raised later stays pending.
 *
 * @param p_events Receives the LINE_SENSOR_EVENT_* bits; 0 on error.
 * @param timeout  Kernel timeout (TMO_POL, TMO_FEVR or milliseconds).
 * @return E_OK, E_PAR for a NULL pointer, or the kernel error.
 */
ER line_sensor_events_wait (uint32_t * p_events, TMO timeout);

#endif /* LINE_SENSOR_H */

/* End of file. */

