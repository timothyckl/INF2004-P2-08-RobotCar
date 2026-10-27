/**
 * @file range_measurement.h
 * @brief Conversion and validation of ultrasonic echo durations.
 */
#ifndef RANGE_MEASUREMENT_H
#define RANGE_MEASUREMENT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Convert a valid HC-SR04 echo duration to whole millimetres.
 *
 * The output is not changed when the duration is invalid.
 *
 * @param echo_us Echo high duration in microseconds.
 * @param p_distance_mm Destination for the converted distance.
 * @return true when the duration is within the supported physical range.
 */
bool range_measurement_from_echo(uint32_t echo_us,
                                 uint16_t * p_distance_mm);

#endif /* RANGE_MEASUREMENT_H */

/* End of file. */
