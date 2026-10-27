/** @file telemetry_encoder.h
 * @brief Provisional binary telemetry format for isolated codec development.
 */
#ifndef TELEMETRY_ENCODER_H
#define TELEMETRY_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_ENCODER_SIZE 8u
#define TELEMETRY_ENCODER_VERSION 1u
#define TELEMETRY_ENCODER_DISTANCE_VALID 0x01u
#define TELEMETRY_ENCODER_STOPPED        0x02u
#define TELEMETRY_ENCODER_LINK_UP        0x04u

typedef struct
{
    uint32_t captured_ms;
    uint16_t distance_mm;
    bool     distance_valid;
    bool     stopped;
    bool     link_up;
} telemetry_encoder_snapshot_t;

/**
 * @brief Encode version, flags, uint32 time and uint16 distance, little-endian.
 * @param p_snapshot Immutable, consistent caller-owned snapshot.
 * @param p_output Separate writable buffer; must not overlap the snapshot.
 * @param capacity Writable bytes available at p_output.
 * @return Eight on success; zero on invalid arguments or insufficient space.
 * @note Failure leaves output unchanged; invalid distance encodes as zero.
 * No allocation, locks, network calls or structure overlays are used.
 */
size_t telemetry_encoder_encode (
    const telemetry_encoder_snapshot_t * p_snapshot,
    uint8_t * p_output, size_t capacity);

#endif /* TELEMETRY_ENCODER_H */

/*** End of file. ***/

