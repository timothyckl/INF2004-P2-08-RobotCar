/**
 * @file telemetry_encoder.h
 * @brief Telemetry wire format, version 3 (S1 decisions 1 and 5).
 *
 * 29 bytes, multi-byte fields little-endian:
 *
 *   Byte   Field
 *   0      version (2)
 *   1      flags0: bit 0 distance valid, 1 stopped, 2 MQTT link up,
 *                  3 STOP latched, 4 motion valid, 5 line fresh,
 *                  6 hump valid, 7 zero
 *   2-5    captured_ms         uint32_t
 *   6-7    distance_mm         uint16_t
 *   8      flags1: bit 0 forward valid, 1 profile valid, 2 shape valid,
 *                  3 barcode valid, 4-7 zero
 *   9      state (bits 0-3) | mode (bits 4-7)
 *   10     fault (bits 0-3) | line (bits 4-7)
 *   11     scan_result (bits 0-3) | bypass_side (bits 4-7)
 *   12-13  left_mm_s           int16_t
 *   14-15  right_mm_s          int16_t
 *   16-17  heading_cdeg        int16_t
 *   18-19  forward_mm          uint16_t
 *   20-21  nearest_mm          uint16_t
 *   22-23  width_mm            uint16_t
 *   24-25  centre_offset_mm    int16_t
 *   26     hump_max_mm         uint8_t
 *   27     barcode letter (bits 0-4: 0..25 = 'A'..'Z')
 *          | barcode command (bits 5-7: S2_NAV_*)
 *   28     stop cause (bits 0-3: S2_STOP_*, while STOP latched)
 *          | barcode count (bits 4-7: low 4 bits, wraps)
 *
 * Bytes 0-7 keep their version 1 positions and meanings; bytes 1-26 keep
 * their version 2 ones. A letter outside 'A'..'Z' is sent as 31 and a
 * command above 6 as 7 (invalid), never masked.
 */
#ifndef TELEMETRY_ENCODER_H
#define TELEMETRY_ENCODER_H

#include <stddef.h>
#include <stdint.h>

#include "telemetry_snapshot.h"

#define TELEMETRY_ENCODER_SIZE           29u
#define TELEMETRY_ENCODER_VERSION        3u

/* flags0 (byte 1) */
#define TELEMETRY_ENCODER_DISTANCE_VALID 0x01u
#define TELEMETRY_ENCODER_STOPPED        0x02u
#define TELEMETRY_ENCODER_LINK_UP        0x04u
#define TELEMETRY_ENCODER_STOP_LATCHED   0x08u
#define TELEMETRY_ENCODER_MOTION_VALID   0x10u
#define TELEMETRY_ENCODER_LINE_FRESH     0x20u
#define TELEMETRY_ENCODER_HUMP_VALID     0x40u

/* flags1 (byte 8) */
#define TELEMETRY_ENCODER_FORWARD_VALID  0x01u
#define TELEMETRY_ENCODER_PROFILE_VALID  0x02u
#define TELEMETRY_ENCODER_SHAPE_VALID    0x04u
#define TELEMETRY_ENCODER_BARCODE_VALID  0x08u

/**
 * @brief Encode one snapshot in the version 3 layout above.
 * @param p_snapshot Immutable, consistent caller-owned snapshot.
 * @param p_output Separate writable buffer; must not overlap the snapshot.
 * @param capacity Writable bytes available at p_output.
 * @return TELEMETRY_ENCODER_SIZE on success; zero on invalid arguments or
 *         insufficient space, leaving the output unchanged.
 * @note A field whose validity flag is clear is sent as zero. No
 *       allocation, locks, network calls or structure overlays are used.
 */
size_t telemetry_encoder_encode (
    const telemetry_encoder_snapshot_t * p_snapshot,
    uint8_t * p_output, size_t capacity);

#endif /* TELEMETRY_ENCODER_H */

/* End of file. */
