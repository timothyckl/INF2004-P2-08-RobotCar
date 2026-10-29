/** @file telemetry_encoder.c
 * @brief Byte encoding independent of structure padding and byte order.
 */
#include "telemetry_encoder.h"

size_t telemetry_encoder_encode (
    const telemetry_encoder_snapshot_t * p_snapshot,
    uint8_t * p_output, size_t capacity)
{
    size_t written = 0u;

    if ((NULL != p_snapshot) && (NULL != p_output) &&
        (capacity >= TELEMETRY_ENCODER_SIZE))
    {
        uint8_t  flags = 0u;
        uint32_t distance = 0u;

        if (p_snapshot->distance_valid)
        {
            flags |= TELEMETRY_ENCODER_DISTANCE_VALID;
            distance = p_snapshot->distance_mm;
        }

        if (p_snapshot->stopped)
        {
            flags |= TELEMETRY_ENCODER_STOPPED;
        }

        if (p_snapshot->link_up)
        {
            flags |= TELEMETRY_ENCODER_LINK_UP;
        }

        p_output[0] = TELEMETRY_ENCODER_VERSION;
        p_output[1] = flags;

        /* Masks limit every narrowed value to the uint8_t range. */
        p_output[2] = (uint8_t)(p_snapshot->captured_ms & UINT32_C(0xff));
        p_output[3] = (uint8_t)((p_snapshot->captured_ms >> 8u) &
                               UINT32_C(0xff));
        p_output[4] = (uint8_t)((p_snapshot->captured_ms >> 16u) &
                               UINT32_C(0xff));
        p_output[5] = (uint8_t)((p_snapshot->captured_ms >> 24u) &
                               UINT32_C(0xff));
        p_output[6] = (uint8_t)(distance & UINT32_C(0xff));
        p_output[7] = (uint8_t)((distance >> 8u) & UINT32_C(0xff));
        written = TELEMETRY_ENCODER_SIZE;
    }

    return written;
}

/*** End of file. ***/

