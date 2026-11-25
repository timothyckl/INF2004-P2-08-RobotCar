/**
 * @file telemetry_encoder.c
 * @brief Byte encoding independent of structure padding and byte order.
 */
#include "telemetry_encoder.h"

#include <stdbool.h>

#define LETTER_INVALID   31u     /* 5-bit letter field */
#define COMMAND_MAX      6u      /* 3-bit command field */
#define COMMAND_INVALID  7u
#define HELD_MAX         4u      /* S2_NAV_UTURN */
#define HELD_INVALID     7u      /* 3-bit held-command field */
#define MOTION_FAULT_MASK 0x07u  /* MOTION_FAULT_* fit in 3 bits */

static uint8_t pack_codes (uint8_t low, uint8_t high);
static uint8_t pack_barcode (char letter, uint8_t command);
static void    put_u16 (uint8_t * p_out, uint16_t value);
static void    put_s16 (uint8_t * p_out, int16_t value);
static void    put_u32 (uint8_t * p_out, uint32_t value);
static uint8_t flag_if (bool b_set, uint8_t flag);
static uint8_t pack_diag (uint8_t motion_fault, uint8_t held,
                          uint8_t junctions);
static uint8_t pack_yaw (uint8_t motion_class, int16_t yaw_ddeg_s);

size_t telemetry_encoder_encode (
    const telemetry_encoder_snapshot_t * p_snapshot,
    uint8_t * p_output, size_t capacity)
{
    size_t written = 0u;

    if ((NULL != p_snapshot) && (NULL != p_output) &&
        (capacity >= TELEMETRY_ENCODER_SIZE))
    {
        const telemetry_encoder_snapshot_t * const p_s = p_snapshot;
        bool const b_width_raw = p_s->b_width_raw && (!p_s->b_shape_valid);
        /* Narrowing casts in this function: each flag word is an OR of single-
           bit masks below 0x100; each field keeps the range of its snapshot
           member (telemetry_snapshot.h): distances are already uint16_t,
           speeds are within +/-400 mm/s, heading +/-18000 cdeg and centre
           offset +/-4000 mm, all inside int16_t, and hump heights are
           saturated at 255 by S2. */
        uint8_t const flags0 = (uint8_t)(
            flag_if(p_s->b_distance_valid, TELEMETRY_ENCODER_DISTANCE_VALID)
            | flag_if(p_s->b_stopped, TELEMETRY_ENCODER_STOPPED)
            | flag_if(p_s->b_link_up, TELEMETRY_ENCODER_LINK_UP)
            | flag_if(p_s->b_stop_latched, TELEMETRY_ENCODER_STOP_LATCHED)
            | flag_if(p_s->b_motion_valid, TELEMETRY_ENCODER_MOTION_VALID)
            | flag_if(p_s->b_line_fresh, TELEMETRY_ENCODER_LINE_FRESH)
            | flag_if(p_s->b_hump_valid, TELEMETRY_ENCODER_HUMP_VALID)
            | flag_if(p_s->b_hump_last_valid,
                      TELEMETRY_ENCODER_HUMP_LAST_VALID));
        uint8_t const flags1 = (uint8_t)(
            flag_if(p_s->b_forward_valid, TELEMETRY_ENCODER_FORWARD_VALID)
            | flag_if(p_s->b_profile_valid, TELEMETRY_ENCODER_PROFILE_VALID)
            | flag_if(p_s->b_shape_valid, TELEMETRY_ENCODER_SHAPE_VALID)
            | flag_if(p_s->b_barcode_valid, TELEMETRY_ENCODER_BARCODE_VALID)
            | flag_if(b_width_raw, TELEMETRY_ENCODER_WIDTH_RAW));

        p_output[0] = TELEMETRY_ENCODER_VERSION;
        p_output[1] = flags0;
        put_u32(&p_output[2], p_s->captured_ms);
        put_u16(&p_output[6],
                (uint16_t)(p_s->b_distance_valid ? p_s->distance_mm : 0u));
        p_output[8]  = flags1;
        p_output[9]  = pack_codes(p_s->state, p_s->mode);
        p_output[10] = pack_codes(p_s->fault,
                                  (uint8_t)(p_s->b_line_fresh ? p_s->line
                                                              : 0u));
        p_output[11] = pack_codes(p_s->scan_result, p_s->bypass_side);
        put_s16(&p_output[12],
                (int16_t)(p_s->b_motion_valid ? p_s->left_mm_s : 0));
        put_s16(&p_output[14],
                (int16_t)(p_s->b_motion_valid ? p_s->right_mm_s : 0));
        put_s16(&p_output[16],
                (int16_t)(p_s->b_motion_valid ? p_s->heading_cdeg : 0));
        put_u16(&p_output[18],
                (uint16_t)(p_s->b_forward_valid ? p_s->forward_mm : 0u));
        put_u16(&p_output[20],
                (uint16_t)(p_s->b_profile_valid ? p_s->nearest_mm : 0u));
        put_u16(&p_output[22],
                (uint16_t)((p_s->b_shape_valid || b_width_raw)
                           ? p_s->width_mm : 0u));
        put_s16(&p_output[24],
                (int16_t)(p_s->b_shape_valid ? p_s->centre_offset_mm : 0));
        p_output[26] = (uint8_t)(p_s->b_hump_valid ? p_s->hump_max_mm : 0u);
        p_output[27] = p_s->b_barcode_valid
                       ? pack_barcode(p_s->barcode, p_s->barcode_command)
                       : 0u;
        /* The count is a wrapping counter, so its low bits are kept. */
        p_output[28] = (uint8_t)(
            pack_codes((uint8_t)(p_s->b_stop_latched ? p_s->stop_cause : 0u),
                       0u)
            | (uint8_t)((p_s->b_barcode_valid
                         ? (p_s->barcode_count & 0x0fu) : 0u) << 4u));
        p_output[29] = (uint8_t)(p_s->b_hump_last_valid
                                 ? p_s->hump_last_mm : 0u);
        p_output[30] = p_s->hump_count;
        p_output[31] = pack_diag(p_s->motion_fault, p_s->nav_held,
                                 p_s->junction_count);
        p_output[32] = pack_codes(p_s->motion_class, 0u);
        p_output[33] = pack_yaw(p_s->motion_class, p_s->yaw_ddeg_s);
        written = TELEMETRY_ENCODER_SIZE;
    }

    return written;
}

/** @brief Version 6 yaw rate: 0.1 deg/s to 2 deg/s per count, rounded half
 *  away from zero, saturated at +/-127, and 0 while the class is unknown. */
static uint8_t pack_yaw (uint8_t motion_class, int16_t yaw_ddeg_s)
{
    /* Widening: yaw is int16_t, so +/-10 fits int32_t. */
    int32_t const yaw = (int32_t)yaw_ddeg_s;
    int32_t counts = (yaw >= 0) ? ((yaw + 10) / 20) : ((yaw - 10) / 20);

    counts = (counts > INT8_MAX) ? INT8_MAX : counts;
    counts = (counts < -INT8_MAX) ? -INT8_MAX : counts;
    counts = (0u == motion_class) ? 0 : counts;

    /* counts is -127..127; its two's-complement byte is the int8_t on the
       wire: adding 256 to a negative value keeps it in 129..255. */
    return (uint8_t)((counts < 0) ? (counts + 256) : counts);
}

/** @brief Two codes in one byte; an out-of-range code becomes INVALID. */
static uint8_t pack_codes (uint8_t low, uint8_t high)
{
    uint8_t const low_code = (low > TELEMETRY_SNAPSHOT_CODE_MAX)
                       ? TELEMETRY_SNAPSHOT_CODE_INVALID : low;
    uint8_t const high_code = (high > TELEMETRY_SNAPSHOT_CODE_MAX)
                       ? TELEMETRY_SNAPSHOT_CODE_INVALID : high;

    /* Both codes are at most 15 (clamped above), so high_code << 4 is at most
       0xF0. */
    return (uint8_t)(low_code | (uint8_t)(high_code << 4u));
}

/** @brief Letter in bits 0-4, command in bits 5-7; out of range is INVALID. */
static uint8_t pack_barcode (char letter, uint8_t command)
{
    uint8_t const code = ((letter >= 'A') && (letter <= 'Z'))
                         ? (uint8_t)(letter - 'A') : LETTER_INVALID;
    uint8_t const cmd  = (command > COMMAND_MAX) ? COMMAND_INVALID : command;

    /* code is at most 31 and cmd at most 7, so the byte is at most 0xFF. */
    return (uint8_t)(code | (uint8_t)(cmd << 5u));
}

/** @brief Version 5 diagnostics: fault detail, held command, junctions. A
 *  fault detail above 7 or a held command above 4 is sent as 7 (invalid);
 *  the junction count is a wrapping counter, so its low 2 bits are kept. */
static uint8_t pack_diag (uint8_t motion_fault, uint8_t held,
                          uint8_t junctions)
{
    uint8_t const fault = (motion_fault > MOTION_FAULT_MASK)
                          ? MOTION_FAULT_MASK : motion_fault;
    uint8_t const cmd = (held > HELD_MAX) ? HELD_INVALID : held;

    /* fault <= 7, cmd <= 7 and junctions & 3 <= 3, so the shifted fields
       fill bits 0-2, 3-5 and 6-7 and the byte is at most 0xFF. */
    return (uint8_t)(fault | (uint8_t)(cmd << 3u)
                     | (uint8_t)((junctions & 0x03u) << 6u));
}

static void put_u16 (uint8_t * p_out, uint16_t value)
{
    /* Masks limit every narrowed value to the uint8_t range. */
    p_out[0] = (uint8_t)(value & 0xffu);
    p_out[1] = (uint8_t)((value >> 8u) & 0xffu);
}

static void put_s16 (uint8_t * p_out, int16_t value)
{
    /* Two's complement bit pattern, defined by the uint16_t conversion. */
    put_u16(p_out, (uint16_t)value);
}

static void put_u32 (uint8_t * p_out, uint32_t value)
{
    /* The masks keep each byte within 0..0xFF. */
    p_out[0] = (uint8_t)(value & UINT32_C(0xff));
    p_out[1] = (uint8_t)((value >> 8u) & UINT32_C(0xff));
    p_out[2] = (uint8_t)((value >> 16u) & UINT32_C(0xff));
    p_out[3] = (uint8_t)((value >> 24u) & UINT32_C(0xff));
}

static uint8_t flag_if (bool b_set, uint8_t flag)
{
    return (b_set ? flag : 0u);
}

/* End of file. */
