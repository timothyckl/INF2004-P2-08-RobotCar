/**
 * @file telemetry_snapshot.h
 * @brief Decoded telemetry snapshot that subsystem 2 fills for subsystem 1.
 *
 * Kernel-free on purpose: only <stdbool.h> and <stdint.h>. The kernel and
 * <stddef.h> define size_t differently, so kernel-side code (subsystem 2's
 * robot_state.c) includes this header, never telemetry_encoder.h.
 *
 * Enumerated fields carry the providers' own codes unchanged, so this file
 * holds only their valid ranges (S1 decision 1 lists the codes):
 *   state  S2_SYS_*            mode   S2_MODE_*         fault S2_FAULT_*
 *   line   S2_LINE_*           scan_result SCAN_TELEMETRY_* result
 *   bypass_side SCAN_TELEMETRY_SIDE_*
 * Each enumerated field is sent in a nibble; a value above
 * TELEMETRY_SNAPSHOT_CODE_MAX is sent as TELEMETRY_SNAPSHOT_CODE_INVALID.
 */
#ifndef TELEMETRY_SNAPSHOT_H
#define TELEMETRY_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#define TELEMETRY_SNAPSHOT_CODE_MAX      14u
#define TELEMETRY_SNAPSHOT_CODE_INVALID  15u

/*
 * One internally consistent view. A field is meaningful only while its
 * validity flag is set; the encoder sends zero otherwise.
 */
typedef struct
{
    uint32_t captured_ms;       /* controller clock, wraps */
    uint16_t distance_mm;       /* run distance;   distance_valid */
    int16_t  left_mm_s;         /* wheel speeds;   motion_valid */
    int16_t  right_mm_s;
    int16_t  heading_cdeg;      /* left positive, -18000..18000;
                                   motion_valid */
    uint16_t forward_mm;        /* ultrasonic, 20..4000; forward_valid */
    uint16_t nearest_mm;        /* nearest obstacle ray; profile_valid */
    uint16_t width_mm;          /* observed span;  shape_valid */
    int16_t  centre_offset_mm;  /* left positive;  shape_valid */
    uint8_t  state;             /* S2_SYS_* */
    uint8_t  mode;              /* S2_MODE_* */
    uint8_t  fault;             /* S2_FAULT_* */
    uint8_t  line;              /* S2_LINE_*;      line_fresh */
    uint8_t  scan_result;       /* SCAN_TELEMETRY_* result */
    uint8_t  bypass_side;       /* SCAN_TELEMETRY_SIDE_* */
    uint8_t  hump_max_mm;       /* run maximum, saturated; hump_valid */
    bool     distance_valid;
    bool     stopped;           /* both wheels confirmed still */
    bool     link_up;           /* set by subsystem 1 */
    bool     stop_latched;
    bool     motion_valid;
    bool     line_fresh;        /* line report no older than 100 ms */
    bool     hump_valid;
    bool     forward_valid;
    bool     profile_valid;
    bool     shape_valid;
} telemetry_encoder_snapshot_t;

#endif /* TELEMETRY_SNAPSHOT_H */

/* End of file. */
