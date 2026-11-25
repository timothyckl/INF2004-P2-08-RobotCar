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
 *   barcode_command S2_NAV_*   stop_cause S2_STOP_*
 * Each nibble field is sent so that a value above
 * TELEMETRY_SNAPSHOT_CODE_MAX becomes TELEMETRY_SNAPSHOT_CODE_INVALID.
 * The 3-bit barcode command and 5-bit letter use the same rule with their
 * own limits (telemetry_encoder.h).
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
    uint16_t width_mm;          /* observed span;  shape_valid, or the
                                   unrefined raw span; width_raw */
    int16_t  centre_offset_mm;  /* left positive;  shape_valid */
    uint8_t  state;             /* S2_SYS_* */
    uint8_t  mode;              /* S2_MODE_* */
    uint8_t  fault;             /* S2_FAULT_* */
    uint8_t  line;              /* S2_LINE_*;      line_fresh */
    uint8_t  scan_result;       /* SCAN_TELEMETRY_* result */
    uint8_t  bypass_side;       /* SCAN_TELEMETRY_SIDE_* */
    uint8_t  hump_max_mm;       /* run maximum, saturated; hump_valid */
    uint8_t  hump_last_mm;      /* last completed hump, saturated;
                                   hump_last_valid */
    uint8_t  hump_count;        /* humps this run, saturates at 255;
                                   0 when S4 status is unavailable */
    char     barcode;           /* last letter, 'A'..'Z'; barcode_valid */
    uint8_t  barcode_command;   /* S2_NAV_*; barcode_valid */
    uint8_t  barcode_count;     /* letters decoded, wraps; barcode_valid */
    uint8_t  stop_cause;        /* S2_STOP_*; stop_latched */
    uint8_t  motion_fault;      /* MOTION_FAULT_* 0..7 while fault is
                                   motion, else 0 (version 5) */
    uint8_t  nav_held;          /* S2_NAV_* held for the next junction,
                                   0 none (version 5) */
    uint8_t  junction_count;    /* junctions confirmed, wraps (version 5) */
    uint8_t  motion_class;      /* S4_MOTION_* 0..8, 0 unknown (version 6) */
    int16_t  yaw_ddeg_s;        /* encoder-estimated yaw rate, 0.1 deg/s,
                                   left positive; with motion_class
                                   (version 6) */
    bool     b_distance_valid;
    bool     b_stopped;           /* both wheels confirmed still */
    bool     b_link_up;           /* set by subsystem 1 */
    bool     b_stop_latched;
    bool     b_motion_valid;
    bool     b_line_fresh;        /* line report no older than 100 ms */
    bool     b_hump_valid;
    bool     b_forward_valid;
    bool     b_profile_valid;
    bool     b_shape_valid;
    bool     b_barcode_valid;     /* a letter was decoded since power-up */
    bool     b_hump_last_valid;
    bool     b_width_raw;         /* width_mm is the raw span (an edge is
                                   missing); ignored when shape_valid */
} telemetry_encoder_snapshot_t;

#endif /* TELEMETRY_SNAPSHOT_H */

/* End of file. */
