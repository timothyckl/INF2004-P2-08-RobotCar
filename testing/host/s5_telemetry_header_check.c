/**
 * @file s5_telemetry_header_check.c
 * @brief Verify S1 and S5 public telemetry headers coexist without RTOS types.
 * Subsystem 5 -- Timothy.  Student ID: 2501530.
 */
#include "../../app_program/subsystem1/telemetry_encoder.h"
#include "../../app_program/subsystem5/include/scan_telemetry.h"

typedef char scan_telemetry_header_check[
    (sizeof(scan_telemetry_t) > 0u) ? 1 : -1];

/* End of file. */
