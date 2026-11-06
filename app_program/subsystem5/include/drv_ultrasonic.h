/**
 * @file drv_ultrasonic.h
 * @brief HC-SR04 driver: TRIG on GP4, ECHO on GP5 (AGENTS.md).
 *
 * Datasheet (docs/datasheets/HCSR04-datasheet.pdf): a TRIG high pulse of
 * at least 10 us starts a measurement; ECHO is then high for the
 * round-trip time; a measurement cycle of over 60 ms is advised. The
 * datasheet's working voltage is DC 5 V. The module is wired to 3V3 on
 * this robot: ECHO then cannot exceed the Pico's 3.3 V input level, but
 * ranging at 3V3 is NOT validated (decision 2; S5-U46, S5-U48).
 *
 * ECHO edges are timestamped by subsystem 2's shared GPIO dispatcher and
 * queued; only the scan task reads them. Call only from the scan task.
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#include "../../app.h"

/** Configure TRIG low, ECHO as an input, and register the ECHO interrupt. */
ER drv_ultrasonic_init (void);

/** Send one TRIG pulse of at least 10 us; returns its start time (us). */
uint32_t drv_ultrasonic_trigger (void);

/**
 * @brief Take the oldest queued ECHO edge.
 *
 * @param p_high Receives the ECHO level after the edge.
 * @param p_time_us Receives the edge time from the interrupt.
 * @return false when no edge is queued.
 */
bool drv_ultrasonic_edge (bool * p_high, uint32_t * p_time_us);

/** True once if edges were lost (queue full) since the last call. */
bool drv_ultrasonic_overrun (void);

/** Current ECHO level. */
bool drv_ultrasonic_echo_high (void);

#endif /* DRV_ULTRASONIC_H */

/* End of file. */
