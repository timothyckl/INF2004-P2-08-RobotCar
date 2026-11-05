/**
 * @file drv_watchdog.h
 * @brief Subsystem 2 RP2040 hardware watchdog driver.
 *
 * If the CPU hangs, the watchdog resets the chip within the timeout. The
 * reset puts the PWM block back in reset and the GPIO pads back to their
 * reset state (pull-down), so the motor driver inputs go low and the
 * motors stop. It is the only protection that works when no code runs.
 *
 * Only Motion Control kicks it, once per control period, so a hung or
 * starved Motion Control task is what triggers the reset.
 */
#ifndef DRV_WATCHDOG_H
#define DRV_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

/* Longest timeout: the 24-bit LOAD register at 2 counts per microsecond
   (erratum RP2040-E1): 0xFFFFFF / 2000 = 8388 ms. */
#define DRV_WATCHDOG_MAX_MS     8388u

/* Arm the watchdog. E_PAR for 0 or over DRV_WATCHDOG_MAX_MS; E_OBJ if the
   watchdog tick is not running (it is never reconfigured here, because
   the kernel's 1 MHz timer runs from it). */
ER       drv_watchdog_start (uint32_t timeout_ms);

/* Reload the counter. No effect before drv_watchdog_start(). */
void     drv_watchdog_kick (void);

/* True if the last reset came from this driver's watchdog timing out
   (REASON.TIMER and the start marker in SCRATCH0); false after power-on,
   the RUN pin, or a boot-ROM reboot such as after flashing a UF2. */
bool     drv_watchdog_caused_reset (void);

/* LOAD value for a timeout (pure): timeout_ms x 1000 x 2, or 0 if the
   timeout is 0 or over DRV_WATCHDOG_MAX_MS. */
uint32_t drv_watchdog_load_value (uint32_t timeout_ms);

#endif /* DRV_WATCHDOG_H */

/* End of file. */
