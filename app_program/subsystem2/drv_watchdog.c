/**
 * @file drv_watchdog.c
 * @brief Subsystem 2 RP2040 hardware watchdog driver (see drv_watchdog.h).
 *
 * The port has no watchdog or PSM defines, so the registers are defined
 * here. Values are from the Pico SDK's generated register headers
 * (hardware/regs/watchdog.h, psm.h, addressmap.h), which follow the
 * RP2040 datasheet's Watchdog and Power-On State Machine (PSM) chapters
 * (docs/datasheets/rp2040-datasheet.pdf).
 *
 * - Erratum RP2040-E1: the counter decrements twice per 1 us tick, so the
 *   load value is twice the microsecond count (as the SDK does).
 * - PSM WDSEL resets to 0, which means a watchdog timeout resets nothing.
 *   Start sets every PSM domain except the two oscillators (as the SDK
 *   does), so the processors, RESETS (and so PWM) and IO are reset.
 * - CTRL's PAUSE_DBG0/DBG1/JTAG bits are 1 at reset and left alone: the
 *   counter pauses while a debugger halts the CPU.
 * - The watchdog tick (TICK) also clocks the 1 MHz system timer the
 *   kernel and this subsystem use. The kernel sets it in startup_clock();
 *   this driver only checks that it runs.
 * - SCRATCH0 holds a marker written at start, so a timeout can be told
 *   apart from the boot ROM's own watchdog reboot (after flashing).
 *   SCRATCH0-3 are free for applications; the boot ROM uses SCRATCH4-7.
 */
#include "drv_watchdog.h"

#define WDT_BASE            0x40058000u
#define WDT_CTRL            (WDT_BASE + 0x00u)
#define WDT_LOAD            (WDT_BASE + 0x04u)
#define WDT_REASON          (WDT_BASE + 0x08u)
#define WDT_SCRATCH0        (WDT_BASE + 0x0Cu)
#define WDT_TICK            (WDT_BASE + 0x2Cu)

#define WDT_CTRL_ENABLE     (1u << 30)
#define WDT_REASON_TIMER    (1u << 0)
#define WDT_TICK_RUNNING    (1u << 10)
#define WDT_LOAD_MAX        0x00FFFFFFu     /* 24-bit counter */
#define WDT_COUNTS_PER_MS   2000u           /* 1000 us x 2 (E1) */
#define WDT_MARKER          0x53325744u     /* "S2WD" */

#define PSM_WDSEL           (0x40010000u + 0x08u)
#define PSM_WDSEL_ALL       0x0001FFFFu
#define PSM_WDSEL_ROSC      (1u << 0)
#define PSM_WDSEL_XOSC      (1u << 1)

#define REG_ALIAS_SET       0x2000u         /* atomic set alias */
#define REG_ALIAS_CLR       0x3000u         /* atomic clear alias */

static uint32_t volatile g_load = 0u;       /* 0 until started */

uint32_t
drv_watchdog_load_value (uint32_t timeout_ms)
{
    if ((0u == timeout_ms) || (timeout_ms > DRV_WATCHDOG_MAX_MS))
    {
        return (0u);
    }

    return (timeout_ms * WDT_COUNTS_PER_MS);  /* <= WDT_LOAD_MAX */
}

ER
drv_watchdog_start (uint32_t timeout_ms)
{
    uint32_t const load = drv_watchdog_load_value(timeout_ms);

    if (0u == load)
    {
        return (E_PAR);
    }

    if (0u == (in_w(WDT_TICK) & WDT_TICK_RUNNING))
    {
        return (E_OBJ);
    }

    out_w(WDT_CTRL + REG_ALIAS_CLR, WDT_CTRL_ENABLE);
    out_w(PSM_WDSEL + REG_ALIAS_SET,
          PSM_WDSEL_ALL & ~(PSM_WDSEL_ROSC | PSM_WDSEL_XOSC));
    out_w(WDT_SCRATCH0, WDT_MARKER);
    g_load = load;
    out_w(WDT_LOAD, load);
    out_w(WDT_CTRL + REG_ALIAS_SET, WDT_CTRL_ENABLE);

    return (E_OK);
}

void
drv_watchdog_kick (void)
{
    uint32_t const load = g_load;

    if (0u != load)
    {
        out_w(WDT_LOAD, load);
    }
}

bool
drv_watchdog_caused_reset (void)
{
    return ((0u != (in_w(WDT_REASON) & WDT_REASON_TIMER))
            && (WDT_MARKER == in_w(WDT_SCRATCH0)));
}

/* End of file. */
