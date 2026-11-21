/**
 * @file drv_gpio_irq.h
 * @brief Shared GPIO edge-interrupt dispatcher (owner: subsystem 2).
 *
 * The RP2040 has ONE interrupt line for all of IO bank 0 per core
 * (IO_IRQ_BANK0, IRQ 13), so every subsystem that needs a GPIO edge
 * interrupt registers a callback here instead of installing its own
 * handler (AAI08_PostTri1: single dispatcher for encoder, ECHO and any IR
 * edge interrupts). PROVISIONAL until agreed in TEAM.md.
 *
 * Usage:
 *   subsystem2_init() calls drv_gpio_irq_init() once at start-up, before
 *   any subsystemN_start(). Then the task that owns a pin (TEAM.md
 *   section 2) calls, for example:
 *       drv_gpio_irq_register(3u, DRV_GPIO_IRQ_EDGE_RISE |
 *                                 DRV_GPIO_IRQ_EDGE_FALL, echo_isr);
 *   Configure the pin as an input (gpio_set_pin) before registering.
 *
 * Limits:
 * - One callback per pin; at most 26 callbacks (GPIO0-22 and GP26-28;
 *   the radio pins GP23-25 and GP29 are refused). A second registration
 *   of a pin returns E_OBJ until it is unregistered.
 * - Edge interrupts only (rising, falling or both). Level interrupts are
 *   refused: they would fire continuously while the level holds.
 * - Registering the same pin from two tasks at once is not protected:
 *   each pin has a single owner by TEAM.md section 2.
 * - All callbacks share one interrupt and run one after another, lowest
 *   pin first, so a slow callback delays every other pin's edges.
 *
 * Callback rules: it runs in interrupt context on the core that called
 * drv_gpio_irq_init(), at NVIC priority 2. Keep it to a few
 * microseconds: read the pin level, store a timestamp, bump a counter, or
 * write to a buffer that only this callback writes. Never block, never
 * call a tk_ function that waits, never take a mutex, never print. To
 * wake a task use only tk_set_flg, tk_sig_sem or tk_wup_tsk.
 *
 * events may hold both edge bits if both happened since the last service;
 * read the pin level if the order matters. now_us is the low 32 bits of
 * the 1 MHz system timer (TIMERAWL), read once on entry, shared by every
 * callback in that interrupt (wraps after 71.6 minutes).
 */
#ifndef DRV_GPIO_IRQ_H
#define DRV_GPIO_IRQ_H

#include <stdint.h>

#include "../app.h"

/* Per-pin event bits, as laid out in the IO_BANK0 INTR/INTE registers. */
#define DRV_GPIO_IRQ_EDGE_FALL  0x4u
#define DRV_GPIO_IRQ_EDGE_RISE  0x8u

typedef void (*drv_gpio_irq_cb_t)(uint32_t gpio, uint32_t events,
                                  uint32_t now_us);

ER drv_gpio_irq_init (void);
ER drv_gpio_irq_register (uint32_t gpio, uint32_t events,
                          drv_gpio_irq_cb_t cb);
ER drv_gpio_irq_unregister (uint32_t gpio);

#endif /* DRV_GPIO_IRQ_H */

/* End of file. */
