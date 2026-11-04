/**
 * @file drv_gpio_irq.c
 * @brief Shared GPIO edge-interrupt dispatcher (see drv_gpio_irq.h).
 *
 * Register layout from the Pico SDK's RP2040 register headers (generated
 * from the RP2040 datasheet, IO_BANK0 chapter): four 32-bit registers
 * cover GPIO0-29, 8 pins each, 4 bits per pin (level low, level high,
 * edge low, edge high). INTR holds raw latched edges (write 1 to clear);
 * PROCn_INTE enables and PROCn_INTS shows enabled events for core n.
 *
 * The interrupt is enabled in the NVIC of the core that runs init, so only
 * that core's INTE/INTS registers are used. INTE is shared by several
 * owners (GP3 and GP7 are in the same register), so it is changed only
 * through the RP2040's atomic SET/CLR register aliases, never by
 * read-modify-write, which would race with the other core.
 */
#include "drv_gpio_irq.h"

#include <stdbool.h>

#include "hardware/platform_defs.h"     /* _u() used by the next two */
#include "hardware/regs/intctrl.h"
#include "hardware/regs/io_bank0.h"

#define DRV_GPIO_IRQ_PRI        2       /* same as the kernel ptimer */
#define DRV_GPIO_IRQ_REGS       4u      /* registers covering GPIO0-29 */
#define DRV_GPIO_IRQ_PER_REG    8u      /* pins per register */
#define DRV_GPIO_IRQ_BITS       4u      /* event bits per pin */
#define DRV_GPIO_IRQ_PIN_MASK   0xFu
#define DRV_GPIO_IRQ_EDGES      (DRV_GPIO_IRQ_EDGE_FALL | DRV_GPIO_IRQ_EDGE_RISE)
#define DRV_GPIO_IRQ_ALL_EDGES  0xCCCCCCCCu /* edge bits of all 8 pins */
#define DRV_GPIO_IRQ_REG_STRIDE 4u
#define DRV_GPIO_IRQ_ALIAS_SET  0x2000u /* RP2040 atomic alias, = SDK */
#define DRV_GPIO_IRQ_ALIAS_CLR  0x3000u /* REG_ALIAS_SET/CLR_BITS */

/* Pico W radio pins (design review section 2): never usable. */
#define DRV_GPIO_IRQ_RADIO_MASK ((1u << 23) | (1u << 24) | (1u << 25) \
                                 | (1u << 29))

static drv_gpio_irq_cb_t volatile g_callbacks[GPIO_NUM];
static UW g_inte_base;      /* PROCn_INTE0 of the init core */
static UW g_ints_base;      /* PROCn_INTS0 of the init core */
static bool g_b_ready = false;

static void drv_gpio_irq_handler (UINT intno);
static UW   drv_gpio_irq_reg (UW base, uint32_t gpio);
static UW   drv_gpio_irq_bits (uint32_t gpio, uint32_t events);

/**
 * @brief Install the IO_IRQ_BANK0 handler on the calling core.
 *
 * Call once, from task context, before any registration. Repeat calls
 * return E_OK without changing anything.
 */
ER
drv_gpio_irq_init (void)
{
    T_DINT const dint = {TA_HLNG, (FP)drv_gpio_irq_handler};

    if (g_b_ready)
    {
        return (E_OK);
    }

    if (0u == in_w(SIO_CPUID_ADDR))
    {
        g_inte_base = IO_BANK0_BASE + IO_BANK0_PROC0_INTE0_OFFSET;
        g_ints_base = IO_BANK0_BASE + IO_BANK0_PROC0_INTS0_OFFSET;
    }
    else
    {
        g_inte_base = IO_BANK0_BASE + IO_BANK0_PROC1_INTE0_OFFSET;
        g_ints_base = IO_BANK0_BASE + IO_BANK0_PROC1_INTS0_OFFSET;
    }

    for (UW r = 0u; r < DRV_GPIO_IRQ_REGS; r++)
    {
        out_w(g_inte_base + (r * DRV_GPIO_IRQ_REG_STRIDE), 0u);
    }

    ER const er = tk_def_int(IO_IRQ_BANK0, &dint);

    if (er < E_OK)
    {
        return (er);
    }

    g_b_ready = true;
    EnableInt(IO_IRQ_BANK0, DRV_GPIO_IRQ_PRI);

    return (E_OK);
}

/**
 * @brief Route edge interrupts on one pin to a callback.
 * @param events DRV_GPIO_IRQ_EDGE_FALL and/or DRV_GPIO_IRQ_EDGE_RISE.
 * @return E_PAR for an invalid pin, radio pin, events or NULL callback;
 *         E_OBJ if the dispatcher is not initialised or the pin is taken.
 */
ER
drv_gpio_irq_register (uint32_t gpio, uint32_t events, drv_gpio_irq_cb_t cb)
{
    if ((gpio >= (uint32_t)GPIO_NUM)
        || (0u != ((1u << gpio) & DRV_GPIO_IRQ_RADIO_MASK))
        || (0u == events) || (0u != (events & ~DRV_GPIO_IRQ_EDGES))
        || (NULL == cb))
    {
        return (E_PAR);
    }

    if ((!g_b_ready) || (NULL != g_callbacks[gpio]))
    {
        return (E_OBJ);
    }

    UW const bits = drv_gpio_irq_bits(gpio, events);

    g_callbacks[gpio] = cb;

    /* Drop any edge latched before registration, then enable. */
    out_w(drv_gpio_irq_reg(IO_BANK0_BASE + IO_BANK0_INTR0_OFFSET, gpio),
          bits);
    out_w(drv_gpio_irq_reg(g_inte_base, gpio) + DRV_GPIO_IRQ_ALIAS_SET,
          bits);

    return (E_OK);
}

/**
 * @brief Disable a pin's interrupts and free its slot.
 * @return E_PAR for an invalid pin; E_OBJ if nothing is registered.
 */
ER
drv_gpio_irq_unregister (uint32_t gpio)
{
    if (gpio >= (uint32_t)GPIO_NUM)
    {
        return (E_PAR);
    }

    if ((!g_b_ready) || (NULL == g_callbacks[gpio]))
    {
        return (E_OBJ);
    }

    out_w(drv_gpio_irq_reg(g_inte_base, gpio) + DRV_GPIO_IRQ_ALIAS_CLR,
          drv_gpio_irq_bits(gpio, DRV_GPIO_IRQ_PIN_MASK));
    g_callbacks[gpio] = NULL;   /* the handler skips a NULL slot */

    return (E_OK);
}

/*
 * Each enabled pin's edges are acknowledged before its callback runs, so
 * an edge that arrives during the callback latches again and is not lost.
 */
static void
drv_gpio_irq_handler (UINT intno)
{
    UW const now_us = in_w(TIMER_TIMERAWL);

    (void)intno;

    for (UW r = 0u; r < DRV_GPIO_IRQ_REGS; r++)
    {
        UW const ints = in_w(g_ints_base + (r * DRV_GPIO_IRQ_REG_STRIDE))
                        & DRV_GPIO_IRQ_ALL_EDGES;

        if (0u == ints)
        {
            continue;
        }

        out_w(IO_BANK0_BASE + IO_BANK0_INTR0_OFFSET
              + (r * DRV_GPIO_IRQ_REG_STRIDE), ints);

        for (UW p = 0u; p < DRV_GPIO_IRQ_PER_REG; p++)
        {
            UW const events = (ints >> (p * DRV_GPIO_IRQ_BITS))
                              & DRV_GPIO_IRQ_EDGES;
            uint32_t const gpio = (r * DRV_GPIO_IRQ_PER_REG) + p;

            if ((0u != events) && (gpio < (uint32_t)GPIO_NUM))
            {
                drv_gpio_irq_cb_t const cb = g_callbacks[gpio];

                if (NULL != cb)
                {
                    cb(gpio, events, now_us);
                }
            }
        }
    }
}

/* Address of the register holding this pin's bits, from a register 0. */
static UW
drv_gpio_irq_reg (UW base, uint32_t gpio)
{
    return (base + ((gpio / DRV_GPIO_IRQ_PER_REG) * DRV_GPIO_IRQ_REG_STRIDE));
}

/* events shifted to this pin's 4-bit field. */
static UW
drv_gpio_irq_bits (uint32_t gpio, uint32_t events)
{
    return ((events & DRV_GPIO_IRQ_PIN_MASK)
            << ((gpio % DRV_GPIO_IRQ_PER_REG) * DRV_GPIO_IRQ_BITS));
}

/* End of file. */
