/**
 * @file drv_ultrasonic.c
 * @brief HC-SR04 driver (see drv_ultrasonic.h).
 */
#include "../include/drv_ultrasonic.h"

#include "../../subsystem2/drv_gpio_irq.h"

#define DRV_US_TRIG_PIN         (4u)
#define DRV_US_ECHO_PIN         (5u)
#define DRV_US_PULSE_US         (12u)   /* datasheet minimum is 10 us */
#define DRV_US_RING             (16u)

static volatile uint32_t g_us_edge_time[DRV_US_RING];
static volatile uint8_t  g_us_edge_high[DRV_US_RING];
static volatile uint32_t g_us_head     = 0u;    /* interrupt only */
static volatile uint32_t g_us_tail     = 0u;    /* scan task only */
static volatile bool     gb_us_overrun = false;

/** Dispatcher callback, interrupt context: one read and a ring store. */
static void drv_ultrasonic_isr (uint32_t gpio, uint32_t events,
                                uint32_t now_us);

/** Order ring accesses as seen by the other core (ARMv6-M DMB). */
static void drv_ultrasonic_barrier (void);

ER
drv_ultrasonic_init (void)
{
    ER er = gpio_set_val(DRV_US_TRIG_PIN, 0u);

    if (E_OK == er)
    {
        er = gpio_set_pin(DRV_US_TRIG_PIN, GPIO_MODE_OUT);
    }

    if (E_OK == er)
    {
        er = gpio_set_pin(DRV_US_ECHO_PIN, GPIO_MODE_IN);
    }

    if (E_OK == er)
    {
        /* Pull-down: an unconnected ECHO reads low, never as a pulse. */
        clr_w(GPIO(DRV_US_ECHO_PIN), GPIO_PUE);
        set_w(GPIO(DRV_US_ECHO_PIN), GPIO_PDE);
        er = drv_gpio_irq_register(DRV_US_ECHO_PIN,
                                   DRV_GPIO_IRQ_EDGE_RISE
                                   | DRV_GPIO_IRQ_EDGE_FALL,
                                   drv_ultrasonic_isr);
    }

    return (er);
}

uint32_t
drv_ultrasonic_trigger (void)
{
    uint32_t const start_us = in_w(TIMER_TIMERAWL);

    (void) gpio_set_val(DRV_US_TRIG_PIN, 1u);

    /* A 12 us busy wait: shorter than one kernel tick, and interrupts can
       only lengthen the pulse, which the module accepts. */
    while ((in_w(TIMER_TIMERAWL) - start_us) < DRV_US_PULSE_US)
    {
        /* Wait for the end of the pulse. */
    }

    (void) gpio_set_val(DRV_US_TRIG_PIN, 0u);

    return (start_us);
}

bool
drv_ultrasonic_edge (bool * p_high, uint32_t * p_time_us)
{
    bool b_has_edge = false;

    if ((NULL != p_high) && (NULL != p_time_us)
        && (g_us_tail != g_us_head))
    {
        uint32_t const index = g_us_tail;

        drv_ultrasonic_barrier();
        *p_high    = (0u != g_us_edge_high[index]);
        *p_time_us = g_us_edge_time[index];
        drv_ultrasonic_barrier();
        g_us_tail  = (index + 1u) % DRV_US_RING;
        b_has_edge = true;
    }

    return (b_has_edge);
}

bool
drv_ultrasonic_overrun (void)
{
    bool const b_overrun = gb_us_overrun;

    gb_us_overrun = false;

    return (b_overrun);
}

bool
drv_ultrasonic_echo_high (void)
{
    return (0u != gpio_get_val(DRV_US_ECHO_PIN));
}

static void
drv_ultrasonic_isr (uint32_t gpio, uint32_t events, uint32_t now_us)
{
    uint32_t const head = g_us_head;
    uint32_t const next = (head + 1u) % DRV_US_RING;

    (void) gpio;
    (void) events;

    if (next == g_us_tail)
    {
        gb_us_overrun = true;
    }
    else
    {
        g_us_edge_high[head] =
            (uint8_t) ((0u != (in_w(GPIO_IN) & (1u << DRV_US_ECHO_PIN)))
                       ? 1u : 0u);
        g_us_edge_time[head] = now_us;
        drv_ultrasonic_barrier();
        g_us_head = next;
    }

}

static void
drv_ultrasonic_barrier (void)
{
    __asm volatile ("dmb" ::: "memory");
}

/* End of file. */
