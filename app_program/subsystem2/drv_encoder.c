/**
 * @file drv_encoder.c
 * @brief Subsystem 2 quadrature wheel-encoder driver (see drv_encoder.h).
 *
 * Sequence lock: the interrupt makes g_seq odd, updates, then makes it
 * even. A reader that sees the same even g_seq before and after its copy
 * got a consistent snapshot. There is one writer (the dispatcher runs on
 * one core and does not nest), so the writer needs no lock and never
 * waits. The data memory barriers keep the counter and data accesses in
 * order as seen from the other core.
 */
#include "drv_encoder.h"

#include "drv_gpio_irq.h"

#define DRV_ENCODER_SNAP_TRIES  8u  /* an update takes ~1 us; 8 is ample */
#define DRV_ENCODER_EDGES       (DRV_GPIO_IRQ_EDGE_FALL | DRV_GPIO_IRQ_EDGE_RISE)

typedef struct
{
    uint32_t pin_a;
    uint32_t pin_b;
} drv_encoder_hw_t;

static const drv_encoder_hw_t g_encoder_hw[DRV_ENCODER_COUNT] =
{
    {7u,  28u},     /* DRV_ENCODER_1: left wheel, blue = A GP7, green = B GP28 */
    {0u,  1u}       /* DRV_ENCODER_2: right wheel, blue = A GP0, green = B GP1 */
};

static volatile uint32_t g_seq;
static volatile uint32_t g_count[DRV_ENCODER_COUNT];
static volatile uint32_t g_edge_us[DRV_ENCODER_COUNT];
static volatile uint32_t g_errors[DRV_ENCODER_COUNT];

static void drv_encoder_isr (uint32_t gpio, uint32_t events,
                             uint32_t now_us);
static void drv_encoder_barrier (void);
static void drv_encoder_input (uint32_t pin);

/**
 * @brief Configure the four encoder pins and start counting.
 *
 * Needs drv_gpio_irq_init() first. Inputs get the internal pull-up: it
 * is needed if the Hall outputs are open collector and harmless if they
 * drive both ways (the module type is unverified).
 */
ER
drv_encoder_init (void)
{
    for (UINT e = 0u; e < (UINT)DRV_ENCODER_COUNT; e++)
    {
        drv_encoder_input(g_encoder_hw[e].pin_a);
        drv_encoder_input(g_encoder_hw[e].pin_b);
    }

    for (UINT e = 0u; e < (UINT)DRV_ENCODER_COUNT; e++)
    {
        ER const er = drv_gpio_irq_register(g_encoder_hw[e].pin_a,
                                            DRV_ENCODER_EDGES,
                                            drv_encoder_isr);

        if (er < E_OK)
        {
            return (er);
        }
    }

    return (E_OK);
}

/**
 * @brief Copy both encoders' state consistently, from any task or core.
 * @return E_PAR for NULL; E_BUSY, leaving *p_snap unchanged, if every try
 *         overlapped an update (only under an extreme edge rate).
 */
ER
drv_encoder_snapshot (drv_encoder_snapshot_t * p_snap)
{
    if (NULL == p_snap)
    {
        return (E_PAR);
    }

    for (uint32_t t = 0u; t < DRV_ENCODER_SNAP_TRIES; t++)
    {
        drv_encoder_snapshot_t copy;
        uint32_t const before = g_seq;

        drv_encoder_barrier();

        if (0u != (before & 1u))
        {
            continue;   /* update in progress */
        }

        for (UINT e = 0u; e < (UINT)DRV_ENCODER_COUNT; e++)
        {
            copy.count[e]   = g_count[e];
            copy.edge_us[e] = g_edge_us[e];
            copy.errors[e]  = g_errors[e];
        }

        drv_encoder_barrier();

        if (g_seq == before)
        {
            *p_snap = copy;
            return (E_OK);
        }
    }

    return (E_BUSY);
}

/**
 * @brief x2 quadrature decode of one A interrupt.
 *
 * With A leading B, A rises while B is low and falls while B is high.
 * Both edges in one interrupt means an edge pair was serviced late: the
 * direction cannot be known, so it counts nothing (the caller records an
 * error).
 */
int32_t
drv_encoder_step (uint32_t events, bool b_level_b)
{
    uint32_t const edges = events & DRV_ENCODER_EDGES;
    int32_t        step  = 0;

    if (DRV_GPIO_IRQ_EDGE_RISE == edges)
    {
        step = b_level_b ? -1 : 1;
    }
    else if (DRV_GPIO_IRQ_EDGE_FALL == edges)
    {
        step = b_level_b ? 1 : -1;
    }
    else
    {
        /* No edge, or both edges: not decodable. */
    }

    return (step);
}

/* Dispatcher callback, interrupt context: a few loads and stores only. */
static void
drv_encoder_isr (uint32_t gpio, uint32_t events, uint32_t now_us)
{
    UW const levels = in_w(GPIO_IN);

    for (UINT e = 0u; e < (UINT)DRV_ENCODER_COUNT; e++)
    {
        if (gpio != g_encoder_hw[e].pin_a)
        {
            continue;
        }

        bool const    b_level_b = (0u != (levels & (1u << g_encoder_hw[e].pin_b)));
        int32_t const step      = drv_encoder_step(events, b_level_b);

        g_seq = g_seq + 1u;
        drv_encoder_barrier();

        if (0 != step)
        {
            /* Unsigned wrap is defined; -1 wraps to UINT32_MAX. */
            g_count[e]   = g_count[e] + (uint32_t)step;
            g_edge_us[e] = now_us;
        }
        else if (0u != (events & DRV_ENCODER_EDGES))
        {
            g_errors[e] = g_errors[e] + 1u;
        }
        else
        {
            /* Nothing to record. */
        }

        drv_encoder_barrier();
        g_seq = g_seq + 1u;
    }
}

/* Orders memory accesses as seen by the other core (ARMv6-M DMB). */
static void
drv_encoder_barrier (void)
{
    __asm volatile ("dmb" ::: "memory");
}

static void
drv_encoder_input (uint32_t pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

/* End of file. */
