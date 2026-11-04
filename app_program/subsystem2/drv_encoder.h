/**
 * @file drv_encoder.h
 * @brief Subsystem 2 quadrature wheel-encoder driver.
 *
 * Hardware: TT motors with two-channel Hall encoders (confirmed on the
 * robot; part number and datasheet unknown, so counts per revolution are
 * provisionally measured by test S2-H03). The agreed mapping is encoder 1 =
 * right motor 1 = GP26 (A) / GP27 (B), and encoder 2 = left motor 2 = GP7
 * (A) / GP28 (B). Keep the GP28 battery-sense jumper open and keep the ADC
 * driver disabled on GP26 through GP28.
 *
 * Decoding (x2): an interrupt on both edges of A only; A and B levels at
 * that moment give the direction. If RP2040 has latched both A edge bits
 * before service, the current A level identifies the latest edge and the
 * pair is counted as two steps. This doubles the resolution of rising-edge
 * counting while taking half the interrupts of full x4 decoding (both edges
 * of A and B), which keeps the combined GPIO interrupt load down
 * (AAI08_PostTri1). Counts per wheel revolution = 2 x A pulses per rev.
 *
 * Counts are free-running uint32_t, +1 when A leads B, wrapping, as
 * odom_count_delta() expects. Which sign is "forward" depends on the
 * wiring; Motion Control maps it.
 *
 * Concurrency: only the dispatcher interrupt (one core) writes the counts.
 * drv_encoder_snapshot() is a lock-free sequence read: it never stops
 * the interrupt, and retries if an update happened during the copy, so it
 * is safe from either core and the ISR never waits.
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

typedef enum
{
    DRV_ENCODER_1 = 0,  /* right wheel (motor 1), GP26/GP27 */
    DRV_ENCODER_2,      /* left wheel (motor 2), GP7/GP28 */
    DRV_ENCODER_COUNT
} drv_encoder_id_t;

/** One consistent copy of both encoders. */
typedef struct
{
    uint32_t count[DRV_ENCODER_COUNT];      /* +1 per A edge, A leading */
    uint32_t edge_us[DRV_ENCODER_COUNT];    /* time of last counted edge */
    uint32_t errors[DRV_ENCODER_COUNT];     /* edges that could not be
                                               decoded (both A edges seen) */
} drv_encoder_snapshot_t;

ER drv_encoder_init (void);
ER drv_encoder_snapshot (drv_encoder_snapshot_t * p_snap);

/* Pure x2 decode used by the interrupt; exposed for unit tests. A single
   edge returns +1/-1. A coalesced rise+fall pair returns +2/-2 based on the
   current A/B levels. No edge returns 0. */
int32_t drv_encoder_step (uint32_t events, bool b_level_a, bool b_level_b);

#endif /* DRV_ENCODER_H */

/* End of file. */
