/**
 * @file drv_motor.c
 * @brief Subsystem 2 DC motor driver (see drv_motor.h).
 *
 * PWM registers from include/sys/sysdepend/cpu/rp2040/sysdef.h and the
 * RP2040 PWM block (counter counts 0..TOP; output is high while the
 * counter is below CC, so CC = TOP + 1 is always high). Both channels of
 * a slice share one CC register, so A and B change in a single 32-bit
 * write and the hardware applies it at the next wrap: no glitch where the
 * two outputs disagree mid-period.
 */
#include "drv_motor.h"

#define DRV_MOTOR_TOP       ((UW)DRV_MOTOR_DUTY_MAX - 1u)
#define DRV_MOTOR_DIV_ONE   (1u << 4)   /* DIV is 8.4 fixed point: 1.0 */
#define DRV_MOTOR_CC_B_SHIFT 16u        /* B compare is CC bits 31:16 */
#define DRV_MOTOR_SLICE_STRIDE 0x14u    /* bytes between slice registers */

/* One row per motor: pins and the slice they share (slice = gpio / 2). */
typedef struct
{
    UINT pin_a;
    UINT pin_b;
    UW   slice;
} drv_motor_hw_t;

static const drv_motor_hw_t g_motor_hw[DRV_MOTOR_COUNT] =
{
    {8u,  9u,  4u},     /* DRV_MOTOR_1: M1A, M1B */
    {10u, 11u, 5u}      /* DRV_MOTOR_2: M2A, M2B */
};

static UW   drv_motor_reg (UW slice, UW offset);
static void drv_motor_write_cc (drv_motor_id_t motor,
                                drv_motor_cc_t const * p_cc);

/**
 * @brief Configure both motors for 20 kHz PWM, starting braked.
 *
 * Call once from the Motion Control task. The pins are switched to PWM
 * only after the slices run with both outputs low, so the driver never
 * sees a stray drive pulse.
 */
ER
drv_motor_init (void)
{
    drv_motor_cc_t const brake = {0u, 0u};

    /* The kernel releases the PWM block from reset only when it uses PWM
       as its physical timer (USE_PTMR in config.h, 0 in this build), so
       release it here. Only clear the reset, never assert it: another
       subsystem (the servo) may already be using the block. Found on the
       bench (S2-H01): motors did not move at any duty. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* Wait a few clock cycles for the block to come out of reset. */
    }

    for (UINT m = 0u; m < (UINT)DRV_MOTOR_COUNT; m++)
    {
        drv_motor_hw_t const * p_hw = &g_motor_hw[m];

        out_w(drv_motor_reg(p_hw->slice, PWM_CHx_DIV), DRV_MOTOR_DIV_ONE);
        (void)pwm_set_wrap(p_hw->pin_a, DRV_MOTOR_TOP);
        drv_motor_write_cc((drv_motor_id_t)m, &brake);
        (void)pwm_set_enabled(p_hw->pin_a, TRUE);
    }

    /* Let one PWM period (50 us) pass so the buffered CC = 0 is live. */
    ER const er = tk_dly_tsk(1);

    if (er < E_OK)
    {
        return (er);
    }

    for (UINT m = 0u; m < (UINT)DRV_MOTOR_COUNT; m++)
    {
        (void)pwm_set_pin(g_motor_hw[m].pin_a);
        (void)pwm_set_pin(g_motor_hw[m].pin_b);
    }

    return (E_OK);
}

/**
 * @brief Drive one motor at a signed duty (positive = A side, "forward").
 * @return E_PAR, changing nothing, for a bad motor or out-of-range duty.
 */
ER
drv_motor_set (drv_motor_id_t motor, int32_t duty)
{
    drv_motor_cc_t cc;

    if ((motor >= DRV_MOTOR_COUNT) || (!drv_motor_map(duty, &cc)))
    {
        return (E_PAR);
    }

    drv_motor_write_cc(motor, &cc);

    return (E_OK);
}

/** @brief Both outputs low: motor terminals shorted (truth table: brake). */
ER
drv_motor_brake (drv_motor_id_t motor)
{
    return (drv_motor_set(motor, 0));
}

/** @brief Both outputs high: motor terminals open (truth table: coast). */
ER
drv_motor_coast (drv_motor_id_t motor)
{
    drv_motor_cc_t const coast = {(uint16_t)DRV_MOTOR_DUTY_MAX,
                                  (uint16_t)DRV_MOTOR_DUTY_MAX};

    if (motor >= DRV_MOTOR_COUNT)
    {
        return (E_PAR);
    }

    drv_motor_write_cc(motor, &coast);

    return (E_OK);
}

/**
 * @brief The one place that maps duty sign to the A/B outputs.
 *
 * Positive drives A with B low (forward); negative drives B with A low
 * (backward); zero is both low (brake). The undriven side stays low, so
 * the PWM off-time brakes rather than coasts.
 * @return false, leaving *p_cc unchanged, if |duty| > DRV_MOTOR_DUTY_MAX
 *         or p_cc is NULL.
 */
bool
drv_motor_map (int32_t duty, drv_motor_cc_t * p_cc)
{
    if ((NULL == p_cc) || (duty > DRV_MOTOR_DUTY_MAX)
        || (duty < -DRV_MOTOR_DUTY_MAX))
    {
        return (false);
    }

    if (duty >= 0)
    {
        p_cc->cc_a = (uint16_t)duty;
        p_cc->cc_b = 0u;
    }
    else
    {
        p_cc->cc_a = 0u;
        p_cc->cc_b = (uint16_t)(-duty);
    }

    return (true);
}

static UW
drv_motor_reg (UW slice, UW offset)
{
    return (PWM_BASE + (slice * DRV_MOTOR_SLICE_STRIDE) + offset);
}

static void
drv_motor_write_cc (drv_motor_id_t motor, drv_motor_cc_t const * p_cc)
{
    UW const value = ((UW)p_cc->cc_b << DRV_MOTOR_CC_B_SHIFT)
                     | (UW)p_cc->cc_a;

    out_w(drv_motor_reg(g_motor_hw[motor].slice, PWM_CHx_CC), value);
}

/* End of file. */
