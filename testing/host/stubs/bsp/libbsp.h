/**
 * @file libbsp.h
 * @brief Host stand-in for <bsp/libbsp.h>. Pure-logic tests must not touch
 *        hardware, so only the register helpers' names are declared and
 *        every access reads 0.
 */
#ifndef HOST_LIBBSP_H
#define HOST_LIBBSP_H

#include <tk/tkernel.h>

UW   in_w (UW addr);
void out_w (UW addr, UW value);
void set_w (UW addr, UW value);
void clr_w (UW addr, UW value);
ER   gpio_set_pin (UINT no, UINT mode);

#define TIMER_TIMERAWL  (0u)
/* GPIO names the GP20 button code uses; only the integration model
   (kernel_model.c) gives GPIO_IN a value. */
#define GPIO_IN         (4u)
#define GPIO(n)         (0x100u + (UW)(n))
#define GPIO_PDE        (0x4u)
#define GPIO_PUE        (0x8u)
#define GPIO_MODE_IN    (0)

#endif /* HOST_LIBBSP_H */

/* End of file. */
