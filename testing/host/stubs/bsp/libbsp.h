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

#define TIMER_TIMERAWL  (0u)

#endif /* HOST_LIBBSP_H */

/* End of file. */
