/**
 * @file app_stack.h
 * @brief Stack high-water marks of the application tasks (review F-24).
 *
 * app_start_task() gives each task a stack from a static pool, painted with
 * APP_STACK_FILL before the task starts. A stack grows down from its top,
 * so the painted words still intact at the bottom were never used. The
 * scan is pure and host-tested (S2-U83).
 */
#ifndef APP_STACK_H
#define APP_STACK_H

#include <stdint.h>

#define APP_STACK_FILL  UINT32_C(0xA5A5A5A5)

/** Fill count words with APP_STACK_FILL. */
void     app_stack_paint (uint32_t * p_words, uint32_t count);

/** Bytes at the bottom (lowest address) of a painted stack never
 *  written: the run of APP_STACK_FILL words from p_words[0]. */
uint32_t app_stack_unused_bytes (uint32_t const * p_words, uint32_t count);

#endif /* APP_STACK_H */

/* End of file. */
