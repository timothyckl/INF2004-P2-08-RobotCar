/**
 * @file app_stack.c
 * @brief Stack painting and high-water scan (app_stack.h). Pure.
 */
#include "app_stack.h"

#include <stddef.h>

void
app_stack_paint (uint32_t * p_words, uint32_t count)
{
    if (NULL != p_words)
    {
        for (uint32_t idx = 0u; idx < count; idx++)
        {
            p_words[idx] = APP_STACK_FILL;
        }
    }
}

uint32_t
app_stack_unused_bytes (uint32_t const * p_words, uint32_t count)
{
    uint32_t unused = 0u;

    if (NULL != p_words)
    {
        while ((unused < count) && (APP_STACK_FILL == p_words[unused]))
        {
            unused++;
        }
    }

    /* unused <= count, and a stack is far below 1 GB, so x 4 cannot wrap. */
    return (unused * 4u);
}

/* End of file. */
