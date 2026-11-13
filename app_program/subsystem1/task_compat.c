/**
 * @file task_compat.c
 * @brief Hardware-free compatibility target for subsystem 4's starter IF-05.
 */
#include "subsystem1.h"

EXPORT void s1_compat_task (INT start_code, void * p_exinf)
{
    (void) start_code;
    (void) p_exinf;

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
    }
}

/* End of file. */
