/**
 * @file subsystem4_test.c
 * @brief On-target standalone tests for Subsystem 4 terrain logic.
 */
#include "subsystem4.h"
#include "terrain_logic.h"

#if APP_TEST
#include <tm/tmonitor.h>

#define S4_INITIAL_LOGIC_CASES 15u

void
subsystem4_test (void)
{
#if S4_TERRAIN_LOGIC_AVAILABLE
    /* Implemented with the production terrain logic in the next milestone. */
#else
    static char const * const p_case_ids[S4_INITIAL_LOGIC_CASES] =
    {
        "S4-U03", "S4-U04", "S4-U05", "S4-U06", "S4-U07",
        "S4-U08", "S4-U09", "S4-U10", "S4-U11", "S4-U12",
        "S4-U13", "S4-U14", "S4-U15", "S4-U16", "S4-U17"
    };

    for (uint32_t index = 0u; index < S4_INITIAL_LOGIC_CASES; index++)
    {
        tm_printf((UB const *) "[TEST] %s BLOCKED  "
                  "terrain logic not implemented\n", p_case_ids[index]);
    }

    tm_printf((UB const *) "[TEST] subsystem4 SUMMARY 0 passed, "
              "0 failed, %u blocked\n", (UINT) S4_INITIAL_LOGIC_CASES);
#endif
}
#endif

/* End of file. */
