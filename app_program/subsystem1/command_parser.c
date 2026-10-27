/** @file command_parser.c
 * @brief Strict command parsing with no network or controller dependencies.
 */
#include "command_parser.h"

#include <string.h>

command_parser_command_t command_parser_parse (const void * p_payload,
                                               size_t length)
{
    command_parser_command_t command = COMMAND_PARSER_INVALID;

    if ((NULL != p_payload) && (length <= COMMAND_PARSER_MAX))
    {
        if ((4u == length) && (0 == memcmp(p_payload, "STOP", length)))
        {
            command = COMMAND_PARSER_STOP;
        }
        else if ((5u == length) && (0 == memcmp(p_payload, "START", length)))
        {
            command = COMMAND_PARSER_START;
        }
        else if ((5u == length) && (0 == memcmp(p_payload, "RESET", length)))
        {
            command = COMMAND_PARSER_RESET;
        }
        else
        {
            command = COMMAND_PARSER_INVALID;
        }
    }

    return command;
}

/*** End of file. ***/

