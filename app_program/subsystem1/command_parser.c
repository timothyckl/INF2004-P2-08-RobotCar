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

bool command_parser_parse_assign (const void * p_payload, size_t length,
                                  uint8_t * p_route, char * p_letter)
{
    bool    valid = false;
    uint8_t route = 0u;

    if ((NULL != p_payload) && (NULL != p_route) && (NULL != p_letter) &&
        (COMMAND_PARSER_ASSIGN_LENGTH == length))
    {
        const uint8_t * const p_bytes = (const uint8_t *) p_payload;

        switch (p_bytes[0])
        {
            case (uint8_t) 'L':
                route = COMMAND_PARSER_ROUTE_LEFT;
            break;
            case (uint8_t) 'R':
                route = COMMAND_PARSER_ROUTE_RIGHT;
            break;
            case (uint8_t) 'S':
                route = COMMAND_PARSER_ROUTE_STRAIGHT;
            break;
            case (uint8_t) 'U':
                route = COMMAND_PARSER_ROUTE_UTURN;
            break;
            default:
                route = 0u;
            break;
        }

        valid = (0u != route) && ((uint8_t) '=' == p_bytes[1]) &&
                ((uint8_t) 'A' <= p_bytes[2]) &&
                ((uint8_t) 'Z' >= p_bytes[2]);

        if (valid)
        {
            *p_route  = route;
            *p_letter = (char) p_bytes[2];
        }
    }

    return valid;
}

/*** End of file. ***/


