/** @file command_parser.h
 * @brief Length-delimited parsing of provisional remote command tokens.
 */
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stddef.h>
#include <stdint.h>

typedef uint8_t command_parser_command_t;

#define COMMAND_PARSER_INVALID 0u
#define COMMAND_PARSER_START   1u
#define COMMAND_PARSER_STOP    2u
#define COMMAND_PARSER_RESET   3u
#define COMMAND_PARSER_MAX     5u

/**
 * @brief Parse exactly START, STOP or RESET; reject all other byte sequences.
 * @param p_payload Readable payload of length bytes, without a terminator.
 * @param length Number of payload bytes; bounded before any payload access.
 * @return A command token, or COMMAND_PARSER_INVALID, including for NULL.
 * @note This validates syntax only, without forwarding or authorising motion.
 */
command_parser_command_t command_parser_parse (const void * p_payload,
                                               size_t length);

#endif /* COMMAND_PARSER_H */

/*** End of file. ***/

