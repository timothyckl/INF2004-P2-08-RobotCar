/** @file command_parser.h
 * @brief Length-delimited parsing of provisional remote command tokens.
 */
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint8_t command_parser_command_t;

#define COMMAND_PARSER_INVALID 0u
#define COMMAND_PARSER_START   1u
#define COMMAND_PARSER_STOP    2u
#define COMMAND_PARSER_RESET   3u
#define COMMAND_PARSER_MAX     5u

/* Junction routes of a barcode letter assignment, "L=A" style (S1
   decision 6). Values match the S2_NAV_* and S3_NAV_* codes. */
#define COMMAND_PARSER_ROUTE_LEFT      1u
#define COMMAND_PARSER_ROUTE_RIGHT     2u
#define COMMAND_PARSER_ROUTE_STRAIGHT  3u
#define COMMAND_PARSER_ROUTE_UTURN     4u
#define COMMAND_PARSER_ASSIGN_LENGTH   3u

/**
 * @brief Parse exactly START, STOP or RESET; reject all other byte sequences.
 * @param p_payload Readable payload of length bytes, without a terminator.
 * @param length Number of payload bytes; bounded before any payload access.
 * @return A command token, or COMMAND_PARSER_INVALID, including for NULL.
 * @note This validates syntax only, without forwarding or authorising motion.
 */
command_parser_command_t command_parser_parse (const void * p_payload,
                                               size_t length);

/**
 * @brief Parse a barcode letter assignment: exactly three bytes, a route
 *        L, R, S or U, then '=', then an upper-case letter A to Z.
 * @param p_payload Readable payload of length bytes, without a terminator.
 * @param length Number of payload bytes; bounded before any payload access.
 * @param p_route Receives a COMMAND_PARSER_ROUTE_* code on success.
 * @param p_letter Receives the letter on success.
 * @return True only for a valid assignment; outputs are unchanged otherwise.
 * @note Syntax only; the letter table belongs to subsystem 3.
 */
bool command_parser_parse_assign (const void * p_payload, size_t length,
                                  uint8_t * p_route, char * p_letter);

#endif /* COMMAND_PARSER_H */

/*** End of file. ***/


