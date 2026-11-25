/** @file s3_nav.c Barcode letter to command table (S2 decision 26,
 * S1 decision 6). */
#include "s3_nav.h"

#define S3_NAV_LETTERS 26U

/* Letter A-Z to s3_command_t, one byte each. Written only by
 * s3_nav_set_letter() (WiFi service task), read by s3_nav_command(). */
static volatile uint8_t g_s3_nav_map[S3_NAV_LETTERS] =
{
    [0] = (uint8_t)S3_NAV_LEFT,      /* A */
    [1] = (uint8_t)S3_NAV_RIGHT,     /* B */
    [2] = (uint8_t)S3_NAV_STRAIGHT,  /* C */
    [3] = (uint8_t)S3_NAV_UTURN,     /* D */
    [5] = (uint8_t)S3_NAV_F          /* F: test letter, 0..4 at build time */
};

s3_command_t s3_nav_command(char symbol)
{
    s3_command_t command = S3_NAV_NONE;

    if ((symbol >= 'A') && (symbol <= 'Z'))
    {
        /* symbol is 'A'..'Z' here, so the index is 0..25; map entries are
           s3_command_t values 0..4. */
        command = (s3_command_t)g_s3_nav_map[(uint8_t)(symbol - 'A')];
    }

    return command;
}

bool s3_nav_set_letter(s3_command_t command, char letter)
{
    bool const b_valid = (command >= S3_NAV_LEFT) &&
                       (command <= S3_NAV_UTURN) &&
                       (letter >= 'A') && (letter <= 'Z');

    if (b_valid)
    {
        /* command is S3_NAV_LEFT..S3_NAV_UTURN (checked above) and letter
           'A'..'Z', so both casts below are in range. */
        for (uint8_t index = 0U; index < S3_NAV_LETTERS; index++)
        {
            if ((uint8_t)command == g_s3_nav_map[index])
            {
                g_s3_nav_map[index] = (uint8_t)S3_NAV_NONE;
            }
        }
        g_s3_nav_map[(uint8_t)(letter - 'A')] = (uint8_t)command;
    }

    return b_valid;
}

/** End of file. */

