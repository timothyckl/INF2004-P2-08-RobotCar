/** @file s3_nav.h Barcode letter to command table; never drives motors.
 * The command is held for the next junction by the Vehicle Controller
 * (nav_control.c, S2 decisions 26 and 27), not here. */
#ifndef S3_NAV_H
#define S3_NAV_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    S3_NAV_NONE = 0,
    S3_NAV_LEFT,
    S3_NAV_RIGHT,
    S3_NAV_STRAIGHT,
    S3_NAV_UTURN
} s3_command_t;

/* The course letters are announced on the day. Until then, F is the test
 * letter: build with S3_NAV_F = 0 (none), 1 (left), 2 (right),
 * 3 (straight) or 4 (U-turn). */
#ifndef S3_NAV_F
#define S3_NAV_F 0
#endif

#if (S3_NAV_F < 0) || (S3_NAV_F > 4)
#error "S3_NAV_F must be 0..4"
#endif

/** Letter to command. At boot: the brief's example A-D plus the F test
 * letter; s3_nav_set_letter() changes it at run time (S1 decision 6).
 * Any unmapped letter is NONE, which the car reports and ignores.
 */
s3_command_t s3_nav_command(char symbol);
/** Make letter the only letter for command (LEFT..UTURN): every letter
 * mapped to command is first unmapped, then letter is mapped to it,
 * replacing whatever letter had before. Single writer: the WiFi service
 * task. Each entry is one byte, so a concurrent s3_nav_command() sees an
 * old or new entry, never a torn one; mid-update a letter may read NONE,
 * which is reported and ignored. The table resets to the boot map only on
 * a restart.
 * @return false, changing nothing, for a bad command or a letter not A-Z.
 */
bool s3_nav_set_letter(s3_command_t command, char letter);

#endif /* S3_NAV_H */

/* End of file. */
