/** @file s3_nav.h Controller request staging; never drives motors. */
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

typedef struct
{
    s3_command_t pending;
    uint32_t     scan_id;
    uint32_t     staged_ms;
    bool         have_scan;
} s3_navigation_t;

/** Example A-D mapping from the brief, pending team confirmation. */
s3_command_t s3_nav_command(char symbol);
/** Only call with a validated complete frame; one unique ID per scan.
 * Does not overwrite a pending command. Duplicate IDs are rejected.
 */
bool s3_nav_stage(s3_navigation_t * p_nav, char symbol,
                   uint32_t scan_id, uint32_t now_ms);
/** Call every control iteration, including while stopped.
 * STOP or age expiry discards pending work. The scan ID stays remembered.
 * Returns a request only at an authorised junction, never a motor action.
 * On delivery failure the controller must stop/reject, not execute stale work.
 */
s3_command_t s3_nav_take(s3_navigation_t * p_nav, bool stopped,
                         bool authorised, bool junction, uint32_t now_ms,
                         uint32_t max_age_ms);

#endif /* S3_NAV_H */

