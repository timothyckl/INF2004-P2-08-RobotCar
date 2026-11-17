/** @file s3_code39.c Standard Code 39 A-to-Z lookup table. */
#include "s3_code39.h"

s3_symbol_t const g_s3_code39_alphabet[S3_CODE39_SYMBOL_COUNT] =
{
    {S3_CODE39_PATTERN_A, 'A'}, {S3_CODE39_PATTERN_B, 'B'},
    {S3_CODE39_PATTERN_C, 'C'}, {S3_CODE39_PATTERN_D, 'D'},
    {S3_CODE39_PATTERN_E, 'E'}, {S3_CODE39_PATTERN_F, 'F'},
    {S3_CODE39_PATTERN_G, 'G'}, {S3_CODE39_PATTERN_H, 'H'},
    {S3_CODE39_PATTERN_I, 'I'}, {S3_CODE39_PATTERN_J, 'J'},
    {S3_CODE39_PATTERN_K, 'K'}, {S3_CODE39_PATTERN_L, 'L'},
    {S3_CODE39_PATTERN_M, 'M'}, {S3_CODE39_PATTERN_N, 'N'},
    {S3_CODE39_PATTERN_O, 'O'}, {S3_CODE39_PATTERN_P, 'P'},
    {S3_CODE39_PATTERN_Q, 'Q'}, {S3_CODE39_PATTERN_R, 'R'},
    {S3_CODE39_PATTERN_S, 'S'}, {S3_CODE39_PATTERN_T, 'T'},
    {S3_CODE39_PATTERN_U, 'U'}, {S3_CODE39_PATTERN_V, 'V'},
    {S3_CODE39_PATTERN_W, 'W'}, {S3_CODE39_PATTERN_X, 'X'},
    {S3_CODE39_PATTERN_Y, 'Y'}, {S3_CODE39_PATTERN_Z, 'Z'}
};

/** End of file. */
