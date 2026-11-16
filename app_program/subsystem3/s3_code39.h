/** @file s3_code39.h Standard Code 39 letter lookup table. */
#ifndef S3_CODE39_H
#define S3_CODE39_H

#include "s3_bar.h"

/* ISO/IEC 16388 Code 39 patterns. Bit 8 is the first element and a set bit
 * denotes a wide element. Every letter contains exactly three wide elements. */
#define S3_CODE39_SENTINEL  0x094U
#define S3_CODE39_PATTERN_A 0x109U
#define S3_CODE39_PATTERN_B 0x049U
#define S3_CODE39_PATTERN_C 0x148U
#define S3_CODE39_PATTERN_D 0x019U
#define S3_CODE39_PATTERN_E 0x118U
#define S3_CODE39_PATTERN_F 0x058U
#define S3_CODE39_PATTERN_G 0x00DU
#define S3_CODE39_PATTERN_H 0x10CU
#define S3_CODE39_PATTERN_I 0x04CU
#define S3_CODE39_PATTERN_J 0x01CU
#define S3_CODE39_PATTERN_K 0x103U
#define S3_CODE39_PATTERN_L 0x043U
#define S3_CODE39_PATTERN_M 0x142U
#define S3_CODE39_PATTERN_N 0x013U
#define S3_CODE39_PATTERN_O 0x112U
#define S3_CODE39_PATTERN_P 0x052U
#define S3_CODE39_PATTERN_Q 0x007U
#define S3_CODE39_PATTERN_R 0x106U
#define S3_CODE39_PATTERN_S 0x046U
#define S3_CODE39_PATTERN_T 0x016U
#define S3_CODE39_PATTERN_U 0x181U
#define S3_CODE39_PATTERN_V 0x0C1U
#define S3_CODE39_PATTERN_W 0x1C0U
#define S3_CODE39_PATTERN_X 0x091U
#define S3_CODE39_PATTERN_Y 0x190U
#define S3_CODE39_PATTERN_Z 0x0D0U

#define S3_CODE39_SYMBOL_COUNT 26U

extern s3_symbol_t const
g_s3_code39_alphabet[S3_CODE39_SYMBOL_COUNT];

#endif /* S3_CODE39_H */
