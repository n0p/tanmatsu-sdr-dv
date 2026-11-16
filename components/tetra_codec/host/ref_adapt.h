// SPDX-License-Identifier: AGPL-3.0-only
// Force-included in front of the separately compiled reference files of the host test: the same
// three adaptations as in ../src/tetra_codec.c (fixed-width types, round(), no printf / exit).
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define TYPEDEF_H
typedef int16_t Word16;
typedef int32_t Word32;
typedef int     Flag;
#define round       etsi_round
#define printf(...) ((void)0)
#define exit(code)  ((void)0)
