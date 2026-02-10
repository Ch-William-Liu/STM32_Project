#include "ascii_bin.h"
#include <stdint.h>
#include <string.h>

static void byte_to_bin8(uint8_t b , char *dst8)
{
    for (int i = 7; i>=0; --i)
    {
        *dst8++ = (b & (1u << i)) ? '1' : '0';
    } // end for 
} // end function byte_to_bin8

void ascii_to_binary_string(const char *ascii , char *out , size_t out_size)
{
    if (!ascii || !out || out_size == 0) return;
    out[0] = '\0';

    size_t used = 0;

    for (size_t i = 0; ascii[i] != '\0'; ++i)
    {
        char bin[8];
        byte_to_bin8((uint8_t)ascii[i], bin);

        // Need: 8 bits + optional space + null
        size_t need = 8 + ((ascii[i + 1] != '\0') ? 1 : 0) + 1;
        if (used + need > out_size) break;

        memcpy(out + used , bin , 8);
        used += 8;

        if (ascii[i + 1] != '\0') out[used++] = ' ';
        out[used] = '\0';
    } // end for 
} // end function ascii_to_binary_string