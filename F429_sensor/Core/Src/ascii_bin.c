#include "ascii_bin.h"
#include <stdint.h>
#include <string.h>

void ascii_to_binary_string_no_space(const char *ascii, char *out, size_t out_size)
{
    if (!ascii || !out || out_size == 0) return;

    size_t w = 0;
    for (size_t i = 0; ascii[i] != '\0'; ++i)
    {
        uint8_t c = (uint8_t)ascii[i];
        for (int b = 7; b >= 0; --b)
        {
            if (w + 1 >= out_size)
            {
                out[w] = '\0';
                return;
            } // end if
            
            out[w++] = (c & (1u << b)) ? '1' : '0';
        } // end inner for
    } // end outer for
    out[w] = '\0';
} // end function ascii_to_binary_string_no_space