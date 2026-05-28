#ifndef APP_UTILS_H
#define APP_UTILS_H

#include <stdint.h>

void APP_Int16ToFloat(const int16_t *input, float *output, uint32_t length);
void APP_RemoveDC(float *x, uint32_t length);
void APP_NormalizeFrame(float *x, uint32_t length);

#endif