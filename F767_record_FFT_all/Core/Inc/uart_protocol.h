#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "main.h"
#include <stdint.h>

void UART_Protocol_Init(void);
void UART_Protocol_Process(void);

void UART_SendText(const char *text);

void UART_SendFFTFrame(
    uint32_t frame_index,
    uint32_t fs,
    uint32_t fft_size,
    float *mag,
    uint32_t bin_count
);

#endif