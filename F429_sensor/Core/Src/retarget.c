#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

extern UART_HandleTypeDef huart3;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3 , (uint8_t *)ptr , (uint16_t)len , HAL_MAX_DELAY);
    return len;
} // end _write

