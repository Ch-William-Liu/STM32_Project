#include "stm32f4xx_hal.h"
#include <sys/unistd.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>

extern UART_HandleTypeDef huart3;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3 , (uint8_t *)ptr, len , HAL_MAX_DELAY);
    return len;
}