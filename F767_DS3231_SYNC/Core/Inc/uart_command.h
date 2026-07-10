#ifndef INC_UART_COMMAND_H_
#define INC_UART_COMMAND_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef UART_Command_Init(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c);

void UART_Command_Process(void);

void UART_Command_RxCpltCallback(UART_HandleTypeDef *huart);

void UART_Command_ErrorCallback(UART_HandleTypeDef *huart);

#endif