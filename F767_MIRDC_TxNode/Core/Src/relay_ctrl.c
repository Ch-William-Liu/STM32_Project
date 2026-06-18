#include "main.h"
#include "relay_ctrl.h"

void Relay_On(void)
{
    HAL_GPIO_WritePin(GPIOC , GPIO_PIN_0 , GPIO_PIN_SET);
} // end function Relay_On

void Relay_Off(void)
{
    HAL_GPIO_WritePin(GPIOC , GPIO_PIN_0 , GPIO_PIN_RESET);
} // end function Relay_Off