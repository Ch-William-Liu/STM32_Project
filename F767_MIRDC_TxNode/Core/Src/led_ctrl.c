#include "main.h"
#include "led_ctrl.h"

void LED_AllOff(void)
{
    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_7 , GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_14 , GPIO_PIN_RESET);
} // end function LED_AllOff

void LED_GreenOnly(void)
{
    LED_AllOff();
    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_SET);
} // end function LED_GreenOnly

void LED_BlueOnly(void)
{
    LED_AllOff();
    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_7 , GPIO_PIN_SET);
} // end function LED_BlueOnly

void LED_RedOnly(void)
{
    LED_AllOff();
    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_14 , GPIO_PIN_SET);
} // end function LED_RedOnly