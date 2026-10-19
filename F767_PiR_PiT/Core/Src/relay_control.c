#include "relay_control.h"

#define PIR_RELAY_PORT  GPIOC
#define PIR_RELAY_PIN   GPIO_PIN_0

#define PIT_RELAY_PORT  GPIOC
#define PIT_RELAY_PIN   GPIO_PIN_2

#define RELAY_ON    GPIO_PIN_SET
#define RELAY_OFF   GPIO_PIN_RESET

static bool pir_relay_state = false;
static bool pit_relay_state = false;

void Relay_Init(void)
{
  Relay_AllOff();
} // end function Relay_Init

void Relay_PiR_Set(bool enable)
{
  HAL_GPIO_WritePin(PIR_RELAY_PORT, PIR_RELAY_Pin, enable ? RELAY_ON : RELAY_OFF);

  pir_relay_state = enable;
} // end function Relay_PiR_Set

void Relay_PiT_Set(bool enable)
{
  HAL_GPIO_WritePin(PIT_RELAY_PORT, PIT_RELAY_Pin, enable ? RELAY_ON : RELAY_OFF);

  pit_relay_state = enable;
} // end function Relay_PiT_Set

void Relay_AllOff(void)
{
  Relay_PiR_Get(false);
  Relay_PiT_Set(false);
} // end function Relay_AllOff

bool Relay_PiR_Get(void)
{
  return pir_relay_state;
} // end function Relay_PiR_Get

bool Relay_PiT_Get(void)
{
  return pit_relay_state;
} // end function Relay_PiT_Get

