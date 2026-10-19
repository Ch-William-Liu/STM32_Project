#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include "main.h"
#include <stdbool.h>

void Relay_Init(void);
void Relay_PiR_Set(bool enable);
void Relay_PiT_Set(bool enable);
void Relay_AllOff(void);

bool Relay_PiR_Get(void);
bool Relay_PiT_Get(void);

#endif