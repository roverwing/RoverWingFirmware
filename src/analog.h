#ifndef _ROVERWING_ANALOG_H
#define _ROVERWING_ANALOG_H
#include <Arduino.h>
#include "wiring_private.h"
#define NUM_ANALOGS 7
void setupAnalogPins();
void updateAnalogs();
void printAnalogs();
uint16_t myAnalogRead(uint8_t pin);

#endif
