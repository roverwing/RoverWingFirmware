#ifndef _ROVERWING_SONARS_H
#define _ROVERWING_SONARS_H
#include <Arduino.h>
#include "wiring_private.h"
#define MICROS_TO_MM (0.171f) //half of speed of sound, which is 343 m/s = 0.343 mm/us
#define NO_ECHO 6000; //distance in mm to set if no echo was received. 6000 mm = 6m
#define NUM_SONARS 3 //number of sonars
//#define SONAR_DELAY 440// in us;to compensate for sonar not starting immediately after we send the pulse

void setupSonarPins();
void updateSonars();
void ISR_sonar1();
void ISR_sonar2();
void ISR_sonar3();

#endif //for ifndef _ROVERWING_SONARS_H
