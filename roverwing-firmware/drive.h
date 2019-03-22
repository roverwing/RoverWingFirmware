#ifndef _ROVERWING_DRIVE_H
#define _ROVERWING_DRIVE_H
#include <Arduino.h>
#include "motors.h"
#include "regmap.h"
//drive modes
#define DRIVE_OFF 0x00
#define DRIVE_STRAIGHT 0x01
#define DRIVE_STRAIGHT_DISTANCE 0x02
#define DRIVE_TURN 0x03
//drive statuses
#define DRIVE_STATUS_COMPLETE 0x00
#define DRIVE_STATUS_INPROGRESS 0x01
extern int8_t  motorDir[2];

//functions
void driveSetup();
void driveUpdateMotors();
//now, two inline functions
int32_t inline driveCurPosition(){
  return (encoder[0]*motorDir[0]+encoder[1]*motorDir[1])/2 ;
}
//for making angles be between -180..180. Variable a is nagle in units of 0.1 degree
int16_t inline normalizeAngle(int16_t a){
  if (a>1800) return (a-3600);
  else if (a<-1800) return (a+3600);
  else return a;
}


#endif
