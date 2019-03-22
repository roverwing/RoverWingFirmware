#ifndef _ROVERWING_MAG_H
#define _ROVERWING_MAG_H
#include "Arduino.h"
#include "Wire.h"
//statuses
#define MAG_OFF 0x00
#define MAG_OK  0x01
#define MAG_CALIBRATING 0x02
//configuration mode used in communication with feather
#define MAG_CONFIG_BEGIN 0x01
#define MAG_CONFIG_CALIBRATE 0x02
#define MAG_CONFIG_END 0x00


#define MAG_ADDRESS  (0x3C >> 1)         // 0011110x
#define MAG_MEASUREMENT_CONFIG (B01110100)   // to be placed to config register A
                                             //8 samples per output, 30 Hz refresh rate, normal measurement config
#define MAG_GAIN      0x20  // corresponds to range of  +/- 1.3 Gauss
#define MAG_MG_PER_LSB (0.92f) // milligauss per LSB
#define MAG_CONTINUOUS_MODE 0x00 //continuous measurement
#define MAG_IDLE_MODE 0x02 //idle
//Registers
#define MAG_REG_CRA  0x00   //config register A
#define MAG_REG_CRB  0x01   //config register B
#define MAG_REG_MR   0x02   //mode register
#define MAG_REG_OUT_X_H 0x03
#define MAG_REG_OUT_X_L 0x04
#define MAG_REG_OUT_Z_H 0x05
#define MAG_REG_OUT_Z_L 0x06
#define MAG_REG_OUT_Y_H 0x07
#define MAG_REG_OUT_Y_L 0x08
#define MAG_REG_IRA 0x0A

//constants
#define DEG_PER_RADIAN (180.0f/PI)

extern int16_t magSoftIronMatrix[][3];

boolean magBegin();
void magEnd();
void magUpdate();
void magCalibrate();
void magSetCalData();
void magResetCalData();


#endif
