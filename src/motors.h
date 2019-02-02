#ifndef _ROVERWING_MOTORS_H
#define _ROVERWING_MOTORS_H
#include <Arduino.h>
#include "wiring_private.h"

// Motor configuration modes
#define MOTOR_MODE_BRAKE 0
#define MOTOR_MODE_FLOAT 1
#define MOTOR_MODE_SPEEDPID 2
#define MOTOR_MODE_POWER 3
// For floating a motor:
// Normally, motor power is an int between -500...500
// this special value indicates that the motor should be stopped in floating state
#define POWER_FLOAT 1000


/*
 * Configures clock sources and others for timers TCC0 (used by motors),
 * TCC1 and TCC2 (used by servos)
 */
void setupTimers();
/*
 * Correctly sets pin mode and peripherals for  motors, encoders, and servos pins
 */
void setupMotorPins();
//updates motors PID coefficients and target if they have been changed
void updateMotorsConfig();
//resets motor encoders as needed
void resetEncoders();
//computes what power to give to motors, using PID if so instructed,
//and actually sets motors to this power
void setMotors();
/* Sets power for motors. Range is -500..500 */
void setMotorsPower(int16_t power1, int16_t power2);
/* Sets servo positions. Position should be a pointer to an array
 * of four numbers, representing pulsewidths  for 4 servos
 */

void setServos(volatile uint16_t * positions);
/* Interrupt service routines for encoders */
void ISR_enc1A();
void ISR_enc1B();
void ISR_enc2A();
void ISR_enc2B();
#endif //for ifndef _ROVERWING_MOTORS_H
