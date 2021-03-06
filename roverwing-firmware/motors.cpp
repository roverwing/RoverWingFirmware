#include "motors.h"
#include "pid.h"
#include "regmap.h"

//
int32_t  prevEncoder[]={0,0};//to hold previous values of encoders - for computing speed




//motor speed controllers
PIDcontroller SpeedController1, SpeedController2;

void setupTimers () {
  //setup the clock
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                   GCLK_GENCTRL_GENEN |         // Enable GCLK4
                   GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                   GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  //feed GCLK4 to TCC0, TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |          // Enable GCLK
                       GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |          // Enable GCLK
                       GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC2_TC3;  // Feed GCLK4 to TCC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


  //configure TCC0 timer for motors
  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  REG_TCC0_PER = 500;                            // period = 501 ticks, so freq is  16Mhz/501 approx 32 Khz
  while(TCC0->SYNCBUSY.bit.PER);
   // Set prescaler and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                        TCC_CTRLA_ENABLE;         // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  //configure TCC1 timer for servos
  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC1
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  REG_TCC1_PER = 19999;                          // period = 20 000 us, so freq is  50 hz
  while(TCC1->SYNCBUSY.bit.PER);
  //configure TCC2 timer for servos
  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC2_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC2
  while (TCC2->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  REG_TCC2_PER = 19999;                          // period = 20 000 us, so freq is  50 hz
  while(TCC2->SYNCBUSY.bit.PER);



  // Set prescaler and enable the outputs
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    // Divide GCLK4 by 16
                        TCC_CTRLA_ENABLE;          // Enable the TCC1 output
  while (TCC1->SYNCBUSY.bit.ENABLE);               // Wait for synchronization

  // Set prescaler and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    // Divide GCLK4 by 16
                        TCC_CTRLA_ENABLE;          // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);               // Wait for synchronization

}
void setupMotorPins(){
  pinPeripheral(PIN_SERVO1, PIO_TIMER);
  pinPeripheral(PIN_SERVO2, PIO_TIMER);
  pinPeripheral(PIN_SERVO3, PIO_TIMER_ALT);
  pinPeripheral(PIN_SERVO4, PIO_TIMER_ALT);
  pinPeripheral(PIN_MOTOR_1A, PIO_TIMER_ALT);
  pinPeripheral(PIN_MOTOR_1B, PIO_TIMER_ALT);
  pinPeripheral(PIN_MOTOR_2A, PIO_TIMER_ALT);
  pinPeripheral(PIN_MOTOR_2B, PIO_TIMER_ALT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_1A), ISR_enc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_1B), ISR_enc1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_2A), ISR_enc2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_2B), ISR_enc2B, CHANGE);
}

void resetEncoders(){
  byte b=*encoderReset;
  *encoderReset = 0x00;
  if (b&0x01) { //reset encoder for motor1
    encoder[0]=0;
    //also, reset previous value, otherwise speed computation will go haywire
    prevEncoder[0]=0;
  }
  if (b&0x02) { //reset encoder for motor2
    encoder[1]=0;
    prevEncoder[1]=0;
  }
}

void updateMotorsConfig(){
  //first, check if PID coefficients have changed
  if (isSet(FLAG_MOTOR1_PID)){
    //Serial.println("Configuring PID");
    //Serial.println(motor1PID[0]);
    clearFlag(FLAG_MOTOR1_PID);
    SpeedController1.configure(motor1PID);
    SpeedController1.reset();
    SpeedController1.setTarget(motorTarget[0]);
    //Serial.print("Motor 1 max speed: "); Serial.println(motorMaxspeed[0]);
  }
  if (isSet(FLAG_MOTOR2_PID)){
    clearFlag(FLAG_MOTOR2_PID);
    SpeedController2.configure(motor2PID);
    SpeedController2.reset();
    SpeedController2.setTarget(motorTarget[1]);
  }
  //now, set target
  if (isSet(FLAG_MOTOR_TARGET)){
    clearFlag(FLAG_MOTOR_TARGET);
    SpeedController1.setTarget(motorTarget[0]);
    SpeedController2.setTarget(motorTarget[1]);
  }
}
//computes what power to give to motors, using PID if so instructed,
//and actually sets motors to this power
void setMotors(){
  int16_t power1=0, power2=0;
  // determine powers
  switch (motorMode[0]){
    case MOTOR_MODE_POWER:
      power1=motorPower[0];
      //Serial.print("Setting motor 1 in MOTOR_MODE_POWER to power "); Serial.println(power1);
      break;
    case MOTOR_MODE_COAST:
      power1=POWER_COAST; break;//special value to indicate that it should be floating
      //Serial.println("Setting motor 1 to coast ");
    case MOTOR_MODE_SPEEDPID:
      //get the current measured speed and use it to update PID controller
      power1=MOTOR_MAX_POWER*(SpeedController1.update((float)speed[0]) + (float)motorTarget[0]/motorMaxspeed[0]); //500 is maximal power
      //cap power
      if (power1>MOTOR_MAX_POWER) power1=MOTOR_MAX_POWER;
      else if (power1<-MOTOR_MAX_POWER) power1=-MOTOR_MAX_POWER;
      //Serial.print("Setting motor 1 in MOTOR_MODE_SPEEDPID to power "); Serial.println(power1);

      //Serial.print("Power1: "); Serial.println(power1);
      break;
  }
  //now same for motor2
  switch (motorMode[1]){
    case MOTOR_MODE_POWER:
      power2=motorPower[1]; break;
    case MOTOR_MODE_COAST:
      power2=POWER_COAST; break;//special value to indicate that it should be floating
    case MOTOR_MODE_SPEEDPID:
      //get the current measured speed and use it to update PID controller
      power2=MOTOR_MAX_POWER*(SpeedController2.update((float)speed[1])+(float)motorTarget[1]/motorMaxspeed[1]);
      if (power2>MOTOR_MAX_POWER) power2=MOTOR_MAX_POWER;
      else if (power2<-MOTOR_MAX_POWER) power2=-MOTOR_MAX_POWER;
      //Serial.print("Power2: "); Serial.println(power2);
      break;
  }
  //finally, use the found values to set motor powers
  setMotorsPower(power1, power2);
}

//setting motors to given power, -500 ... 500 for each motor
//special value of power POWER_COAST is used to indicate float state
void setMotorsPower(int16_t power1, int16_t power2){
  //motor1; controlled by registers REG_TCC0_CCB2, REG_TCC0_CCB3:
  // pin duty pode is REG_TCC0_CCBx/500
  if (power1==POWER_COAST) {
    //motor shoudl be coasted
    REG_TCC0_CCB2=0;
    REG_TCC0_CCB3=0;
  } else  if (power1 >= 0) {
    REG_TCC0_CCB2=MOTOR_MAX_POWER;
    REG_TCC0_CCB3=MOTOR_MAX_POWER-power1;
    //pin A should be always HIGH, pin B duty cycle should be 1- (power/500)
  } else {  //speed<0; pin B high, pin A duty cycle 1 - (|power|/500)=1+(power/500)
    REG_TCC0_CCB2=MOTOR_MAX_POWER+power1;
    REG_TCC0_CCB3=MOTOR_MAX_POWER;
  }
  //motor2; controlled by registers REG_TCC0_CCB0, REG_TCC0_CCB1:
  // pin duty pode is REG_TCC0_CCBx/500
  if (power2==POWER_COAST) {
    REG_TCC0_CCB0=0;
    REG_TCC0_CCB1=0;
  } else if (power2 >= 0) {
    //pin A should be always HIGH, pin B duty cycle should be 1- (power/500)
    REG_TCC0_CCB0=MOTOR_MAX_POWER;
    REG_TCC0_CCB1=MOTOR_MAX_POWER-power2;
  } else {  //speed<0; pin B high, pin A duty cycle 1 - (|power|/500)=1+(power/500)
    REG_TCC0_CCB0=MOTOR_MAX_POWER+power2;
    REG_TCC0_CCB1=MOTOR_MAX_POWER;
  }
  //Serial.print("Motor powers: "); Serial.print( power1); Serial.print(", "); Serial.println(power2);
}
//setting servo positions
void setServos(){
  REG_TCC2_CCB0=servoPosition[0]; //servo1
  REG_TCC2_CCB1=servoPosition[1]; //servo2
  REG_TCC1_CCB0=servoPosition[2]; //servo3
  REG_TCC1_CCB1=servoPosition[3]; //servo4
}
/* ISR for encoders */
void ISR_enc1A() {
  bool pina, pinb;
  pina = REG_PORT_IN0 & PORT_PA15; //fast way to read digital pin PA15=ENC1A
  pinb = REG_PORT_IN0 & PORT_PA14; //PA14 = ENC1B
  if ( pina==pinb ) {
    encoder[0]--;
  } else {
    encoder[0]++;
  }
}
void ISR_enc1B() {
  bool pina, pinb;
  pina = REG_PORT_IN0 & PORT_PA15; // PA15=ENC1A
  pinb = REG_PORT_IN0 & PORT_PA14; // PA14=ENC1B
  if ( pina==pinb ) {
    encoder[0]++;
  } else {
    encoder[0]--;
  }
}
void ISR_enc2A() {
  bool pina, pinb;
  pina = REG_PORT_IN0 & PORT_PA20; //PA20=ENC2A
  pinb = REG_PORT_IN0 & PORT_PA21; //PA21=ENC2B
  if ( pina==pinb ) {
    encoder[1]--;
  } else {
    encoder[1]++;
  }
}
void ISR_enc2B() {
  bool pina, pinb;
  pina =  REG_PORT_IN0 & PORT_PA20; //PA20=ENC2A
  pinb =  REG_PORT_IN0 & PORT_PA21; //PA21=ENC2B
  if ( pina==pinb ) {
    encoder[1]++;
  } else {
    encoder[1]--;
  }
}
