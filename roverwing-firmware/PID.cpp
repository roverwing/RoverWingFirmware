#include "PID.h"

void PIDcontroller::configure(volatile  float p, volatile  float i, volatile  float d){
  Kp=p;
  Ki=i;
  Kd=d;
  Ilim=0;
}
void PIDcontroller::configure(volatile  float p, volatile  float i, volatile  float d, volatile float lim){
  Kp=p;
  Ki=i;
  Kd=d;
  Ilim=lim;
}
void PIDcontroller::configure(volatile float * pidCoef){
  Kp=pidCoef[0];
  Ki=pidCoef[1];
  Kd=pidCoef[2];
  Ilim=pidCoef[3];
}
void PIDcontroller::reset(){
  target=0;
  prevError=0;
  intError=0;
  justReset=true;
}
void PIDcontroller::setTarget(volatile float t){
  target=t;
}
float PIDcontroller::update(float reading){
  float error,dError,correction;
  float deltat;
  uint32_t now=micros();
  error=target-reading;
  if (!justReset) {
    //if this is not the first reading, compute integral and derivative error
    deltat=(now-lastUpdate)*0.000001f; //time in seconds
    intError+=error*deltat;
    if (Ilim>0){ //limit the integral error, to prevent integral windup 
      if (intError>Ilim) intError=Ilim;
      else if (intError<-Ilim) intError=-Ilim;
    }
    dError=(error-prevError)/deltat;
  } else {
    //first reading after reset
    justReset=false;
  }
  lastUpdate=now;
  prevError=error;
  // now, compute the feedback\
  correction=Kp*error+Ki*intError+Kd*dError;
  return correction;
}
