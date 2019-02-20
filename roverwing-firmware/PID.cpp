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
  //Serial.print("PID configured: ");
  //Serial.print(Kp,5);Serial.print(Ki,5);Serial.print(Kd,5);Serial.println(Ilim,5);
}
void PIDcontroller::reset(){
  target=0;
  prevError=0;
  intError=0;
  justReset=true;
}
void PIDcontroller::setTarget(volatile float t){
  target=t;
  //Serial.print("PID target: ");Serial.println(t);
}
float PIDcontroller::update(float reading){
  float error,dError;
  //double controlOutput;
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
  // now, compute the feedback
  float output=Kp*error+Ki*intError+Kd*dError;
  //Serial.print("Kp: "); Serial.println(Kp,4);
  //Serial.print("Target: "); Serial.println(target,4);
  //Serial.print("Error: "); Serial.println(error,4);
  //Serial.print("Deltat: "); Serial.println(deltat,8);
  //Serial.print("Int Error: "); Serial.println(intError,4);
  //Serial.print("D Error: "); Serial.println(dError,4);
  //Serial.print("Kp*Error: "); Serial.println(Kp*error,4);
  //Serial.print("PID output: "); Serial.println(output,4);
  //Serial.print("PID returned: "); Serial.println(controlOutput,4);
  return output;
}
