#ifndef _ROVERWING_PID_H
#define _ROVERWING_PID_H
#include <Arduino.h>

//generic pid controller
class PIDcontroller {
  public:
    void configure(volatile float p, volatile float i, volatile float d);
    void configure(volatile float p, volatile float i, volatile float d, volatile float lim);
    void configure(volatile float * pidCoef); //pidCoef[0]=Kp,...
    void reset();
    void setTarget(volatile float t);
    float update(float reading);
  private:
    float Kp, Ki, Kd, Ilim;
    float target;
    float prevError, intError;
    uint32_t lastUpdate; //time in us
    bool justReset;
};
#endif
