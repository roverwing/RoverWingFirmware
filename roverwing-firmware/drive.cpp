#include "drive.h"
#include "regmap.h"
#include "motors.h"
#include "pid.h"
#include "MPU6050.h"

//PID controller used for driving in straight line
PIDcontroller drivePID;
int8_t motorDir[2]; // +1 means that positivie power to that motor causes rover to mvoe forward;
                    //   -1, backwards
int8_t motorTurnDir[2]; //+1 means that positive power to that motor causes rover to rotate clockwise

int16_t driveCurPower; // current power to motors
int16_t driveTargetYaw; //target yaw angle, in units of 0.1 deg
int32_t driveTargetPosition;
int8_t turnDir; //+1 for cw, -1 for ccw
int8_t driveDir; //+1 for forward, -1 for backwards
//in addition, the following variаblеs are defined in regmap.cpp
// uint8_t * driveStatus
// uint8_t  * driveMode;
// uint8_t  * driveMotorConfig;
//             driveMotorConfig contians the following info:
//             bit 0: 0 if motor1 direction is positive, 1 otherwise
//             bit 1: 0 if motor 1 turn direction is positive
//                   (i.e., postitive power to thsi motor causes robot turn clockwise)
//             bit 2: motor2 direction
//             bit 3: motor2 turn direction
// uint16_t * driveMaxSpeed;
// uint16_t * driveMaxTurnSpeed;
// uint16_t * driveMinPower;
// float    * drivePIDcoef;
// int32_t  * driveDistance;
// int16_t  * driveTurnAngle;
// int16_t  * driveTargetPower;

void driveSetup(){
  if (*driveMode==DRIVE_OFF){
    *driveStatus=DRIVE_STATUS_COMPLETE;
    setMotorsPower(0,0);
    return;
  }
  //in all other cases, we have real work to do
  //first, override the motors mode
  motorMode[0]=MOTOR_MODE_TANKDRIVE;
  motorMode[1]=MOTOR_MODE_TANKDRIVE;
  *driveStatus=DRIVE_STATUS_INPROGRESS;
  int16_t curYaw=getYaw()*10.0f; // current rover direction, in units of 0.1 degree
  //now, determine all motor directions
  if (*driveMotorConfig & B00000001) motorDir[0]=-1;
  else motorDir[0]=1;
  if (*driveMotorConfig & B00000100) motorDir[1]=-1;
  else motorDir[1]=1;
  if (*driveMotorConfig & B00000010) motorTurnDir[0]=-1;
  else motorTurnDir[0]=1;
  if (*driveMotorConfig & B00001000) motorTurnDir[1]=-1;
  else motorTurnDir[1]=1;
  //now, do everything else
  switch (*driveMode){
    case DRIVE_STRAIGHT:
      driveTargetYaw=curYaw; //direction to hold
      if (*driveTargetPower>0) driveDir=+1;
      else driveDir=-1;
      //initialize pid
      drivePID.configure(drivePIDcoef);
      drivePID.setTarget(0.0f);
      break;
    case DRIVE_STRAIGHT_DISTANCE:
      driveTargetYaw=curYaw; //direction to hold
      if (*driveTargetPower>0) driveDir=+1;
      else driveDir=-1;
      //target position, as halfsum of two positions (with signs as necessary)
      driveTargetPosition=driveCurPosition()+ driveDir * (*driveDistance);
      //initialize pid
      drivePID.configure(drivePIDcoef);
      drivePID.setTarget(0.0f);
      break;
    case DRIVE_TURN:
      driveTargetYaw=normalizeAngle(curYaw + *driveTurnAngle);
      if (*driveTurnAngle>0) turnDir=+1;
      else turnDir=-1;
      break;
  }
}
void driveUpdateMotors(){
  int16_t power, power1,power2, correction;
  int16_t dYaw; //difference in angle
  switch (*driveMode){
    case DRIVE_STRAIGHT:
      //get default powers
      power1=(*driveTargetPower)*motorDir[0];
      power2=(*driveTargetPower)*motorDir[1];
      //now, compute and apply the course correction
      dYaw=normalizeAngle(driveTargetYaw-*yaw);
      correction=MOTOR_MAX_POWER*drivePID.update((float)dYaw);//positive correction means we need to turn cw
      power1+=correction*motorTurnDir[0];
      power2+=correction*motorTurnDir[1];
      break;
    case DRIVE_STRAIGHT_DISTANCE:
      //FIXME
      break;
    case DRIVE_TURN:
      dYaw=turnDir*normalizeAngle(driveTargetYaw-*yaw); //should be positive
      if (dYaw<0) {
        //we overshoot!
        *driveStatus=DRIVE_STATUS_COMPLETE;
        *driveMode=DRIVE_OFF;
        power=0;
      } else if (dYaw<100) {
        //ten degrees left - time to slow down
        power=*driveMinPower * 2;
      } else {
        //not yet there, more than 10 degrees left to turn
        power=*driveTargetPower;
      }
      //now, setup power1, power2
      power1=power*turnDir*motorTurnDir[0];
      power2=power*turnDir*motorTurnDir[1];
  }
  setMotorsPower(power1, power2);

}
