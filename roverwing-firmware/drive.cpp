#include "drive.h"
#include "regmap.h"
#include "motors.h"
#include "pid.h"
#include "ICM42605.h"

//PID controller used for driving in straight line
PIDcontroller drivePID;
int8_t motorDir[2]; // +1 means that positivie power to that motor causes rover to mvoe forward;
                    //   -1, backwards
int8_t motorTurnDir[2]; //+1 means that positive power to that motor causes rover to rotate clockwise

int16_t driveCurPower; // current power to motors
int32_t driveTargetPosition;
int8_t turnDir; //+1 for cw, -1 for ccw
int8_t driveDir; //+1 for forward, -1 for backwards
uint16_t driveAccel; //maximal acceleration in power units (0-500) per sec
uint32_t driveStartTime; //timestamp in ms
float distanceUnit; //how many encoder ticks rover travels in 1 sec at power=1 (out of 500);

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
// int16_t  * driveHeading; //target yaw angle, in units of 0.1 deg
// int16_t  * driveTargetPower;
// uint16_t * driveRampTime;

void driveSetup(){
  if (*driveMode==DRIVE_OFF){
    *driveStatus=DRIVE_STATUS_COMPLETE;
    //Serial.println("Drive: turnign motors off 3");
    setMotorsPower(0,0);
    return;
  }
  //in all other cases, we have real work to do
  //first, override the motors mode
  motorMode[0]=MOTOR_MODE_TANKDRIVE;
  motorMode[1]=MOTOR_MODE_TANKDRIVE;
  *driveStatus=DRIVE_STATUS_INPROGRESS;
  driveStartTime=millis();
  //int16_t curYaw=getYaw()*10.0f; // current rover direction, in units of 0.1 degree
  //now, determine all motor directions
  if (*driveMotorConfig & B00000001) motorDir[0]=-1;
  else motorDir[0]=1;
  if (*driveMotorConfig & B00000100) motorDir[1]=-1;
  else motorDir[1]=1;
  if (*driveMotorConfig & B00000010) motorTurnDir[0]=-1;
  else motorTurnDir[0]=1;
  if (*driveMotorConfig & B00001000) motorTurnDir[1]=-1;
  else motorTurnDir[1]=1;
  //figure out maximal acceleration
  if (*driveRampTime>0){
    driveAccel=((1000ul)*MOTOR_MAX_POWER)/ (*driveRampTime); //remember, we want accel in power.sec, and ramp time is in ms
  } else {
    driveAccel=10000; // corresponds to ramp time of 1/20 sec
  }
  //compute distance unit
  distanceUnit=(float)(*driveMaxSpeed)/MOTOR_MAX_POWER;
  //now, do everything else
  switch (*driveMode){
    case DRIVE_STRAIGHT:
      //driveHeading=curYaw; //direction to hold
      if (*driveTargetPower>0) driveDir=+1;
      else driveDir=-1;
      //initialize pid
      drivePID.configure(drivePIDcoef);
      drivePID.reset();
      drivePID.setTarget(0.0f);
      break;
    case DRIVE_STRAIGHT_DISTANCE:
      //driveHeading=curYaw; //direction to hold
      if (*driveTargetPower>0) driveDir=+1;
      else driveDir=-1;
      //target position, as halfsum of two positions (with signs as necessary)
      driveTargetPosition=driveCurPosition()+ driveDir * (*driveDistance);
      //initialize pid
      drivePID.configure(drivePIDcoef);
      drivePID.reset();
      drivePID.setTarget(0.0f);
      break;
    case DRIVE_TURN:
      int16_t turnAngle=normalizeAngle(*driveHeading-*yaw);
      if (turnAngle>0) turnDir=+1;
      else turnDir=-1;
      break;
  }
}
void driveUpdateMotors(){
  int16_t power, power1,power2, correction;
  int16_t dYaw; //difference in angle, in units of 0.1 deg
  int32_t dPosition;
  uint32_t elapsedTime=millis()-driveStartTime;
  //get the  power (0--500), taking into account acceleration/decceleration
  if (elapsedTime<5000){
    power = min (abs(*driveTargetPower), elapsedTime*driveAccel/1000);
  } else {
    power = abs(*driveTargetPower);
  }
  //now, for driving fixed distance, let us see if it is time to start slowing down
  // we use familiar formula v=sqrt(2*a*s), where a is acceleration, and s distance to travel
  // and we should start slowing down when s=v_max^2/2a
  if (*driveMode==DRIVE_STRAIGHT_DISTANCE){
    float dPosition=driveDir*(driveTargetPosition-driveCurPosition()); //in encoder ticks
    dPosition/=distanceUnit; //now, in distance units

    if ( (dPosition/abs(*driveTargetPower))<0.1f){
      //less than 1/10 sec travel time at our speed - time to stop
      *driveStatus=DRIVE_STATUS_COMPLETE;
      *driveMode=DRIVE_OFF;
      //Serial.println("Drive: turnign motors off");
      setMotorsPower(0,0);
      return;
    } else {
      float p= sqrtf(2*driveAccel*dPosition);
      if (p<power) power=p;
      if (power<(*driveMinPower)) power=(*driveMinPower);
    }
    //debug[0]=dPosition;
    //debug[1]=power;
    //debug[2]=driveAccel;
  }
  //now taht we have desired power, let us apply it
  switch (*driveMode){
    case DRIVE_STRAIGHT:
    case DRIVE_STRAIGHT_DISTANCE:
      //get default powers
      power1=power*motorDir[0];
      power2=power*motorDir[1];
      //now, compute and apply the course correction
      dYaw=normalizeAngle(*driveHeading-*yaw );
      correction=-(int16_t)(drivePID.update(0.1f*dYaw) * MOTOR_MAX_POWER);//positive correction means we need to turn cw
      //correction=(int16_t)(0.3f*dYaw); //dYaw=900=90 deg will give correction of 300, or 60%
      power1+=correction*motorTurnDir[0];
      power2+=correction*motorTurnDir[1];
        //FIXME
      break;
    case DRIVE_TURN:
      dYaw=turnDir*normalizeAngle(*driveHeading-*yaw); //should be positive
      if (dYaw<30) {
        //less than 3 degree to turn, time to stop
        *driveStatus=DRIVE_STATUS_COMPLETE;
        *driveMode=DRIVE_OFF;
        //Serial.println("Drive: turnign motors off 2");
        setMotorsPower(0,0);
        return;
      } else if (dYaw<200) {
        //20 degrees left - time to slow down; have speed proportional to angle to turn
        power=(*driveTargetPower)*(float)dYaw/200.0f;
        //but do not go below the minimal power
        if (power < *driveMinPower) power=*driveMinPower;
      }
      //otherwise, do not change power from what it was
      //now, setup power1, power2
      power1=power*turnDir*motorTurnDir[0];
      power2=power*turnDir*motorTurnDir[1];
  }
  //Serial.println("Drive: setting motors");
  setMotorsPower(power1, power2);
}
