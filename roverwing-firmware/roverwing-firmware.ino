#include "wiring_private.h"
//registers
#include "regmap.h"
//code for motors, encoders, and servos
#include "motors.h"
#include "sonars.h"
#include "analog.h"
#include "i2c.h"
#include "ICM42605.h"
#include "gps.h"
#include "mag.h"
#include "neopixel.h"
#include "drive.h"


#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 3
//uncomment to allow debugging print to Serial.
//#define DEBUG_PRINT


//script own variables
uint8_t i;
uint32_t lastPrint=0; //time of last printing, in ms
uint32_t lastLoop2update=0; //time when we last ran low-priority updates, in us
uint32_t loop2deltat=1;
bool blink=false;
//int16_t motorDelta = 25;
//int16_t servoDelta = 50;
float voltage=0;
const float voltageScale=(3.3/1023.0)*(1+(37.4/9.53)); //voltage divider uses 37.4K  and 9.53K resistors
uint32_t loopCount=0;
uint32_t loop2Count=0; //for low priority loop
uint32_t intPixelColor=GREEN; //color for neopixel

///////////////////////////////////////////////////
void setup() {
  //SerialGPS.begin(9600);
  i2cMasterBegin(100000); //start I2C bus on Wire1 as master, in regular mode (100 kHz)
  i2cSlaveBegin();        //start i2c bus on Wire, as a slave
  initRegmap();
  Serial.begin(9600);
  //delay(2000);
  Serial.println("Starting RoverWing");

  setupTimers();
  setupMotorPins();
  setupSonarPins();
  setupAnalogPins();
  fwVersion[0]=FW_VERSION_MINOR;
  fwVersion[1]=FW_VERSION_MAJOR;
  *whoAmI=SLAVE_ADDRESS;
  *lowVoltage=0; //set low voltage threshold to 0, effectively disabling low voltage warnings
  *sonarBitmask = 0x00; // sonars inactive
  *analogBitmask = 0xFF; //analog inputs are active
  //initialize servo positions
  for (int i=0; i<4; i++){
    servoPosition[i] = 1500;
  }
  //
  *imuStatus = IMU_OFF;
  *gpsStatus = GPS_OFF;
  *magStatus = MAG_OFF;
  //neopixels
  *pixelBrightness=(uint8_t)32;
  pixelBegin();
  intPixelUpdate(BLUE);
  pixelShow();
  //delay(3000);
}

void loop() {
  loopCount++;
  //High priority: done every cycle
  //First, update configuration/motors/servos
  if (isSet(FLAG_SERVO)){
    clearFlag(FLAG_SERVO);//unset the servo flag bit
#ifdef DEBUG_PRINT
    Serial.print("setting servo to new positions: ");
    Serial.print(servoPosition[0]); Serial.println(" ");
#endif
    setServos();
  }
  if (isSet(FLAG_DRIVE_MODE)) {
    clearFlag(FLAG_DRIVE_MODE);
#ifdef DEBUG_PRINT
    Serial.println("Setting drive mode");
#endif
    driveSetup();
  }
  if (isSet(FLAG_ENC_RESET)) {
    clearFlag(FLAG_ENC_RESET);
#ifdef DEBUG_PRINT
    Serial.println("Resetting encoder(s)");
#endif
    resetEncoders();
  }
  if (isSet(FLAG_MOTOR_PID)){
#ifdef DEBUG_PRINT
  Serial.println("Updating motor PID  configuration");
#endif
    updateMotorsConfig();
  }
  if (isSet(FLAG_MOTOR_MODE)||isSet(FLAG_MOTOR_POWER)){
    clearFlag(FLAG_MOTOR_MODE);
    clearFlag(FLAG_MOTOR_POWER);
#ifdef DEBUG_PRINT
    Serial.println("Updating motor mode/power  configuration");
    Serial.print("Motor1 mode: "); Serial.print(motorMode[0] ); Serial.print(" power: "); Serial.println(motorPower[0]);
    Serial.print("Motor2 mode: "); Serial.print(motorMode[1] ); Serial.print(" power: "); Serial.println(motorPower[0]);
#endif
    setMotors();
  }
  if (isSet(FLAG_IMU_CONFIG)){
    clearFlag(FLAG_IMU_CONFIG);
#ifdef DEBUG_PRINT
  Serial.println("Configuring IMU");
#endif
  switch (*imuConfig){
  case IMU_CONFIG_END: //stop
    *imuStatus = IMU_OFF;
    break;
  case IMU_CONFIG_BEGIN://begin
    *imuStatus = 33; //FIXME
    Serial.println("starting IMU");
    ICM42605begin();
    Serial.println("IMU Started");
    //*imuStatus += 7;
    break;
  case IMU_CONFIG_CALIBRATE: //calibrate
    ICM42605calibrate();
    break;
}
  }
  if (isSet(FLAG_ACCEL_OFFSET)){
    clearFlag(FLAG_ACCEL_OFFSET);
#ifdef DEBUG_PRINT
  Serial.println("Applying accelerometer calibration;");
#endif
    for (i=0; i<3; i++){
      accelOffset[i]=accelUsrOffset[i];
    }
  }
  if (isSet(FLAG_GYRO_OFFSET)){
    clearFlag(FLAG_GYRO_OFFSET);
#ifdef DEBUG_PRINT
    Serial.println("Applying gyro calibration;");
#endif
    for (i=0; i<3; i++){
      gyroOffset[i]=gyroUsrOffset[i];
    }
  }
  if (isSet(FLAG_MAG_CALIBRATION)){
    clearFlag(FLAG_MAG_CALIBRATION);
#ifdef DEBUG_PRINT
    Serial.println("Applying mag calibration;");
#endif
    //user provided offsets and matrix  for magnetometer; let us use them
    magSetCalData();
  }
  if (isSet(FLAG_MAG_CONFIG)){
    clearFlag(FLAG_MAG_CONFIG);
#ifdef DEBUG_PRINT
    Serial.println("Configuring magentometer");
#endif
    switch (*magConfig){
      case MAG_CONFIG_END: //stop
        magEnd();
        break;
      case MAG_CONFIG_BEGIN://begin
        magBegin();
        break;
      case MAG_CONFIG_CALIBRATE: //calibrate
        magCalibrate();
        break;
    }
  }
  if (isSet(FLAG_GPS_CONFIG)){
    clearFlag(FLAG_GPS_CONFIG);
#ifdef DEBUG_PRINT
    Serial.println("Configuring GPS");
#endif
    if (*gpsConfig) GPSbegin();
    else GPSend();
  }
  if (isSet(FLAG_PIXEL_CONFIG)) {
    clearFlag(FLAG_PIXEL_CONFIG);
#ifdef DEBUG_PRINT
    Serial.println("Setting Neopixel brightness");
#endif
    pixelUpdateConfig(); //updates brightness
  }
  if (isSet(FLAG_PIXEL_COMMAND)) {
    clearFlag(FLAG_PIXEL_COMMAND);
#ifdef DEBUG_PRINT
    Serial.println("Showing Neopixels");
#endif
    if (*pixelCommand){
      pixelShow(); //pushes changes to hardware
    }
  }

  /**********************************************
   * now, update all sensors
   **********************************************/
  if ((*imuStatus)==IMU_OK ) {
#ifdef DEBUG_PRINT
    Serial.println("Updating IMU");
#endif
    ICM42605update();
  }
  if (*gpsStatus){
#ifdef DEBUG_PRINT
    Serial.println("Updating GPS");
#endif
      GPSupdate();
  }
  if (*magStatus==MAG_OK) {
#ifdef DEBUG_PRINT
    Serial.println("Updating magnetometer");
#endif
       //magnetometer active
    magUpdate();
  }
  updateAnalogs();

  //low priority loop: updated 25 times/s, i.e. every 40 ms
  if (micros()- lastLoop2update>40000){
    loop2Count++;
    loop2deltat=micros()-lastLoop2update;//duration in us
    lastLoop2update = micros();
#ifdef DEBUG_PRINT
    Serial.print("Loop 2: count "); Serial.print(loop2Count);
    Serial.print("time since last update (us): "); Serial.println(loop2deltat);
    Serial.println("Updatign sonars");
#endif
    updateSonars();
#ifdef DEBUG_PRINT
    Serial.println("Sonars updated");
#endif    //compute  measured motor speed  using encoders
    float freq = 1000000.0f/loop2deltat; //frequency of loop2
    for (uint8_t i = 0; i<2; i++){
      speed[i] = (int16_t) ( (encoder[i]-prevEncoder[i])*freq ); //speed in enc ticks/s
      prevEncoder[i]=encoder[i];
    }
    //if using PID for motors, update motor power
    if ((motorMode[0] == MOTOR_MODE_SPEEDPID) ||(motorMode[1] == MOTOR_MODE_SPEEDPID) ){
#ifdef DEBUG_PRINT
     Serial.println("Updatign motors using PID");
#endif
      setMotors();
      //Serial.println("Setting PID");
    }
    if ((*imuStatus)==IMU_OK){
#ifdef DEBUG_PRINT
     Serial.println("Getting Yaw, pitch, and roll");
#endif
      //compute yaw, pitch, roll and write to register
      *yaw=(int16_t) (getYaw()*10.0f);
      *pitch=(int16_t) (getPitch()*10.0f);
      *roll=(int16_t) (getRoll()*10.0f);
    }
    if (*driveStatus==DRIVE_STATUS_INPROGRESS){
      driveUpdateMotors();
    }
    //update internal  neopixel, blinking 2 times a second
    if ((loop2Count%5)==0) {
      //every 5 cycles = 4 times /s
      blink =!blink;
      //compute voltage
      voltage = (float) analogAvg[0]*0.1*voltageScale;
      //Serial.print("Voltage: "); Serial.println(voltage);
      //check voltage limit
      if (voltage<(*lowVoltage)*0.1f) {
        intPixelColor = RED;
      } else {
        intPixelColor = GREEN;
      }
      //change color
      if (blink) intPixelUpdate(intPixelColor);
      else intPixelUpdate(OFF);
    }

  }
  //debugging print
#ifdef DEBUG_PRINT
  // info prints, once every 500 ms
  if (millis()-lastPrint>500){
    uint32_t duration=millis()-lastPrint; //in ms
    //
    Serial.print("Time since last print (ms): "); Serial.println(duration);
    Serial.print("Main loop: number of loops: "); Serial.print(loopCount);
    Serial.print("   freq (hz): "); Serial.println((float)loopCount*1000.0f/duration);
    loopCount=0;

    //print IMU values
    if (ICM42605isAvailable()) {
      Serial.print("IMU is aavailable. Status is ");
      Serial.println(*imuStatus);
    }
    if ((*imuStatus==IMU_OK)){
     ICM42605print();
    }
    //print encoders
    Serial.print("Encoders: "); Serial.print(encoder[0]); Serial.print("    "); Serial.println(encoder[1]);
    //print sonars
    Serial.print("Sonars: "); Serial.print((sonarAvg[0]+5)/10);
        Serial.print("    "); Serial.print((sonarAvg[1]+5)/10);
        Serial.print("    "); Serial.println((sonarAvg[2]+5)/10);
    //print analogs
    printAnalogs();
    lastPrint=millis();
  }
#endif
}
