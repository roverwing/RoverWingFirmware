#include "wiring_private.h"
//registers
#include "regmap.h"
//code for motors, encoders, and servos
#include "motors.h"
#include "sonars.h"
#include "analog.h"
#include "i2c.h"
#include "MPU6050.h"
#include "gps.h"
#include "mag.h"
#include "neopixel.h"
#include "drive.h"


#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
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
const float voltageScale=(3.3/1023.0)*(122.0/22.0); //voltage divider uses 100k and 22k resistors
uint32_t loopCount=0;
uint32_t loop2Count=0; //for low priority loop
uint32_t intPixelColor=GREEN; //color for neopixel


///////////////////////////////////////////////////
void setup() {
  //SerialGPS.begin(9600);
  i2cMasterBegin(400000); //start I2C bus on Wire1 as master, in fast mode (400 kHz)
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
  *analogBitmask = 0x00; //analog inputs are inactive
  //initialize servo positions
  for (int i=0; i<4; i++){
    servoPosition[i] = 1500;
  }
  //
  *imuStatus = 0x05;
  *gpsStatus = GPS_OFF;
  *magStatus = MAG_OFF;
  //neopixels
  *pixelBrightness=(uint8_t)32;
  pixelBegin();
  pixelShow();
  //delay(3000);
  if  (MPU6050isAvailable()) {
    Serial.println("IMU is available");
  }
}

void loop() {
  loopCount++;
  //High priority: done every cycle
  //First, update configuration/motors/servos
  if (isSet(FLAG_SERVO)){
    clearFlag(FLAG_SERVO);//unset the servo flag bit
    //Serial.print("setting servo to new positions: ");
    //Serial.print(servoPosition[0]); Serial.println(" ");
    setServos();
  }
  if (isSet(FLAG_ENC_RESET)) {
    clearFlag(FLAG_ENC_RESET);
    resetEncoders();
  }
  if (isSet(FLAG_MOTOR_PID)){
    updateMotorsConfig();
    Serial.println("Updating motor PID  configuration");
    //Serial.print("Motor1 mode: "); Serial.println(motorMode[0] );
    //Serial.print("Motor2 mode: "); Serial.println(motorMode[1] );
  }
  if (isSet(FLAG_MOTOR_MODE)||isSet(FLAG_MOTOR_POWER)){
    clearFlag(FLAG_MOTOR_MODE);
    clearFlag(FLAG_MOTOR_POWER);
    setMotors();
  }
  if (isSet(FLAG_IMU_CONFIG)){
    clearFlag(FLAG_IMU_CONFIG);
    switch (*imuConfig){
      case IMU_CONFIG_END: //stop
        *imuStatus = IMU_OFF;
        break;
      case IMU_CONFIG_BEGIN://begin
        *imuStatus = 33;
        Serial.println("starting IMU");
        MPU6050begin();
        Serial.println("IMU Started");
        //*imuStatus += 7;
        break;
      case IMU_CONFIG_CALIBRATE: //calibrate
        MPU6050calibrate();
        break;
    }
  }
  if (isSet(FLAG_ACCEL_OFFSET)){
    clearFlag(FLAG_ACCEL_OFFSET);
    //Serial.println("Applying accelerometer calibration;");
    for (i=0; i<3; i++){
      accelOffset[i]=accelUsrOffset[i];
    }
  }
  if (isSet(FLAG_GYRO_OFFSET)){
    clearFlag(FLAG_GYRO_OFFSET);
    Serial.println("Applying gyro calibration;");
    for (i=0; i<3; i++){
      gyroOffset[i]=gyroUsrOffset[i];
    }
  }
  if (isSet(FLAG_MAG_CALIBRATION)){
    clearFlag(FLAG_MAG_CALIBRATION);
    Serial.println("Applying mag calibration;");
    //user provided offsets and matrix  for magnetometer; let us use them
    magSetCalData();
  }
  if (isSet(FLAG_MAG_CONFIG)){
    clearFlag(FLAG_MAG_CONFIG);
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
    if (*gpsConfig) GPSbegin();
    else GPSend();
  }
  if (isSet(FLAG_PIXEL_CONFIG)) {
    clearFlag(FLAG_PIXEL_CONFIG);
    pixelUpdateConfig(); //updates brightness
  }
  if (isSet(FLAG_PIXEL_COMMAND)) {
    clearFlag(FLAG_PIXEL_COMMAND);
    if (*pixelCommand){
      pixelShow(); //pushes changes to hardware
    }
  }
  if (isSet(FLAG_DRIVE_MODE)) {
    clearFlag(FLAG_DRIVE_MODE);
    driveSetup();
  }


  /**********************************************
   * now, update all sensors
   **********************************************/
  if ((*imuStatus)==IMU_OK ) {
    MPU6050update();
  }
  if (*gpsStatus){
    GPSupdate();
  }
  if (*magStatus==MAG_OK) {
    //magnetometer active
    magUpdate();
  }
  updateSonars();
  updateAnalogs();

  //low priority loop: updated 25 times/s, i.e. every 40 ms
  if (micros()- lastLoop2update>40000){
    loop2Count++;
    loop2deltat=micros()-lastLoop2update;//duration in us
    lastLoop2update = micros();
    //Serial.print("last Loop 2 update, us: "); Serial.println(lastLoop2update);
    //compute  measured motor speed  using encoders
    for (uint8_t i = 0; i<2; i++){
      speed[i] = (float)(encoder[i]-prevEncoder[i])*(1000000.0f/loop2deltat);
      prevEncoder[i]=encoder[i];
    }
    //if using PID for motors, update motor power
    if ((motorMode[0] == MOTOR_MODE_SPEEDPID) ||(motorMode[1] == MOTOR_MODE_SPEEDPID) ){
      setMotors();
      //Serial.println("Setting PID");
    }
    if ((*imuStatus)==IMU_OK){
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
      //check voltage limit
      if (voltage<(*lowVoltage)*0.1f) {
        intPixelColor = YELLOW;
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
    if (MPU6050isAvailable()) {
      Serial.print("IMU is aavailable. Status is ");
      Serial.println(*imuStatus);
    }
    if ((*imuStatus==IMU_OK)){
      MPU6050print();
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
