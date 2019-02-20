#include "wiring_private.h"
//registers
#include "regmap.h"
//code for motors, encoders, and servos
#include "motors.h"
#include "sonars.h"
#include "analog.h"
#include "i2c.h"
#include "MPU6050.h"
#include "neopixel.h"

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 9
//uncomment to allow debugging print to Serial.
//#define DEBUG_PRINT


//script own variables
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
  *imuStatus = 0x00;
  //neopixels
  *pixelBrightness=(uint8_t)32;
  *numPixels= (uint8_t)8;

  setupPixels();
  for (int i=0; i<*numPixels; i++) {
    pixelColors[i]=GREEN;
  }
  updatePixels();
  //temporary, FIXME
  //startMPU6050();
  //*imuStatus=0x01;
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
  if (isSet(FLAG_MOTOR)){
    updateMotorsConfig();
    Serial.println("Updating motor configuration");
    clearFlag(FLAG_MOTOR);//unset the motor flag bits
    //now, compute the power to give to motors and actually
    //set motors to that power
    setMotors();
    //Serial.print("Motor1 mode: "); Serial.println(motorMode[0] );
    //Serial.print("Motor2 mode: "); Serial.println(motorMode[1] );

  }
  if (isSet(FLAG_PIXEL_CONFIG)) {
    clearFlag(FLAG_PIXEL_CONFIG);
    //FIXME
    updatePixels(); //updates brightness, copies pixel colors and then calls pixel.show();
  }
  if (isSet(FLAG_IMU_CONFIG)){
    clearFlag(FLAG_IMU_CONFIG);
    //debug!
    /*servoPosition[0]=1000;
    setServos();
    delay(300);
    servoPosition[0]=1500;
    setServos();*/
    //
    bool activate = (*imuConfig)&0x01; //get the last bit of the config byte
    if (activate) {
      //we just received the command to activate IMU!
      //let us try actvating it; if we succeed, let us set imuStatus to true.
      // WARNING: activating IMU takes aabout 1.5 seconds!!!
      delay(500);
      startMPU6050();
      *imuStatus=(uint8_t) isAvailableMPU6050();
      Serial.print("Activating IMU: "); Serial.println(*imuStatus);
    } else {
      //we received the command to deactivate imu
      *imuStatus = false;
    }
  }
  //now, update all sensors
  if ((*imuStatus)) {
    updateMPU6050();
  }
  updateSonars();
  updateAnalogs();
  //low priority loop: updated 20 times/s, i.e. every 50 ms
  if (micros()- lastLoop2update>50000){
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
    if ((*imuStatus)){
      //compute yaw, pitch, roll and write to register
      *yaw=(int16_t) (getYaw()*100.0f);
      *pitch=(int16_t) (getPitch()*100.0f);
      *roll=(int16_t) (getRoll()*100.0f);
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
      if (blink) updateIntPixel(intPixelColor);
      else updateIntPixel(OFF);
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
    if ((*imuStatus)){
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
