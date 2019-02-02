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


// pointers to registers; defined in regmap.cpp
extern uint8_t * fwVersion;
extern uint8_t * whoAmI; 
extern volatile int32_t *  encoder;
extern volatile int16_t  * speed;  //actual current motor speed as measured by encoder,
                                  //in encoder ticks/s
extern uint16_t * sonarRaw; //raw readings, in mm
extern volatile uint8_t * motorMode;
extern uint16_t * sonarAvg;
extern uint16_t * analogRaw; //raw readings, scale 0-1023
extern uint16_t * analogAvg; //filtered values, scale 0 -10230
extern volatile uint16_t *  servoPosition;
extern volatile int16_t *  motorPower;
extern volatile byte *  sonarBitmask;
extern volatile byte *  analogBitmask;
/////////////////////////////////////////////////
//IMU data
////////////////////////////////////////////////
//acceleration data: accel[0]=x accel, accel[1]=y, accel[2]=z
//scale: LSB=1/16384 g
extern int16_t *  accel;
//gyro data: gyro[0]=x rotation, gyro[1]=y, gyro[2]=z
//LSB=250.0 / 32768.0 deg/s
extern int16_t *  gyro;
//orientation, as a quaternion
//quat[0] is real part, quat[1], quat[2], quat[3] are i-, j- and k-components respectively
extern float * quat;
//pointers to register addresses holding yaw, pitch, and roll values
//in units of 1/100 degree
extern int16_t * yaw;
extern int16_t * pitch;
extern int16_t * roll;

//llow voltahe threshold
extern volatile uint8_t * lowVoltage;
///////////////////////////////
//neopixels
//////////////////////////////
extern volatile uint32_t * pixelColors;
extern volatile uint8_t  * numPixels;
extern volatile uint8_t  * pixelBrightness;


//encoder computation
extern uint32_t * prevEncoder;

//flags
// Flags
extern volatile uint32_t  changeFlag; //defined in regmap.cpp
extern  uint32_t * registerFlag;


//script own variables
uint32_t lastPrint=0; //time of last printing, in ms
uint32_t lastLPupdate=0; //time when we last ran low-priority updates
uint32_t LPdeltat=1;
bool blink=false;
//int16_t motorDelta = 25;
//int16_t servoDelta = 50;
float voltage=0;
const float voltageScale=(3.3/1023.0)*(122.0/22.0); //voltage divider uses 100k and 22k resistors
uint32_t loopCount=0;
uint32_t LPloopCount=0; //for low priority loop
uint32_t intPixelColor=GREEN; //color for neopixel
bool IMUactive=true;


///////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  delay(2000);


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
  i2cMasterBegin(400000); //start I2C bus on Wire1 as master, in fast mode (400 kHz)
  i2cSlaveBegin();        //start i2c bus on Wire, as a slave
  if (isAvailableMPU6050()){
    startMPU6050();
    delay(2000);
    Serial.println("IMU started...");
    IMUactive = true;
  } else {
    Serial.println("Failed to start IMU");
    IMUactive = false;
  }
  //neopixels
  *pixelBrightness=(uint8_t)32;
  *numPixels= (uint8_t)8;

  setupPixels();
  for (int i=0; i<*numPixels; i++) {
    pixelColors[i]=GREEN;
  }
  updatePixels();
}

void loop() {
  loopCount++;
  //High priority: done every cycle
  if (IMUactive) {
    updateMPU6050();
  }
  if (isSet(FLAG_SERVO)){
    clearFlag(FLAG_SERVO);//unset the servo flag bit
    setServos(servoPosition);
  }
  if (isSet(FLAG_ENC_RESET)) {
    clearFlag(FLAG_ENC_RESET);
    resetEncoders();
  }
  if (isSet(FLAG_MOTOR)){
    updateMotorsConfig();
    clearFlag(FLAG_MOTOR);//unset the motor flag bit
    //now, compute the power to give to motors and actually
    //set motors to that power
    setMotors();
  }
  if (isSet(FLAG_PIXEL_CONFIG)) {
    clearFlag(FLAG_PIXEL_CONFIG);
    //FIXME
    updatePixels(); //updates brightness, copies pixel colors and then calls pixel.show();
  }
  updateSonars();
  updateAnalogs();
  //low priority:updated 20 times/s, i.e. every 50 ms
  if (millis()- lastLPupdate>50){
    LPloopCount++;
    LPdeltat=millis()-lastLPupdate;//duration in ms
    lastLPupdate = millis();
    //compute  measured motor speed  using encoders
    //FIXME: using ms gives low precision....
    for (uint8_t i = 0; i<2; i++){
      int32_t s= (encoder[i]-prevEncoder[i])*1000/LPdeltat;
      speed[i]= (int16_t) s;
      prevEncoder[i]=encoder[i];
    }
    //if using PID for motors, update motor power
    if ((motorMode[0] == MOTOR_MODE_SPEEDPID) ||(motorMode[0] == MOTOR_MODE_SPEEDPID) ){
      setMotors();
    }

    //compute yaw, pitch, roll and write to register
    *yaw=(int16_t) (getYaw()*100.0f);
    *pitch=(int16_t) (getPitch()*100.0f);
    *roll=(int16_t) (getRoll()*100.0f);
    //update internal  neopixel, blinking 2 times a second
    if ((LPloopCount%5)==0) {
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

  // info prints, once every 500 ms
  if (millis()-lastPrint>500){
    uint32_t duration=millis()-lastPrint; //in ms
    //
    //Serial.print("Time since last print (ms): "); Serial.println(duration);
    //Serial.print("Number of loops: "); Serial.println(loopCount);
    Serial.print("Main loop freq (hz): "); Serial.println((float)loopCount*1000.0f/duration);
    loopCount=0;

    //print IMU values
    if (IMUactive){
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
}
