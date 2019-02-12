#include "regmap.h"
//allocate memory for registers, aligned with 32-bit words
int32_t REGA32[REGA_SIZE32];
int32_t REGB32[REGB_SIZE32];
//cast them as byte arrays
byte * REGA = (byte *)REGA32;
byte * REGB = (byte *)REGB32;
//change flags.  Whenever one of the registers is written to,
// it sets one of the bits in changeFlag, to indicate
// to the main loop that it needs processing
uint32_t volatile changeFlag=0;

// array to map register offsets (for REGB) to flag bits
// instead of giving it static values, we will initialize it using a function
// initRegmap
uint32_t registerFlag[REGB_SIZE32*4];
void initRegmap(){
  int i;
  //initialize registerFlag array, which shows, for each register, which flag it should set
  //Note: this uses hardcoded block sizes
  registerFlag[REGB_ANALOG_BITMASK] = FLAG_ANALOG;
  for (i=0; i<3; i++)  registerFlag[REGB_SONAR_BITMASK+i] = FLAG_SONAR_BITMASK;
  for (i=0; i<8; i++)  registerFlag[REGB_SERVO+i] = FLAG_SERVO;
  for (i=0; i<16; i++) registerFlag[REGB_MOTOR1_PID+i] = FLAG_MOTOR1_PID;
  for (i=0; i<16; i++) registerFlag[REGB_MOTOR2_PID+i] = FLAG_MOTOR2_PID;
  registerFlag[REGB_ENC_RESET] = FLAG_ENC_RESET;
  for (i=0; i<2; i++)  registerFlag[REGB_MOTOR_MODE+i] = FLAG_MOTOR_MODE;
  for (i=0; i<4; i++)  registerFlag[REGB_MOTOR_POWER+i] = FLAG_MOTOR_POWER;
  for (i=0; i<8; i++)  registerFlag[REGB_MOTOR_TARGET+i] = FLAG_MOTOR_TARGET;
  registerFlag[REGB_IMU_CONFIG] = FLAG_IMU_CONFIG;
  registerFlag[REGB_MAG_CONFIG] = FLAG_MAG_CONFIG;
  registerFlag[REGB_GPS_CONFIG] = FLAG_GPS_CONFIG;
  registerFlag[REGB_LOW_VOLTAGE] = FLAG_LOW_VOLTAGE;
  for (i=0; i<3; i++)  registerFlag[REGB_NUM_PIXELS+i] = FLAG_PIXEL_CONFIG;
  for (i=REGB_PIXEL_COLORS; i<REGB_SIZE32*4; i++)  registerFlag[i] = FLAG_PIXEL_COLORS;

}


//////////////////////////////
// Pointers to register A, for easier access
// THis way, you can access, e.g., reading of motor1 encoder by simply using
// value=encoder[0];
// instead of
// value =   REGA[REGA_ENCODER]|(REGA[REGA_ENCODER+1]<<8)|(REGA[REGA_ENCODER+2]<<16)|(REGA[REGA_ENCODER+3]<<24);
/////////////////////////////
//Firmware version
uint8_t * fwVersion    = &REGA[REGA_FW_VERSION];
//whoami
uint8_t * whoAmI       = &REGA[REGA_WHO_AM_I];
//analog inputs
uint16_t * analogRaw   =(uint16_t *) &REGA[REGA_ANALOG_RAW]; //scale 0-1023
uint16_t * analogAvg   =(uint16_t *) &REGA[REGA_ANALOG]; //filtered values, scale 0 -10230
//sonars
uint16_t * sonarRaw    =(uint16_t *) &REGA[REGA_SONAR_RAW]; //in mm
uint16_t * sonarAvg    =(uint16_t *) &REGA[REGA_SONAR]; //10*(distance in mm), after low pass filter
//encoders
volatile int32_t *  encoder = (int32_t *) &REGA[REGA_ENCODER];
int16_t *  speed   = (int16_t *) &REGA[REGA_SPEED]; //speed in encoder counts/s
// IMU
uint8_t * imuStatus         = &REGA[REGA_IMU_STATUS];
//acceleration data: accel[0]=x accel, accel[1]=y, accel[2]=z
//scale: LSB=1/16384 g
int16_t *  accel            = (int16_t *) &REGA[REGA_ACCEL];
//gyro data: gyro[0]=x rotation, gyro[1]=y, gyro[2]=z
//LSB=250.0 / 32768.0 deg/s
int16_t *  gyro             = (int16_t *) &REGA[REGA_GYRO];
//orientation, as a quaternion
//quat[0] is real part, quat[1], quat[2], quat[3] are i-, j- and k-components respectively
float * quat                =(float *) &REGA[REGA_QUAT];
// yaw, pitch, roll, in units of 1/100 degree
int16_t * yaw               =  (int16_t *) &REGA[REGA_YAW];
int16_t * pitch             =  (int16_t *) &REGA[REGA_PITCH];
int16_t * roll              =  (int16_t *) &REGA[REGA_ROLL];
// MAGNETOMETER
uint8_t * magStatus         = &REGA[REGA_MAG_STATUS];
volatile int16_t * mag     = (int16_t *) &REGA[REGA_MAG];
volatile int16_t *  heading = (int16_t *) &REGA[REGA_HEADING];
// GPS
uint8_t * gpsStatus         = &REGA[REGA_GPS_STATUS];
volatile int32_t * gpsLat   = (int32_t *) &REGA[REGA_GPS_LAT];
volatile int32_t * gpsLong  = (int32_t *) &REGA[REGA_GPS_LONG];
volatile uint32_t * gpsTimestampg  = (uint32_t *) &REGA[REGA_GPS_TIMESTAMP];

//////////////////////////////
// Pointers to register B
/////////////////////////////
//Analog
volatile byte * analogBitmask     = &REGB[REGB_ANALOG_BITMASK];
//Sonars
volatile byte * sonarBitmask      = &REGB[REGB_SONAR_BITMASK];
volatile uint16_t * sonarTimeout  = (uint16_t *) &REGB[REGB_SONAR_TIMEOUT];

// Servos
volatile uint16_t * servoPosition =(uint16_t *) &REGB[REGB_SERVO];
//motors
volatile float    * motor1PID     = (float *) &REGB[REGB_MOTOR1_PID];
volatile float    * motor2PID     = (float *) &REGB[REGB_MOTOR2_PID];
volatile byte     * encoderReset  = &REGB[REGB_ENC_RESET];
volatile uint8_t  * motorMode     = &REGB[REGB_MOTOR_MODE];
volatile int16_t  * motorPower    = (int16_t *) &REGB[REGB_MOTOR_POWER];
volatile int32_t  * motorTarget   = (int32_t *) &REGB[REGB_MOTOR_TARGET];
//imu, gps, magnetometer
volatile uint8_t * imuConfig      = &REGB[REGB_IMU_CONFIG];
volatile uint8_t * gpsConfig      = &REGB[REGB_GPS_CONFIG];
volatile uint8_t * magConfig      = &REGB[REGB_MAG_CONFIG];
// low voltage threshold
volatile uint8_t * lowVoltage     = &REGB[REGB_LOW_VOLTAGE];
//neopixels
volatile uint8_t * numPixels      = (uint8_t *) &REGB[REGB_NUM_PIXELS];
volatile uint8_t * pixelBrightness = (uint8_t *) &REGB[REGB_PIXEL_BRIGHTNESS];
volatile uint8_t * pixelCommand   = (uint8_t *) &REGB[REGB_PIXEL_COMMAND];
volatile uint32_t * pixelColors   = (uint32_t *) &REGB[REGB_PIXEL_COLORS];
