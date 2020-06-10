#include "regmap.h"
//allocate memory for registers, aligned with 32-bit words
volatile int32_t REGA32[REGA_SIZE32];
volatile int32_t REGB32[REGB_SIZE32];
//cast them as byte arrays
volatile byte * REGA = (byte *)REGA32;
volatile byte * REGB = (byte *)REGB32;
//change flags.  Whenever one of the B registers is written to,
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
  for (i=0; i<6; i++)  registerFlag[REGB_GYRO_OFFSET+i] = FLAG_GYRO_OFFSET;
  for (i=0; i<6; i++)  registerFlag[REGB_ACCEL_OFFSET+i] = FLAG_ACCEL_OFFSET;
  registerFlag[REGB_MAG_CONFIG] = FLAG_MAG_CONFIG;
  for (i=0; i<24; i++)  registerFlag[REGB_MAG_OFFSET+i] = FLAG_MAG_CALIBRATION;
  registerFlag[REGB_GPS_CONFIG] = FLAG_GPS_CONFIG;
  registerFlag[REGB_LOW_VOLTAGE] = FLAG_LOW_VOLTAGE;
  for (i=0; i<2; i++)  registerFlag[REGB_NUM_PIXELS+i] = FLAG_PIXEL_CONFIG;
  for (i=0; i<4; i++) registerFlag[REGB_PIXEL_COLOR] = FLAG_PIXEL_COLOR;
  registerFlag[REGB_PIXEL_COMMAND] = FLAG_PIXEL_COMMAND;
  registerFlag[REGB_DRIVE_MODE] = FLAG_DRIVE_MODE;
  for (i=0; i<26; i++) registerFlag[REGB_DRIVE_MOTORCONFIG] = FLAG_DRIVE_CONFIG;
  for (i=0; i<10; i++) registerFlag[REGB_DRIVE_DISTANCE] = FLAG_DRIVE_TARGET;
}


//////////////////////////////
// Pointers to register A, for easier access
// THis way, you can access, e.g., reading of motor1 encoder by simply using
// value=encoder[0];
// instead of
// value =   REGA[REGA_ENCODER]|(REGA[REGA_ENCODER+1]<<8)|(REGA[REGA_ENCODER+2]<<16)|(REGA[REGA_ENCODER+3]<<24);
/////////////////////////////
//Firmware version
volatile uint8_t * fwVersion    = &REGA[REGA_FW_VERSION];
//whoami
volatile uint8_t * whoAmI       = &REGA[REGA_WHO_AM_I];
//analog inputs
volatile uint16_t * analogRaw   =(uint16_t *) &REGA[REGA_ANALOG_RAW]; //scale 0-1023
volatile uint16_t * analogAvg   =(uint16_t *) &REGA[REGA_ANALOG]; //filtered values, scale 0 -10230
//sonars
volatile uint16_t * sonarRaw    =(uint16_t *) &REGA[REGA_SONAR_RAW]; //in mm
volatile uint16_t * sonarAvg    =(uint16_t *) &REGA[REGA_SONAR]; //10*(distance in mm), after low pass filter
//encoders
volatile int32_t *  encoder = (int32_t *) &REGA[REGA_ENCODER];
volatile int16_t *  speed   = (int16_t *) &REGA[REGA_SPEED]; //speed in encoder counts/s
// IMU
volatile uint8_t * imuStatus         = &REGA[REGA_IMU_STATUS];
//acceleration data: accel[0]=x accel, accel[1]=y, accel[2]=z
//scale: LSB=1/16384 g
volatile int16_t *  accel            = (int16_t *) &REGA[REGA_ACCEL];
//gyro data: gyro[0]=x rotation, gyro[1]=y, gyro[2]=z
//LSB=250.0 / 32768.0 deg/s
volatile int16_t *  gyro             = (int16_t *) &REGA[REGA_GYRO];
//orientation, as a quaternion
//quat[0] is real part, quat[1], quat[2], quat[3] are i-, j- and k-components respectively
volatile float * quat                =(float *) &REGA[REGA_QUAT];
// yaw, pitch, roll, in units of 1/100 degree
volatile int16_t  * yaw               =  (int16_t *) &REGA[REGA_YAW];
volatile int16_t  * pitch             =  (int16_t *) &REGA[REGA_PITCH];
volatile int16_t  * roll              =  (int16_t *) &REGA[REGA_ROLL];
volatile int16_t  * accelOffset         = (int16_t *) &REGA[REGA_ACCEL_OFFSET];
volatile int16_t  * gyroOffset         = (int16_t *) &REGA[REGA_GYRO_OFFSET];
// MAGNETOMETER
volatile uint8_t  * magStatus         = &REGA[REGA_MAG_STATUS];
volatile int16_t  * mag               = (int16_t *) &REGA[REGA_MAG];
volatile int16_t  * magOffset         = (int16_t *) &REGA[REGA_MAG_OFFSET];
// GPS
volatile uint8_t  * gpsStatus         = &REGA[REGA_GPS_STATUS];
volatile int32_t  * gpsLat            = (int32_t *) &REGA[REGA_GPS_LAT];
volatile int32_t  * gpsLong           = (int32_t *) &REGA[REGA_GPS_LONG];
volatile uint32_t * gpsTimestamp      = (uint32_t *) &REGA[REGA_GPS_TIMESTAMP];
volatile uint8_t  * driveStatus       = (uint8_t *) &REGA[REGA_DRIVE_STATUS];
volatile int16_t  * debug             = (int16_t *) &REGA[REGA_DEBUG];

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
volatile uint16_t * motorMaxspeed = (uint16_t *) &REGB[REGB_MOTOR_MAXSPEED];
volatile int16_t  * motorPower    = (int16_t *) &REGB[REGB_MOTOR_POWER];
volatile int32_t  * motorTarget   = (int32_t *) &REGB[REGB_MOTOR_TARGET];
//imu, gps, magnetometer
volatile uint8_t * imuConfig      = &REGB[REGB_IMU_CONFIG];
volatile int16_t * gyroUsrOffset  = (int16_t *) &REGB[REGB_GYRO_OFFSET];
volatile int16_t * accelUsrOffset = (int16_t *) &REGB[REGB_ACCEL_OFFSET];
volatile uint8_t * gpsConfig      = &REGB[REGB_GPS_CONFIG];
volatile uint8_t * magConfig      = &REGB[REGB_MAG_CONFIG];
volatile int16_t * magUsrOffset   = (int16_t *) &REGB[REGB_MAG_OFFSET];
volatile int16_t * magUsrMatrix   = (int16_t *) &REGB[REGB_MAG_MATRIX];
// low voltage threshold
volatile uint8_t * lowVoltage     = &REGB[REGB_LOW_VOLTAGE];
//neopixels
volatile uint8_t * numPixels      = (uint8_t *) &REGB[REGB_NUM_PIXELS];
volatile uint8_t * pixelBrightness = (uint8_t *) &REGB[REGB_PIXEL_BRIGHTNESS];
volatile uint8_t * pixelCommand   = (uint8_t *) &REGB[REGB_PIXEL_COMMAND];
volatile uint32_t * pixelColor    = (uint32_t *) &REGB[REGB_PIXEL_COLOR];
//DRIVE
volatile uint8_t  * driveMode     = (uint8_t *) &REGB[REGB_DRIVE_MODE];
volatile uint8_t  * driveMotorConfig= (uint8_t *) &REGB[REGB_DRIVE_MOTORCONFIG];
volatile uint16_t * driveMaxSpeed = (uint16_t *) &REGB[REGB_DRIVE_MAXSPEED];
volatile uint16_t * driveMaxTurnSpeed = (uint16_t *) &REGB[REGB_DRIVE_MAXTURNSPEED];
volatile uint16_t * driveMinPower = (uint16_t *) &REGB[REGB_DRIVE_MINPOWER];
volatile float    * drivePIDcoef  = (float *) &REGB[REGB_DRIVE_PID_COEF];
volatile int32_t  * driveDistance = (int32_t *) &REGB[REGB_DRIVE_DISTANCE];
volatile int16_t  * driveHeading  = (int16_t *) &REGB[REGB_DRIVE_HEADING];
volatile int16_t  * driveTargetPower= (int16_t *) &REGB[REGB_DRIVE_TARGETPOWER];
volatile uint16_t * driveRampTime = (uint16_t *) &REGB[REGB_DRIVE_RAMPTIME];
