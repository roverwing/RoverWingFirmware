#ifndef _ROVERWING_REGMAP_H
#define _ROVERWING_REGMAP_H
#include <Arduino.h>
#define REGA_SIZE32 30 //size of regiter A, in 4-byte (32 bit) units
#define REGB_SIZE32 63 //size of regiter B, in 4-byte (32 bit) units
//this will be defined in regmap.cpp
extern  volatile byte * REGA;
extern  volatile byte * REGB;

/* REGISTER A - read-only

Bytes  Offset name     value               data type  description
0      REGA_FW_VERSION fwVersion[0]        uint8    firmware version - minor
1                      fwVersion[1]        uint8    fw version major
2-3    REGA_ANALOG_RAW analogRaw[0]        uint16    scale 0-1023;
4-5                    analogRaw[1]        uint16    1023 = 3.3V
6-7                    analogRaw[2]        uint16
8-9                    analogRaw[3]        uint16
10-11                  analogRaw[4]        uint16
12-13                  analogRaw[5]        uint16
14-15                  analogRaw[6]        uint16
16-17   REGA_ANALOG    analogAvg[0]=vsense uint16    scale 0-10230;
18-19                  analogAvg[1]        uint16    10230 = 3.3V
20-21                  analogAvg[2]        uint16
22-23                  analogAvg[3]        uint16
24-25                  analogAVg[4]        uint16
26-27                  analogAvg[5]        uint16
28-29                  analogAvg[6]        uint16
30-31   REGA_SONAR_RAW sonarRaw[0]         uint16     raw sonar distance values,
32-33                  sonarRaw[1]         uint16     in mm
34-35                  sonarRaw[2]         uint16
36-37   REGA_SONAR     sonarAvg[0]         uint16    filtered sonar distance
38-39                  sonarAvg[1]         uint16    values, in units of 0.1 mm
40-41                  sonarAvg[2]         uint16
---------------------------------------------
42      REGA_WHO_AM_I                      uint8_t   must contain value 0x11 (decimal 17); used for testing connection
43      unused
------- MOTORS AND ENCODERS ------------------
44-47   REGA_ENCODER   encoder[0]          int32      encoder values, in ticks
48-51                  encoder[1]          int32
52-53   REGA_SPEED     speed[0]            int16      speed, in encoder ticks/s
54-55                  speed[1]            int16
------- IMU ----------------------------------
56      REGA_IMU_STATUS imuStatus          uint8
57-59    unused
60-61   REGA_ACCEL     accel[0]            int16    acceleration x, LSB=1/16384 g
62-63                  accel[1]            int16    acceleration y
64-65                  accel[2]            int16    acceleration z
66-67   REGA_GYRO      gyro[0]             int16    angular speed x-axis, LSB=250.0 / 32768.0 deg/s
68-69                  gyro[1]             int16    angular speed y
70-71                  gyro[2]             int16    angualr speed z
72-75   REGA_QUAT      quat[0]             float    orientation as quaternion, real part
76-79                  quat[1]             float    i-part
80-83                  quat[2]             float    j-part
84-87                  quat[3]             float    k-part
88-89   REGA_YAW       yaw                 int16    yaw angle, in units of 1/100 degree
90-91   REGA_PITCH     pitch               int16    pitch angle, in units of 1/100 degree
92-93   REGA_ROLL      roll                int16    roll  angle, in units of 1/100 degree
------- MAGNETOMETER --- ----------------------
94      REGA_MAG_STATUS magStatus          uint8
95      unused
96-97   REGA_MAG       mag[0]              int16   x-comp of magnetic field, in units of 1 milligauss=0.1uT
98-99                  mag[1]              int16   y
100-101                mag[2]              int16   z
102-103 REGA_HEADING   heading             int16   heading in degrees; north=0, east=90
------ GPS -----------------------------------
104     REGA_GPS_STATUS
105     unused
106-109 REGA_GPS_LAT   gpsLat              int32  latitude, in units of 10^{-7} degree (about 10cm)
110-113 REGA_GPS_LONG  gpsLong             int32  longitude
114-117 REGA_GPS_TIMESTAMP gpsTimestamp    uint32 timestamp of last measurement, in ms

*/
#define REGA_FW_VERSION    0
#define REGA_ANALOG_RAW    2
#define REGA_ANALOG        16
#define REGA_SONAR_RAW     30
#define REGA_SONAR         36
#define REGA_WHO_AM_I      42
#define REGA_ENCODER       44
#define REGA_SPEED         52
#define REGA_IMU_STATUS    56
#define REGA_ACCEL         60
#define REGA_GYRO          66
#define REGA_QUAT          72
#define REGA_YAW           88
#define REGA_PITCH         90
#define REGA_ROLL          92
#define REGA_MAG_STATUS    94
#define REGA_MAG           96
#define REGA_HEADING       102
#define REGA_GPS_STATUS    104
#define REGA_GPS_LAT       106
#define REGA_GPS_LONG      110
#define REGA_GPS_TIMESTAMP 114


/*
   REGISTER B - writable
-------- ANALOG ----------------
0      REGB_ANALOG_BITMASK analogBitmask   byte     bitmask of active analog sensors, LSB=analog1
                                                     analog0=vsense - always active
------- SONAR -------------------
1      REGB_SONAR_BITMASK  sonarBitmask    byte     bitmask of active sonars, LSB=sonar1
2-3    REGB_SONAR_TIMEOUT  sonarTimeout    uint16   timeout for waiting for sonar echo, in us
------- SERVO --------------------
4-5    REGB_SERVO      servoPosition[0]    uint16   pulse width for servo1, in us (500-2500)
6-7                    servoPosition[1]    uint16   servo2
8-9                    servoPosition[2]    uint16   servo3
10-11                  servoPosition[3]    uint16   servo4
------ MOTOR1 PID CONFIG -------------
12-27  REGB_MOTOR1_PID  motor1PID []       float[4]
------ MOTOR2 PID CONFIG -------------
28-41  REGB_MOTOR2_PID  motor2PID []       float[4]
------ ENCODER RESET     -------------
42     REGB_ENC_RESET   encoderReset       byte     bit 0: reset motor1 encoder; bit 1: motor 2
------ MOTOR  MODE       -------------
43     REGB_MOTOR_MODE  motorMode[0]        byte
44                      motorMode[1]        byte
45-47  reserved
------ MOTOR POWER       -------------
48-49  REGB_MOTOR_POWER motorPower[0]      int16    motor power, -500...500
50-51                   motorPower[1]      int16
------ MOTOR TARGET
52-55  REGB_MOTOR_TARGET motorTarget[0]    int32   in speed PID mode: speed  in enc ticks/s
56-59                    motorTarget[1]    int32
------ IMU
60    REGB_IMU_CONFIG                      byte     bit0: is IMU active?
                                                    bits 2:1 : IMU orientation
------ MAGNETOMETER
61    REGB_MAG_CONFIG                      byte
------ GPS
62    REGB_GPS_CONFIG
------ Low voltage cutoff
63    REGB_LOW_VOLTAGE   lowVoltage       uint8_t  the low voltage threshold, in units of 0.1V
                                                   if voltage is below that, internal neopixel will blink yellow
----- NEOPIXELS_CONFIG
64    REGB_NUM_PIXELS  numPixels          uint8    actual number of used neopixels (at most 45)
                                                   doesn't include internal neopixel
65    REGB_PIXEL_BRIGHTNESS  pixelBrightness uint8 neopixel brightness, 0-255
66    REGB_PIXEL_COMMAND                  uint8    command to start/update/stop pixels
67     unused
----- NEOPIXELS_COlORS
68-71 REGB_PIXEL_COLORS                   uint32   color of 1st neopixel (00RRGGBB)
72-75                                     uint32   color of 2nd neopixel
...
244-247                                   uint32   color of 45th (last) neopixel
*/
#define REGB_ANALOG_BITMASK    0
#define REGB_SONAR_BITMASK     1
#define REGB_SONAR_TIMEOUT     2
#define REGB_SERVO             4
#define REGB_MOTOR1_PID        12
#define REGB_MOTOR2_PID        28
#define REGB_ENC_RESET         42
#define REGB_MOTOR_MODE        43
#define REGB_MOTOR_POWER       48
#define REGB_MOTOR_TARGET      52
#define REGB_IMU_CONFIG        60
#define REGB_MAG_CONFIG        61
#define REGB_GPS_CONFIG        62
#define REGB_LOW_VOLTAGE       63
#define REGB_NUM_PIXELS        64
#define REGB_PIXEL_BRIGHTNESS  65
#define REGB_PIXEL_COMMAND     66
#define REGB_PIXEL_COLORS      68



// now, pointer/aliases - for direct access to registers. These are forward declarations,
//the definitions are in regmap.cpp

//Firmware version
extern volatile uint8_t * fwVersion;
//whoami
extern volatile uint8_t * whoAmI;
//analog inputs
extern volatile uint16_t * analogRaw; //scale 0-1023
extern volatile uint16_t * analogAvg; //filtered values, scale 0 -10230
//sonars
extern volatile uint16_t * sonarRaw; //in mm
extern volatile uint16_t * sonarAvg; //10*(distance in mm), after low pass filter
//encoders
extern volatile int32_t *  encoder;
extern volatile int16_t *  speed; //speed in encoder counts/s
// IMU
extern volatile uint8_t * imuStatus;
//acceleration data: accel[0]=x accel, accel[1]=y, accel[2]=z
//scale: LSB=1/16384 g
extern volatile int16_t *  accel;
//gyro data: gyro[0]=x rotation, gyro[1]=y, gyro[2]=z
//LSB=250.0 / 32768.0 deg/s
extern volatile int16_t *  gyro;
//orientation, as a quaternion
//quat[0] is real part, quat[1], quat[2], quat[3] are i-, j- and k-components respectively
extern volatile float * quat;
// yaw, pitch, roll, in units of 1/100 degree
extern volatile int16_t * yaw;
extern volatile int16_t * pitch;
extern volatile int16_t * roll;
// MAGNETOMETER
extern volatile uint8_t * magStatus;
extern volatile int16_t * mag;
extern volatile int16_t *  heading;
// GPS
extern volatile uint8_t * gpsStatus;
extern volatile int32_t * gpsLat;
extern volatile int32_t * gpsLong;
extern volatile uint32_t * gpsTimestamp;

//////////////////////////////
// Pointers to register B
/////////////////////////////
//Analog
extern volatile byte * analogBitmask;
//Sonars
extern volatile byte * sonarBitmask;
extern volatile uint16_t * sonarTimeout;

// Servos
extern volatile uint16_t * servoPosition;
//motors
extern volatile float    * motor1PID;
extern volatile float    * motor2PID;
extern volatile byte     * encoderReset;
extern volatile uint8_t  * motorMode;
extern volatile int16_t  * motorPower;
extern volatile int32_t  * motorTarget;
//imu, gps, magnetometer
extern volatile uint8_t * imuConfig;
extern volatile uint8_t * gpsConfig;
extern volatile uint8_t * magConfig;
// low voltage threshold
extern volatile uint8_t * lowVoltage;
//neopixels
extern volatile uint8_t * numPixels;
extern volatile uint8_t * pixelBrightness;
extern volatile uint8_t * pixelCommand;
extern volatile uint32_t * pixelColors;




// flags. Whenever a register is written to, it sets one of the bits in
// changeFlag (see regmap.cpp) These bits are given names below
//change flags.  Whenever one of the registers is written to,
// it sets one of the bits in changeFlag, to indicate
// to the main loop that it needs processing
extern uint32_t volatile changeFlag;

// array to map register offsets (for REGB) to flag bits
// instead of giving it static values, we will initialize it using a function
// initRegmap
extern uint32_t registerFlag[];

#define FLAG_NONE 0x0
#define FLAG_ANALOG (1ul)
#define FLAG_SONAR_BITMASK ((1ul)<<1)
#define FLAG_SERVO ((1ul)<<2)
#define FLAG_MOTOR1_PID ((1ul)<<3)
#define FLAG_MOTOR2_PID ((1ul)<<4)
#define FLAG_ENC_RESET ((1ul)<<5)
#define FLAG_MOTOR_MODE ((1ul)<<6)
#define FLAG_MOTOR_POWER ((1ul)<<7)
#define FLAG_MOTOR_TARGET ((1ul)<<8)
#define FLAG_IMU_CONFIG ((1ul)<<9)
#define FLAG_MAG_CONFIG ((1ul)<<10)
#define FLAG_GPS_CONFIG ((1ul)<<11)
#define FLAG_LOW_VOLTAGE ((1ul)<<12)
#define FLAG_NUM_PIXELS ((1ul)<<13)
#define FLAG_PIXEL_CONFIG ((1ul)<<14)
#define FLAG_PIXEL_COLORS ((1ul)<<15)
//composite flags
#define FLAG_MOTOR (FLAG_MOTOR1_PID | FLAG_MOTOR2_PID |   FLAG_MOTOR_MODE | FLAG_MOTOR_POWER | FLAG_MOTOR_TARGET)
//finally, function-like macros and function  declarations
#define isSet(F) (changeFlag & (uint32_t) F)
#define setFlag(F)  changeFlag |= (uint32_t) F
#define clearFlag(F) changeFlag &=~ (uint32_t)F

void initRegmap();

#endif
