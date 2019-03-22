#ifndef _ROVERWING_REGMAP_H
#define _ROVERWING_REGMAP_H
#include <Arduino.h>
#define REGA_SIZE32 42 //size of regiter A, in 4-byte (32 bit) units
#define REGB_SIZE32 40 //size of regiter B, in 4-byte (32 bit) units
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
88-89   REGA_YAW       yaw                 int16    yaw angle, in units of 1/10 degree
90-91   REGA_PITCH     pitch               int16    pitch angle, in units of 1/10 degree
92-93   REGA_ROLL      roll                int16    roll  angle, in units of 1/10 degree
94-95   REGA_ACCEL_OFFSET accelOffset[0]   int16    x-component of accel offset, in raw units
96-97                  accelOffset[1]      int16    y
98-99                  accelOffset[2]      int16    z
100-101 REGA_GYRO_OFFSET gyroOffset[0]     int16    x-component of gyro offset, in raw units
102-103                gyroOffset[1]       int16    y
104-105                gyroOffset[2]       int16    z
------- MAGNETOMETER --- ----------------------
106      REGA_MAG_STATUS magStatus         uint8
107      unused
108-109  REGA_MAG       mag[0]             int16   x-comp of magnetic field, in units of 0.92 milligauss=0.092uT
110-111                 mag[1]             int16   y
112-113                 mag[2]             int16   z
114-115 REGA_MAG_OFFSET magOffset[0]       int16   x-comp of magnetometer offset currently in use (either from calibration of copied form magUsrOffset)
116-117                magOffset[1]        int16   y
118-119                magOffset[2]        int16   z
------ GPS -----------------------------------
120     REGA_GPS_STATUS
121-123    unused
124-127 REGA_GPS_LAT   gpsLat              int32  latitude, in units of 10^{-7} degree (about 10cm)
128-131 REGA_GPS_LONG  gpsLong             int32  longitude
132-135 REGA_GPS_TIMESTAMP gpsTimestamp    uint32 timestamp of last measurement, in ms
------ DRIVE
136    REGA_DRIVE_STATUS
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
#define REGA_ACCEL_OFFSET  94
#define REGA_GYRO_OFFSET   100
#define REGA_MAG_STATUS    106
#define REGA_MAG           108
#define REGA_MAG_OFFSET    114
#define REGA_GPS_STATUS    120
#define REGA_GPS_LAT       124
#define REGA_GPS_LONG      128
#define REGA_GPS_TIMESTAMP 132
#define REGA_DRIVE_STATUS  136


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
45     unused
------ MOTOR POWER       -------------
46-47  REGB_MOTOR_POWER motorPower[0]      int16    motor power, -500...500
48-49                   motorPower[1]      int16
------ MOTOR STEERING    -------------
50-51  REGB_MOTOR_STEERING steering[0]     int16  see description in motor.h
52-53                      steering[1]     int16
54-55  unused
------ MOTOR TARGET      -------------
56-59  REGB_MOTOR_TARGET motorTarget[0]    int32   in speed PID mode: speed  in enc ticks/s
60-63                    motorTarget[1]    int32
------ IMU               -------------
64    REGB_IMU_CONFIG                      byte     bit0: is IMU active?
                                                    bits 2:1 : IMU orientation
65    unused
66-67 REGB_GYRO_OFFSET   gyroUsrOffset[0]  int16    gyro offset x, provided by user
68-69                    gyroUsrOffset[1]  int16    y
70-71                    gyroUsrOffset[2]  int16    z
72-73 REGB_ACCEL_OFFSET  accelUsrOffset[0] int16    accel offset x, provided by user
74-75                    accelUsrOffset[1] int16    y
76-77                    accelUsrOffset[2] int16    z
------ MAGNETOMETER
78    REGB_MAG_CONFIG                      byte    see definition of modes in mag.h
79     unused
80-81 REGB_MAG_OFFSET    magUsrOffset[0]   int16   x-offset for magnetometer provided by user
82-83                    magUsrOffset[1]   int16   y-offset for magnetometer
84-85                    magUsrOffset[2]   int16   z-offset for magnetometer
86-87 REGB_MAG_MATRIX    magUsrMatrix[0][0] int16   (soft iron matrix)*1000
88-89                    magUsrMatrix[0][1]  int16
90-91                    magUsrMatrix[0][2]  int16
92-93                    magUsrMatrix[1][0]  int16
94-95                    magUsrMatrix[1][1]  int16
96-97                    magUsrMatrix[1][2]  int16
98-99                    magUsrMatrix[2][0]  int16
100-101                  magUsrMatrix[2][1]  int16
102-103                  magUsrMatrix[2][2]  int16

------ GPS
104    REGB_GPS_CONFIG                     byte    1 if we need to activate gps
------ Low voltage cutoff
105    REGB_LOW_VOLTAGE   lowVoltage       uint8_t  the low voltage threshold, in units of 0.1V
                                                   if voltage is below that, internal neopixel will blink yellow
----- NEOPIXELS_CONFIG
106    REGB_NUM_PIXELS    numPixels        uint8    actual number of used neopixels (at most 255)
                                                   doesn't include internal neopixel
107    REGB_PIXEL_BRIGHTNESS  pixelBrightness uint8 neopixel brightness, 0-255
----- NEOPIXELS_COlORS
108-111 REGB_PIXEL_COLOR  pixelColor       uint32   neopixel color data  (nnRRGGBB), where nn is the pixel index (1-255)
112    REGB_PIXEL_COMMAND pixelCommand     uint8    command to start/update/stop pixels
----- TANK DRIVE
113   REGB_DRIVE_MODE           driveMode         byte
----- TANK DRIVE CONFIGURATION
114   REGB_DRIVE_MOTORCONFIG    driveMotorConfig  byte    info about which motor is left/right, etc. See details in drive.cpp
115   unused
116-117 REGB_DRIVE_MAXSPEED     driveMaxSpeed     uint16  maximal possible speed, in encoder ticks/s; determined by the motor
118-119 REGB_DRIVE_MAXTURNSPEED driveMaxTurnSPeed uint16  maximal turning speed, in deg/s
120-121 REGB_DRIVE_MINPOWER     driveMinPower     uint16  minimal power necessary for the robot to move when driving straight
122-123 unused
124-139 REGB_DRIVE_PID_COEF    drivePIDcoef       float[4] PID coefficients for driving straight using IMU
------ TANK DRIVE TARGETS
140-143 REGB_DRIVE_DISTANCE    driveDistance      int32   distance to drive, in encoder ticks. Always positive, even when going backwards
144-145 REGB_DRIVE_TURNANGLE   driveTurnAngle     int16   angle to turn, in units of 0.1 degree. Positive is clockwise
146-147 REGB_DRIVE_TARGETPOWER driveTargetPower   int16   requested  power, -500...500. For driving backwards must be negative


*/
#define REGB_ANALOG_BITMASK    0
#define REGB_SONAR_BITMASK     1
#define REGB_SONAR_TIMEOUT     2
#define REGB_SERVO             4
#define REGB_MOTOR1_PID        12
#define REGB_MOTOR2_PID        28
#define REGB_ENC_RESET         42
#define REGB_MOTOR_MODE        43
#define REGB_MOTOR_POWER       46
#define REGB_MOTOR_STEERING    50
#define REGB_MOTOR_TARGET      56
#define REGB_IMU_CONFIG        64
#define REGB_GYRO_OFFSET       66
#define REGB_ACCEL_OFFSET      72
#define REGB_MAG_CONFIG        78
#define REGB_MAG_OFFSET        80
#define REGB_MAG_MATRIX        86
#define REGB_GPS_CONFIG        104
#define REGB_LOW_VOLTAGE       105
#define REGB_NUM_PIXELS        106
#define REGB_PIXEL_BRIGHTNESS  107
#define REGB_PIXEL_COLOR       108
#define REGB_PIXEL_COMMAND     112
#define REGB_DRIVE_MODE        113
#define REGB_DRIVE_MOTORCONFIG 114
#define REGB_DRIVE_MAXSPEED    116
#define REGB_DRIVE_MAXTURNSPEED 118
#define REGB_DRIVE_MINPOWER    120
#define REGB_DRIVE_PID_COEF    124
#define REGB_DRIVE_DISTANCE    140
#define REGB_DRIVE_TURNANGLE   144
#define REGB_DRIVE_TARGETPOWER 146

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
extern volatile int16_t * accelOffset;
extern volatile int16_t * gyroOffset;
// MAGNETOMETER
extern volatile uint8_t * magStatus;
extern volatile int16_t * mag;
extern volatile int16_t * magOffset;
// GPS
extern volatile uint8_t * gpsStatus;
extern volatile int32_t * gpsLat;
extern volatile int32_t * gpsLong;
extern volatile uint32_t * gpsTimestamp;
//DRIVE
extern volatile uint8_t * driveStatus;

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
extern volatile int16_t  * steering;
extern volatile int32_t  * motorTarget;
//imu, gps, magnetometer
extern volatile uint8_t * imuConfig;
extern volatile int16_t * gyroUsrOffset;
extern volatile int16_t * accelUsrOffset;
extern volatile uint8_t * gpsConfig;
extern volatile uint8_t * magConfig;
extern volatile int16_t * magUsrOffset;
extern volatile int16_t * magUsrMatrix;
// low voltage threshold
extern volatile uint8_t * lowVoltage;
//neopixels
extern volatile uint8_t  * numPixels;
extern volatile uint8_t  * pixelBrightness;
extern volatile uint32_t * pixelColor;
extern volatile uint8_t  * pixelCommand;
//DRIVE
extern volatile uint8_t  * driveMode;
extern volatile uint8_t  * driveMotorConfig;
extern volatile uint16_t * driveMaxSpeed;
extern volatile uint16_t * driveMaxTurnSpeed;
extern volatile uint16_t * driveMinPower;
extern volatile float    * drivePIDcoef;
extern volatile int32_t  * driveDistance;
extern volatile int16_t  * driveTurnAngle;
extern volatile int16_t  * driveTargetPower;








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
#define FLAG_ANALOG        (1ul)
#define FLAG_SONAR_BITMASK ((1ul)<<1)
#define FLAG_SERVO         ((1ul)<<2)
#define FLAG_MOTOR1_PID    ((1ul)<<3)
#define FLAG_MOTOR2_PID    ((1ul)<<4)
#define FLAG_ENC_RESET     ((1ul)<<5)
#define FLAG_MOTOR_MODE    ((1ul)<<6)
#define FLAG_MOTOR_POWER   ((1ul)<<7)
#define FLAG_MOTOR_STEERING ((1ul)<<8)
#define FLAG_MOTOR_TARGET  ((1ul)<<9)
#define FLAG_IMU_CONFIG    ((1ul)<<10)
#define FLAG_GYRO_OFFSET   ((1ul)<<11)
#define FLAG_ACCEL_OFFSET  ((1ul)<<12)
#define FLAG_MAG_CONFIG    ((1ul)<<13)
#define FLAG_MAG_CALIBRATION ((1ul)<<14)
#define FLAG_GPS_CONFIG    ((1ul)<<15)
#define FLAG_LOW_VOLTAGE   ((1ul)<<16)
#define FLAG_PIXEL_CONFIG  ((1ul)<<17)
#define FLAG_PIXEL_COLOR   ((1ul)<<18)
#define FLAG_PIXEL_COMMAND ((1ul)<<19)
#define FLAG_DRIVE_MODE    ((1ul)<<20)
#define FLAG_DRIVE_CONFIG  ((1ul)<<21)
#define FLAG_DRIVE_TARGET  ((1ul)<<22)

//composite flags
//FIXME: motor steering?
#define FLAG_MOTOR_PID (FLAG_MOTOR1_PID | FLAG_MOTOR2_PID |  FLAG_MOTOR_TARGET)
//finally, function-like macros and function  declarations
#define isSet(F) (changeFlag & (uint32_t) F)
#define setFlag(F)  changeFlag |= (uint32_t) F
#define clearFlag(F) changeFlag &=~ (uint32_t)F

void initRegmap();

#endif
