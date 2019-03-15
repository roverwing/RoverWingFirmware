#include "Arduino.h"
#include "Wire.h"
#define MPU6050_ADDRESS 0x69  // Device address when ADO = 1
// Set initial input parameters
#define ASCALE 0x00 //accelereation scale: 0x00 is 2g max
#define GSCALE 0x00 //gyro scale: 0x00 is 250 deg/s
const float gRes = 250.0f / 32768.0f; //gyro resolution, in (deg/s)/LSB
const float gResRad = gRes * PI/180.0f; //gyro resolution, in (rad/s)/LSB
const float aRes = 2.0f/32768.0f;    //accel resolution, in g/LSB
const float radToDeg=180.0f/PI;      //converstion factor from radians to degrees s
const float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
const float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
const float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
const float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


//Registers. Note that thsi si not a complete list, only those that we need

#define MPU6050_REG_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define MPU6050_REG_XA_OFFSET_L_TC   0x07
#define MPU6050_REG_YA_OFFSET_H      0x08
#define MPU6050_REG_YA_OFFSET_L_TC   0x09
#define MPU6050_REG_ZA_OFFSET_H      0x0A
#define MPU6050_REG_ZA_OFFSET_L_TC   0x0B
#define MPU6050_REG_SELF_TEST_X      0x0D
#define MPU6050_REG_SELF_TEST_Y      0x0E
#define MPU6050_REG_SELF_TEST_Z      0x0F
#define MPU6050_REG_SELF_TEST_A      0x10
#define MPU6050_REG_XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define MPU6050_REG_XG_OFFS_USRL     0x14
#define MPU6050_REG_YG_OFFS_USRH     0x15
#define MPU6050_REG_YG_OFFS_USRL     0x16
#define MPU6050_REG_ZG_OFFS_USRH     0x17
#define MPU6050_REG_ZG_OFFS_USRL     0x18
#define MPU6050_REG_SMPLRT_DIV       0x19
#define MPU6050_REG_CONFIG           0x1A
#define MPU6050_REG_GYRO_CONFIG      0x1B
#define MPU6050_REG_ACCEL_CONFIG     0x1C
#define MPU6050_REG_FIFO_EN          0x23
#define MPU6050_REG_I2C_MST_CTRL     0x24
#define MPU6050_REG_INT_PIN_CFG      0x37
#define MPU6050_REG_INT_ENABLE       0x38
#define MPU6050_REG_INT_STATUS       0x3A
#define MPU6050_REG_ACCEL_XOUT_H     0x3B
#define MPU6050_REG_ACCEL_XOUT_L     0x3C
#define MPU6050_REG_ACCEL_YOUT_H     0x3D
#define MPU6050_REG_ACCEL_YOUT_L     0x3E
#define MPU6050_REG_ACCEL_ZOUT_H     0x3F
#define MPU6050_REG_ACCEL_ZOUT_L     0x40
#define MPU6050_REG_GYRO_XOUT_H      0x43
#define MPU6050_REG_GYRO_XOUT_L      0x44
#define MPU6050_REG_GYRO_YOUT_H      0x45
#define MPU6050_REG_GYRO_YOUT_L      0x46
#define MPU6050_REG_GYRO_ZOUT_H      0x47
#define MPU6050_REG_GYRO_ZOUT_L      0x48
#define MPU6050_REG_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define MPU6050_REG_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define MPU6050_REG_PWR_MGMT_2       0x6C
#define MPU6050_REG_FIFO_COUNTH      0x72
#define MPU6050_REG_FIFO_COUNTL      0x73
#define MPU6050_REG_FIFO_R_W         0x74
#define MPU6050_REG_WHO_AM_I         0x75 // Should return 0x68


//checks if MPU6050 is available
//assumes Wire1 has already been started
bool MPU6050isAvailable();
// configures and calibrates MPU6050
//retunrs 1 on success, 0 on failure
bool MPU6050begin();
void readAccelData();
void readGyroData();

//need to be called regularly - as frequently as possible - to update the orientation;
//saves accel, gyro, and orientation to regmap
void MPU6050update();
// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
// In this coordinate system, the positive z-axis is down toward Earth.
// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
// applied in the correct order which for this configuration is yaw, pitch, and then roll.
// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
float getYaw();
float getPitch();
float getRoll();
//print IMU readings to serial monitor, for debugging
void MPU6050print();
void MPU6050SelfTest(float * destination);
void _MadgwickQuaternionUpdate(float deltat);
