#include "i2c.h"
#include "MPU6050.h"
#include "regmap.h"


//for keeping track of time
float IMUdeltat = 0.0f;                              // integration time interval
uint32_t IMUlastUpdate=0;



bool MPU6050isAvailable(){
  uint8_t c = i2cMasterReadByte(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I);  // Read WHO_AM_I register for MPU-6050
  return (bool) (c == 0x68);
}

bool MPU6050begin() {
  quat[0]=1.0f; quat[1]=0.0f;  quat[2]=0.0f; quat[3]=0.0f;
  if (!MPU6050isAvailable()) {
    Serial.println("Failed to connect to MPU050");
    *imuStatus = IMU_ERROR;
    return false;
  }
  // Initialization proper
  // wake up device-don't need this here if using calibration function below
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
 // reset device, reset all registers, clear gyro and accelerometer bias registers
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  i2cMasterReadByte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG);
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, c | GSCALE << 3); // Set full scale range for the gyro

  // Set accelerometer configuration
  c =  i2cMasterReadByte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG);
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, c | ASCALE << 3); // Set full scale range for the accelerometer

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_INT_PIN_CFG, 0x22);
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

  //finishing up
  *imuStatus = IMU_OK;
  return true;
}

void MPU6050calibrate(){
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  *imuStatus = IMU_CALIBRATING;
  // reset device, reset all registers, clear gyro and accelerometer bias registers
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x01);
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_INT_ENABLE, 0x00);   // Disable all interrupts
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_FIFO_EN, 0x00);      // Disable FIFO
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_I2C_MST_CTRL, 0x00); // Disable I2C master
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_USER_CTRL, 0x40);   // Enable FIFO
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  i2cMasterWriteByte(MPU6050_ADDRESS, MPU6050_REG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  i2cMasterReadBytes(MPU6050_ADDRESS, MPU6050_REG_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    i2cMasterReadBytes(MPU6050_ADDRESS, MPU6050_REG_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }
  //finally, save to global variables
  for (ii=0; ii<3; ii++){
    accelOffset[ii]=accel_bias[ii];
    gyroOffset[ii]=gyro_bias[ii];
  }
  *imuStatus = IMU_OK;

}

void readAccelData() {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  i2cMasterReadBytes(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  accel[0] = (int16_t)((rawData[0] << 8) | rawData[1]) - accelOffset[0];  // Turn the MSB and LSB into a signed 16-bit value
  accel[1] = (int16_t)((rawData[2] << 8) | rawData[3]) - accelOffset[1];
  accel[2] = (int16_t)((rawData[4] << 8) | rawData[5]) - accelOffset[2];
}
void readGyroData() {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  i2cMasterReadBytes(MPU6050_ADDRESS, MPU6050_REG_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gyro[0] = (int16_t)((rawData[0] << 8) | rawData[1]) - gyroOffset[0];  // Turn the MSB and LSB into a signed 16-bit value
  gyro[1] = (int16_t)((rawData[2] << 8) | rawData[3]) - gyroOffset[1];
  gyro[2] = (int16_t)((rawData[4] << 8) | rawData[5]) - gyroOffset[2];
}
void MPU6050update(){
  uint32_t Now; //timestamp in us
  // If data ready bit set, all data registers have new data
  if(i2cMasterReadByte(MPU6050_ADDRESS, MPU6050_REG_INT_STATUS) & 0x01) {  // check if data ready interrupt

    readAccelData();  // Read the x/y/z adc values into register map
    readGyroData();  // Read the x/y/z adc values
  }

  Now = micros();
  IMUdeltat = ((Now - IMUlastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  IMUlastUpdate = Now;
  _MadgwickQuaternionUpdate(IMUdeltat);
}

float getYaw() {
  float a;
  a=  - atan2f(2.0f * (quat[1] * quat[2] + quat[0] * quat[3]), quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] - quat[3] * quat[3]);
  return (a*radToDeg);
}
float getPitch() {
  float a;
  a = -asinf(2.0f * (quat[1] * quat[3] - quat[0] * quat[2]));
  return (a*radToDeg);
}
float getRoll(){
  float a;
  a  = - atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]), quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3]);
  return (a*radToDeg);
}

void MPU6050print(){
  /* Serial.print("Quat: ");
  Serial.print(quat[0]); Serial.print('\t');
  Serial.print(quat[1]); Serial.print('\t');
  Serial.print(quat[2]); Serial.print('\t');
  Serial.println(quat[3]); */

  Serial.println(" x\t  y\t  z  ");

  Serial.print((int)(1000.0f * accel[0]*aRes)); Serial.print('\t');
  Serial.print((int)(1000.0f * accel[1]*aRes)); Serial.print('\t');
  Serial.print((int)(1000.0f * accel[2]*aRes));
  Serial.println(" mg");

  Serial.print((int)(gyro[0]*gRes)); Serial.print('\t');
  Serial.print((int)(gyro[1]*gRes)); Serial.print('\t');
  Serial.print((int)(gyro[2]*gRes));
  Serial.println(" o/s");

  Serial.print((*yaw)/100.0f); Serial.print('\t');
  Serial.print((*pitch)/100.0f); Serial.print('\t');
  Serial.print((*roll)/100.0f);
  Serial.println(" ypr");
}

void _MadgwickQuaternionUpdate(float deltat) {
  float q1 = quat[0], q2 = quat[1], q3 = quat[2], q4 = quat[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float ax, ay, az, gx, gy, gz;     // accel in g/s; gyro in rad/s
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;

  ax = (float)accel[0]*aRes ;  // get actual g value, this depends on scale being set
  ay = (float)accel[1]*aRes ;
  az = (float)accel[2]*aRes ;

  gx = (float)gyro[0]*gResRad ;
  gy = (float)gyro[1]*gResRad ;
  gz = (float)gyro[2]*gResRad ;


  // Normalise accelerometer measurement
  float accnorm = sqrtf(ax * ax + ay * ay + az * az);
  if (accnorm == 0.0f) return; // handle NaN
  accnorm = 1.0f/accnorm;
  /*if (isnan(accnorm)){
    Serial.print("Accnorm is NAN!  acc : ");
    Serial.print(ax*1000); Serial.print('\t');
    Serial.print(ay*1000); Serial.print('\t');
    Serial.println(az*1000);
    Serial.print("Raw values: ");
    Serial.print(accel[0]); Serial.print('\t');
    Serial.print(accel[1]); Serial.print('\t');
    Serial.println(accel[2]);
    Serial.print("Recompute: ");
    ax = (float)accel[0]*aRes ;  // get actual g value, this depends on scale being set
    ay = (float)accel[1]*aRes ;
    az = (float)accel[2]*aRes ;
    Serial.print(ax*1000); Serial.print('\t');
    Serial.print(ay*1000); Serial.print('\t');
    Serial.println(az*1000);
  }*/
  ax *= accnorm;
  ay *= accnorm;
  az *= accnorm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  float hatnorm = sqrtf(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);

  hatDot1 /= hatnorm;
  hatDot2 /= hatnorm;
  hatDot3 /= hatnorm;
  hatDot4 /= hatnorm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  gx -= gbiasx;
  gy -= gbiasy;
  gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * deltat;
  q2 += (qDot2 -(beta * hatDot2)) * deltat;
  q3 += (qDot3 -(beta * hatDot3)) * deltat;
  q4 += (qDot4 -(beta * hatDot4)) * deltat;
  /*Serial.print("q1 : ");
  Serial.print(q1); Serial.print('\t');
  Serial.print(q2); Serial.print('\t');
  Serial.print(q3); Serial.print('\t');
  Serial.println(q4);*/
  /*if (isnan(q1)) {
    //stop now
    Serial.println("Not a number!!!");
    Serial.print("hatnorm: "); Serial.println(hatnorm*1000);
    Serial.print("qDot1: "); Serial.println(qDot1);

    Serial.print("hatDot1: "); Serial.println(hatDot1);
    //print all arguments

    Serial.print("acc*1000 : ");
    Serial.print(ax*1000); Serial.print('\t');
    Serial.print(ay*1000); Serial.print('\t');
    Serial.println(az*1000);

    Serial.print("gyro : ");
    Serial.print(gx); Serial.print('\t');
    Serial.print(gy); Serial.print('\t');
    Serial.println(gz);

    Serial.print("q : ");
    Serial.print(quat[0]); Serial.print('\t');
    Serial.print(quat[1]); Serial.print('\t');
    Serial.print(quat[2]); Serial.print('\t');
    Serial.println(quat[3]);

    Serial.print("Deltat: "); Serial.println(deltat*1000);
    while(1);
  } */
  // Normalize the quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion

  norm = 1.0f/norm;
  quat[0] = q1 * norm;
  quat[1] = q2 * norm;
  quat[2] = q3 * norm;
  quat[3] = q4 * norm;
}
