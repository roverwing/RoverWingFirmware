#include "i2c.h"
#include "ICM42605.h"
#include "regmap.h"


//for keeping track of time
float IMUdeltat = 0.0f;                              // integration time interval
uint32_t IMUlastUpdate=0;



bool ICM42605isAvailable(){
  uint8_t c = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_WHO_AM_I);  // Read WHO_AM_I register for ICM42605
  return (bool) (c == 0x42);
}

bool ICM42605begin() {
  quat[0]=1.0f; quat[1]=0.0f;  quat[2]=0.0f; quat[3]=0.0f;
  if (!ICM42605isAvailable()) {
    Serial.println("Failed to connect to MPU050");
    *imuStatus = IMU_ERROR;
    return false;
  }

  uint8_t temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0); // make sure not to distrub reserved bit values
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0, temp | 0x0F);  // enable gyro and accel in low noise mode

   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0, temp | GODR | GSCALE << 5); // gyro full scale and data rate

   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0, temp | AODR | ASCALE << 5); // set accel full scale and data rate

   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1, temp | 0xD0); // set temperature sensor low pass filter to 5Hz, use first order gyro filter
/*
   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG, temp | 0x18 | 0x03 ); // set both interrupts active high, push-pull, pulsed

   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2

   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0, temp | 0x08 ); // route data ready interrupt to INT1

   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3, temp | 0x01 ); // route AGC interrupt interrupt to INT2
*/
  // Select Bank 4
   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp | 0x04 ); // select Bank 4

     temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5, temp & ~(0x07) ); // select unitary mounting matrix

   temp = i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL);
  i2cMasterWriteByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp & ~(0x07) ); // select Bank 0



  //finishing up
  *imuStatus = IMU_OK;
  return true;
}


void readAccelData() {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  i2cMasterReadBytes(ICM42605_ADDRESS, ICM42605_ACCEL_DATA_X1, 6, &rawData[0]);  // Read the six raw data registers into data array
  accel[0] = (int16_t)((rawData[0] << 8) | rawData[1]) - accelOffset[0];  // Turn the MSB and LSB into a signed 16-bit value
  accel[1] = (int16_t)((rawData[2] << 8) | rawData[3]) - accelOffset[1];
  accel[2] = (int16_t)((rawData[4] << 8) | rawData[5]) - accelOffset[2];
}
void readGyroData() {
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  i2cMasterReadBytes(ICM42605_ADDRESS, ICM42605_GYRO_DATA_X1, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gyro[0] = (int16_t)((rawData[0] << 8) | rawData[1]) - gyroOffset[0];  // Turn the MSB and LSB into a signed 16-bit value
  gyro[1] = (int16_t)((rawData[2] << 8) | rawData[3]) - gyroOffset[1];
  gyro[2] = (int16_t)((rawData[4] << 8) | rawData[5]) - gyroOffset[2];
}

void ICM42605calibrate(){
  uint8_t ii;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  *imuStatus = IMU_CALIBRATING;
  //zero the offsets
  for (ii=0; ii<3;ii++){
      accelOffset[ii]=0;
      gyroOffset[ii]=0;
  }

  for (ii = 0; ii < 128; ii++) {
    readAccelData();
    accel_bias[0] += accel[0];
    accel_bias[1] += accel[1];
    accel_bias[2] += accel[2];
    readGyroData();
    gyro_bias[0] += gyro[0];
    gyro_bias[1] += gyro[1];
    gyro_bias[2] += gyro[2];
    delay(50);
}
  accel_bias[0] /= 128; // Normalize sums to get average count biases
  accel_bias[1] /= 128;
  accel_bias[2] /= 128;
  //Serial.print("Accel bias (z): "); Serial.println(accel_bias[2]);
  gyro_bias[0] /= 128;
  gyro_bias[1] /= 128;
  gyro_bias[2] /= 128;


  //remove gravity
  if (accel_bias[0] > G/2)  {
    accel_bias[0] -= G; // Remove gravity from the x-axis accelerometer bias calculation
  } else if (accel_bias[0] < -G/2) {
    accel_bias[0] += G; // Remove gravity from the x-axis accelerometer bias calculation
  }

  if (accel_bias[1] > G/2)  {
    accel_bias[1] -= G; // Remove gravity from the y-axis accelerometer bias calculation
  } else if (accel_bias[1] < -G/2) {
    accel_bias[1] += G; // Remove gravity from the y-axis accelerometer bias calculation
  }
  //remove gravity
  if (accel_bias[2] > G/2)  {
    accel_bias[2] -= G; // Remove gravity from the z-axis accelerometer bias calculation
  } else if (accel_bias[2] < -G/2) {
    accel_bias[2] += G; // Remove gravity from the z-axis accelerometer bias calculation
  }
  

  //save offsets
  for (ii=0; ii<3; ii++){
    accelOffset[ii]=accel_bias[ii];
    gyroOffset[ii]=gyro_bias[ii];
  }
  *imuStatus = IMU_OK;

}

void ICM42605update(){
  uint32_t Now; //timestamp in us
  // If data ready bit set, all data registers have new data
  if(i2cMasterReadByte(ICM42605_ADDRESS, ICM42605_INT_STATUS) & 0x08) {  // check  data ready interrupt
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

void ICM42605print(){
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

__attribute__((optimize("O3"))) void _MadgwickQuaternionUpdate(float deltat) {
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
