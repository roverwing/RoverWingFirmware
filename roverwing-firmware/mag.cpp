#include "regmap.h"
#include "i2c.h"
#include "mag.h"

int16_t magSoftIronMatrix[3][3]; // this is the soft iron transformation matrix *1000 (for performance reasons, we work with int rather than floats )

bool magBegin(){
  int i,j;
  if ( i2cMasterReadByte(MAG_ADDRESS, MAG_REG_IRA)!= 72 ){
    //reading ID register didn't produce correct value - abort
    //Serial.println(i2cMasterReadByte(MAG_ADDRESS, MAG_REG_IRA));
    *magStatus=MAG_OFF;
    return false;
  }
  //reset calibration
  magResetCalData();
  // Enable the magnetometer
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_CRA, MAG_MEASUREMENT_CONFIG);
  // Set the gain to a known level
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_CRB, MAG_GAIN);
  //configure mode
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_MR, MAG_CONTINUOUS_MODE);
  *magStatus=MAG_OK;
  return true;
}
void magEnd(){
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_MR, MAG_IDLE_MODE);
  *magStatus = MAG_OFF;
}
void magUpdate(){
  byte buf[6];
  int16_t xRaw, yRaw, zRaw;
  //read 6 bytes
  i2cMasterReadBytes(MAG_ADDRESS,MAG_REG_OUT_X_H, 6, (uint8_t *) buf);
  xRaw=(int16_t)(buf[0]<<8)|buf[1];
  zRaw=(int16_t)(buf[2]<<8)|buf[3];
  yRaw=(int16_t)(buf[4]<<8)|buf[5];
  //apply offsets
  xRaw-=magOffset[0];
  yRaw-=magOffset[1];
  zRaw-=magOffset[2];
  //apply transformation matrix, and remember that it needs to be divided by 1000
  mag[0]=((int32_t)magSoftIronMatrix[0][0]*xRaw + magSoftIronMatrix[0][1]*yRaw + magSoftIronMatrix[0][2]*zRaw)/1000;
  mag[1]=((int32_t)magSoftIronMatrix[1][0]*xRaw + magSoftIronMatrix[1][1]*yRaw + magSoftIronMatrix[1][2]*zRaw)/1000;
  mag[2]=((int32_t)magSoftIronMatrix[2][0]*xRaw + magSoftIronMatrix[2][1]*yRaw + magSoftIronMatrix[2][2]*zRaw)/1000;
}
void magCalibrate(){
  //first, set status
  *magStatus=MAG_CALIBRATING;
  //reset calibration
  magResetCalData();
  //now, really begin calibration
  uint32_t endTime=millis()+20000; //time to stop, in ms
  int16_t m1[3];
  int16_t m2[3];
  int16_t dm[3];
  float n1,n2, dn;  // lengths of m1, m2, dm respectively
  float t;
  while (millis()<endTime){
    magUpdate();
    //save the results in m1:
    for (int i=0; i<3; i++) m1[i]=mag[i];
    delay(200);//give time to rotate
    magUpdate();
    //save the results in m2:
    for (int i=0; i<3; i++) m2[i]=mag[i];
    //compute difference
    dm[0]=m2[0]-m1[0]; dm[1]=m2[1]-m1[1];dm[2]=m2[2]-m1[2];
    n1=sqrtf(1.0f*m1[0]*m1[0] +m1[1]*m1[1]+m1[2]*m1[2]);
    n2=sqrtf(1.0f*m2[0]*m2[0] +m2[1]*m2[1]+m2[2]*m2[2]);
    dn=sqrtf(1.0f*dm[0]*dm[0] +dm[1]*dm[1]+dm[2]*dm[2]);
    if (dn>100.0){
      t=0.3*(n2-n1)/dn;
      //update offset values
      magOffset[0]+=(int16_t)(dm[0]*t);
      magOffset[1]+=(int16_t)(dm[1]*t);
      magOffset[2]+=(int16_t)(dm[2]*t);
      //Serial.print(offset_x); Serial.print(" "); Serial.print(offset_y);
      //Serial.print(" "); Serial.println(offset_z);
    }
  }
  //set status, to signal we are done calibrating
  *magStatus=MAG_OK;
}
void magSetCalData(){
  int i,j;
  for (i=0; i<3;i++){
    magOffset[i]=magUsrOffset[i];
  }
  for (i=0;i<3;i++){
    for (j=0;j<3; j++){
      magSoftIronMatrix[i][j]=(int16_t)magUsrMatrix[3*i+j];
      //Serial.println(magSoftIronMatrix[i][j]);
    }
  }
}
void magResetCalData(){
  int i,j;
  for (i=0; i<3; i++){
    magOffset[i]=0;
  }
  for (i=0;i<3;i++){
    for (j=0;j<3; j++){
      magSoftIronMatrix[i][j]=0;
    }
  }
  for (i=0; i<3; i++){
    magSoftIronMatrix[i][i]=1000;
  }

}
