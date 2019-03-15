#include "regmap.h"
#include "i2c.h"
#include "mag.h"
bool magBegin(){
  if ( i2cMasterReadByte(MAG_ADDRESS, MAG_REG_IRA)!= 72 ){
    //reading ID register didn't produce correct value - abort
    //Serial.println(i2cMasterReadByte(MAG_ADDRESS, MAG_REG_IRA));
    *magStatus=MAG_STATUS_OFF;
    return false;
  }
  // Enable the magnetometer
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_CRA, MAG_MEASUREMENT_CONFIG);
  // Set the gain to a known level
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_CRB, MAG_GAIN);
  //configure mode
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_MR, MAG_CONTINUOUS_MODE);
  *magStatus=MAG_STATUS_ON;
  return true;
}
void magEnd(){
  i2cMasterWriteByte(MAG_ADDRESS, MAG_REG_MR, MAG_IDLE_MODE);
  *magStatus = 0x00;
}
void magUpdate(){
  byte buf[6];
  int16_t xRaw, yRaw, zRaw;
  //read 6 bytes
  i2cMasterReadBytes(MAG_ADDRESS,MAG_REG_OUT_X_H, 6, (uint8_t *) buf);
  xRaw=(int16_t)(buf[0]<<8)|buf[1];
  zRaw=(int16_t)(buf[2]<<8)|buf[3];
  yRaw=(int16_t)(buf[4]<<8)|buf[5];
  mag[0]=xRaw-magOffset[0];
  mag[1]=yRaw-magOffset[1];
  mag[2]=zRaw-magOffset[2];
}
void magApplyUsrOffsets(){
  for (int i=0; i<3;i++){
    magOffset[i]=magUsrOffset[i];
  }
}
void magCalibrate(){
  int16_t xMax,xMin,yMax,yMin,zMax,zMin;
  uint32_t endTime=millis()+20000; //time to stop, in ms
  *magStatus=MAG_STATUS_CALIBRATING; //set the status
  //clear previous offsets if any
  magOffset[0]=0;magOffset[1]=0;magOffset[2]=0;
  //get first reading
  magUpdate();
  xMax=mag[0]; xMin=mag[0];
  yMax=mag[1]; yMin=mag[1];
  zMax=mag[2]; zMin=mag[2];
  while (millis()<endTime){
    magUpdate();
    if (mag[0]>xMax) xMax=mag[0];
    else if (mag[0]<xMin) xMin=mag[0];
    if (mag[1]>yMax) yMax=mag[1];
    else if (mag[1]<yMin) yMin=mag[1];
    if (mag[2]>zMax) zMax=mag[2];
    else if (mag[2]<zMin) zMin=mag[2];
  }
  /*Serial.print("xMax: "); Serial.println(xMax);
  Serial.print("xMin: "); Serial.println(xMin);
  Serial.print("yMax: "); Serial.println(yMax);
  Serial.print("yMin: "); Serial.println(yMin);
  Serial.print("zMax: "); Serial.println(zMax);
  Serial.print("zMin: "); Serial.println(zMin);*/
  magOffset[0]=(xMax+xMin)/2;
  magOffset[1]=(yMax+yMin)/2;
  magOffset[2]=(zMax+zMin)/2;
  /*Serial.print("Calibration completed. Offsets are: ");
  Serial.print(offset_x); Serial.print(" "); Serial.print(offset_y);
  Serial.print(" "); Serial.println(offset_z);*/
  *magStatus=MAG_STATUS_ON;
}
