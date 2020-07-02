#include "i2c.h"
#include <Wire.h>
#include "wiring_private.h"
#include "regmap.h"
#include "neopixel.h" //neopixel is treated specially
volatile uint8_t requestAddress;
// Flags

void i2cMasterBegin(int freq){
  Wire1.begin();
  Wire1.setClock(freq);
  pinPeripheral(PIN_WIRE1_SDA,PIO_SERCOM);
  pinPeripheral(PIN_WIRE1_SCL,PIO_SERCOM);
}

void i2cMasterWriteByte(uint8_t address, uint8_t regAddress, uint8_t data) {
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  Wire1.write(regAddress);           // Put slave register address in Tx buffer
  Wire1.write(data);                 // Put data in Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}

uint8_t i2cMasterReadByte(uint8_t address, uint8_t regAddress) {
  uint8_t data; // `data` will store the register data
  Wire1.beginTransmission(address);         // Initialize the Tx buffer
  Wire1.write(regAddress);	                 // Put slave register address in Tx buffer
  Wire1.endTransmission();             // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire1.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void i2cMasterReadBytes(uint8_t address, uint8_t regAddress, uint8_t count, uint8_t * dest) {
  Wire1.beginTransmission(address);   // Initialize the Tx buffer
  Wire1.write(regAddress);            // Put slave register address in Tx buffer
  Wire1.endTransmission();       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire1.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire1.available()) {
    dest[i++] = Wire1.read();
  }         // Put read results in the Rx buffer
}

void i2cSlaveBegin(){
  Wire.begin(SLAVE_ADDRESS);
  pinPeripheral(PIN_WIRE_SDA,PIO_SERCOM);
  pinPeripheral(PIN_WIRE_SCL,PIO_SERCOM);
  Wire.onRequest(i2cSlaveRequestEvent);
  Wire.onReceive(i2cSlaveReceiveEvent);
}

void i2cSlaveRequestEvent(){
  //put maximum possible number of bytes in the buffer - the master will stop transimssion
  //after reading as many as it needs
  //start at offset requestAddress - the one received from master at last transimssion
  // All reads use REGA
  Wire.write((char *)REGA+requestAddress, MAX_TRANSMIT_SIZE);
  //Serial.print("Sent bytes starting at offset "); Serial.println(requestAddress);
}

void i2cSlaveReceiveEvent(int bytesReceived){
  uint8_t offset=Wire.read();//get the register offset, always first byte sent
  if (bytesReceived>1) {
    //this was to write data to register B
    for (int i=0; i<bytesReceived-1; i++){
      REGB[offset+i]=Wire.read();
    }
    //Serial.print("Wrote "); Serial.print(bytesReceived-1); Serial.print(" bytes to register "); Serial.println(offset);
    if (offset==REGB_PIXEL_COLOR){
      //let's deal with it right away, before it gets overwritten
      uint32_t c=(REGB[REGB_PIXEL_COLOR+2]<<16)|(REGB[REGB_PIXEL_COLOR+1]<<8)|(REGB[REGB_PIXEL_COLOR]); //RGB
      uint8_t n=(uint8_t)(REGB[REGB_PIXEL_COLOR+3]); //pixel index
      pixelColorArray[n]=c;
    } else {
      //set flag and let the main loop deal with it
      setFlag(registerFlag[offset]);
      //Serial.print("received data at offset ");Serial.println(offset);
    }
  } else {
    requestAddress=offset; //save for request handler
  }
}
