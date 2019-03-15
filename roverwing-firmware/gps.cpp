#include "gps.h"
#include "regmap.h"
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
uint32_t fixTimestamp; //timestamp of last time we got a fix, in ms
void GPSbegin(){
  gpsPort.begin(9600);
  *gpsStatus = GPS_WAITING;
  *gpsLat =0;
  *gpsLong=0;
}
void GPSend(){
  gpsPort.end();
  *gpsStatus = GPS_OFF;
}
void GPSupdate(){
  while (gps.available( gpsPort )) {
    fix = gps.read();
    if (fix.valid.location) {
      *gpsLat=fix.latitudeL();
      *gpsLong=fix.longitudeL();
      *gpsTimestamp=millis();
      *gpsStatus=GPS_OK;
    }
  }
  //check - if we haven't received updates in a while, it means we lost the signal...
  if (millis()-*gpsTimestamp>GPS_TIMEOUT) {
    *gpsStatus=GPS_WAITING;
  }

}
