#ifndef _ROVERWING_GPS_H
#define _ROVERWING_GPS_H
#include <Arduino.h>
#include <NMEAGPS.h>
#define gpsPort SerialGPS
#define GPS_PORT_NAME "SerialGPS"
#define DEBUG_PORT Serial
//various statuses, for variable *gpsStatus
#define GPS_OFF 0
#define GPS_WAITING 1 //waiting for fix
#define GPS_OK 2      // has fix
//timeout - if we do no get new location fix in that time, we decide that we lost signal
// in ms
#define GPS_TIMEOUT 3000
void GPSbegin();
//reads GPS data and saves it to register; also updates *gpsStatus
void GPSupdate();
void GPSend();




#endif
