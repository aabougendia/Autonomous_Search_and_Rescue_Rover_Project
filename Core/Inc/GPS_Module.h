#ifndef INC_GPS_MODULE_H_
#define INC_GPS_MODULE_H_

#include "NMEA.h"


extern char GGA[100];
extern char RMC[100];

extern GPSSTRUCT gpsData;

extern int flagGGA, flagRMC;

extern char timeBuffer[50];
extern char locationBuffer[50];
extern char mapsBuffer[150];

extern int VCCTimeout;


void GPS_Init();
char* GPS_getLocation();
char* GPS_getGoogleMapsLink();

#endif /* INC_GPS_MODULE_H_ */
