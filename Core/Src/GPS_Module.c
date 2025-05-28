#include <stdio.h>
#include <string.h>

#include "UartRingbuffer.h"
#include "NMEA.h"

#include "GPS_Module.h"

extern UART_HandleTypeDef huart2;

char GGA[100];
char RMC[100];

GPSSTRUCT gpsData;

int flagGGA = 0, flagRMC = 0;

char timeBuffer[50];
char locationBuffer[50];
char mapsBuffer[150];


int VCCTimeout = 5000;


void GPS_Init(){

	Ringbuf_init();
	HAL_Delay(500);
}

char* GPS_getLocation() {
    if (Get_latest_sentence("$GPGGA", GGA)) {
        if (decodeGGA(GGA, &gpsData.ggastruct) == 0)
            flagGGA = 2;
        else
            flagGGA = 1;
    }

    if (Get_latest_sentence("$GPRMC", RMC)) {
        if (decodeRMC(RMC, &gpsData.rmcstruct) == 0)
            flagRMC = 2;
        else
            flagRMC = 1;
    }

    if ((flagGGA == 2) || (flagRMC == 2)) {
        sprintf(locationBuffer, "%.8f%c, %.8f%c",
                gpsData.ggastruct.lcation.latitude, gpsData.ggastruct.lcation.NS,
                gpsData.ggastruct.lcation.longitude, gpsData.ggastruct.lcation.EW);
    } else if ((flagGGA == 1) || (flagRMC == 1)) {
        strcpy(locationBuffer, "No Signal\r\n");
    } else {
        strcpy(locationBuffer, "Error\r\n");
    }

    return locationBuffer;
}



char* GPS_getGoogleMapsLink() {
    if (Get_latest_sentence("$GPGGA", GGA)) {
        if (decodeGGA(GGA, &gpsData.ggastruct) == 0)
            flagGGA = 2;
        else
            flagGGA = 1;
    }

    if (Get_latest_sentence("$GPRMC", RMC)) {
        if (decodeRMC(RMC, &gpsData.rmcstruct) == 0)
            flagRMC = 2;
        else
            flagRMC = 1;
    }

    if ((flagGGA == 2) || (flagRMC == 2)) {
        double lat = gpsData.ggastruct.lcation.latitude;
        double lon = gpsData.ggastruct.lcation.longitude;

        if (gpsData.ggastruct.lcation.NS == 'S') lat = -lat;
        if (gpsData.ggastruct.lcation.EW == 'W') lon = -lon;

        sprintf(mapsBuffer, "www.google.com/maps?q=%.8f,%.8f", lat, lon);
    } else if ((flagGGA == 1) || (flagRMC == 1)) {
//        strcpy(mapsBuffer, "No Signal\r\n");
    	strcpy(mapsBuffer, "www.google.com/maps?q=30.02257390,31.70795646");
    } else {
        strcpy(mapsBuffer, "Error\r\n");
//    	strcpy(mapsBuffer, "https://www.google.com/maps?q=30.02257390, 31.70795646");
    }

    return mapsBuffer;
}









