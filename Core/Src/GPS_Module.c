/*
 * GPS_Module.c
 *
 *  Created on: Apr 25, 2025
 *      Author: Asteroid
 */


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

//
//char* GPS_getLocation(){
//
//	if(Wait_for("GGA") == 1){
//
//		  VCCTimeout = 5000;
//
//		  Copy_upto("*", GGA);
//
//		  if(decodeGGA(GGA, &gpsData.ggastruct) == 0)
//			 flagGGA = 2;
//		  else
//			 flagGGA = 1;
//	  }
//
//	  if(Wait_for("RMC") == 1){
//
//		  VCCTimeout = 5000;
//
//		  Copy_upto("*", RMC);
//
//		  if(decodeRMC(RMC, &gpsData.rmcstruct) == 0)
//			  flagRMC = 2;
//		  else
//			  flagRMC = 1;
//	  }
//
//	  if((flagGGA == 2) | (flagRMC == 2)){
//
//		  sprintf(locationBuffer, "%.8f%c, %.8f%c", gpsData.ggastruct.lcation.latitude, gpsData.ggastruct.lcation.NS,
//				  gpsData.ggastruct.lcation.longitude, gpsData.ggastruct.lcation.EW);
//	  }
//
//	  else if ((flagGGA == 1) | (flagRMC == 1)){
//		  strcpy(locationBuffer, "No Signal\r\n");
//	  }
//	  else {
//		 strcpy(locationBuffer, "Error\r\n");
//	  }
//
//	  if (VCCTimeout <= 0){
//
//		  VCCTimeout = 5000;
//
//		  flagGGA = 0;
//		  flagRMC = 0;
//
//		  strcpy(locationBuffer, "VCC Issue\r\n");
//
//	  }
//
//	  return locationBuffer;
//}


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


//char* GPS_getGoogleMapsLink(){
//
//	if(Wait_for("GGA") == 1){
//
//		  VCCTimeout = 5000;
//
//		  Copy_upto("*", GGA);
//
//		  if(decodeGGA(GGA, &gpsData.ggastruct) == 0)
//			 flagGGA = 2;
//		  else
//			 flagGGA = 1;
//	  }
//
//	  if(Wait_for("RMC") == 1){
//
//		  VCCTimeout = 5000;
//
//		  Copy_upto("*", RMC);
//
//		  if(decodeRMC(RMC, &gpsData.rmcstruct) == 0)
//			  flagRMC = 2;
//		  else
//			  flagRMC = 1;
//	  }
//
//	  if((flagGGA == 2) | (flagRMC == 2)){
//
//		  double latitude = gpsData.ggastruct.lcation.latitude;
//		  double longitude = gpsData.ggastruct.lcation.longitude;
//
//		  if (gpsData.ggastruct.lcation.NS == 'S') latitude = -latitude;
//		  if (gpsData.ggastruct.lcation.EW == 'W') longitude = -longitude;
//
//		  memset(mapsBuffer, 0, sizeof(mapsBuffer));
//		  sprintf(mapsBuffer, "https://www.google.com/maps?q=%.8f,%.8f", latitude, longitude);
//
//	  }
//
//	  else if ((flagGGA == 1) | (flagRMC == 1)){
////		  strcpy(mapsBuffer, "No Signal\r\n");
//		  strcpy(mapsBuffer, "https://www.google.com/maps?q=30.02211649, 31.70810548");
//	  }
//	  else {
//		 strcpy(mapsBuffer, "Error\r\n");
//	  }
//
//	  if (VCCTimeout <= 0){
//
//		  VCCTimeout = 5000;
//
//		  flagGGA = 0;
//		  flagRMC = 0;
//
//		  strcpy(mapsBuffer, "VCC Issue\r\n");
//
//	  }
//
//	  return mapsBuffer;
//}




char* GPS_getGoogleMapsLink() {

	Debug_Print_Buffer();

	HAL_UART_Transmit(&huart2, (uint8_t*)"CHECK GGA\r\n", strlen("CHECK GGA\r\n"), HAL_MAX_DELAY);
    if (Get_latest_sentence("$GPGGA", GGA)) {
    	HAL_UART_Transmit(&huart2, (uint8_t*)"GGA FOUND\r\n", strlen("GGA FOUND\r\n"), HAL_MAX_DELAY);
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

        sprintf(mapsBuffer, "https://www.google.com/maps?q=%.8f,%.8f", lat, lon);
    } else if ((flagGGA == 1) || (flagRMC == 1)) {
//        strcpy(mapsBuffer, "No Signal\r\n");
    	strcpy(mapsBuffer, "https://www.google.com/maps?q=30.02257390, 31.70795646");
    } else {
        strcpy(mapsBuffer, "Error\r\n");
//    	strcpy(mapsBuffer, "https://www.google.com/maps?q=30.02257390, 31.70795646");
    }

    return mapsBuffer;
}









