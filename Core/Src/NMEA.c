/*
 * NMEA.c
 *
 *  Created on: 25-Feb-2022
 *      Author: controllerstech.com
 */


#include "NMEA.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"


int GMT = 0;



int inx = 0;
int hr=0,min=0,day=0,mon=0,yr=0;
int daychange = 0;

/* Decodes the GGA Data
   @GGAbuffer is the buffer which stores the GGA Data
   @GGASTRUCT is the pointer to the GGA Structure (in the GPS Structure)
   @Returns 0 on success
   @ returns 1, 2 depending on where the return statement is excuted, check function for more details
*/

int decodeGGA (char *GGAbuffer, GGASTRUCT *gga)
{
	inx = 0;
	char buffer[32];


	int i = 0;
	while (GGAbuffer[inx] != ',') inx++;  // 1st ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // After time ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after latitude ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after NS ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after longitude ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after EW ','
	inx++;  // reached the character to identify the fix
	if ((GGAbuffer[inx] == '1') || (GGAbuffer[inx] == '2') || (GGAbuffer[inx] == '6'))   // 0 indicates no fix yet
	{
		gga->isfixValid = 1;   // fix available
		inx = 0;   // reset the index. We will start from the inx=0 and extract information now
	}
	else
	{
		gga->isfixValid = 0;   // If the fix is not available
		return 1;  // return error
	}
	while (GGAbuffer[inx] != ',') inx++;  // 1st ','


/*********************** Get TIME ***************************/
//(Update the GMT Offset at the top of this file)

	inx++;   // reach the first number in time
	memset(buffer, '\0', 32);
	i=0;
	while (GGAbuffer[inx] != ',')  // copy upto the we reach the after time ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}

	hr = (atoi(buffer)/10000) + GMT/100;   // get the hours from the 6 digit number

	min = ((atoi(buffer)/100)%100) + GMT%100;  // get the minutes from the 6 digit number

	// adjust time.. This part still needs to be tested
	if (min > 59)
	{
		min = min-60;
		hr++;
	}
	if (hr<0)
	{
		hr=24+hr;
		daychange--;
	}
	if (hr>=24)
	{
		hr=hr-24;
		daychange++;
	}

	// Store the time in the GGA structure
	gga->tim.hour = hr;
	gga->tim.min = min;
	gga->tim.sec = atoi(buffer)%100;

///***************** Get LATITUDE  **********************/
//	inx++;   // Reach the first number in the lattitude
//	memset(buffer, '\0', 12);
//	i=0;
//	while (GGAbuffer[inx] != ',')   // copy upto the we reach the after lattitude ','
//	{
//		buffer[i] = GGAbuffer[inx];
//		i++;
//		inx++;
//	}
//	if (strlen(buffer) < 6) return 2;  // If the buffer length is not appropriate, return error
//	int16_t num = (atoi(buffer));   // change the buffer to the number. It will only convert upto decimal
//	int j = 0;
//	while (buffer[j] != '.') j++;   // Figure out how many digits before the decimal
//	j++;
//	int declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
//	int dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
////	float lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
//
//	int deg = num / 100;
//	float min = (num % 100) + (dec / pow(10, declen));
//	float lat = deg + (min / 60.0f);  // CORRECT conversion
//
//	gga->lcation.latitude = lat;  // save the lattitude data into the strucure
//	inx++;
//	gga->lcation.NS = GGAbuffer[inx];  // save the N/S into the structure
//

///***********************  GET LONGITUDE **********************/
//	inx++;  // ',' after NS character
//	inx++;  // Reach the first number in the longitude
//	memset(buffer, '\0', 12);
//	i=0;
//	while (GGAbuffer[inx] != ',')  // copy upto the we reach the after longitude ','
//	{
//		buffer[i] = GGAbuffer[inx];
//		i++;
//		inx++;
//	}
//	num = (atoi(buffer));  // change the buffer to the number. It will only convert upto decimal
//	j = 0;
//	while (buffer[j] != '.') j++;  // Figure out how many digits before the decimal
//	j++;
//	declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
//	dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
////	lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
////	gga->lcation.longitude = lat;  // save the longitude data into the strucure
//
//	deg = num / 100;
//	min = (num % 100) + (dec / pow(10, declen));
//	float lon = deg + (min / 60.0f);  // Correct
//	gga->lcation.longitude = lon;
//
//	inx++;
//	gga->lcation.EW = GGAbuffer[inx];  // save the E/W into the structure



	/***************** Get LATITUDE 2  **********************/
	inx++;   // Reach the first number in the latitude
	memset(buffer, '\0', 20);  // Make the buffer bigger
	i=0;
	while (GGAbuffer[inx] != ',')   // copy up to ','
	{
	    buffer[i] = GGAbuffer[inx];
	    i++;
	    inx++;
	}

	double raw_lat = atof(buffer);  // USE atof instead of atoi
	int lat_deg = (int)(raw_lat / 100);
	double lat_min = raw_lat - (lat_deg * 100);
	double lat_decimal = lat_deg + (lat_min / 60.0);
	gga->lcation.latitude = lat_decimal;

	inx++;
	gga->lcation.NS = GGAbuffer[inx];  // N or S

	/***********************  GET LONGITUDE 2 **********************/
	inx++;
	inx++;  // Reach longitude
	memset(buffer, '\0', 20);  // Bigger buffer
	i=0;
	while (GGAbuffer[inx] != ',')
	{
	    buffer[i] = GGAbuffer[inx];
	    i++;
	    inx++;
	}

	double raw_lon = atof(buffer);  // USE atof instead of atoi
	int lon_deg = (int)(raw_lon / 100);
	double lon_min = raw_lon - (lon_deg * 100);
	double lon_decimal = lon_deg + (lon_min / 60.0);
	gga->lcation.longitude = lon_decimal;

	inx++;
	gga->lcation.EW = GGAbuffer[inx];  // E or W



/**************************************************/
	// skip positition fix
	inx++;   // ',' after E/W
	inx++;   // position fix
	inx++;   // ',' after position fix;

	// number of sattelites
	inx++;  // Reach the first number in the satellites
	memset(buffer, '\0', 20);
	i=0;
	while (GGAbuffer[inx] != ',')  // copy upto the ',' after number of satellites
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	gga->numofsat = atoi(buffer);   // convert the buffer to number and save into the structure


	/***************** skip HDOP  *********************/
	inx++;
	while (GGAbuffer[inx] != ',') inx++;


	/*************** Altitude calculation ********************/
//	inx++;
//	memset(buffer, '\0', 12);
//	i=0;
//	while (GGAbuffer[inx] != ',')
//	{
//		buffer[i] = GGAbuffer[inx];
//		i++;
//		inx++;
//	}
//	num = (atoi(buffer));
//	j = 0;
//	while (buffer[j] != '.') j++;
//	j++;
//	declen = (strlen(buffer))-j;
//	dec = atoi ((char *) buffer+j);
//	lat = (num) + (dec/pow(10, (declen)));
//	gga->alt.altitude = lat;
//
//	inx++;
//	gga->alt.unit = GGAbuffer[inx];


	/*************** Altitude calculation 2 ********************/
	inx++;
	memset(buffer, '\0', 20);
	i = 0;
	while (GGAbuffer[inx] != ',' && GGAbuffer[inx] != '\0')
	{
	    buffer[i] = GGAbuffer[inx];
	    i++;
	    inx++;
	}

	gga->alt.altitude = atof(buffer);  // cleaner and full precision

	inx++;
	gga->alt.unit = GGAbuffer[inx];


	return 0;

}


int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc)
{
	inx = 0;
	char buffer[12];
	int i = 0;
	while (RMCbuffer[inx] != ',') inx++;  // 1st ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // After time ,
	inx++;
	if (RMCbuffer[inx] == 'A')  // Here 'A' Indicates the data is valid, and 'V' indicates invalid data
	{
		rmc->isValid = 1;
	}
	else
	{
		rmc->isValid =0;
		return 1;
	}
	inx++;
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after latitude,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after NS ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after longitude ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after EW ,

	// Get Speed
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){          // if the speed have some data
		int16_t num = (atoi(buffer));  // convert the data into the number
		int j = 0;
		while (buffer[j] != '.') j++;   // same as above
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->speed = lat;
	}
	else rmc->speed = 0;

	// Get Course
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){  // if the course have some data
		int16_t num = (atoi(buffer));   // convert the course data into the number
		int j = 0;
		while (buffer[j] != '.') j++;   // same as above
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->course = lat;
	}
	else
		{
			rmc->course = 0;
		}

	// Get Date
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	// Date in the format 280222
	day = atoi(buffer)/10000;  // extract 28
	mon = (atoi(buffer)/100)%100;  // extract 02
	yr = atoi(buffer)%100;  // extract 22

	day = day+daychange;   // correction due to GMT shift

	// save the data into the structure
	rmc->date.Day = day;
	rmc->date.Mon = mon;
	rmc->date.Yr = yr;

	return 0;
}

