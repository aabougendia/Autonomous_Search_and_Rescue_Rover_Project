#ifndef __PIR_SENSOR_H
#define __PIR_SENSOR_H

#include <Arduino.h>

#define PIR_OUTPUT_PIN	15

typedef enum {
	PIR_NO_MOTION = 0,
	PIR_MOTION_DETECTED = 1
} PIR_OUT;


void PIR_Init();
uint8_t PIR_Read();

#endif
