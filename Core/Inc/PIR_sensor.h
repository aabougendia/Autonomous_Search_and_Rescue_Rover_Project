#ifndef __PIR_SENSOR_H
#define __PIR_SENSOR_H

#include "main.h"


typedef enum {
	PIR_NO_MOTION = 0,
	PIR_MOTION_DETECTED = 1
} PIR_OUT;


PIR_OUT PIR_Read();

#endif
