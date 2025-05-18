#ifndef INC_SYSTEM_FLOW_H_
#define INC_SYSTEM_FLOW_H_


#include <stdint.h>
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "std_types.h"

#include "servo.h"
#include "GPS_Module.h"
#include "PIR_sensor.h"
#include "ultrasonic.h"
#include "motor.h"
#include "alerts.h"
#include "CommBus.h"

typedef enum {
	 _000_RECONNING              = 0,
	 _001_AVOID_OBSTACLE         = 1,
	 _010_THM_DETECTED           = 2,
	 _011_SEND_DATA_TO_OPERATOR  = 3,
	 _100_MANUAL_MODE            = 4,
	 UNDEFINED_STATE             = 5
} System_State;

void SystemFlow_Init(void);
void SystemFlow_Run(void);



#endif /* INC_SYSTEM_FLOW_H_ */
