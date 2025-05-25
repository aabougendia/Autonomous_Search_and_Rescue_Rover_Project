/*
 * SystemFlow_STM32.h
 *
 *  Created on: May 23, 2025
 *      Author: Asteroid
 */

#ifndef INC_SYSTEMFLOW_STM32_H_
#define INC_SYSTEMFLOW_STM32_H_

#include "CommBus_STM32.h"

#include <stdint.h>
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "std_types.h"

#include "CommBus_STM32.h"
#include "mpu6050.h"
#include "servo.h"
#include "GPS_Module.h"
#include "PIR_sensor.h"
#include "ultrasonic.h"
#include "motor.h"
#include "alerts.h"


void SystemFlow_Init();
void SystemFlow_Run();


#endif /* INC_SYSTEMFLOW_STM32_H_ */
