#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

// Function prototypes
void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_Forward(uint8_t speed);
void Motor_Backward(uint8_t speed);
void Motor_TurnLeft(uint8_t speed);
void Motor_TurnRight(uint8_t speed);

#endif // MOTOR_H
