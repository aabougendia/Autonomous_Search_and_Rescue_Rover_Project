/* stepper_driver.h */
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

void Stepper_Init(void);
void Stepper_MoveForward(uint8_t speed);
void Stepper_MoveBackward(uint8_t speed);
void Stepper_Stop(void);
void Stepper_TurnRight(uint8_t speed);
void Stepper_TurnLeft(uint8_t speed);


#endif /* MOTOR_H */
