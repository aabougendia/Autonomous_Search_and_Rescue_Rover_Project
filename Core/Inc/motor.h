/* stepper_driver.h */
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "stm32f1xx_hal.h"

// Pins for Motor A
#define DIR1_PORT MOTOR_DIR1_GPIO_Port
#define DIR1_PIN  MOTOR_DIR1_Pin
#define EN1_PORT  MOTOR_EN1_GPIO_Port
#define EN1_PIN   MOTOR_EN1_Pin

// Pins for Motor B
#define DIR2_PORT MOTOR_DIR2_GPIO_Port
#define DIR2_PIN  MOTOR_DIR2_Pin
#define EN2_PORT  MOTOR_EN2_GPIO_Port
#define EN2_PIN   MOTOR_EN2_Pin

// PWM channels
#define STEP_TIMER &htim1
#define STEP_CHANNEL_A TIM_CHANNEL_1
#define STEP_CHANNEL_B TIM_CHANNEL_2


void Stepper_Init(void);
void Stepper_MoveForward(uint16_t speed);
void Stepper_MoveBackward(uint16_t speed);
void Stepper_Stop(void);
void Stepper_TurnRight(uint16_t speed);
void Stepper_TurnLeft(uint16_t speed);


#endif /* MOTOR_H */
