/* stepper_driver.c */
#include "motor.h"

// Pins for Motor A
#define DIR1_PORT GPIOA
#define DIR1_PIN  GPIO_PIN_4
#define EN1_PORT  GPIOA
#define EN1_PIN   GPIO_PIN_5

// Pins for Motor B
#define DIR2_PORT GPIOA
#define DIR2_PIN  GPIO_PIN_6
#define EN2_PORT  GPIOA
#define EN2_PIN   GPIO_PIN_7

// PWM channels
#define STEP_TIMER &htim1
#define STEP_CHANNEL_A TIM_CHANNEL_1
#define STEP_CHANNEL_B TIM_CHANNEL_2

extern TIM_HandleTypeDef htim1;

void Stepper_Init(void)
{
    // Enable both motors (assuming EN is active low, adjust if opposite)
    HAL_GPIO_WritePin(EN1_PORT, EN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN2_PORT, EN2_PIN, GPIO_PIN_RESET);

    // Start PWM on both channels
    HAL_TIM_PWM_Start(STEP_TIMER, STEP_CHANNEL_A);
    HAL_TIM_PWM_Start(STEP_TIMER, STEP_CHANNEL_B);

    // Set initial PWM duty cycle to 50%
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_A, htim1.Init.Period / 2);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_B, htim1.Init.Period / 2);
}

void Stepper_MoveForward(uint8_t speed)
{
    HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, GPIO_PIN_SET);   // Forward
    HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_AUTORELOAD(STEP_TIMER, 1000 - speed); // Speed sets frequency
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_A, (1000 - speed) / 2);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_B, (1000 - speed) / 2);
}

void Stepper_MoveBackward(uint8_t speed)
{
    HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, GPIO_PIN_RESET);  // Backward
    HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_AUTORELOAD(STEP_TIMER, 1000 - speed); // Speed sets frequency
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_A, (1000 - speed) / 2);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_B, (1000 - speed) / 2);
}

void Stepper_Stop(void)
{
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_A, 0);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_B, 0);
}

void Stepper_TurnRight(uint8_t speed)
{
    HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, GPIO_PIN_SET);    // Motor A Forward
    HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_RESET);  // Motor B Backward

    __HAL_TIM_SET_AUTORELOAD(STEP_TIMER, 1000 - speed);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_A, (1000 - speed) / 2);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_B, (1000 - speed) / 2);
}

void Stepper_TurnLeft(uint8_t speed)
{
    HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, GPIO_PIN_RESET);  // Motor A Backward
    HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_SET);    // Motor B Forward

    __HAL_TIM_SET_AUTORELOAD(STEP_TIMER, 1000 - speed);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_A, (1000 - speed) / 2);
    __HAL_TIM_SET_COMPARE(STEP_TIMER, STEP_CHANNEL_B, (1000 - speed) / 2);
}

