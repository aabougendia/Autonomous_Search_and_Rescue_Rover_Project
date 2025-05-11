#include "motor.h"

// Motor A (left)
#define IN1_PORT GPIOA
#define IN1_PIN  GPIO_PIN_4
#define IN2_PORT GPIOA
#define IN2_PIN  GPIO_PIN_5
#define ENA_CHANNEL TIM_CHANNEL_1

// Motor B (right)
#define IN3_PORT GPIOA
#define IN3_PIN  GPIO_PIN_6
#define IN4_PORT GPIOA
#define IN4_PIN  GPIO_PIN_7
#define ENB_CHANNEL TIM_CHANNEL_2

static TIM_HandleTypeDef *_motor_htim;

void Motor_Init(TIM_HandleTypeDef *htim)
{
    _motor_htim = htim;

    // Start PWM channels for both motors
    HAL_TIM_PWM_Start(_motor_htim, ENA_CHANNEL);
    HAL_TIM_PWM_Start(_motor_htim, ENB_CHANNEL);
}

void Motor_Forward(uint8_t speed)
{
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(_motor_htim, ENA_CHANNEL, speed);
    __HAL_TIM_SET_COMPARE(_motor_htim, ENB_CHANNEL, speed);
}

void Motor_Backward(uint8_t speed)
{
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(_motor_htim, ENA_CHANNEL, speed);
    __HAL_TIM_SET_COMPARE(_motor_htim, ENB_CHANNEL, speed);
}

void Motor_TurnLeft(uint8_t speed)
{
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(_motor_htim, ENA_CHANNEL, speed);
    __HAL_TIM_SET_COMPARE(_motor_htim, ENB_CHANNEL, speed);
}

void Motor_TurnRight(uint8_t speed)
{
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(_motor_htim, ENA_CHANNEL, speed);
    __HAL_TIM_SET_COMPARE(_motor_htim, ENB_CHANNEL, speed);
}
