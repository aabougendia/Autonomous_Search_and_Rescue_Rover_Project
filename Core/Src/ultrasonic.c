#include "ultrasonic.h"


extern TIM_HandleTypeDef htim3;

volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t Is_First_Captured = 0;
volatile uint8_t ULT_Distance = 0;

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(ULTRASONIC_TIMER, 0);
	while (__HAL_TIM_GET_COUNTER (ULTRASONIC_TIMER) < time);
}

void Ultrasonic_Init (void)
{
	HAL_TIM_IC_Start_IT(ULTRASONIC_TIMER, TIM_CHANNEL_1);
}

void Ultrasonic_Read (void)
{
	HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(ULTRASONIC_TIMER, TIM_IT_CC1);
}


