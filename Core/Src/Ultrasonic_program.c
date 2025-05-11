/*
 * Ultrasonic_program.c
 *
 *  Created on: May 11, 2025
 *      Author: rouka
 */


volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t Is_First_Captured = 0;
volatile uint8_t Distance = 0;

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

void HCSR04_init (void)
{
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}


