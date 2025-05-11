/*
 * HALERTS_program.c
 *
 *  Created on: May 7, 2025
 *      Author: rouka
 */
#include "std_types.h"
#include "main.h"
#include "HALERTS_interface.h"


void LED_vON(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void LED_vOFF(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void BUZZER_vON(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void BUZZER_vOFF(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void RGB_LED_vON(u8 R_State, u8 G_State, u8 B_State)
{

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, R_State);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, G_State);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, B_State);
}

void RGB_LED_vOFF()
{

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, LED_OFF);
}

void LED_vBlink(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, u32 delay)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(delay);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void RGB_LED_vBlink(u8 R_State, u8 G_State, u8 B_State, u32 delay)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, R_State);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, G_State);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, B_State);

	HAL_Delay(delay);

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, LED_OFF);
}


