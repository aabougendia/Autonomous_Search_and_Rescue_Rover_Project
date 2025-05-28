/*
 * CommBus_STM32.c
 *
 *  Created on: May 23, 2025
 *      Author: Asteroid
 *
 *      1. Setters and getters based on the direction of the data.
 *      2. GPS UART3 in STM and UART2 in ESP.
 *      3.
 *
 */

#include "CommBus_STM32.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

ManualState Get_Man_Stat() {

    uint8_t b0 = HAL_GPIO_ReadPin(STATE_PORT, MAN0_AUTO_ACK_Pin);
    uint8_t b1 = HAL_GPIO_ReadPin(STATE_PORT, MAN1_Pin);
    uint8_t b2 = HAL_GPIO_ReadPin(STATE_PORT, MAN2_Pin);

    ManualState manual_state = (b2 << 2) | (b1 << 1) | (b0);

    return manual_state;
}




THM_State Get_THM_HUM() {
    return HAL_GPIO_ReadPin(FLAG_PORT, HUM_FLAG_Pin);
}


void Set_Auto_State(AutoState state){
    HAL_GPIO_WritePin(STATE_PORT, AUTO0_Pin, (state & 1));
    HAL_GPIO_WritePin(STATE_PORT, AUTO1_Pin, (state & 2));
}


ControlState Get_Ctrl_State(){
	return HAL_GPIO_ReadPin(STATE_PORT, COMM_AUTOMAN_STATE_Pin);
}

void Send_GPSLink(char* link){
    HAL_UART_Transmit(&huart3, (uint8_t*)link, strlen(link), HAL_MAX_DELAY);
}

uint8_t Get_ESP_ACK(){
	return HAL_GPIO_ReadPin(STATE_PORT, MAN0_AUTO_ACK_Pin);
}
void Set_STM_ACK(){
	HAL_GPIO_WritePin(STATE_PORT, AUTO0_Pin, GPIO_PIN_SET);
}
void Clear_STM_ACK(){
	HAL_GPIO_WritePin(STATE_PORT, AUTO0_Pin, GPIO_PIN_RESET);
}
