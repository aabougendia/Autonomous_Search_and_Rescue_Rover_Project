/*
 * Comm_Bus.c
 *
 *  Created on: May 4, 2025
 *      Author: Asteroid
 */
#include "main.h"
#include "CommBus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define RX_BUFFER_SIZE	256
#define TX_BUFFER_SIZE	256

static UART_HandleTypeDef *CommBus_huart;

static char rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_index = 0;
static uint8_t message_ready = 0;
static uint8_t rx_byte;

static char tx_buffer[TX_BUFFER_SIZE];

CommBus_Detected THM_Decision = THM_HUM_NOT_DETECTED;


void CommBus_Init(UART_HandleTypeDef *huart){
	CommBus_huart = huart;
	rx_index = 0;
	message_ready = 0;

	HAL_UART_Receive_IT(CommBus_huart, &rx_byte, 1);
}



static void CommBus_ProcessByte(uint8_t byte){
	if(message_ready)	return;

	if (byte == '\n'){
		rx_buffer[rx_index] = '\0';
		message_ready = 1;
		rx_index = 0;
	}
	else {
		rx_buffer[rx_index++] = byte;

		if(rx_index >= RX_BUFFER_SIZE){
			rx_index = 0;
		}
	}

}

void CommBus_RxHandler(void){
	CommBus_ProcessByte(rx_byte);
    HAL_UART_Receive_IT(CommBus_huart, &rx_byte, 1);
}

uint8_t CommBus_MessageAvailable(void){
	return message_ready;
}

char* CommBus_GetMessage(void){
	return rx_buffer;
}

CommBus_MessageType CommBus_GetMessageType(char *msg){
	CommBus_MessageType message_type;

	if(msg[0] == 'G' && msg[1] == 'P' && msg[2] == 'S' && msg[3] == ':'){
		message_type = GPS;
	}
	else if (msg[0] == 'U' && msg[1] == 'L' && msg[2] == 'T' && msg[3] == ':'){
		message_type = ULT;
	}
	else if (msg[0] == 'T' && msg[1] == 'H' && msg[2] == 'M' && msg[3] == ':'){
		message_type = THM;
	}
	else if (msg[0] == 'P' && msg[1] == 'I' && msg[2] == 'R' && msg[3] == ':'){
		message_type = PIR;
	}

	return message_type;
}

char* CommBus_GetPayload(char *msg){
	char* colon = strchr(msg, ':');
	if(colon != NULL){
		return colon + 1;
	}
	else {
		return "";
	}
}

void CommBus_SendMessage(CommBus_MessageType type, const char* payload){
	char* msg_type;
	switch(type){
		case GPS:
			msg_type = "GPS";
		break;
		case ULT:
			msg_type = "ULT";
		break;
		case PIR:
			msg_type = "PIR";
		break;
		default:
			msg_type = "UNKNOWN";
	}

	snprintf(tx_buffer, TX_BUFFER_SIZE, "%s:%s\n", msg_type, payload);
	HAL_UART_Transmit(CommBus_huart, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
}


void CommBus_ResetMessageReady(){
	message_ready = 0;
}

void CommBus_ParseTHM(char* payload){
	switch (atoi(payload)){
	case 0:
		THM_Decision = THM_HUM_NOT_DETECTED;
		break;
	case 1:
		THM_Decision = THM_HUM_DETECTED;
		break;
	}
}

void CommBus_ResetTHMDecision(){
	THM_Decision = THM_HUM_NOT_DETECTED;
}

/* To call in main.c */
void CommBus_HandleIncoming(){
	if(!CommBus_MessageAvailable())	return;

	char *msg = CommBus_GetMessage();
	CommBus_MessageType type = CommBus_GetMessageType(msg);

	char *payload = CommBus_GetPayload(msg);

	switch (type){
		case THM:
			CommBus_ParseTHM(payload);
			break;

		default:
			HAL_UART_Transmit(CommBus_huart, (uint8_t*)"[WARN] Unsupported msg type\r\n", 30, HAL_MAX_DELAY);
			break;
	}

	CommBus_ResetMessageReady();

}


