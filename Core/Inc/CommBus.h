/*
 * Comm_Bus.h
 *
 *  Created on: May 4, 2025
 *      Author: Asteroid
 */

#ifndef INC_COMMBUS_H_
#define INC_COMMBUS_H_

typedef enum {
	GPS,
	ULT,
	PIR,
	THM,
	DRV,
	SRV,
	UNKNOWN
} CommBus_MessageType;

typedef enum {
	THM_HUM_DETECTED = 1,
	THM_HUM_NOT_DETECTED = 0
}CommBus_Detected;

extern CommBus_Detected THM_Decision;

void CommBus_Init(UART_HandleTypeDef *huart);

void CommBus_RxHandler(void);
uint8_t CommBus_MessageAvailable(void);
char* CommBus_GetMessage(void);

CommBus_MessageType CommBus_GetMessageType(char *msg);
char* CommBus_GetPayload(char *msg);

void CommBus_SendMessage(CommBus_MessageType type, const char* payload);
void CommBus_HandleIncoming();

void CommBus_ParseTHM(char* payload);
void CommBus_ParseDRV(char* payload);
void CommBus_ParseCAM(char* payload);


void CommBus_Enable(void);
void CommBus_Disable(void);


void CommBus_ResetMessageReady();
void CommBus_ResetTHMDecision();

#endif /* INC_COMMBUS_H_ */
