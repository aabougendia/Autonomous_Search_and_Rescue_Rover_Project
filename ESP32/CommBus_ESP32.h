#ifndef COMMBUS_ESP32_H
#define COMMBUS_ESP32_H

#include <Arduino.h>

// ESP32 -> STM32 
#define COMM_AUTOMAN_STATE_PIN    5    // Connects to PB12

// Auto States
#define COMM_AUTO0_PIN            18   // Connects to PB13
#define COMM_AUTO1_PIN            19   // Connects to PB14
  
// Manual States
#define COMM_MAN0_AUTO_ACK_PIN    2    // Connects to PB0 (used as ACK in Auto too)
#define COMM_MAN1_PIN             4    // Connects to PB1
#define COMM_MAN2_PIN             23   // Connects to PB3

// Flags
#define COMM_HUM_FLAG_PIN         14   // Connects to PA15 (Human detection from ESP)


typedef enum {
	STATE_MANUAL = 1,
	STATE_AUTO = 0
} ControlState;

typedef enum {
	RECONNING = 0,
	SEND_INFO = 1,
	IDLE = 2
} AutoState;

typedef enum {
	DRV_STOP = 0,
	DRV_FWD = 1,
	DRV_BWD	= 2,
	DRV_RIGHT = 3,
	DRV_LEFT = 4,
} ManualState;

typedef enum {
	THM_HUM_NOT_DETECTED = 0,
	THM_HUM_DETECTED = 1
} THM_State;

void Set_THM_HUM(THM_State state);
void Set_Ctrl_State(ControlState state);
void Set_Manual_State(ManualState state);
AutoState Get_Auto_State();
void Set_ESP_ACK();
void Clear_ESP_ACK();
uint8_t Get_STM_ACK();
String Get_GPSLink();

void CommBus_Init();

#endif