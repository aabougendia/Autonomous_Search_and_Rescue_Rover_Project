#ifndef INC_COMMBUS_STM32_H_
#define INC_COMMBUS_STM32_H_

#include "ultrasonic.h"

#define STATE_PORT     				COMM_AUTOMAN_STATE_GPIO_Port

#define AUTOMAN_STATE         		COMM_AUTOMAN_STATE_Pin
#define AUTO0_Pin               	COMM_AUTO0_Pin
#define AUTO1_Pin               	COMM_AUTO1_Pin

#define MAN0_AUTO_ACK_Pin           COMM_MAN0_AUTO_ACK_Pin
#define MAN1_Pin                    COMM_MAN1_Pin
#define MAN2_Pin                    COMM_MAN2_Pin


#define FLAG_PORT       			COMM_MOTION_FLAG_GPIO_Port

#define HUM_FLAG_Pin                COMM_HUM_FLAG_Pin



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
	DRV_LEFT = 4
} ManualState;

typedef enum {
	THM_HUM_NOT_DETECTED = 0,
	THM_HUM_DETECTED = 1
} THM_State;

ManualState Get_Man_Stat();
THM_State Get_THM_HUM();
void Set_Auto_State(AutoState state);
ManualState Get_Manual_State();
uint8_t Get_ESP_ACK();
void Set_STM_ACK();
void Clear_STM_ACK();
ControlState Get_Ctrl_State();
void Send_GPSLink(char* link);

#endif /* INC_COMMBUS_STM32_H_ */
