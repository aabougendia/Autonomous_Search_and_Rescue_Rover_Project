#ifndef INC_COMMBUS_STM32_H_
#define INC_COMMBUS_STM32_H_


#include "PIR_sensor.h"
#include "ultrasonic.h"
// Logical GPIO Macros (No COMM_ prefix)

// --- Auto Mode Control Pins ---
// Shorter aliases for optional internal usage
#define STATE_PORT     				COMM_AUTOMAN_STATE_GPIO_Port

#define AUTOMAN_STATE         		COMM_AUTOMAN_STATE_Pin
#define AUTO0_Pin               	COMM_AUTO0_Pin
#define AUTO1_Pin               	COMM_AUTO1_Pin

#define MAN0_AUTO_ACK_Pin           COMM_MAN0_AUTO_ACK_Pin
#define MAN1_Pin                    COMM_MAN1_Pin
#define MAN2_Pin                    COMM_MAN2_Pin


#define FLAG_PORT       			COMM_MOTION_FLAG_GPIO_Port

#define MOTION_FLAG_Pin             COMM_MOTION_FLAG_Pin
#define HUM_FLAG_Pin                COMM_HUM_FLAG_Pin


#define LOG_UART(...) do { \
    char buf[100]; \
    snprintf(buf, sizeof(buf), __VA_ARGS__); \
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY); \
     HAL_UART_Transmit(&huart2, (uint8_t*)"_______________________________\r\n", strlen("_______________________________\r\n"), HAL_MAX_DELAY);\
} while(0)


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
	CAM_STOP = 5,
	CAM_RIGHT = 6,
	CAM_LEFT = 7
} ManualState;

typedef enum {
	THM_HUM_NOT_DETECTED = 0,
	THM_HUM_DETECTED = 1
} THM_State;

ManualState Get_Man_Stat();
THM_State Get_THM_HUM();
void Set_PIR(PIR_OUT state);
void Set_Auto_State(AutoState state);
uint8_t Get_ESP_ACK();
void Set_STM_ACK();
void Clear_STM_ACK();
ControlState Get_Ctrl_State();
void Send_GPSLink(char* link);

#endif /* INC_COMMBUS_STM32_H_ */
