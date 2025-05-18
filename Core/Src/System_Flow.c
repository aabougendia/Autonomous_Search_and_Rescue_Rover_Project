

#include "System_Flow.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;


#define STATE_PORT 	STATE0_GPIO_Port
#define STATE0 		STATE0_Pin
#define STATE1 		STATE1_Pin
#define STATE2 		STATE2_Pin
#define STATE_ACK 	STATE_ACK_Pin



// Current FSM state
static System_State system_state = 0; // Start in autonomous





char GPS_outputBuffer[150];


// Forward declarations
static void Set_State(System_State state);
static GPIO_PinState Get_Ack();

static void Handle_State_000_Reconning(void);
static void Handle_State_001_Avoid_Obstacle(void);
static void Handle_State_010_THM_Detected(void);
static void Handle_State_011_Send_Data_To_Operator(void);
static void Handle_State_100_Manual(void);





void SystemFlow_Init(void){
	  Servo_Init();
	  GPS_Init();
	  Ultrasonic_Init();
	  Stepper_Init();
	  CommBus_Init(&huart3);

	  Set_State(_000_RECONNING);
}

void SystemFlow_Run(void) {
	Set_State(system_state);

    switch(system_state) {
        case _000_RECONNING:
        	Handle_State_000_Reconning();
            break;
        case _001_AVOID_OBSTACLE:
        	Handle_State_001_Avoid_Obstacle();
            break;
        case _010_THM_DETECTED:
        	Handle_State_010_THM_Detected();
            break;
        case _011_SEND_DATA_TO_OPERATOR:
        	Handle_State_011_Send_Data_To_Operator();
            break;
        case _100_MANUAL_MODE:
        	Handle_State_100_Manual();
            break;
        default:
            system_state = UNDEFINED_STATE;
            break;
    }
}


static void Set_State(System_State state){
	HAL_GPIO_WritePin(STATE_PORT, STATE0, (GPIO_PinState)(state & (1 << 0)));
	HAL_GPIO_WritePin(STATE_PORT, STATE1, (GPIO_PinState)(state & (1 << 1)));
	HAL_GPIO_WritePin(STATE_PORT, STATE2, (GPIO_PinState)(state & (1 << 2)));
}

static GPIO_PinState Get_Ack(){
	return HAL_GPIO_ReadPin(STATE_PORT, STATE_ACK);
}



static void Handle_State_000_Reconning(void){
	CommBus_HandleIncoming();
	/* Run Reconning Algorithm */

	// check Obstacle
	// check THM

//	if(THM_Decision == THM_HUM_DETECTED){
//		system_state = _010_THM_DETECTED;
//	}

}
static void Handle_State_001_Avoid_Obstacle(void){

}
static void Handle_State_010_THM_Detected(void){

}
static void Handle_State_011_Send_Data_To_Operator(void){

}
static void Handle_State_100_Manual(void){

}
