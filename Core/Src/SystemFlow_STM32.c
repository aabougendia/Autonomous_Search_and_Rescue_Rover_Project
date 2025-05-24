
#include "SystemFlow_STM32.h"



extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

char GPS_GoogleMapsLink[150];
PIR_OUT pir_state;
THM_State thm_state;


ControlState control_state = STATE_AUTO;
AutoState sys_auto_state = RECONNING;
ManualState sys_manual_state = DRV_STOP;

/**************   Auto States  ****************/
static void Handle_AutoState_Reconning(void);
static void Handle_AutoState_SendInfo(void);
static void Handle_AutoState_Idle(void);

/**************   Manual States  ****************/
static void Handle_ManualState_DRV_STOP(void);
static void Handle_ManualState_DRV_FWD(void);
static void Handle_ManualState_DRV_BWD(void);
static void Handle_ManualState_DRV_RIGHT(void);
static void Handle_ManualState_DRV_LEFT(void);

static void Handle_ManualState_CAM_STOP(void);
static void Handle_ManualState_CAM_RIGHT(void);
static void Handle_ManualState_CAM_LEFT(void);

void SystemFlow_Init(){
    Servo_Init();
    Servo_SetAngle(90);
    GPS_Init();
    Ultrasonic_Init();
    Stepper_Init();
//    CommBus_Init(&huart3);
}
void SystemFlow_Run(){

    if(control_state == STATE_AUTO) {

        switch (sys_auto_state) {
            case RECONNING:
                Handle_AutoState_Reconning();
                break;
            case SEND_INFO:
                Handle_AutoState_SendInfo();
                break;
            case IDLE:
                Handle_AutoState_Idle();
        }

        Set_Auto_State(sys_auto_state);
    }
    else if(control_state == STATE_MANUAL){

    	sys_manual_state = Get_Man_Stat();
        switch (sys_manual_state) {
            case DRV_STOP:
                Handle_ManualState_DRV_STOP();
                break;
            case DRV_FWD:
                Handle_ManualState_DRV_FWD();
                break;
            case DRV_BWD:
                Handle_ManualState_DRV_BWD();
                break;
            case DRV_RIGHT:
                Handle_ManualState_DRV_RIGHT();
                break;
            case DRV_LEFT:
                Handle_ManualState_DRV_LEFT();
                break;
            case CAM_STOP:
                Handle_ManualState_CAM_STOP();
                break;
            case CAM_RIGHT:
                Handle_ManualState_CAM_RIGHT();
                break;
            case CAM_LEFT:
                Handle_ManualState_CAM_LEFT();
                break;
        }
    }
    HAL_Delay(100);

}


/**************   Auto States  ****************/

static void Avoid_Obstacle(void){
    volatile uint8_t Right_Distance = 0.0;
	volatile uint8_t Left_Distance = 0.0;

	// Look to the right
	Servo_SetAngle(0);
	Ultrasonic_Read();
	Right_Distance = ULT_Distance;

	// Look to the left
	Servo_SetAngle(180);
	Ultrasonic_Read();
	Left_Distance = ULT_Distance;

	// Backing a bit to be able to rotate
	Stepper_MoveBackward(200);
	HAL_Delay(2000);
	Stepper_Stop();

	// Deciding to turn right or left
	(Right_Distance >= Left_Distance)? Stepper_TurnRight(400) : Stepper_TurnLeft(400);
	HAL_Delay(1000);
	Stepper_Stop();

	// Moving in the direction of more space
	Stepper_MoveForward(400);
	HAL_Delay(5000);
	Stepper_Stop();
}


static void Handle_AutoState_Reconning(void){

	LOG_UART("AUTO STATE 0: RECONNING");

	Ultrasonic_Read();
	char msg[50];
	sprintf(msg, "Distance: %u cm\r\n", ULT_Distance);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);

	thm_state = Get_THM_HUM();

	if(thm_state == THM_HUM_DETECTED){
		HAL_UART_Transmit(&huart2, (uint8_t*)"HUMAN\r\n", strlen("HUMAN\r\n"), HAL_MAX_DELAY);
		sys_auto_state = SEND_INFO;
	}
	else if(thm_state == THM_HUM_NOT_DETECTED){
		HAL_UART_Transmit(&huart2, (uint8_t*)"NOT HUMAN\r\n", strlen("NOT HUMAN\r\n"), HAL_MAX_DELAY);
	}

	HAL_Delay(500);
}

static void Handle_AutoState_SendInfo(void){
	LOG_UART("AUTO STATE 1: SEND INFO");

	strcpy(GPS_GoogleMapsLink, GPS_getGoogleMapsLink());
	HAL_UART_Transmit(&huart2, (uint8_t*)GPS_GoogleMapsLink, strlen(GPS_GoogleMapsLink), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);

	Send_GPSLink(GPS_GoogleMapsLink);
	HAL_Delay(2000);
//	uint32_t start_time = HAL_GetTick();  // Start time in ms
//	pir_state = 0;
//
//	while ((HAL_GetTick() - start_time) < 5000) {  // Run for 5000ms = 5s
//	    pir_state |= PIR_Read();   // Read PIR sensor
//	}

//	HAL_Delay(10);
//	Set_PIR(pir_state);
//
//	while(Get_ESP_ACK() == GPIO_PIN_RESET);
//
//	if(pir_state == PIR_MOTION_DETECTED){
//		HAL_UART_Transmit(&huart2, (uint8_t*)"MOTION\r\n", strlen("MOTION\r\n"), HAL_MAX_DELAY);
//	}
//	else {
//		HAL_UART_Transmit(&huart2, (uint8_t*)"NO MOTION\r\n", strlen("NO MOTION\r\n"), HAL_MAX_DELAY);
//	}

//	while(Get_ESP_ACK() == GPIO_PIN_RESET){
//	}


//	Send_GPSLink(GPS_GoogleMapsLink);
//	HAL_Delay(10);
//	Set_PIR(pir_state);

//	Set_STM_ACK();

//	while(Get_ESP_ACK() == GPIO_PIN_SET);

//	Clear_STM_ACK();

	sys_auto_state = IDLE;

}

static void Handle_AutoState_Idle(void){
	LOG_UART("AUTO STATE 2: IDLE");

	HAL_Delay(1000);

	sys_auto_state = RECONNING;
}


/**************   Manual States  ****************/

static void Handle_ManualState_DRV_STOP(void){

}
static void Handle_ManualState_DRV_FWD(void){

}
static void Handle_ManualState_DRV_BWD(void){

}
static void Handle_ManualState_DRV_RIGHT(void){

}
static void Handle_ManualState_DRV_LEFT(void){

}

static void Handle_ManualState_CAM_STOP(void){

}
static void Handle_ManualState_CAM_RIGHT(void){

}
static void Handle_ManualState_CAM_LEFT(void){

}
