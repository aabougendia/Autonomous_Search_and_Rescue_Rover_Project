
#include "SystemFlow_STM32.h"



extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define ROVER_SPEED 400


char GPS_GoogleMapsLink[150];
PIR_OUT pir_state;
THM_State thm_state;


ControlState control_state = STATE_AUTO;
AutoState sys_auto_state = RECONNING;
ManualState sys_manual_state = DRV_STOP;

typedef struct {
    u8 R, G, B;
    uint32_t on_ms;
    uint32_t off_ms;
    u8 blink;
    u8 beep;
} AlertPattern;

const AlertPattern alert_patterns[] = {
    [RECONNING]  = {0, 0, 1, 100, 2000, 1, 1},
    [SEND_INFO]  = {1, 0, 0, 500, 1000, 1, 1},
    [IDLE]       = {0, 1, 0, 0,    0,   0, 0}
};


void AlertHandler_Update(AutoState state);


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

	HAL_UART_Transmit(&huart2, (uint8_t*)"start run\r\n", strlen("start run\r\n"), HAL_MAX_DELAY);
    if(control_state == STATE_AUTO) {
    	AlertHandler_Update(sys_auto_state);
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
        HAL_Delay(100);
    }
    else if(control_state == STATE_MANUAL){

    	RGB_LED_vON(0, 0, 1);
    	BUZZER_vOFF();

    	HAL_UART_Transmit(&huart2, (uint8_t*)"start manual\r\n", strlen("start manual\r\n"), HAL_MAX_DELAY);
    	sys_manual_state = Get_Man_Stat();

        switch (sys_manual_state) {
            case DRV_STOP:
            	HAL_UART_Transmit(&huart2, (uint8_t*)"stop\r\n", strlen("stop\r\n"), HAL_MAX_DELAY);
                Handle_ManualState_DRV_STOP();
                break;
            case DRV_FWD:
            	HAL_UART_Transmit(&huart2, (uint8_t*)"forward\r\n", strlen("forward\r\n"), HAL_MAX_DELAY);
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
        HAL_Delay(20);
    }


}


void AlertHandler_Update(AutoState state) {
    static uint8_t is_on = 0;
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    AlertPattern cfg = alert_patterns[state];

    if (!cfg.blink) {
        RGB_LED_vON(cfg.R, cfg.G, cfg.B);
        BUZZER_vOFF();
        return;
    }

    uint32_t interval = is_on ? cfg.on_ms : cfg.off_ms;

    if (now - last_tick >= interval) {
        last_tick = now;
        is_on = !is_on;

        if (is_on) {
            RGB_LED_vON(cfg.R, cfg.G, cfg.B);
            if (cfg.beep)
                BUZZER_vON();
        } else {
            RGB_LED_vOFF();
            BUZZER_vOFF();
        }
    }
}



/**************   Auto States  ****************/

uint16_t Get_Average_Distance(void) {
    uint16_t total = 0;
    for (int i = 0; i < 3; i++) {
        Ultrasonic_Read();
        total += ULT_Distance;
        HAL_Delay(100);
    }
    return total / 3;
}

static void Avoid_Obstacle(void){
    volatile uint8_t Right_Distance = 0.0;
	volatile uint8_t Left_Distance = 0.0;


	Stepper_Stop();
	// Look to the right
	for(uint8_t ang = 90; ang > 0; ang -= 10){
		Servo_SetAngle(ang);
		HAL_Delay(50);
	}
	HAL_Delay(200);
	Ultrasonic_Read();
	Right_Distance = Get_Average_Distance();

	// Look to the left
	for(uint8_t ang = 0; ang < 180; ang += 10){
		Servo_SetAngle(ang);
		HAL_Delay(50);
	}
	HAL_Delay(200);
	Ultrasonic_Read();
	Left_Distance = Get_Average_Distance();


	Servo_SetAngle(90);
	HAL_Delay(200);
	// Backing a bit to be able to rotate
	Stepper_MoveBackward(200);
	HAL_Delay(1500);
	Stepper_Stop();

	// Deciding to turn right or left
	(Right_Distance >= Left_Distance)? Stepper_TurnRight(400) : Stepper_TurnLeft(400);
	HAL_Delay(3000);
	Stepper_Stop();

	HAL_Delay(300);
	// Moving in the direction of more space
	Stepper_MoveForward(ROVER_SPEED);
//	HAL_Delay(5000);
//	Stepper_Stop();

}




static void Handle_AutoState_Reconning(void){

	LOG_UART("AUTO STATE 0: RECONNING");
//
//	Ultrasonic_Read();
//	char msg[50];
//	sprintf(msg, "Distance: %u cm\r\n", ULT_Distance);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
//
//	thm_state = Get_THM_HUM();
//
//	if(thm_state == THM_HUM_DETECTED){
//		HAL_UART_Transmit(&huart2, (uint8_t*)"HUMAN\r\n", strlen("HUMAN\r\n"), HAL_MAX_DELAY);
//		sys_auto_state = SEND_INFO;
//	}
//	else if(thm_state == THM_HUM_NOT_DETECTED){
//		HAL_UART_Transmit(&huart2, (uint8_t*)"NOT HUMAN\r\n", strlen("NOT HUMAN\r\n"), HAL_MAX_DELAY);
//	}


    Stepper_MoveForward(ROVER_SPEED);

    HAL_Delay(100);
    thm_state = Get_THM_HUM();

    HAL_Delay(100);

    // Read ultrasonic sensor multiple times to confirm obstacle
//    const int checkCount = 3;
//    int obstacleCount = 0;
//
//    for (int i = 0; i < checkCount; i++) {
//        Ultrasonic_Read();
//        if (ULT_Distance <= 47) {
//            obstacleCount++;
//        }
//        HAL_Delay(30);  // Small delay between checks
//    }
    int dist = Get_Average_Distance();

    if (dist < 45) { // At least 2 out of 3 reads must detect obstacle
        Avoid_Obstacle();
    }

    HAL_Delay(100);
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
	Stepper_Stop();
}
static void Handle_ManualState_DRV_FWD(void){
	HAL_UART_Transmit(&huart2, (uint8_t*)"stepper fwd\r\n", strlen("stepper fwd\r\n"), HAL_MAX_DELAY);
	Stepper_MoveForward(ROVER_SPEED);
}
static void Handle_ManualState_DRV_BWD(void){
	Stepper_MoveBackward(ROVER_SPEED);
}
static void Handle_ManualState_DRV_RIGHT(void){
	Stepper_TurnRight(ROVER_SPEED);
}
static void Handle_ManualState_DRV_LEFT(void){
	Stepper_TurnLeft(ROVER_SPEED);
}

static void Handle_ManualState_CAM_STOP(void){

}
static void Handle_ManualState_CAM_RIGHT(void){

}
static void Handle_ManualState_CAM_LEFT(void){

}
