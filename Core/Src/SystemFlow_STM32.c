
#include "SystemFlow_STM32.h"

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define ROVER_SPEED 400

#define MOVE_DISTANCE 1000 // Distance to move in mm (1 meter)
#define TURN_SPEED 400    // Speed for turning
#define OBSTACLE_THRESHOLD 45 // Distance in cm for obstacle detection
#define TILT_THRESHOLD 15.0f  // Max allowable tilt in degrees
#define MOVE_TIME_MS 5000     // Approximate time to move 1 meter at ROVER_SPEED (adjust based on testing)
#define SENSOR_CHECK_INTERVAL 100 // Check sensors every 100 ms during movement

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
	Stepper_Init();
    Servo_Init();
    Stepper_Stop();
    Servo_SetAngle(90);
    GPS_Init();
    Ultrasonic_Init();
    MPU6050_init(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98f, 0.004);
    MPU6050_calibrateGyro(&hi2c1, 1000);

//    CommBus_Init(&huart3);
}
void SystemFlow_Run(){

	HAL_UART_Transmit(&huart2, (uint8_t*)"start run\r\n", strlen("start run\r\n"), HAL_MAX_DELAY);
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



/**************   Auto States  ****************/

static void trigger_Gyro(){
    // Calculate dynamic time step (dt) using DWT
    uint32_t now_cycle = DWT->CYCCNT;
    static uint32_t last_cycle = 0;
    uint32_t cpu_freq = HAL_RCC_GetHCLKFreq(); // e.g., 84,000,000 Hz
    float dt = (now_cycle - last_cycle) / (float)cpu_freq;
    last_cycle = now_cycle;

    // Update MPU6050 attitude with dynamic dt
    MPU6050_calibrateGyro(&hi2c1, 1000);
    MPU6050_calcAttitude(&hi2c1, dt);
}

uint16_t Get_Average_Distance(void) {
    uint16_t total = 0;
    for (int i = 0; i < 3; i++) {
        Ultrasonic_Read();
        total += ULT_Distance;
        HAL_Delay(100);
    }
    return total / 3;
}

uint8_t Get_Consistent_THM(void){

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
	if(Right_Distance >= Left_Distance){
		Stepper_TurnRight(400);
		while(1){
			trigger_Gyro();
			  char buffer[50];  // Make sure the buffer is large enough

			  sprintf(buffer,"gz raw: %d, gz deg/s: %.2f, yaw: %.2f\r\n", rawData.gz, sensorData.gz, attitude.y);
			  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			  // Convert float to string
			  sprintf(buffer, "Value: %.2f\r\n", attitude.y);  // Format with 2 decimal places
			if(attitude.y >= -1.4){
				break;
			}
		}
	}
	else {
		Stepper_TurnLeft(400);
		while(1){
			trigger_Gyro();
			  char buffer[50];  // Make sure the buffer is large enough

			  sprintf(buffer,"gz raw: %d, gz deg/s: %.2f, yaw: %.2f\r\n", rawData.gz, sensorData.gz, attitude.y);
			  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			  // Convert float to string
			  sprintf(buffer, "Value: %.2f\r\n", attitude.y);  // Format with 2 decimal places
			if(attitude.y <= 1.4){
				break;
			}
		}
	}

//	HAL_Delay(3000);
	Stepper_Stop();

	HAL_Delay(300);
	// Moving in the direction of more space
	Stepper_MoveForward(ROVER_SPEED);

}



#define TIME_BETWEEN_SWEEPS  5000



static void Handle_AutoState_Reconning(void){

	LOG_UART("AUTO STATE 0: RECONNING");
//
//	Ultrasonic_Read();
//	char msg[50];
//	sprintf(msg, "Distance: %u cm\r\n", ULT_Distance);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
//
	Stepper_MoveForward(ROVER_SPEED);

	uint32_t start_time = HAL_GetTick();
	while(HAL_GetTick() - start_time < TIME_BETWEEN_SWEEPS){
		thm_state = Get_THM_HUM();
		HAL_Delay(10);
		if(thm_state == THM_HUM_DETECTED){
			Stepper_Stop();
			Servo_SetAngle(90);
			sys_auto_state = SEND_INFO;
			return;
		}
	    int dist = Get_Average_Distance();
	    if (dist < 50) {
	        Avoid_Obstacle();
	    }
	}

	Stepper_Stop();
	// Look to the right
	for(uint8_t ang = 90; ang > 0; ang -= 5){
		Servo_SetAngle(ang);
		thm_state = Get_THM_HUM();
		HAL_Delay(50);
		if(thm_state == THM_HUM_DETECTED){
			Stepper_Stop();
//			Servo_SetAngle(90);
			sys_auto_state = SEND_INFO;
			return;
		}
	}
	// Look to the left
	for(uint8_t ang = 0; ang < 180; ang += 5){
		Servo_SetAngle(ang);
		thm_state = Get_THM_HUM();
		HAL_Delay(50);
		if(thm_state == THM_HUM_DETECTED){
			Stepper_Stop();
//			Servo_SetAngle(90);
			sys_auto_state = SEND_INFO;
			return;
		}
	}
	for(uint8_t ang = 180; ang > 90; ang -= 5){
		Servo_SetAngle(ang);
		thm_state = Get_THM_HUM();
		HAL_Delay(50);
		if(thm_state == THM_HUM_DETECTED){
			Stepper_Stop();
//			Servo_SetAngle(90);
			sys_auto_state = SEND_INFO;
			return;
		}
	}
}

static void Handle_AutoState_SendInfo(void){
	LOG_UART("AUTO STATE 1: SEND INFO");

	Stepper_Stop();

	strcpy(GPS_GoogleMapsLink, GPS_getGoogleMapsLink());
	HAL_UART_Transmit(&huart2, (uint8_t*)GPS_GoogleMapsLink, strlen(GPS_GoogleMapsLink), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);

	Send_GPSLink(GPS_GoogleMapsLink);
	HAL_Delay(10000);

	sys_auto_state = IDLE;

}

static void Handle_AutoState_Idle(void){
	LOG_UART("AUTO STATE 2: IDLE");

	Stepper_Stop();

	HAL_Delay(8000);

	while(Get_Ctrl_State() != STATE_MANUAL);
	control_state = STATE_MANUAL;
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
