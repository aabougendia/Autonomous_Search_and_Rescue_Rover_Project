
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
    MPU6050_init(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98f, 0.004);
    MPU6050_calibrateGyro(&hi2c1, 1000);
    Stepper_Init();
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


#define MOVE_DISTANCE 1000 // Distance to move in mm (1 meter)
#define TURN_SPEED 400    // Speed for turning
#define OBSTACLE_THRESHOLD 45 // Distance in cm for obstacle detection
#define TILT_THRESHOLD 15.0f  // Max allowable tilt in degrees
#define MOVE_TIME_MS 5000     // Approximate time to move 1 meter at ROVER_SPEED (adjust based on testing)
#define SENSOR_CHECK_INTERVAL 100 // Check sensors every 100 ms during movement

static void Handle_AutoState_Reconning(void) {
    LOG_UART("AUTO STATE 0: RECONNING");

    // Initialize variables
    static uint8_t turn_direction = 0; // 0 for right, 1 for left (alternate for grid pattern)
    static uint32_t move_start_time = 0;
    static uint8_t is_moving = 0;
    static uint32_t last_sensor_check = 0;


    static uint32_t lastCycle = 0;
    uint32_t nowCycle = DWT->CYCCNT;

    // Get system clock frequency
    uint32_t cpuFreq = HAL_RCC_GetHCLKFreq();  // e.g. 84,000,000 Hz

    // Convert cycle count difference to seconds
    float dt = (nowCycle - lastCycle) / (float)cpuFreq;
    lastCycle = nowCycle;

    // Use real dt in your attitude function
    MPU6050_calcAttitude(&hi2c1, dt);


    // Check for excessive tilt to ensure stability
    if (fabs(attitude.r) > TILT_THRESHOLD || fabs(attitude.p) > TILT_THRESHOLD) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"UNSTABLE ORIENTATION DETECTED\r\n", strlen("UNSTABLE ORIENTATION DETECTED\r\n"), HAL_MAX_DELAY);
        Stepper_Stop(); // Stop to prevent tipping
        HAL_Delay(1000); // Wait to stabilize
        is_moving = 0;
        return;
    }

    // Check thermal sensor for human detection
    thm_state = Get_THM_HUM();
    if (thm_state == THM_HUM_DETECTED) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"HUMAN DETECTED\r\n", strlen("HUMAN DETECTED\r\n"), HAL_MAX_DELAY);
        Stepper_Stop(); // Stop the rover
        sys_auto_state = SEND_INFO; // Transition to SEND_INFO state
        is_moving = 0;
        return;
    }

    // Check for obstacles using ultrasonic sensor
    if (HAL_GetTick() - last_sensor_check >= SENSOR_CHECK_INTERVAL) {
        uint16_t dist = Get_Average_Distance();
        char msg[50];
        sprintf(msg, "Distance: %u cm\r\n", dist);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        if (dist < OBSTACLE_THRESHOLD) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"OBSTACLE DETECTED\r\n", strlen("OBSTACLE DETECTED\r\n"), HAL_MAX_DELAY);
            Stepper_Stop(); // Stop immediately
            Avoid_Obstacle(); // Handle obstacle avoidance
            is_moving = 0; // Reset movement state
            return;
        }
        last_sensor_check = HAL_GetTick();
    }

    // Exploration pattern: Move 1 meter, then turn 90 degrees
    if (!is_moving) {
        // Start moving forward
        Stepper_MoveForward(ROVER_SPEED);
        move_start_time = HAL_GetTick(); // Record start time
        is_moving = 1;
        last_sensor_check = HAL_GetTick(); // Initialize sensor check timer
    } else {
        // Check if the rover has moved approximately 1 meter
        if (HAL_GetTick() - move_start_time >= MOVE_TIME_MS) {
            Stepper_Stop(); // Stop after moving 1 meter

            // Perform a 90-degree turn using MPU6050 yaw
            float initial_yaw = attitude.y; // Current yaw
            if (turn_direction == 0) {
                Stepper_TurnRight(TURN_SPEED);
                HAL_UART_Transmit(&huart2, (uint8_t*)"TURNING RIGHT\r\n", strlen("TURNING RIGHT\r\n"), HAL_MAX_DELAY);
            } else {
                Stepper_TurnLeft(TURN_SPEED);
                HAL_UART_Transmit(&huart2, (uint8_t*)"TURNING LEFT\r\n", strlen("TURNING LEFT\r\n"), HAL_MAX_DELAY);
            }

            // Wait until a 90-degree turn is completed or timeout
            uint32_t turn_start_time = HAL_GetTick();
            while (fabs(attitude.y - initial_yaw) < 90.0f && (HAL_GetTick() - turn_start_time) < 5000) {
                MPU6050_calcAttitude(&hi2c1, dt); // Update yaw during turn
                HAL_Delay(10);
            }
            Stepper_Stop(); // Stop turning

            // Alternate turn direction for next segment
            turn_direction = !turn_direction;
            is_moving = 0; // Reset for next movement segment
        }
    }

    // Periodically log GPS coordinates for mapping (every 5 seconds)
//    static uint32_t last_gps_log = 0;
//    if (HAL_GetTick() - last_gps_log >= 5000) {
//        strcpy(GPS_GoogleMapsLink, GPS_getGoogleMapsLink());
//        HAL_UART_Transmit(&huart2, (uint8_t*)"GPS: ", 5, HAL_MAX_DELAY);
//        HAL_UART_Transmit(&huart2, (uint8_t*)GPS_GoogleMapsLink, strlen(GPS_GoogleMapsLink), HAL_MAX_DELAY);
//        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
//        last_gps_log = HAL_GetTick();
//    }

    HAL_Delay(10); // Small delay for system stability
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

	HAL_Delay(8000);

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
