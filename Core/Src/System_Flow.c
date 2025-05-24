//
//
//#include "System_Flow.h"
//
//
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
//
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;
//
//
//#define STATE_PORT 	STATE0_GPIO_Port
//#define STATE0 		STATE0_Pin
//#define STATE1 		STATE1_Pin
//#define STATE2 		STATE2_Pin
//#define STATE_ACK_S STATE_ACK_S_Pin
//#define STATE_ACK_R STATE_ACK_R_Pin
//
//
//
//// Current FSM state
//static System_State system_state = _000_RECONNING; // Start in autonomous
//
//
//char GPS_outputBuffer[150];
//PIR_OUT PIR_motion_decision;
//
//
//// Forward declarations
//static void Set_State(System_State state);
//static GPIO_PinState Get_StateAck();
//static GPIO_PinState Get_ReceiveAck();
//
//static void Handle_State_000_Reconning(void);
//static void Handle_State_001_Avoid_Obstacle(void);
//static void Handle_State_010_THM_Detected(void);
//static void Handle_State_011_Send_Data_To_Operator(void);
//static void Handle_State_100_Manual(void);
//
//
//
//const char* StateToString(System_State s) {
//    switch(s) {
//        case _000_RECONNING: 				return "_000_RECONNING";
//        case _001_AVOID_OBSTACLE: 			return "_001_AVOID_OBSTACLE";
//        case _010_THM_DETECTED: 			return "_010_THM_DETECTED";
//        case _011_SEND_DATA_TO_OPERATOR: 	return "_011_SEND_DATA_TO_OPERATOR";
//        case _100_MANUAL_MODE: 				return "_100_MANUAL_MODE";
//        default: 							return "UNDEFINED_STATE";
//    }
//}
//
//
//
//
//
//
//
//void SystemFlow_Init(void){
//	  Servo_Init();
//	  GPS_Init();
//	  Ultrasonic_Init();
//	  Stepper_Init();
//	  CommBus_Init(&huart3);
//
//	  LOG_UART("in flow init\r\n");
//	  Set_State(_000_RECONNING);
//}
//
//void SystemFlow_Run(void) {
////	LOG_UART("in flow run\r\n");
//	LOG_UART("start RUN\n");
//
//
//	// Inside SystemFlow_Run:
////	printf(">> STATE: %s\r\n", StateToString(system_state));
//
//	LOG_UART("start switch\n");
//    switch(system_state) {
//        case _000_RECONNING:
//        	Handle_State_000_Reconning();
//            break;
//        case _001_AVOID_OBSTACLE:
//        	Handle_State_001_Avoid_Obstacle();
//            break;
//        case _010_THM_DETECTED:
//        	Handle_State_010_THM_Detected();
//            break;
//        case _011_SEND_DATA_TO_OPERATOR:
//        	Handle_State_011_Send_Data_To_Operator();
//            break;
//        case _100_MANUAL_MODE:
//        	Handle_State_100_Manual();
//            break;
//        default:
//            system_state = UNDEFINED_STATE;
//            break;
//
//    }
//    Set_State(system_state);
//    HAL_Delay(500);
//}
//
//
//static void Set_State(System_State state){
//	HAL_GPIO_WritePin(STATE_PORT, STATE0, (GPIO_PinState)(state & (1 << 0)));
//	HAL_GPIO_WritePin(STATE_PORT, STATE1, (GPIO_PinState)(state & (1 << 1)));
//	HAL_GPIO_WritePin(STATE_PORT, STATE2, (GPIO_PinState)(state & (1 << 2)));
//}
//
//static GPIO_PinState Get_StateAck(){
//	return HAL_GPIO_ReadPin(STATE_PORT, STATE_ACK_S);
//}
//static GPIO_PinState Get_ReceiveAck(){
//	return HAL_GPIO_ReadPin(STATE_PORT, STATE_ACK_R);
//}
//
//
//static void Handle_State_000_Reconning(void){
////	Set_State(_000_RECONNING);
//
//
//    LOG_UART("STM: Entered STATE 000 (RECONNING)\r\n");
//    HAL_Delay(1000);  // simulate scan
////
////	CommBus_Enable();
////	CommBus_HandleIncoming();
////
////	while(THM_Decision == THM_EMPTY){
////		char buffer[50];
////		sprintf(buffer, "CHECK DECISION: %d\r\n", THM_Decision);
////		LOG_UART(buffer);
////
////		CommBus_HandleIncoming();
////
////		LOG_UART("[IN FLOW] ");
////		if(THM_Decision == THM_HUM_DETECTED){
////			LOG_UART("VAR DETECTED 1");
////		}
////		else if (THM_Decision == THM_HUM_NOT_DETECTED){
////			LOG_UART("VAR DETECTED 0");
////		}
////		else {
////			LOG_UART("ELSE");
////		}
////		LOG_UART("\r\n");
////
////
////		LOG_UART("STUCK\r\n");
////		HAL_Delay(100);
////	}
////
////	system_state = _111_STM_ACK;
////	Set_State(_111_STM_ACK);
////
////	if(THM_Decision == THM_HUM_DETECTED){
////		LOG_UART("HUMAN DETECTED\r\n");
////	    system_state = _010_THM_DETECTED;
////	}
////	else if (THM_Decision == THM_HUM_NOT_DETECTED){
////		LOG_UART("NO HUMAN\r\n");
////		system_state = _000_RECONNING;
////	}
////	else {
////		LOG_UART("THM ERROR\r\n");
////	}
////
////	THM_Decision = THM_EMPTY;
////
////
////   Ultrasonic_Read();
////   char msg[50];
////   sprintf(msg, "Distance: %u cm\r\n", ULT_Distance);
////   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
////
////   CommBus_Disable();
//
//
//	/* Run Reconning Algorithm */
//
//	// check Obstacle
//	// check THM
//
////	if(THM_Decision == THM_HUM_DETECTED){
//		system_state = _010_THM_DETECTED;
////	}
//
//}
//static void Handle_State_001_Avoid_Obstacle(void){
////	Set_State(_001_AVOID_OBSTACLE);
////
////	volatile uint8_t Right_Distance = 0.0;
////	volatile uint8_t Left_Distance = 0.0;
////
////	// Look to the right
////	Servo_SetAngle(0);
////	Ultrasonic_Read();
////	Right_Distance = ULT_Distance;
////
////	// Look to the left
////	Servo_SetAngle(180);
////	Ultrasonic_Read();
////	Left_Distance = ULT_Distance;
////
////	// Backing a bit to be able to rotate
////	Stepper_MoveBackward(200);
////	HAL_Delay(2000);
////	Stepper_Stop();
////
////	// Deciding to turn right or left
////	(Right_Distance >= Left_Distance)? Stepper_TurnRight(400) : Stepper_TurnLeft(400);
////	HAL_Delay(1000);
////	Stepper_Stop();
////
////	// Moving in the direction of more space
////	Stepper_MoveForward(400);
////	HAL_Delay(5000);
////	Stepper_Stop();
////
////	// Setting system_state back to RECONNING
////	system_state = _000_RECONNING;
//
//    LOG_UART("STM: Entered STATE 001 (AVOID OBSTACLE)\r\n");
//    HAL_Delay(1000);  // simulate avoidance
//
//    CommBus_Enable();
//    CommBus_HandleIncoming();
//
//
//    system_state = _010_THM_DETECTED;
//
//
//
//}
//static void Handle_State_010_THM_Detected(void){
////	Set_State(_010_THM_DETECTED);
////
////
//	// Read motion decision from PIR
////	PIR_motion_decision = PIR_Read();
////	// Get GPS google maps link
////	strcpy(GPS_outputBuffer, GPS_getGoogleMapsLink());
////
////	HAL_Delay(100);
////
////	// Send Data through CommBus
////	CommBus_SendMessage(GPS, GPS_outputBuffer);
////	HAL_Delay(100);
////
////	if(PIR_motion_decision == PIR_MOTION_DETECTED)
////		CommBus_SendMessage(PIR, "1");
////	else
////		CommBus_SendMessage(PIR, "0");
////
////	HAL_Delay(100);
////	while(Get_Ack() == GPIO_PIN_RESET);
////
////	system_state = _011_SEND_DATA_TO_OPERATOR;
//
//
////		Set_State(_010_THM_DETECTED);
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
////
////
//	    LOG_UART("STM32: 010 state : THM DETECTED\n");
////
//	    // Simulate dummy PIR motion detected and GPS
//	    PIR_motion_decision = PIR_MOTION_DETECTED;
//	    strcpy(GPS_outputBuffer, "https://maps.google.com/?q=30.0444,31.2357");
////
////	    // getting PIR Reading
////	    PIR_motion_decision = PIR_Read();
//	    char PIR_OutBuf[10];
//	    if(PIR_motion_decision == PIR_MOTION_DETECTED){
//	    	strcpy(PIR_OutBuf, "1");
//	    }
//	    else if(PIR_motion_decision == PIR_NO_MOTION){
//	    	strcpy(PIR_OutBuf, "0");
//	    }
//	    else {
//		    strcpy(PIR_OutBuf, "PIR Err");
//	    }
////
////	    // getting GPS Reading
////	    strcpy(GPS_outputBuffer, GPS_getGoogleMapsLink());
////
////	    // Wait for ESP to signal ACK R HIGH
//	    while(Get_ReceiveAck() == GPIO_PIN_RESET);
////
//	    HAL_Delay(100); // NEW: Let ESP enter receive mode fully
////
//	    LOG_UART("STM32: Sending GPS and PIR\n");
//	    while(Get_ReceiveAck() == GPIO_PIN_RESET){
//	    	CommBus_SendMessage(GPS, GPS_outputBuffer);
//	    	HAL_Delay(50);
//	    	CommBus_SendMessage(PIR, PIR_OutBuf);
//////	    	LOG_UART("-");
//	    }
////
////	    // Wait for ESP to signal ACK HIGH
//	    while(Get_StateAck() == GPIO_PIN_RESET){
////	    	LOG_UART(".");
//	    }
////
//	    LOG_UART("STM32: ACK received\n");
//////	    Set_State(_011_SEND_DATA_TO_OPERATOR);  // Required to update GPIOs
//	    system_state = _011_SEND_DATA_TO_OPERATOR;
//////	    Set_State(system_state);
//	    LOG_UART("finished THM\n");
////
////
////
////
//
//
//
//
//
//
//
//}
//static void Handle_State_011_Send_Data_To_Operator(void){
//    LOG_UART("STM: STATE 011 - Send data\n");
//    HAL_Delay(1000);
//    system_state = _000_RECONNING;
//}
//static void Handle_State_100_Manual(void){
//
//}
//
//
