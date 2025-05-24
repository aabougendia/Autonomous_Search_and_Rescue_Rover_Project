// #include "esp32-hal.h"
// #include "System_Flow.h"

// #define STATE_0_PIN 5
// #define STATE_1_PIN 18
// #define STATE_2_PIN 19

// // ACK output pin (ESP32 → STM32)
// #define STATE_ACK_S_PIN 23
// #define STATE_ACK_R_PIN 4

// SystemState sys_state = _000_RECONNING;

// String GPS_ReceivedGoogleMapsLink;
// PIR_MotionDecision PIR_ReceivedMotionDecision;

// AMG8833 thermalSensor;
// float temperatureData[64];


// CommBus comm(Serial2);

// static SystemState Get_State();
// static void Set_StateAck();
// static void Reset_StateAck();
// static void Set_ReceiveAck();
// static void Reset_RecieveAck();

// static void waitBeforeClearingAck();

// static void Handle_State_000_Reconning(void);
// static void Handle_State_001_Avoid_Obstacle(void);
// static void Handle_State_010_THM_Detected(void);
// static void Handle_State_011_Send_Data_To_Operator(void);
// static void Handle_State_100_Manual(void);


// void SystemFlow_Init(){
//     // Serial.println("in system init\n");


//     pinMode(STATE_0_PIN, INPUT);
//     pinMode(STATE_1_PIN, INPUT);
//     pinMode(STATE_2_PIN, INPUT);
//     pinMode(STATE_ACK_S_PIN, OUTPUT);
//     pinMode(STATE_ACK_R_PIN, OUTPUT);

//     digitalWrite(STATE_ACK_S_PIN, LOW); // Ensure ACK starts LOW
//     digitalWrite(STATE_ACK_R_PIN, LOW);
//     // comm.begin(115200, 16, 17);
//     // Serial2.begin(115200, SERIAL_8N1, 16, 17);

//     Serial.begin(115200);
//     comm.begin(115200, 16, 17);

//     if (!thermalSensor.begin()) {
//         Serial.println("Sensor not found!");
//         while (true);
//     }
// }

// void SystemFlow_Run(){

//     sys_state = Get_State();

//     switch(sys_state){
//         case _000_RECONNING:
//             Handle_State_000_Reconning();
//             break;
//         case _001_AVOID_OBSTACLE:
//             Handle_State_001_Avoid_Obstacle();
//             break;
//         case _010_THM_DETECTED:
//             Handle_State_010_THM_Detected();
//             break;
//         case _011_SEND_DATA_TO_OPERATOR:
//             Handle_State_011_Send_Data_To_Operator();
//             break;
//         case _100_MANUAL_MODE:
//             Handle_State_100_Manual();
//             break;
//         default:
//             // Unknown state — do nothing
//             break;
//     }
//     delay(500);
//     Serial.println("___________________________________");
// }


// static SystemState Get_State(){
//     uint8_t b0 = digitalRead(STATE_0_PIN);
//     uint8_t b1 = digitalRead(STATE_1_PIN);
//     uint8_t b2 = digitalRead(STATE_2_PIN);
//     return static_cast<SystemState>((b2 << 2) | (b1 << 1) | b0);
// }

// static void Set_StateAck(){
//     digitalWrite(STATE_ACK_S_PIN, HIGH);
// }
// static void Reset_StateAck(){
//     digitalWrite(STATE_ACK_S_PIN, LOW);
// }
// static void Set_ReceiveAck(){
//     digitalWrite(STATE_ACK_R_PIN, HIGH);
// }
// static void Reset_RecieveAck(){
//     digitalWrite(STATE_ACK_R_PIN, LOW);
// }

// static void waitBeforeClearingAck() {
//     delay(100); // based on STM32 response
// }

// static void Handle_State_000_Reconning(void){
//     Reset_StateAck();
//     Reset_RecieveAck();

//     Serial.println("state 000 reconning\n");
//     delay(500);

//     while(Get_State() != _111_STM_ACK){
//         if (thermalSensor.detectHumanRelative()) {
//             comm.sendMessage(THM, "1");
//             Serial.println("THM:1\n");
//         } else {
//             comm.sendMessage(THM, "0");
//             Serial.println("THM:0\n");
//         }
//     }

//     Set_ReceiveAck();

// }
// static void Handle_State_001_Avoid_Obstacle(void){
//     Reset_StateAck();
//     Reset_RecieveAck();

//     Serial.println("state 001 avoid obstacle\n");


//     // delay(500);



// }
// static void Handle_State_010_THM_Detected(void){
//     // Reset_Ack();

//     // comm.enable();
//     // comm.handleIncoming();

//     // while(comm.GPS_GoogleMapsLink == "" || comm.PIR_Decision == PIR_EMPTY);

//     // GPS_ReceivedGoogleMapsLink = comm.GPS_GoogleMapsLink;
//     // PIR_ReceivedMotionDecision = comm.PIR_Decision;

//     // comm.disable();

//     // comm.GPS_GoogleMapsLink = "";
//     // comm.PIR_Decision = PIR_EMPTY;

//     // Set_Ack();

//     Serial.println("state 010 THM Detected\n");
//     delay(500);

//     Reset_StateAck();
//     Reset_RecieveAck();

//     // comm.enable();

//     delay(500);

//     // while(comm.GPS_GoogleMapsLink == "" || comm.PIR_Decision == PIR_EMPTY){
//     //     // Serial.println(".");
//     //     comm.handleIncoming();
//     // }
//     // Set_ReceiveAck();

//     bool gpsReceived = false;
//     bool pirReceived = false;
//     comm.GPS_GoogleMapsLink = "";
//     comm.PIR_Decision = PIR_EMPTY;

//     comm.enable();
//     unsigned long start = millis();
//     while (!(gpsReceived && pirReceived) && millis() - start < 10000) {
//         comm.handleIncoming();

//         if (!gpsReceived && comm.GPS_GoogleMapsLink != "") gpsReceived = true;
//         if (!pirReceived && comm.PIR_Decision == PIR_MOTION_DETECTED || comm.PIR_Decision == PIR_MOTION_NOT_DETECTED) pirReceived = true;
//     }
//     comm.disable();

//     if (gpsReceived && pirReceived) {
//         Serial.println("ESP32: ACK R sent:");
//         Set_ReceiveAck();  // Only after BOTH are confirmed valid
//     }
//     GPS_ReceivedGoogleMapsLink = comm.GPS_GoogleMapsLink;
//     PIR_ReceivedMotionDecision = comm.PIR_Decision;
//     comm.disable();
//     comm.GPS_GoogleMapsLink = "";
//     comm.PIR_Decision = PIR_EMPTY;

//     // Reset_RecieveAck();

//     Serial.println("ESP32: Data received:");
//     Serial.println(" - GPS: " + GPS_ReceivedGoogleMapsLink);
//     Serial.println(" - PIR: " + String(PIR_ReceivedMotionDecision));

//     Serial.println("ESP32: ACK S sent:");
//     Set_StateAck();  // Notify STM that ESP is ready

// }
// static void Handle_State_011_Send_Data_To_Operator(void){
//     Reset_StateAck();
//     Reset_RecieveAck();

//     Serial.println("state 011 send data\n");
//     delay(500);

//     // Set_Ack();
// }
// static void Handle_State_100_Manual(void){
//     Reset_StateAck();
//     Reset_RecieveAck();

//     // switch (input) {
//     //   case 'w': comm.sendDriveCommand('F'); break;
//     //   case 's': comm.sendDriveCommand('B'); break;
//     //   case 'a': comm.sendDriveCommand('L'); break;
//     //   case 'd': comm.sendDriveCommand('R'); break;

//     //   case 'j': comm.sendCameraCommand('L'); break;
//     //   case 'l': comm.sendCameraCommand('R'); break;
//     // }

//     // Set_Ack();
// }