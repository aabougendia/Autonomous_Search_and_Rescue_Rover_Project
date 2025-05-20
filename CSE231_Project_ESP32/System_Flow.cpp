#include "System_Flow.h"

#define STATE_0_PIN 5
#define STATE_1_PIN 18
#define STATE_2_PIN 19

// ACK output pin (ESP32 → STM32)
#define STATE_ACK_PIN 23

SystemState sys_state = _000_RECONNING;

CommBus comm(Serial2);

static SystemState Get_State();
static void Set_Ack();
static void Reset_Ack();
static void waitBeforeClearingAck();

static void Handle_State_000_Reconning(void);
static void Handle_State_001_Avoid_Obstacle(void);
static void Handle_State_010_THM_Detected(void);
static void Handle_State_011_Send_Data_To_Operator(void);
static void Handle_State_100_Manual(void);


void SystemFlow_Init(){
  pinMode(STATE_0_PIN, INPUT);
  pinMode(STATE_1_PIN, INPUT);
  pinMode(STATE_2_PIN, INPUT);
  pinMode(STATE_ACK_PIN, OUTPUT);

  digitalWrite(STATE_ACK_PIN, LOW); // Ensure ACK starts LOW

  comm.begin(115200, 16, 17);
}

void SystemFlow_Run(){
  
  sys_state = Get_State();

  switch(sys_state){
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
      // Unknown state — do nothing
      break;
  }
}


static SystemState Get_State(){
  uint8_t b0 = digitalRead(STATE_0_PIN);
  uint8_t b1 = digitalRead(STATE_1_PIN);
  uint8_t b2 = digitalRead(STATE_2_PIN);
  return static_cast<SystemState>((b2 << 2) | (b1 << 1) | b0);
}

static void Set_Ack(){
  digitalWrite(STATE_ACK_PIN, HIGH);
}
static void Reset_Ack(){
  digitalWrite(STATE_ACK_PIN, LOW);
}

static void waitBeforeClearingAck() {
  delay(100); // based on STM32 response
}

static void Handle_State_000_Reconning(void){
  Reset_Ack();

  Set_Ack();
}
static void Handle_State_001_Avoid_Obstacle(void){
  Reset_Ack();
}
static void Handle_State_010_THM_Detected(void){
  Reset_Ack();
  
  comm.enable();
  comm.handleIncoming();

  while(comm.GPS_GoogleMapsLink == "" || comm.PIR_Decision == PIR_EMPTY);

  comm.disable();

  comm.GPS_GoogleMapsLink = "";
  comm.PIR_Decision = PIR_EMPTY;

  Set_Ack();
  
}
static void Handle_State_011_Send_Data_To_Operator(void){
  Reset_Ack();

  Set_Ack();
}
static void Handle_State_100_Manual(void){
  Reset_Ack();

  // switch (input) {
  //   case 'w': comm.sendDriveCommand('F'); break;
  //   case 's': comm.sendDriveCommand('B'); break;
  //   case 'a': comm.sendDriveCommand('L'); break;
  //   case 'd': comm.sendDriveCommand('R'); break;

  //   case 'j': comm.sendCameraCommand('L'); break;
  //   case 'l': comm.sendCameraCommand('R'); break;
  // }

  Set_Ack();
}
