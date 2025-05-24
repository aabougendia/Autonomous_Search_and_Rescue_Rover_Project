#include <string>
#include "SystemFlow_ESP32.h"
#include "CommBus_ESP32.h"
#include "AMG8833.h"

String GPS_GoogleMapsLink;

AMG8833 thermalSensor;
float temperatureData[64];

ControlState control_state = STATE_AUTO;
AutoState sys_auto_state = RECONNING;
ManualState sys_manual_state = DRV_STOP;

static void Handle_AutoState_Reconning(void);
static void Handle_AutoState_SendInfo(void);
static void Handle_AutoState_Idle(void);

static void Handle_ManualState_DRV_STOP(void);
static void Handle_ManualState_DRV_FWD(void);
static void Handle_ManualState_DRV_BWD(void);
static void Handle_ManualState_DRV_RIGHT(void);
static void Handle_ManualState_DRV_LEFT(void);

static void Handle_ManualState_CAM_STOP(void);
static void Handle_ManualState_CAM_RIGHT(void);
static void Handle_ManualState_CAM_LEFT(void);

void SystemFlow_Init(){

    CommBus_Init();
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    Serial.begin(115200);

    if (!thermalSensor.begin()) {
    Serial.println("Sensor not found!");
    while (true);
    }

}
void SystemFlow_Run(){

    
    if(control_state == STATE_AUTO) {
        sys_auto_state = Get_Auto_State();
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
    }
    else if(control_state == STATE_MANUAL){

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
}

static void Handle_AutoState_Reconning(void){
    Serial.println("AUTO 0 : RECONNING");

    if (thermalSensor.detectHumanRelative()) {
        Serial.println("THM:1\n");
        Set_THM_HUM(THM_HUM_DETECTED);
    } else {
        Serial.println("THM:0\n");
        Set_THM_HUM(THM_HUM_NOT_DETECTED);
    }
}

static void Handle_AutoState_SendInfo(void){
    Serial.println("AUTO 1 : SEND INFO");

    Set_ESP_ACK();

    while(Get_STM_ACK() == LOW){
        Serial.println("getting info\n");
    }

    GPS_GoogleMapsLink = Get_GPSLink();
    pir_state = Get_PIR();

    Serial.println("info received\n");

    Clear_ESP_ACK();

    // while(Get_STM_ACK() != LOW){
    //     Serial.println("waiting for stm ack low\n");
    // }

    Serial.println("GPS: " + GPS_GoogleMapsLink + "\n");
    Serial.println("PIR: " + (pir_state == PIR_MOTION_DETECTED) ? "1" : "0");
}

static void Handle_AutoState_Idle(void){

}

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