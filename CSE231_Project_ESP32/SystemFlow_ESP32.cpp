#include "esp32-hal.h"
#include <string>
#include "SystemFlow_ESP32.h"
#include "CommBus_ESP32.h"
#include "AMG8833.h"
#include "PIR_sensor.h"

String GPS_GoogleMapsLink;
uint8_t pir_state = PIR_NO_MOTION;
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
    PIR_Init();
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
    delay(100);
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

    // Set_ESP_ACK();

    // while(Get_STM_ACK() == LOW){
    //     Serial.println("getting info\n");
    // }

    String ReceivedGoogleMapsLink = Get_GPSLink();
    if(ReceivedGoogleMapsLink != ""){
        GPS_GoogleMapsLink = ReceivedGoogleMapsLink;

        // delay(2000);
        unsigned long startTime = millis();

        pir_state = 0;
        while (millis() - startTime < 1000) {
            pir_state |= PIR_Read();
        }

        Serial.println("info received\n");
    }



    // Clear_ESP_ACK();
    // if(GPS_GoogleMapsLink != ""){
        // Set_ESP_ACK();
        // pir_state = Get_PIR();
    // }

    // // while(Get_STM_ACK() != LOW){
    // //     Serial.println("waiting for stm ack low\n");
    // // }

    Serial.println("GPS: " + GPS_GoogleMapsLink + "\n");
    if(pir_state)
        Serial.println("PIR: 1\n");
    else
         Serial.println("PIR: 0\n");
    delay(500);





    GPS_GoogleMapsLink = "";
    pir_state = PIR_NO_MOTION;
}

static void Handle_AutoState_Idle(void){
    Serial.println("AUTO 2 : IDLE");
    Clear_ESP_ACK();
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