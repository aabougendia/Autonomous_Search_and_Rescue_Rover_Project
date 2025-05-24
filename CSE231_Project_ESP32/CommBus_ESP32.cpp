#include "esp32-hal-gpio.h"
#include "CommBus_ESP32.h"

// Global variables (definition)
ManualState Manual_state = DRV_STOP;
THM_State thm_hum_state = THM_HUM_NOT_DETECTED;
// PIR_OUT pir_state = PIR_NO_MOTION;
AutoState auto_stat = IDLE;
ControlState ctrl_stat = STATE_MANUAL;






void CommBus_Init() {
    pinMode(COMM_AUTOMAN_STATE_PIN, OUTPUT);
    pinMode(COMM_AUTO0_PIN, INPUT);
    pinMode(COMM_AUTO1_PIN, INPUT);
    pinMode(COMM_HUM_FLAG_PIN, OUTPUT);

    pinMode(COMM_MAN0_AUTO_ACK_PIN, OUTPUT);
    pinMode(COMM_MAN1_PIN, OUTPUT);
    pinMode(COMM_MAN2_PIN, OUTPUT);
}

void Set_Man_Stat(ManualState state) {
    Manual_state = state;  // Update global state
    switch (state) {
        case DRV_STOP:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, LOW);
            digitalWrite(COMM_MAN1_PIN, LOW);
            digitalWrite(COMM_MAN2_PIN, LOW);
            break;
        case DRV_FWD:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, HIGH);
            digitalWrite(COMM_MAN1_PIN, LOW);
            digitalWrite(COMM_MAN2_PIN, LOW);
            break;
        case DRV_BWD:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, LOW);
            digitalWrite(COMM_MAN1_PIN, HIGH);
            digitalWrite(COMM_MAN2_PIN, LOW);
            break;
        case DRV_RIGHT:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, HIGH);
            digitalWrite(COMM_MAN1_PIN, HIGH);
            digitalWrite(COMM_MAN2_PIN, LOW);
            break;
        case DRV_LEFT:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, LOW);
            digitalWrite(COMM_MAN1_PIN, LOW);
            digitalWrite(COMM_MAN2_PIN, HIGH);
            break;
        case CAM_STOP:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, HIGH);
            digitalWrite(COMM_MAN1_PIN, LOW);
            digitalWrite(COMM_MAN2_PIN, HIGH);
            break;
        case CAM_RIGHT:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, LOW);
            digitalWrite(COMM_MAN1_PIN, HIGH);
            digitalWrite(COMM_MAN2_PIN, HIGH);
            break;
        case CAM_LEFT:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, HIGH);
            digitalWrite(COMM_MAN1_PIN, HIGH);
            digitalWrite(COMM_MAN2_PIN, HIGH);
            break;
        default:
            digitalWrite(COMM_MAN0_AUTO_ACK_PIN, LOW);
            digitalWrite(COMM_MAN1_PIN, LOW);
            digitalWrite(COMM_MAN2_PIN, LOW);
            break;
    }
}

void Set_THM_HUM(THM_State state) {
    thm_hum_state = state;  // Update global state
    digitalWrite(COMM_HUM_FLAG_PIN, state == THM_HUM_DETECTED ? HIGH : LOW);
}

void Set_Ctrl_State(ControlState state) {
    ctrl_stat = state;
    digitalWrite(COMM_AUTOMAN_STATE_PIN, state == STATE_MANUAL ? HIGH : LOW);
}

// PIR_OUT Get_PIR() {
//     int reading = digitalRead(COMM_MOTION_FLAG_PIN);
//     pir_state = (reading == HIGH) ? PIR_MOTION_DETECTED : PIR_NO_MOTION;
//     return pir_state;
// }

AutoState Get_Auto_State() {
    int auto0 = digitalRead(COMM_AUTO0_PIN);
    int auto1 = digitalRead(COMM_AUTO1_PIN);
    
    // Combine the two bits to form the state
    int state_value = (auto1 << 1) | auto0;
    
    // Cast to AutoState enum
    auto_stat = (AutoState)state_value;
    return auto_stat;
}

void Set_ESP_ACK(){
    digitalWrite(COMM_MAN0_AUTO_ACK_PIN, HIGH);
}
void Clear_ESP_ACK(){
    digitalWrite(COMM_MAN0_AUTO_ACK_PIN, LOW);
}
uint8_t Get_STM_ACK(){
    return digitalRead(COMM_AUTO1_PIN);
}
String Get_GPSLink(){
    String link = "";
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') break;
        link += c;
    }
    return link;
}