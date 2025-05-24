#include "esp32-hal.h"
#include <Arduino.h>
#include "SystemFlow_ESP32.h"
#include "CommBus_ESP32.h"
#include "AMG8833.h"
#include "PIR_sensor.h"
#include "ManualControl.h"

#define LED_BUILTIN 2
#define PHONE_NUMBER "+201283600006" // Ensure correct country code

String GPS_GoogleMapsLink;
uint8_t pir_state = PIR_NO_MOTION;
AMG8833 thermalSensor;
float temperatureData[64];

// WebServer server(80);

ControlState control_state = STATE_AUTO;
AutoState sys_auto_state = RECONNING;

ManualState sys_manual_state = DRV_STOP;

// GSM function declarations
static void initGSM();
static bool sendSMS(const String& message);
static bool sendAT(const String& command, unsigned long timeout);
static String readSIM800Response(unsigned long timeout);
static void log(const String& msg);
static String extractCoordinates(const String& url); // New helper function

// Existing function declarations
static void Handle_AutoState_Reconning();
static void Handle_AutoState_SendInfo();
static void Handle_AutoState_Idle();
static void Handle_ManualState_DRV_STOP();
static void Handle_ManualState_DRV_FWD();
static void Handle_ManualState_DRV_BWD();
static void Handle_ManualState_DRV_RIGHT();
static void Handle_ManualState_DRV_LEFT();
static void Handle_ManualState_CAM_STOP();
static void Handle_ManualState_CAM_RIGHT();
static void Handle_ManualState_CAM_LEFT();

void SystemFlow_Init() {
    pinMode(LED_BUILTIN, OUTPUT);
    CommBus_Init();
    PIR_Init();
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // GPS on UART2
    Serial.begin(115200); // Debug
    Serial1.begin(9600, SERIAL_8N1, 12, 13); // GSM on UART1
    Drive_Init();

    if (!thermalSensor.begin()) {
        Serial.println("Sensor not found!");
        while (true);
    }

    log("Initializing GSM module...");
    initGSM();
}

void SystemFlow_Run() {
    Serial.println("start run");

    if (control_state == STATE_AUTO) {
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
                break;
        }
        delay(100);
    }
    else if (control_state == STATE_MANUAL) {
        Serial.println("start manual");
        server.handleClient();

        switch (man_state) {
            case DRV_STOP:
                Serial.println("DRV_STOP");
                break;
            case DRV_FWD:
                Serial.println("DRV_FWD");
                break;
            case DRV_BWD:
                Serial.println("DRV_BWD");
                break;
            case DRV_RIGHT:
                Serial.println("DRV_RIGHT");
                break;
            case DRV_LEFT:
                Serial.println("DRV_LEFT");
                break;
            case CAM_STOP:
                Serial.println("CAM_STOP");
                break;
            case CAM_RIGHT:
                Serial.println("CAM_RIGHT");
                break;
            case CAM_LEFT:
                Serial.println("CAM_LEFT");
                break;
            default:
                Serial.println("UNKNOWN");
                break;
        }
        Set_Manual_State(man_state);
    }
}

static void Handle_AutoState_Reconning() {
    Serial.println("AUTO 0 : RECONNING");

    if (thermalSensor.detectHumanRelative()) {
        Serial.println("THM:1\n");
        Set_THM_HUM(THM_HUM_DETECTED);
    } else {
        Serial.println("THM:0\n");
        Set_THM_HUM(THM_HUM_NOT_DETECTED);
    }
}

static void Handle_AutoState_SendInfo() {
    Serial.println("AUTO 1 : SEND INFO");

    String ReceivedGoogleMapsLink = Get_GPSLink();
    if (ReceivedGoogleMapsLink != "") {
        GPS_GoogleMapsLink = ReceivedGoogleMapsLink;

        int firstWww = GPS_GoogleMapsLink.indexOf("www");
        if (firstWww != -1) {
            int secondWww = GPS_GoogleMapsLink.indexOf("www", firstWww + 3);
            if (secondWww != -1) {
                GPS_GoogleMapsLink = GPS_GoogleMapsLink.substring(0, secondWww);
                log("Truncated GPS link at second 'www': " + GPS_GoogleMapsLink);
            }
        } else {
            log("Warning: No 'www' found in GPS link: " + GPS_GoogleMapsLink);
        }

        unsigned long startTime = millis();
        pir_state = 0;
        while (millis() - startTime < 1000) {
            pir_state |= PIR_Read();
        }

        Serial.println("info received\n");

        // Construct message with state and coordinates
        String cc = extractCoordinates(GPS_GoogleMapsLink);
        String state = (pir_state == 1) ? "Conscious" : "Not Conscious";
        String message = "HUMAN FOUND\nState: " + state + "\nLocation: " + cc;
        log("Message length: " + String(message.length()));
        if (message.length() > 160) {
            log("Message too long, truncating to 160 chars.");
            message = message.substring(0, 160);
        }

        log("Sending SMS with state and coordinates...");
        if (sendSMS(message)) {
            log("SMS sent successfully.");
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            log("Failed to send SMS.");
            digitalWrite(LED_BUILTIN, LOW);
        }

        GPS_GoogleMapsLink = "";
        pir_state = PIR_NO_MOTION;
    }

    delay(500);
}

static void Handle_AutoState_Idle() {
    Serial.println("AUTO 2 : IDLE");
    Clear_ESP_ACK();
}

static void initGSM() {
    Serial1.begin(9600, SERIAL_8N1, 12, 13);
    delay(10000); // Wait for GSM module to boot
    log("Sending basic AT test...");
    if (!sendAT("AT", 1000)) {
        log("Error: GSM module not responding to AT");
        return;
    }

    log("Checking SIM status...");
    String simStatus = readSIM800Response(1000);
    sendAT("AT+CPIN?", 1000);
    if (simStatus.indexOf("+CPIN: READY") == -1) {
        log("SIM card not ready!");
        return;
    }

    log("Checking signal quality...");
    sendAT("AT+CSQ", 1000);

    log("Getting SIM card info...");
    sendAT("AT+CCID", 1000);

    log("Checking network registration...");
    String regStatus = readSIM800Response(1000);
    sendAT("AT+CREG?", 1000);
    if (regStatus.indexOf("+CREG: 0,1") == -1 && regStatus.indexOf("+CREG: 0,5") == -1) {
        log("Not registered with network!");
        return;
    }

    log("Setting SMS text mode...");
    sendAT("AT+CMGF=1", 1000);

    log("Checking SMSC...");
    sendAT("AT+CSCA?", 1000); // Verify SMSC number

    log("Enabling delivery reports...");
    sendAT("AT+CNMI=2,2,0,0,0", 1000);

    log("GSM module initialization complete.");
}

static bool sendSMS(const String& message) {
    log("Setting SMS text mode before sending...");
    if (!sendAT("AT+CMGF=1", 1000)) {
        log("Failed to set SMS text mode.");
        return false;
    }

    String cmd = "AT+CMGS=\"" + String(PHONE_NUMBER) + "\"";
    log("Sending SMS command: " + cmd);
    if (!sendAT(cmd, 3000)) {
        log("Failed to initiate SMS send.");
        return false;
    }

    log("Sending message body...");
    Serial1.print(message);
    delay(100);
    log("Sending Ctrl+Z to finish message...");
    Serial1.write(26); // Ctrl+Z
    delay(5000);

    log("Waiting for SMS send confirmation...");
    String resp = readSIM800Response(5000);
    log("SMS send response: " + resp);

    bool success = resp.indexOf("OK") != -1 || resp.indexOf("+CMGS") != -1;
    log(success ? "SMS send successful." : "SMS send failed.");
    return success;
}

static bool sendAT(const String& command, unsigned long timeout) {
    Serial1.println(command);
    log("Sent command: " + command);
    String response = readSIM800Response(timeout);
    log("Received response: " + response);

    bool ok = response.indexOf("OK") != -1 || response.indexOf(">") != -1;
    if (!ok) log("Warning: Unexpected response for command: " + command);
    return ok;
}

static String readSIM800Response(unsigned long timeout) {
    String response = "";
    unsigned long start = millis();
    while (millis() - start < timeout) {
        while (Serial1.available()) {
            char c = Serial1.read();
            response += c;
        }
    }
    response.trim();
    if (response.length() == 0) {
        log("Warning: No response received within timeout.");
    }
    return response;
}

static void log(const String& msg) {
    Serial.print("[GSM:");
    Serial.print(millis());
    Serial.print("ms] ");
    Serial.println(msg);
}

static String extractCoordinates(const String& url) {
    // Expecting URL format: https://www.google.com/maps?q=latitude,longitude
    int startIndex = url.indexOf("?q=");
    if (startIndex == -1) {
        log("Invalid GPS URL format.");
        return "Unknown";
    }
    startIndex += 3; // Skip "?q="
    int endIndex = url.indexOf(",", startIndex);
    if (endIndex == -1) {
        log("Invalid GPS coordinates format.");
        return "Unknown";
    }
    String latitude = url.substring(startIndex, endIndex);
    String longitude = url.substring(endIndex + 1);
    return latitude + "," + longitude;
}

static void Handle_ManualState_DRV_STOP() {}
static void Handle_ManualState_DRV_FWD() {}
static void Handle_ManualState_DRV_BWD() {}
static void Handle_ManualState_DRV_RIGHT() {}
static void Handle_ManualState_DRV_LEFT() {}
static void Handle_ManualState_CAM_STOP() {}
static void Handle_ManualState_CAM_RIGHT() {}
static void Handle_ManualState_CAM_LEFT() {}