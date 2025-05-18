#ifndef COMMBUS_H
#define COMMBUS_H

#include <Arduino.h>

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 256

typedef enum {
    GPS,
    ULT,
    PIR,
    THM,
    UNKNOWN
} CommBus_MessageType;

typedef enum {
    PIR_MOTION_NOT_DETECTED = 0,
    PIR_MOTION_DETECTED = 1
} PIR_MotionDecision;


class CommBus {
    public:
        CommBus(HardwareSerial& serialPort);

        String GPS_GoogleMapsLink = "";
        int ULT_Distance = 0;
        PIR_MotionDecision PIR_Decision = PIR_MOTION_NOT_DETECTED;

        void begin(unsigned long baud);
        void handleIncoming();

        void sendMessage(CommBus_MessageType type, const String& payload);

        bool isMessageReady() const;


        void resetGPS();
        void resetULT();
        void resetPIR();
        void resetTHM();

    private:
        HardwareSerial& serial;

        char rxBuffer[RX_BUF_SIZE];
        size_t rxIndex = 0;
        bool messageReady = false;

        CommBus_MessageType getMessageType(const String& msg);
        String getPayload(const String& msg);
        void processByte(char byte);
        void dispatchMessage(const String& msg);

};

#endif















