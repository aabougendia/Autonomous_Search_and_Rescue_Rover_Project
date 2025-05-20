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
    DRV,
    SRV,
    UNKNOWN
} CommBus_MessageType;

typedef enum {
    PIR_MOTION_NOT_DETECTED = 0,
    PIR_MOTION_DETECTED = 1,
    PIR_EMPTY = 2
} PIR_MotionDecision;


class CommBus {
    public:
        CommBus(HardwareSerial& serialPort);

        String GPS_GoogleMapsLink = "";
        int ULT_Distance = 0;
        PIR_MotionDecision PIR_Decision = PIR_EMPTY;



        void begin(unsigned long baud, int rx, int tx);
        void handleIncoming();
        void sendMessage(CommBus_MessageType type, const String& payload);
        bool isMessageReady() const;

        void enable();
        void disable();


        void resetGPS();
        void resetULT();
        void resetPIR();
        void resetTHM();

    private:
        HardwareSerial& serial;

        unsigned long lastBaud = 115200;
        int rxPin = 16;
        int txPin = 17;

        bool enabled = true;

        char rxBuffer[RX_BUF_SIZE];
        size_t rxIndex = 0;
        bool messageReady = false;


        CommBus_MessageType getMessageType(const String& msg);
        String getPayload(const String& msg);
        void processByte(char byte);
        void dispatchMessage(const String& msg);

};

#endif















