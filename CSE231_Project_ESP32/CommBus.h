// #ifndef COMMBUS_H
// #define COMMBUS_H

// #include <Arduino.h>

// enum CommBus_MessageType { GPS, ULT, PIR, THM, DRV, SRV, UNKNOWN };
// enum PIR_MotionDecision { PIR_MOTION_NOT_DETECTED = 0, PIR_MOTION_DETECTED = 1, PIR_EMPTY = 2 };

// class CommBus {
//   public:
//     CommBus(HardwareSerial& serial);
//     void begin(unsigned long baud, int rx, int tx);
//     void handleIncoming();
//     void sendMessage(CommBus_MessageType type, const String& payload);
//     void enable();
//     void disable();

//     String GPS_GoogleMapsLink = "";
//     int ULT_Distance = 0;
//     PIR_MotionDecision PIR_Decision = PIR_EMPTY;

//   private:
//     HardwareSerial& serial;
//     char rxBuffer[256];
//     size_t rxIndex = 0;
//     bool enabled = true;
//     bool messageReady = false;

//     CommBus_MessageType getMessageType(const String& msg);
//     String getPayload(const String& msg);
//     void processByte(char byte);
//     void dispatchMessage(const String& msg);
// };

// #endif
