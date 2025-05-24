#ifndef SYSTEM_FLOW_ESP32_H
#define SYSTEM_FLOW_ESP32_H



void SystemFlow_Init();
void SystemFlow_Run();

static void initGSM();
static bool sendSMS(const String& message);
static bool sendAT(const String& command, unsigned long timeout = 1000);
static String readSIM800Response(unsigned long timeout);
static void log(const String& msg);


#endif
