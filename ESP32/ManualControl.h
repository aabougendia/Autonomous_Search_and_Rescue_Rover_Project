#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <WiFi.h>
#include <WebServer.h>
#include "CommBus_ESP32.h"

extern WebServer server;
extern ManualState man_state;
extern ControlState control_state;

void Drive_Init();

#endif