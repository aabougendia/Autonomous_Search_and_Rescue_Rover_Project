#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <WiFi.h>
#include <WebServer.h>
#include "CommBus_ESP32.h"

extern WebServer server;
extern ManualState man_state;
extern ControlState control_state;
// LED Pins
// const int ledLeft = 23;    // D23 - Left indicator
// const int ledRight = 21;   // D21 - Right indicator
// const int ledCenter = 4;   // D4 - Center indicator


void Wifi_Init();
void Drive_Init();
void Cam_Init();

#endif