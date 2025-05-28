#include <Wire.h>
#include "AMG8833.h"

#include "SystemFlow_ESP32.h"

void setup() {
  SystemFlow_Init();
}

void loop() {
  SystemFlow_Run();
}