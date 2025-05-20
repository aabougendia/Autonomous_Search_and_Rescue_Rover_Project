#ifndef SYSTEM_FLOW_H
#define SYSTEM_FLOW_H

#include <Arduino.h>
#include <Wire.h>

#include "AMG8833.h"
#include "CommBus.h"
#include "System_Flow.h"

enum SystemState {
  _000_RECONNING              = 0,
  _001_AVOID_OBSTACLE         = 1,
  _010_THM_DETECTED           = 2,
  _011_SEND_DATA_TO_OPERATOR  = 3,
  _100_MANUAL_MODE            = 4,
  UNDEFINED_STATE             = 5
};

extern CommBus comm;

void SystemFlow_Init();
void SystemFlow_Run();

#endif
