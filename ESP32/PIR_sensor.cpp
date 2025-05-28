#include "PIR_sensor.h"

void PIR_Init(){
	    pinMode(PIR_OUTPUT_PIN, INPUT);
}

uint8_t PIR_Read(){
	return digitalRead(PIR_OUTPUT_PIN);
}
