#include "PIR_sensor.h"

PIR_OUT PIR_Read(){
	return HAL_GPIO_ReadPin(PIR_SENSOR_GPIO_Port, PIR_SENSOR_Pin);
}
