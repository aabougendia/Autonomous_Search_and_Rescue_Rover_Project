#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "servo.h"


#define SERVO_MIN_PULSE 65   // 1ms = 0°
#define SERVO_MAX_PULSE 300   // 2ms = 180°
extern TIM_HandleTypeDef htim2;

void Servo_Init(){
	  HAL_TIM_PWM_Start(SERVO_TIMER, SERVO_TIMER_PWM_CH);
}
void Servo_SetAngle(uint8_t angle)
{
    uint32_t pulse_length;

    if (angle > 180) angle = 180; // Limit to valid range

    pulse_length = SERVO_MIN_PULSE + (uint32_t)((angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180.0); // in microseconds

    __HAL_TIM_SET_COMPARE(SERVO_TIMER, SERVO_TIMER_PWM_CH, pulse_length);

}



