#include <stdint.h>
#include "stm32f1xx_hal.h"

#define SERVO_MIN_PULSE 1000   // 1ms = 0°
#define SERVO_MAX_PULSE 2000   // 2ms = 180°
extern TIM_HandleTypeDef htim2;
/*
void servo_setangle(uint8_t angle) {
    if (angle > 180) angle = 180;

    // Map angle (0–180) to pulse width (1000–2000 µs)
    uint16_t pulse_width = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180;

    // Set compare value for TIM2 Channel 2 (CCR2)
    TIM2->CCR2 = pulse_width;  // Direct register access
}
*/
void Servo_Init(){
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}
void Servo_SetAngle(uint8_t angle)
{
    uint32_t pulse_length;

    if (angle > 180) angle = 180; // Limit to valid range

    // Map angle to pulse width:
    // Pulse = 500us (0 deg) to 2500us (180 deg)
    pulse_length = 65 + (uint32_t)((angle * (300 - 65)) / 180.0); // in microseconds

    // Convert microseconds to timer counts
    // Timer frequency = 1MHz → 1 count = 1us
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_length);
   //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, angle);

}

// You may need to define how your platform sets PWM values
// Assume TIMx->CCR1 controls PWM on channel 1
/*void setServoPulse(uint16_t pulse_width_us) {
    // Assuming timer is set up for 20ms period (50Hz), and 1 tick = 1us
    TIM1->CCR1 = pulse_width_us;
}*/

/*
typedef enum {
    SERVO_FORWARD,
    SERVO_BACKWARD,
    SERVO_STOP
} ServoDirection;

void controlContinuousServo(ServoDirection direction) {
    uint16_t pulse_width_us;

    switch (direction) {
        case SERVO_FORWARD:
            pulse_width_us = 2000; // Full speed forward (2ms)
            break;
        case SERVO_BACKWARD:
            pulse_width_us = 1000; // Full speed backward (1ms)
            break;
        case SERVO_STOP:
        default:
            pulse_width_us = 1500; // Stop (1.5ms)
            break;
    }

    setServoPulse(pulse_width_us);
}
*/
