#define SERVO_TIMER &htim2
#define SERVO_TIMER_PWM_CH TIM_CHANNEL_2

void Servo_Init();
void Servo_SetAngle(uint8_t angle);
