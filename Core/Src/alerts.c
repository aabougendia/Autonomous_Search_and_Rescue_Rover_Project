#include "std_types.h"
#include "main.h"
#include "alerts.h"
#include "SystemFlow_STM32.h"

extern AutoState sys_auto_state;

void LED_vON(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void LED_vOFF(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void BUZZER_vON()
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void BUZZER_vOFF()
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void RGB_LED_vON(u8 R_State, u8 G_State, u8 B_State)
{

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, R_State);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, G_State);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, B_State);
}

void RGB_LED_vOFF()
{

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, LED_OFF);
}

void LED_vBlink(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, u32 delay)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(delay);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void RGB_LED_vBlink(u8 R_State, u8 G_State, u8 B_State, u32 delay)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, R_State);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, G_State);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, B_State);

	HAL_Delay(delay);

	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, LED_OFF);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, LED_OFF);
}



void AlertHandler_SysTickCallback(void) {
    static AutoState last_state = 255;
    static uint32_t counter = 0;
    static uint8_t is_on = 0;

    static const AlertPattern patterns[] = {
        [RECONNING]  = {0, 0, 1, 100, 2000, 1},    // Blue blink + beep
        [SEND_INFO]  = {1, 0, 0, 400, 1000, 1},   // Red blink
        [IDLE]       = {0, 1, 0, 0, 0, 0}          // Green steady
    };

    static AlertPattern current = {0};

    // Detect state change
    if (sys_auto_state != last_state) {
        last_state = sys_auto_state;
        current = patterns[sys_auto_state];
        counter = 0;
        is_on = 0;
    }

    // Always on
    if (current.on_ms == 0 && current.off_ms == 0) {
        RGB_LED_vON(current.R_on, current.G_on, current.B_on);
        BUZZER_vOFF();
        return;
    }

    counter++;
    uint32_t interval = is_on ? current.on_ms : current.off_ms;

    if (counter >= interval) {
        counter = 0;
        is_on = !is_on;

        if (is_on) {
            RGB_LED_vON(current.R_on, current.G_on, current.B_on);
            if (current.buzzer_on) BUZZER_vON();
        } else {
            RGB_LED_vOFF();
            BUZZER_vOFF();
        }
    }
}
