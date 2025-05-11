/*
 * Ultrasonic_interface.h
 *
 *  Created on: May 11, 2025
 *      Author: rouka
 */

#ifndef INC_ULTRASONIC_INTERFACE_H_
#define INC_ULTRASONIC_INTERFACE_H_

extern volatile uint32_t IC_Val1 = 0;
extern volatile uint32_t IC_Val2 = 0;
extern volatile uint32_t Difference = 0;
extern volatile uint8_t Is_First_Captured = 0;
extern volatile uint8_t Distance = 0;

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA

void delay (uint16_t time);
void HCSR04_init (void);
void HCSR04_Read (void);



#endif /* INC_ULTRASONIC_INTERFACE_H_ */
