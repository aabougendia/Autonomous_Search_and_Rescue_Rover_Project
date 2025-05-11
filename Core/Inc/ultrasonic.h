
#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA

extern volatile uint32_t IC_Val1;
extern volatile uint32_t IC_Val2;
extern volatile uint32_t Difference;
extern volatile uint8_t Is_First_Captured;
extern volatile uint8_t Distance;


void delay (uint16_t time);
void HCSR04_init (void);
void HCSR04_Read (void);



#endif /* INC_ULTRASONIC_H_ */
