/*
 * hardware abstraction layer for LEDs
 *
 * LED 0, 1, 2: red
 * LED 3, 4: green
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f4xx.h"

typedef enum {
	LED_OFF = 0, LED_ON = 1, LED_TOGGLE = 2
} ledState;

uint16_t LED_GPIO[5];

/*
 * initializes output pins
 */
void led_Init(void);
/*
 * sets a new LED state
 */
void led_Cmd(uint8_t led, ledState s);

#endif /* LED_H_ */
