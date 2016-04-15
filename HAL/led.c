#include "led.h"

/*
 * initializes output pins
 */
void led_Init(void) {
	LED_GPIO[0] = GPIO_Pin_0;
	LED_GPIO[1] = GPIO_Pin_1;
	LED_GPIO[2] = GPIO_Pin_2;
	LED_GPIO[3] = GPIO_Pin_3;
	LED_GPIO[4] = GPIO_Pin_4;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = LED_GPIO[0] | LED_GPIO[1] | LED_GPIO[2]
			| LED_GPIO[3] | LED_GPIO[4];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
/*
 * sets a new LED state
 */
void led_Cmd(uint8_t led, ledState s) {
	switch (s) {
	case LED_OFF:
		GPIOE->BSRRH = LED_GPIO[led];
		break;
	case LED_ON:
		GPIOE->BSRRL = LED_GPIO[led];
		break;
	case LED_TOGGLE:
		GPIOE->ODR ^= LED_GPIO[led];
		break;
	}
}
