#include "externalADC.h"

/*
 * initializes pins, PWM, interrupts
 */
void externalADC_Init(void) {
	// Initialize GPIO pins
	/*
	 * ADC serial interface pins
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Pin = ADC_SCK | ADC_CS;
	GPIO_Init(GPIOB, &gpio);
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Pin = ADC_SDO;
	GPIO_Init(GPIOB, &gpio);
	// set initial pin states
	GPIOB->ODR |= ADC_CS;
	time_Wait100us(1);
	GPIOB->ODR |= ADC_SCK;
	time_Wait100us(1);
	GPIOB->ODR &= ~ADC_CS;
}
/*
 * checks whether a new conversion is available
 */
uint8_t externalADC_Ready(void) {
	if (!(GPIOB->IDR & GPIO_Pin_4)) {
		return 1;
	} else
		return 0;
}
/*
 * retrieves the converted data from the ADC
 */
void externalADC_ReadData(void) {
	/*
	 * read 24-Bit ADC data.
	 * 25 clock pulses are sent (last one resets the SDO line)
	 */
	pressure.rawADC = 0;
	int8_t i;
	for (i = 0; i < 32; i++) {
		// SCK low
		GPIOB->BSRRH = ADC_SCK;
		// wait 100ns -> 5MHz clock
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		// SCK high
		GPIOB->BSRRL = ADC_SCK;
		if (i < 24 && (GPIOB->IDR & ADC_SDO)) {
			pressure.rawADC |= (1L << (23 - i));
		}
		// wait 100ns -> 5MHz clock
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
	}
}

