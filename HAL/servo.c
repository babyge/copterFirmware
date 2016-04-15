#include "servo.h"

/*
 * initializes timer, pins and sets up pwm
 */
void servo_Init() {
	// activate timer clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// activate GPIO clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// initialize GPIOs
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	// configure i/o-pins
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOE, &gpio);
	// pins for timer1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

	// configure i/o-pins
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &gpio);
	// pins for timer3
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	// configure i/o-pins
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &gpio);
	// pins for timer4
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	// set timer prescalers
	TIM_TimeBaseInitTypeDef timer1;
	timer1.TIM_Prescaler = 55;  // prescaler = 56 -> 1/3 us resolution
	timer1.TIM_CounterMode = TIM_CounterMode_Up;
	timer1.TIM_Period = 3000000L / SERVOUPDATERATE; // set cycle time according to SERVOUPDATERATE
	timer1.TIM_ClockDivision = TIM_CKD_DIV1;
	timer1.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &timer1);

	TIM_TimeBaseInitTypeDef timer3;
	timer3.TIM_Prescaler = 27;  // prescaler = 28 -> 1/3 us resolution
	timer3.TIM_CounterMode = TIM_CounterMode_Up;
	timer3.TIM_Period = 3000000L / SERVOUPDATERATE; // set cycle time according to SERVOUPDATERATE
	timer3.TIM_ClockDivision = TIM_CKD_DIV1;
	timer3.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timer3);

	TIM_TimeBaseInitTypeDef timer4;
	timer4.TIM_Prescaler = 27;  // prescaler = 28 -> 1/3 us resolution
	timer4.TIM_CounterMode = TIM_CounterMode_Up;
	timer4.TIM_Period = 3000000L / SERVOUPDATERATE; // set cycle time according to SERVOUPDATERATE
	timer4.TIM_ClockDivision = TIM_CKD_DIV1;
	timer4.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timer4);

	// initialize output compare unit
	TIM_OCInitTypeDef oc;
	TIM_OCStructInit(&oc);
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 4500;
	// timer 1
	TIM_OC1Init(TIM1, &oc);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &oc);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &oc);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &oc);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	// timer 3
	TIM_OC1Init(TIM3, &oc);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &oc);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &oc);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &oc);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	// timer 4
	TIM_OC1Init(TIM4, &oc);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &oc);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &oc);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &oc);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	// start timers
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}
/*
 * updates a servo position
 * @param servo servo number from 0 to 11
 * @param position servo position, -1500=1ms; 0=1.5ms; 1500=2ms
 */
void servo_SetPosition(uint8_t servo, int16_t position) {
	if (position > 1500)
		position = 1500;
	else if (position < -1500)
		position = -1500;
	uint16_t compareValue = position + 4500;
	switch(servo) {
	case 0:
		TIM_SetCompare1(TIM1, compareValue);
		break;
	case 1:
		TIM_SetCompare2(TIM1, compareValue);
		break;
	case 2:
		TIM_SetCompare3(TIM1, compareValue);
		break;
	case 3:
		TIM_SetCompare4(TIM1, compareValue);
		break;
	case 4:
		TIM_SetCompare1(TIM3, compareValue);
		break;
	case 5:
		TIM_SetCompare2(TIM3, compareValue);
		break;
	case 6:
		TIM_SetCompare3(TIM3, compareValue);
		break;
	case 7:
		TIM_SetCompare4(TIM3, compareValue);
		break;
	case 8:
		TIM_SetCompare1(TIM4, compareValue);
		break;
	case 9:
		TIM_SetCompare2(TIM4, compareValue);
		break;
	case 10:
		TIM_SetCompare3(TIM4, compareValue);
		break;
	case 11:
		TIM_SetCompare4(TIM4, compareValue);
		break;
	}
//#ifdef DEBUG_UART
//	stdComm_puts("servo ");
//	stdComm_PrintValue(servo);
//	stdComm_puts(": ");
//	stdComm_PrintValue(position);
//	stdComm_putc(0x0a);
//#endif
}
