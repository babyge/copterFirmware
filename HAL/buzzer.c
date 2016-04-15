#include "buzzer.h"

/*
 * initializes GPIO, timer and interrupt
 */
void buzzer_Init() {
	buzzer.readpos = 0;
	buzzer.writepos = 0;
	buzzer.beepcount = 0;
	buzzer.waitcount = 0;
	buzzer.state = 0;

	// configure GPIO (PE8)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_8;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE, &gpio);

	// configure timer 2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef timer;
	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	// overflow: 10 per second
	timer.TIM_Period = 999;
	timer.TIM_Prescaler = 8399;
	timer.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timer);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	//enable timer interrupt
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}
/*
 * adds a alarm signal to the buzzer sequence
 * @param peeps number of buzzer tones
 */
void buzzer_Signal(uint8_t peeps) {
	// check for free space in buffer
	int8_t freeSpace = buzzer.readpos - buzzer.writepos;
	if (freeSpace <= 0)
		freeSpace += BUZZER_BUFFERSIZE;
	// case 0: buffer full -> dismiss command
	if (freeSpace <= 1) {
		return;
	}
	// case 1: buffer empty -> save command
	if (freeSpace == BUZZER_BUFFERSIZE) {
		buzzer.alarmsequence[buzzer.writepos] = peeps;
		buzzer.writepos = (buzzer.writepos + 1) % BUZZER_BUFFERSIZE;
		return;
	}
	// case 2: buffer partial filled -> only add command, when the buffer doesn't already contain it
	uint8_t i;
	for (i = buzzer.readpos; i != buzzer.writepos;
			i = (i + 1) % BUZZER_BUFFERSIZE) {
		if (buzzer.alarmsequence[i] == peeps)
			// command already in buffer -> abort
			return;
	}
	// add command to buffer
	buzzer.alarmsequence[buzzer.writepos] = peeps;
	buzzer.writepos = (buzzer.writepos + 1) % BUZZER_BUFFERSIZE;
}

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (buzzer.state == 0) {
			// buzzer is currently not beeping
			if (buzzer.waitcount > 0)
				buzzer.waitcount--;
			else if (buzzer.readpos != buzzer.writepos) {
				// new comman available
				buzzer.beepcommand = 2 * buzzer.alarmsequence[buzzer.readpos];
				buzzer.beepcount = 0;
				buzzer.state = 1;
				buzzer.readpos = (buzzer.readpos + 1) % BUZZER_BUFFERSIZE;
			}
		} else {
			if (buzzer.beepcount < buzzer.beepcommand) {
				GPIOE->ODR ^= GPIO_Pin_8;
				buzzer.beepcount++;
			} else {
				// all beeps sounded -> go into wait state
				buzzer.waitcount = 5;
				buzzer.state = 0;
			}
		}
	}
}

