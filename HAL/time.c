/*
 * time.c
 *
 *  Created on: Oct 31, 2013
 *      Author: jan
 */

#include "time.h"

/*
 * initializes timer, variables and sets up interrrupt
 */
void time_Init(void) {
	time100us = 0;

	//init timer7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseInitTypeDef timer;
	// overflow 10 times/ms
	timer.TIM_Period = 199;
	timer.TIM_Prescaler = 41;
	TIM_TimeBaseInit(TIM7, &timer);
	TIM_Cmd(TIM7, ENABLE);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	//enable timer interrupt
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM7_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	/*
	 * configure timer 8 for calling copter_ImportantTasks
	 */
	TIM_TimeBaseInitTypeDef timer8;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	// overflow 10 times/ms
	timer8.TIM_ClockDivision = TIM_CKD_DIV1;
	timer8.TIM_CounterMode = TIM_CounterMode_Up;
	timer8.TIM_Period = 99;
	timer8.TIM_Prescaler = 167;
	timer8.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &timer8);
	TIM_Cmd(TIM8, ENABLE);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

	//enable timer interrupt
	NVIC_InitTypeDef nvic8;
	nvic8.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
	nvic8.NVIC_IRQChannelPreemptionPriority = 3;
	nvic8.NVIC_IRQChannelSubPriority = 0;
	nvic8.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic8);
}
/*
 * initializes a timer
 * @param timer pointer to the timer
 * @param time time in ms until the timer should elapse
 */
void time_SetTimer(uint32_t *timer, uint32_t time) {
	*timer = time100us + time * 10;
}
/*
 * checks whether a timer has elapsed
 * @param timer pointer to the timer
 * @return 0 timer still running / 1 timer elapsed
 */
uint8_t time_TimerElapsed(uint32_t *timer) {
	if (*timer >= time100us)
		return 0;
	else
		return 1;
}
/*
 * returns the current system time in milliseconds
 */
int32_t time_GetMillis(void) {
	uint32_t buf = time100us;
	return buf / 10;
}
/*
 * returns the current system time in multiples of 100us
 */
int32_t time_Get100us(void) {
	return time100us;
}
/*
 * waits for n milliseconds
 */
void time_Waitms(uint32_t n) {
	volatile uint32_t starttime = time100us;
	while ((starttime + n * 10) > time100us)
		;
}
/*
 * waits for multiples of 100 microseconds
 */
void time_Wait100us(uint32_t n) {
	volatile uint32_t starttime = time100us;
	while ((starttime + n) > time100us)
		;
}
void TIM7_IRQHandler(void) {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		time100us++;
	}
}

void TIM8_UP_TIM13_IRQHandler(void) {
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
		copter_ImportantTasks();
	}
}

