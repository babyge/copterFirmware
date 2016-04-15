/*
 * time.h
 *
 *  Created on: Oct 31, 2013
 *      Author: jan
 */

#ifndef TIME_H_
#define TIME_H_

#include "stm32f4xx.h"
#include "hal.h"
#include "copter.h"

// contains the time in 100us since calling time_Init()
volatile uint32_t time100us;

/*
 * initializes timer, variables and sets up interrrupt
 */
void time_Init(void);
/*
 * initializes a timer
 * @param timer pointer to the timer
 * @param time time in ms until the timer should elapse
 */
void time_SetTimer(uint32_t *timer, uint32_t time);
/*
 * checks whether a timer has elapsed
 * @param timer pointer to the timer
 * @return 0 timer still running / 1 timer elapsed
 */
uint8_t time_TimerElapsed(uint32_t *timer);
/*
 * returns the current system time in milliseconds
 */
int32_t time_GetMillis(void);
/*
 * returns the current system time in multiples of 100us
 */
int32_t time_Get100us(void);
/*
 * waits for n milliseconds
 */
void time_Waitms(uint32_t n);
/*
 * waits for multiples of 100 microseconds
 */
void time_Wait100us(uint32_t n);

void TIM7_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);
#endif /* TIME_H_ */
