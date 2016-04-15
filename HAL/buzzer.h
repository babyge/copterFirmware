/*
 *
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "hal.h"

#define BUZZER_BUFFERSIZE	16

struct {
	// stores the alarms to be played
	uint8_t alarmsequence[BUZZER_BUFFERSIZE];
	uint8_t readpos, writepos;
	// number of beeps already sounded and total beeps to be sounded
	uint8_t beepcount, beepcommand;
	//
	uint8_t waitcount;
	// 0: waiting, 1: buzzing
	uint8_t state;
} buzzer;

/*
 * initializes GPIO, timer and interrupt
 */
void buzzer_Init();
/*
 * adds a alarm signal to the buzzer sequence
 * @param peeps number of buzzer tones
 */
void buzzer_Signal(uint8_t peeps);

void TIM2_IRQHandler(void);

#endif /* BUZZER_H_ */
