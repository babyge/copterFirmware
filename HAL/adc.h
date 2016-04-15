/*
 * configures the ADC1 to sample the battery voltage and update
 * the battery structure using continuous scan mode and DMA
 */
#ifndef ADC_H_
#define ADC_H_

#include "battery.h"

struct{
	uint16_t raw[6];
} adc;

/*
 * sets up ADC
 */
void adc_Init(void);

#endif /* ADC_H_ */
