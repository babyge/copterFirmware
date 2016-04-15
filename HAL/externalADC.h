/*
 * externalADC.h
 *
 *  Created on: Nov 16, 2013
 *      Author: jan
 */

#ifndef EXTERNALADC_H_
#define EXTERNALADC_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "pressure.h"

/*
 * initializes pins
 */
void externalADC_Init(void);
/*
 * checks whether a new conversion is available
 */
uint8_t externalADC_Ready(void);
/*
 * retrieves the converted data from the ADC
 */
void externalADC_ReadData(void);

#endif /* EXTERNALADC_H_ */
