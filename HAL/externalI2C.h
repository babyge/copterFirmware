/*
 * handles the I2C communication with external hardware
 * (right now, only motor drivers)
 */

#ifndef EXTERNALI2C_H_
#define EXTERNALI2C_H_

#include "hal.h"
#include "motor.h"

struct {
	uint8_t mode;
	uint8_t transmissionNumber;
	uint8_t bytecount;
	uint8_t transmissionRunning;
	// ms until timeout will occur (must be written by software after each
	// successful transmission)
	uint8_t timeout;
	uint32_t timeoutMessageTimer, errorMessageTimer, nackMessageTimer;
} externalI2C;

/*
 * initializes hardware
 */
void externalI2C_Init();

/*
 * starts the transmission of new values to the motor drivers
 * (transmission itself is handled by interrupt)
 */
void externalI2C_TriggerUpdate();

void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void);

#endif /* EXTERNALI2C_H_ */
