/*
 * Controls up to 12 servos using the STM32 timer and PWM channels
 *
 * servo 1-4:
 * timer1:
 * channel1: PE9
 * channel2: PE11
 * channel3: PE13
 * channel4: PE14
 *
 * servo 5-8:
 * timer3:
 * channel1: PA6
 * channel2: PA7
 * channel3: PB0
 * channel4: PB1
 *
 * servo 9-12:
 * timer4:
 * channel1: PD12
 * channel2: PD13
 * channel3: PD14
 * channel4: PD15
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stdcomm.h"

#define SERVOUPDATERATE		50
#if SERVOUPDATERATE < 50
#error "servo update rate must be at least 50Hz"
#elif SERVOUPDATERATE > 490
#error "servo update rate can't be higher than 490Hz"
#endif

/*
 * initializes timer, pins and sets up pwm
 */
void servo_Init();
/*
 * updates a servo position
 * @param servo servo number from 0 to 11
 * @param position servo position, -1500=1ms; 0=1.5ms; 1500=2ms
 */
void servo_SetPosition(uint8_t servo, int16_t position);

#endif /* SERVO_H_ */
