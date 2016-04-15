#include "externalI2C.h"

/*
 * initializes hardware
 */
void externalI2C_Init() {
	externalI2C.transmissionRunning = 0;

	NVIC_DisableIRQ(I2C2_ER_IRQn);
	NVIC_DisableIRQ(I2C2_EV_IRQn);
	NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);

	// initialize I2C
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	I2C_InitTypeDef i2c;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_ClockSpeed = 200000;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_Mode = I2C_Mode_I2C;
	I2C_Init(I2C2, &i2c);
	I2C_Cmd(I2C2, ENABLE);

	// configure interrupts
	I2C_ITConfig(I2C2, I2C_IT_EVT, ENABLE);
	I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);
	I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);

	// configure GPIO pins (PB10 and PB11)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &gpio);
	GPIOB->BSRRL |= GPIO_Pin_10;

	if (!(GPIOB->IDR & GPIO_Pin_11)) {
		stdComm_puts("external I2C: detected low SDA\n");
		log_LogFileEntry("external I2C: detected low SDA");
		// SDA is stuck low -> out-clock slaves by toggling SCL a few times
		uint8_t i;
		for (i = 0; i < 9; i++) {
			GPIOB->BSRRH |= GPIO_Pin_10;
			time_Wait100us(1);
			GPIOB->BSRRL |= GPIO_Pin_10;
			time_Wait100us(1);
//			stdComm_puts("I2C recovery clock\n");
		}
	}
	stdComm_puts("external I2C initialisation\n");

	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	// configure timer9 to detect timeouts
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	TIM_TimeBaseInitTypeDef timer;
	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	// overflow 1/ms
	timer.TIM_Period = 999;
	timer.TIM_Prescaler = 167;
	timer.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM9, &timer);
	TIM_Cmd(TIM9, ENABLE);
	TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);

	// configure NVIC
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = I2C2_ER_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = I2C2_EV_IRQn;
	NVIC_Init(&nvic);
	//enable timer interrupt
	nvic.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_Init(&nvic);
}

/*
 * starts the transmission of new values to the motor drivers
 * (transmission itself is handled by interrupt)
 */
void externalI2C_TriggerUpdate() {
	if (externalI2C.transmissionRunning == 0
			&& I2C_GetITStatus(I2C2, I2C_IT_STOPF) == RESET) {
		externalI2C.timeout = 50;
		externalI2C.transmissionNumber = 0;
		externalI2C.bytecount = 0;
		externalI2C.mode = I2C_Direction_Transmitter;
		externalI2C.transmissionRunning = 1;
		I2C_GenerateSTART(I2C2, ENABLE);
//		led_Cmd(4, LED_ON);
	}
}

void I2C2_EV_IRQHandler(void) {
	//led_Cmd(0, LED_OFF);
	if (I2C_GetITStatus(I2C2, I2C_IT_SB) == SET) {
//		led_Cmd(1, LED_OFF);
		I2C_Send7bitAddress(I2C2, 0x52 + 2 * externalI2C.transmissionNumber,
				externalI2C.mode);
		I2C_AcknowledgeConfig(I2C2, ENABLE);
		externalI2C.bytecount = 0;
	}
	if (I2C_GetITStatus(I2C2, I2C_IT_ADDR) == SET) {
//		led_Cmd(1, LED_OFF);
		// clear ADDR bit
		(void) (I2C2->SR2);
	}
	if (I2C_GetITStatus(I2C2, I2C_IT_TXE) == SET) {
		if (externalI2C.bytecount == 0) {
			// motor communication works
			motor.error[externalI2C.transmissionNumber] = 0;
			if (externalI2C.mode == I2C_Direction_Transmitter) {
				// send velocity data
				I2C_SendData(I2C2,
						motor.velocity[externalI2C.transmissionNumber]);
				externalI2C.bytecount = 1;
//			I2C_GenerateSTART(I2C2, ENABLE);
//			externalI2C.transmissionNumber++;
//			if (externalI2C.transmissionNumber >= config.motor.num) {
//				externalI2C.transmissionNumber = 0;
//				externalI2C.mode = I2C_Direction_Receiver;
//			}
			}
		} else {
			I2C_GenerateSTART(I2C2, ENABLE);
			if (externalI2C.bytecount == 1) {
				externalI2C.transmissionNumber++;
				if (externalI2C.transmissionNumber >= config.motor.num) {
					externalI2C.transmissionNumber = 0;
					externalI2C.mode = I2C_Direction_Receiver;
				}
				externalI2C.bytecount = 2;
			}
		}
	}
//	if (I2C_GetITStatus(I2C2, I2C_IT_TXE) == SET
//			&& I2C_GetITStatus(I2C2, I2C_IT_BTF) == SET) {
////		led_Cmd(1, LED_OFF);
//		I2C_GenerateSTART(I2C2, ENABLE);
//		if (externalI2C.bytecount == 1) {
//			externalI2C.transmissionNumber++;
//			if (externalI2C.transmissionNumber >= config.motor.num) {
//				externalI2C.transmissionNumber = 0;
//				externalI2C.mode = I2C_Direction_Receiver;
//			}
//			externalI2C.bytecount = 2;
//		}
//	}
	if (I2C_GetITStatus(I2C2, I2C_IT_RXNE) == SET
			|| I2C_GetITStatus(I2C2, I2C_IT_BTF) == SET) {
//		led_Cmd(1, LED_OFF);
		if (externalI2C.mode == I2C_Direction_Receiver) {
			if (externalI2C.bytecount == 0) {
				motor.rawCurrent[externalI2C.transmissionNumber] =
						I2C_ReceiveData(I2C2);
				I2C_AcknowledgeConfig(I2C2, DISABLE);
				externalI2C.transmissionNumber++;
				if (externalI2C.transmissionNumber >= config.motor.num) {
					// last motordata has been read
					I2C_GenerateSTOP(I2C2, ENABLE);
					externalI2C.transmissionRunning = 0;
//					led_Cmd(4, LED_OFF);
				} else {
					// setup next receiver cycle
					I2C_GenerateSTART(I2C2, ENABLE);
					externalI2C.bytecount = 0;
				}
				externalI2C.bytecount++;
			} else {
				I2C_ReceiveData(I2C2);
			}
		}
	}
}
void I2C2_ER_IRQHandler(void) {
//	led_Cmd(0, LED_OFF);
//	led_Cmd(1, LED_ON);
	if (I2C_GetITStatus(I2C2, I2C_IT_AF)) {
		// clear error flag
		I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
		// error during communication
		motor.error[externalI2C.transmissionNumber] = 1;
		// move on to next transmission
		externalI2C.transmissionNumber++;
		if (externalI2C.mode == I2C_Direction_Transmitter) {
			if (externalI2C.transmissionNumber >= config.motor.num) {
				externalI2C.transmissionNumber = 0;
				externalI2C.mode = I2C_Direction_Receiver;
			}
			I2C_GenerateSTART(I2C2, ENABLE);
		} else {
			if (externalI2C.transmissionNumber >= config.motor.num) {
				I2C_GenerateSTOP(I2C2, ENABLE);
				externalI2C.transmissionRunning = 0;
//				led_Cmd(4, LED_OFF);
			} else {
				I2C_GenerateSTART(I2C2, ENABLE);
			}
		}
		if (time_TimerElapsed(&externalI2C.nackMessageTimer)) {
			time_SetTimer(&externalI2C.nackMessageTimer, 500);
			log_LogFileEntry("WARNING: received NACK from motor driver");
		}
	} else {
		if (I2C_GetITStatus(I2C2, I2C_IT_SMBALERT)) {
			log_LogFileEntry("I2C error: SMBALERT");
		}
		if (I2C_GetITStatus(I2C2, I2C_IT_TIMEOUT)) {
			log_LogFileEntry("I2C error: TIMEOUT");
		}
		if (I2C_GetITStatus(I2C2, I2C_IT_PECERR)) {
			log_LogFileEntry("I2C error: PECERR");
		}
		if (I2C_GetITStatus(I2C2, I2C_IT_OVR)) {
			log_LogFileEntry("I2C error: OVR");
		}
		if (I2C_GetITStatus(I2C2, I2C_IT_ARLO)) {
			log_LogFileEntry("I2C error: ARLO");
		}
		if (I2C_GetITStatus(I2C2, I2C_IT_BERR)) {
			log_LogFileEntry("I2C error: BERR");
		}
		I2C2->SR1 &= ~0xFF00;
//		led_Cmd(2, LED_ON);
		//externalI2C.transmissionRunning = 0;
		if (time_TimerElapsed(&externalI2C.errorMessageTimer)) {
			time_SetTimer(&externalI2C.errorMessageTimer, 500);
			log_LogFileEntry("ERROR: external I2C");
		}
//		led_Cmd(4, LED_OFF);
		// clear all error flags
		// radical method -> re-initialize the I2C
		//I2C_SoftwareResetCmd(I2C2, ENABLE);
		//I2C_DeInit(I2C2);
		//externalI2C_Init();
	}
}
void TIM1_BRK_TIM9_IRQHandler(void) {
	if (TIM_GetITStatus(TIM9, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM9, TIM_IT_Update);
		if (externalI2C.transmissionRunning) {
			if (externalI2C.timeout > 0) {
				externalI2C.timeout--;
			} else {
//				led_Cmd(0, LED_ON);
//				stdComm_puts("I2C timeout\n");
				if (time_TimerElapsed(&externalI2C.timeoutMessageTimer)) {
					time_SetTimer(&externalI2C.timeoutMessageTimer, 500);
					log_LogFileEntry("TIMEOUT: external I2C");
				}
				externalI2C.transmissionRunning = 0;
				externalI2C_Init();
			}
//			// timeout occured
//			externalI2C.transmissionRunning = 0;
//			// radical method -> re-initialize the I2C
//			I2C_DeInit(I2C2);
//			externalI2C_Init();
		}
	}

}
