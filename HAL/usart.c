#include "usart.h"

/*
 * initializes all used USARTS
 */
void usart_Init(void) {
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	NVIC_InitTypeDef nvic;

	/********************************************************
	 * USART2 (stdComm) initialization
	 *******************************************************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// uart pins are PD5 and PD6
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// connect alternate functions
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	// configure i/o-pins
	gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOD, &gpio);

	// initialize usart
	usart.USART_BaudRate = STDCOMM_BAUD;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &usart);

	// enable interrupt
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&nvic);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART2, ENABLE);
	/********************************************************
	 * USART1 (receiver) initialization
	 *******************************************************/
	// intialitze receiver struct values
	receiver.NumChannels = 0;
	receiver.valid = RESET;
	receiver.newData = RESET;
	receiver.timestamp = 0;
	receiver.byteCount = -1;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// USART pin is A10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// connect alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// configure i/o-pin
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOA, &gpio);

	// initialize USART
	usart.USART_BaudRate = 115200;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &usart);

	// enable interrupt
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&nvic);

	// activate USART receiver interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	// enable USART
	USART_Cmd(USART1, ENABLE);

	/********************************************************
	 * USART3 (HoTT) initialization
	 *******************************************************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// uart pins are PC8 and PC9
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// connect alternate functions
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	// configure i/o-pins
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOD, &gpio);

	// initialize usart
	usart.USART_BaudRate = 19200;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &usart);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART3, ENABLE);

	// enable interrupt
	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);

	// initialize timer 5 for HoTT timing
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseInitTypeDef timer5;
	timer5.TIM_ClockDivision = TIM_CKD_DIV1;
	timer5.TIM_CounterMode = TIM_CounterMode_Up;
	// overflow 1/ms
	timer5.TIM_Prescaler = 839;
	timer5.TIM_Period = 99;
	timer5.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &timer5);
	TIM_Cmd(TIM5, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

	// enable timer interrupt
	nvic.NVIC_IRQChannel = TIM5_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);

	/********************************************************
	 * UART6 (gps) initialization
	 *******************************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	// uart pins are PC6 and PC7
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// connect alternate functions
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	// configure i/o-pins
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOC, &gpio);

	// initialize usart
	usart_GPSSetBaudrate(38400);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART6, ENABLE);

	// enable interrupt
	nvic.NVIC_IRQChannel = USART6_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&nvic);

	/********************************************************
	 * UART4 (external sensor data) initialization
	 *******************************************************/

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	// USART RX pin is C11
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// connect alternate function
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	// configure i/o-pin
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOC, &gpio);

	// initialize USART
	usart.USART_BaudRate = EXTERNAL_DATA_BAUD;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(UART4, &usart);

	// enable interrupt
	nvic.NVIC_IRQChannel = UART4_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);

	// activate USART receiver interrupt
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	// enable USART
	USART_Cmd(UART4, ENABLE);

}
/*
 * wrapper to enable/disable the USART2 (stdComm) TXE interrupt
 */
void usart_NotifyReadyToSend(uint8_t interface, FunctionalState NewState) {
	if (interface == INTERFACE_GPS)
		USART_ITConfig(USART6, USART_IT_TXE, NewState);
	else if (interface == INTERFACE_STDCOMM)
		USART_ITConfig(USART2, USART_IT_TXE, NewState);
}

/*
 * changes the baudrate of the GPS-USART. (after longer periods without power
 * the GPS module switches back to the default 9600 instead of 38400)
 */
void usart_GPSSetBaudrate(uint32_t baud) {
	USART_InitTypeDef usart;
	// initialize usart
	usart.USART_BaudRate = baud;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART6, &usart);
}

void usart_putcStdComm(uint8_t c) {
	int32_t freeBufferSpace;
	do {
		freeBufferSpace = com.SendBufferReadPos - com.SendBufferWritePos;
		if (freeBufferSpace <= 0)
			freeBufferSpace += STDCOMM_SEND_BUFFER_SIZE;
	} while (freeBufferSpace < 2);
	com.SendBuffer[com.SendBufferWritePos] = c;
	stdcommIncSendWritePos();
	// enable TX buffer empty interrupt
	usart_NotifyReadyToSend(INTERFACE_STDCOMM, ENABLE);
}
/*
 * this function must be called as soon as new data can be send
 */
void usart_TransmitByteStdComm(void) {
	// check for further bytes to transmit
	if (com.SendBufferReadPos != com.SendBufferWritePos) {
		USART_SendData(USART2, com.SendBuffer[com.SendBufferReadPos]);
		stdcommIncSendReadPos();
	} else {
		// disable further interrupts
		usart_NotifyReadyToSend(INTERFACE_STDCOMM, DISABLE);
	}
}

void usart_putcGPS(uint8_t c) {
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART6, c);
}

/*
 * redirects the USART2 interrupts
 */
void USART2_IRQHandler(void) {
	if (USART_GetITStatus(USART2, USART_IT_TXE) == SET) {
		// call stdComm handler
		usart_TransmitByteStdComm();
	} else if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
		// store incoming data
		uint8_t data = USART_ReceiveData(USART2);
		// call stdComm handler
		stdComm_IncomingData(data);
	} else {
		// dummy read
		volatile uint8_t d = USART2->DR;
	}
}
/*
 * redirects the USART1 interrupts
 */
void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		// store incoming data
		uint8_t data = USART_ReceiveData(USART1);
		// call receiver handler
		receiver_IncomingData(data);
	} else {
		// dummy read
		volatile uint8_t d = USART1->DR;
	}
}
/*
 * redirects the USART3 interrupts to HoTT implementation
 */
void USART3_IRQHandler(void) {
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
		// store incoming data
		uint8_t data = USART_ReceiveData(USART3);
		// ignore bytes send from the MCU (TX and RX are connected)
		if (data == usart.hottLastByte) {
			usart.hottLastByte = 0xFFFF;
		} else {
			hott_IncomingData(data);
		}
	} else {
		// dummy read
		volatile uint8_t d = USART3->DR;
	}
}
/*
 * redirects the USART6 interrupts to GPS implementation
 */
void USART6_IRQHandler(void) {
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET) {
		// store incoming data
		uint8_t data = USART_ReceiveData(USART6);
		// call GPS data handler
		gps_IncomingData(data);
	} else {
		// dummy read
		volatile uint8_t d = USART6->DR;
	}
//	if (USART_GetITStatus(USART6, USART_IT_TXE) == SET) {
//		USART_ClearITPendingBit(USART6, USART_IT_TXE);
//		// call GPS handler
//		stdComm_TransmitByte();
//	}
}
/*
 * redirects the UART4 interrupts to the external sensor data handler
 */
void UART4_IRQHandler(void) {
	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET) {
		// store incoming data
		uint8_t data = USART_ReceiveData(UART4);
		// call external sensor data handler
		externalData_IncomingData(data);
	} else {
		// dummy read
		volatile uint8_t d = UART4->DR;
	}
}
/*
 * accesses the HoTT buffer and sends byte if necessary
 */
void TIM5_IRQHandler(void) {
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		if (hott.delay > 0) {
			hott.delay--;
			usart.hottChecksum = 0;
		} else if (hott.dataToBeSend == SET) {
			if (hott.bytes > 0) {
				// send next data byte
				usart.hottLastByte = *hott.outputPointer;
				USART_SendData(USART3, *hott.outputPointer);
				usart.hottChecksum += *hott.outputPointer;
				hott.outputPointer++;
				hott.bytes--;
			} else {
				// no further bytes to be transmitted
				// -> transmit checksum and finish HoTT transfer
				usart.hottLastByte = usart.hottChecksum;
				USART_SendData(USART3, usart.hottChecksum);
				hott.dataToBeSend = RESET;
			}
		}
	}
}
