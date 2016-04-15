
#ifndef USART_H_
#define USART_H_

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stdcomm.h"
#include "receiver.h"
#include "hott.h"
#include "externalSensors.h"
#include "stdcomm.h"

#define INTERFACE_GPS		1
#define INTERFACE_STDCOMM	0

struct {
	uint8_t hottChecksum;
	// contains the last transmitted byte (value>255 means no valid data)
	uint16_t hottLastByte;
} usart;

/*
 * initializes all used USARTS
 */
void usart_Init(void);
/*
 * wrapper to enable/disable the USART2 (stdComm) TXE interrupt
 */
void usart_NotifyReadyToSend(uint8_t interface, FunctionalState NewState);
/*
 * changes the baudrate of the GPS-USART. (after longer periods without power
 * the GPS module switches back to the default 9600 instead of 38400)
 */
void usart_GPSSetBaudrate(uint32_t baud);

void usart_putcStdComm(uint8_t c);

void usart_putcGPS(uint8_t c);
/*
 * this function must be called as soon as new data can be send
 */
void usart_TransmitByteStdComm(void);
/*
 * redirects the USART2 interrupts
 */
void USART2_IRQHandler(void);
/*
 * redirects the USART1 interrupts
 */
void USART1_IRQHandler(void);
/*
 * redirects the USART3 interrupts to HoTT implementation
 */
void USART3_IRQHandler(void);
/*
 * accesses the HoTT buffer and sends byte if necessary
 */
void TIM5_IRQHandler(void);
#endif /* USART_H_ */
