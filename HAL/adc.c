#include "adc.h"

/*
 * sets up ADC
 */
void adc_Init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/*
	 * configure pins
	 */
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
			| GPIO_Pin_4 | GPIO_Pin_5;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);

	/*
	 * DMA2, Stream 0, Channel 0 configuration
	 */
	DMA_InitTypeDef dma;
	dma.DMA_Channel = DMA_Channel_0;
	dma.DMA_Memory0BaseAddr = (uint32_t) &adc.raw;
	dma.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = 6;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &dma);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/*
	 * configure ADC
	 */
	ADC_CommonInitTypeDef adcCommon;
	adcCommon.ADC_Mode = ADC_Mode_Independent;
	adcCommon.ADC_Prescaler = ADC_Prescaler_Div2;
	adcCommon.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	adcCommon.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&adcCommon);

	ADC_InitTypeDef adc;
	adc.ADC_Resolution = ADC_Resolution_12b;
	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfConversion = 6;
	ADC_Init(ADC1, &adc);

	// enable DMA on ADC1
	ADC_DMACmd(ADC1, ENABLE);

	// set ADC channels and conversion sequence
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_144Cycles);

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	// enable ADC
	ADC_Cmd(ADC1, ENABLE);

	// start ADC conversion
	ADC_SoftwareStartConv(ADC1);
}
