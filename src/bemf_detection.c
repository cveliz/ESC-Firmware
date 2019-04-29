/*
 * bemf_detection.c
 *
 *  Created on: 12/02/2018
 *      Author: chepe
 */
#include "stm32f0xx.h"
#include <stm32f030x6.h>
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_dma.h"
#include "bemf_detection.h"


void adcInit(void){
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
	LL_ADC_Disable(ADC1);
	while(LL_ADC_IsDisableOngoing(ADC1));
	LL_ADC_StartCalibration(ADC1);
	while(LL_ADC_IsCalibrationOnGoing(ADC1));
	LL_ADC_SetResolution(ADC1,LL_ADC_RESOLUTION_8B);
	LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
	LL_ADC_Enable(ADC1);
	while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_0,LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_1,LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_2,LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_0,LL_GPIO_PULL_NO);
	LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_1,LL_GPIO_PULL_NO);
	LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_2,LL_GPIO_PULL_NO);

	LL_ADC_SetDataAlignment(ADC1,LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetLowPowerMode(ADC1,LL_ADC_LP_MODE_NONE);
	LL_ADC_REG_SetTriggerSource(ADC1,LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetSequencerChannels(ADC1, (LL_ADC_CHANNEL_0 | LL_ADC_CHANNEL_1 | LL_ADC_CHANNEL_2));

	LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_7CYCLES_5);
	LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
	LL_ADC_REG_SetSequencerDiscont(ADC1,LL_ADC_REG_SEQ_DISCONT_DISABLE);
	LL_ADC_REG_SetContinuousMode(ADC1,LL_ADC_REG_CONV_SINGLE);
//	LL_ADC_REG_SetDMATransfer(ADC2,LL_ADC_REG_DMA_TRANSFER_NONE);
	ADC1->CFGR1 |= 0x01 << ADC_CFGR1_DMACFG_Pos;
	ADC1->CFGR1 |= 0x01 << ADC_CFGR1_DMAEN_Pos;
//	ADC1->CFGR1 &= ~(0x01 << ADC_CFGR1_DMACFG_Pos);
//	ADC1->CFGR1 |= ADC_CFGR_DMACFG_Msk;	//DMA Circular Mode
//	ADC1->CFGR1 |= ADC_CFGR_DMAEN_Msk;	//Enable DMA Request

	LL_ADC_EnableIT_EOC(ADC1);
	LL_ADC_DisableIT_EOC(ADC1);

	LL_ADC_Enable(ADC1);
//	LL_ADC_REG_StartConversion(ADC2);
//	LL_ADC_REG_StopConversion(ADC2);
	while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));
}

void dmaInit(void){
	/*Enable clock for DMA2*/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_DMA_SetChannelPriorityLevel(DMA1,LL_DMA_CHANNEL_1,LL_DMA_PRIORITY_VERYHIGH);
	LL_DMA_ConfigTransfer(DMA1,LL_DMA_CHANNEL_1,(LL_DMA_DIRECTION_PERIPH_TO_MEMORY| LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PRIORITY_VERYHIGH | LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD));
//	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
}
void dmaTransferConfig(uint32_t addrs, uint16_t len){
	/*Disable DMA2 Channel1*/
	LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_1);
	/*Configure the source and destination addresses*/
	LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_1,(uint32_t)(&ADC1->DR),(uint32_t)addrs,LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	/*Configure data length*/
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,len);
	/*Enable DMA2 Channel1*/
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
}
