/*
 * uart_utils.c
 *
 *  Created on: 10/02/2018
 *      Author: chepe
 */
#include "stm32f0xx.h"
#include <stm32f030x6.h>
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_bus.h"
#include "uart_utils.h"
#define POWERS	7

uint32_t powersOfTen[] = {1000000, 100000, 10000, 1000, 100, 10, 1};

void int_to_str(uint16_t n, char *vect);
uint8_t getIntegerLength(uint32_t interger);

void uartInit(void){
//	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//	GPIOA->AFR[0] |= GPIO_AFRL_AFRL2&(0x7<<8);
//	GPIOA->AFR[0] |= GPIO_AFRL_AFRL3&(0x7<<12);
////	LL_APB1_GRP2_PERIPH_USART1
//	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
////	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
//	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
//	USART1->CR1 &= ~USART_CR1_UE;
//	USART1->CR1 &= ~USART_CR1_OVER8;
//	USART1->BRR = 26;//26 -> 921600, 312 -> 115200, 32 -> 1125000
//	USART1->CR1 |= USART_CR1_UE;
//	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
//	LL_USART_EnableDMAReq_TX(USART1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_0);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_0);
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
	LL_USART_Disable(USART1);
//	while(LL_USART_IsActiveFlag_TC(USART1));
	LL_USART_SetOverSampling(USART1, LL_USART_OVERSAMPLING_16);
	LL_USART_SetBaudRate(USART1, 48000000, LL_USART_OVERSAMPLING_16, 921600);
	LL_USART_EnableDirectionRx(USART1);
	LL_USART_EnableDirectionTx(USART1);
	LL_USART_EnableDMAReq_TX(USART1);
	LL_USART_Enable(USART1);
	while(!LL_USART_IsActiveFlag_TC(USART1));

	for	(uint32_t d = 0; d<200000; d++){	//Wait for ~40ms
		__asm__("nop");
	}
}

void uartSendCharBlocking(uint8_t character){
	while(!(USART1->ISR&USART_ISR_TXE));
	USART1->TDR = character;
}

void uartSendStringBlocking(char *strToSend){
	uint8_t index = 0;
	while(strToSend[index]!= 0){
		uartSendCharBlocking(strToSend[index]);
		index++;
	}
}

void uartSendIntBlocking(int16_t number, uint8_t maxSize){
	char buffer[12];

//	snprintf(buffer,maxSize,"%i",number);
//	int_to_str(number,buffer);
	intToStr(number, maxSize, buffer);
	uartSendStringBlocking(buffer);
}

void uartSendFloatBlocking(float number, uint8_t maxSize){
	char buffer[12];
	snprintf(buffer, maxSize,"%f",number);
	uartSendStringBlocking(buffer);
}

void int_to_str(uint16_t n, char *vect){
	uint8_t cent;
	uint8_t dec;
	uint8_t uni;
	uint8_t mil;

	if(n>999){
		mil = n/1000;
	}else{
		mil = 0;
	}
	if(n>99) {
		cent = (n - mil*1000)/100;
	}else{
		cent = 0;
	}
	if(n>9){
		dec = (n - mil*1000 - cent*100)/10;
	}else{
		dec = 0;
	}
	uni = (n - mil*1000 - cent*100 - dec*10)/1;
	vect[0] = mil + '0';
	vect[1] = cent + '0';
	vect[2] = dec + '0';
	vect[3] = uni + '0';
	vect[4]	= 0;
}

void intToStr(uint16_t number, uint8_t nDigits, char *returnedArray){
	uint8_t numLength;
	uint8_t resStrIndx = 0;
	uint8_t intChar = 0;

	if(nDigits == 0){
		numLength = getIntegerLength((uint32_t)number);
	}else{
		numLength = nDigits;
	}

	for(uint8_t i = (POWERS - numLength); i < POWERS; i++){
		intChar = number/powersOfTen[i];
		number = number - (intChar*powersOfTen[i]);
		returnedArray[resStrIndx] = intChar + '0';
		resStrIndx++;
	}
	returnedArray[resStrIndx] = 0;
}

uint8_t getIntegerLength(uint32_t integer){
	uint8_t resLength = 0;

	for(uint8_t j = 1; j<POWERS; j++){
		if(integer > powersOfTen[(POWERS - j)]){
			resLength++;
		}else{
			break;
		}
	}

	return resLength;
}

void strConcat(char *strOne, char *strTwo, char *outStr){
	uint8_t outIndx = 0;
	uint8_t strIndx = 0;

	while(outStr[outIndx] != 0){
		outIndx++;
	}

	while(strOne[strIndx] != 0){
		outStr[outIndx] = strOne[strIndx];
		strIndx++;
		outIndx++;
	}

	strIndx = 0;

	while(strTwo[strIndx] != 0){
		outStr[outIndx] = strTwo[strIndx];
		strIndx++;
		outIndx++;
	}
	outStr[outIndx] = 0;
}

uint8_t getStrLen(char *str){
	uint8_t resLen = 0;
	while(str[resLen] != 0){
		resLen++;
	}
	return resLen;
}

void uartDMAInit(void){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_DMA_SetChannelPriorityLevel(DMA1,LL_DMA_CHANNEL_2,LL_DMA_PRIORITY_VERYHIGH);
	LL_DMA_ConfigTransfer(DMA1,LL_DMA_CHANNEL_2,(LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL| LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PRIORITY_VERYHIGH| LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE));
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_2);
}

void uartDMATransfer(uint32_t  msg, uint8_t trLen){
	LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_2);
	LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_2,(uint32_t)msg,(uint32_t)(&USART1->TDR),LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_2,trLen);
//	LL_DMA_ConfigTransfer(DMA1,LL_DMA_CHANNEL_7,(LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL| LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PRIORITY_MEDIUM| LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE));
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_2);
}

char* my_itoa(uint32_t i){
	/* Room for INT_DIGITS digits, - and '\0' */
	static char buf[INT_DIGITS + 2];
	char *p = buf + INT_DIGITS + 1;	/* points to terminating '\0' */
	if (i >= 0) {
		do {
			*--p = '0' + (i % 10);
			i /= 10;
		} while (i != 0);
		return p;
	}
	else {	/* i < 0 */
		do {
			*--p = '0' - (i % 10);
			i /= 10;
		} while (i != 0);
		*--p = '-';
	}
	return p;
}

void debug_uart_print(uint32_t array_values[], uint8_t array_len, char data_string[]){
	char *pointer_char = data_string;
	for(int i = 0; i < 50; i++){
		data_string[i] = 0;
	}
	if(!LL_USART_IsActiveFlag_BUSY(USART1) && LL_DMA_IsActiveFlag_TC2(DMA1)){
		LL_DMA_ClearFlag_TC2(DMA1);
		for(int j = 0; j < array_len; j++){
			strcat(data_string, my_itoa(array_values[j]));
			if(j < (array_len - 1)){
				strcat(data_string, ",");
			}
		}
		strcat(data_string, "\r\n");
		uartDMATransfer((uint32_t)pointer_char, (uint8_t)strlen(data_string));
	}

}
