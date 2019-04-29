/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stm32f030x6.h>
#include "stdint.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_dma.h"
#include "uart_utils.h"
#include "motor_control.h"
#include "bemf_detection.h"
#include "stdlib.h"
#include "stdbool.h"
#include "string.h"
#define	EMF_FSIZE	4
#define	EMF_SBITS	2
#define	POT_FSIZE	4
#define	POT_SBITS	2

void initDebugPot(void);
void local_delay(uint32_t dly);
void littleDelay(void);

uint32_t timer_duty = 0;
uint32_t prev_timer_duty = 0;

float fdty = 0;
uint32_t dty;

controlstate_t controlState = INIT;
controlstate_t prevControlState = INIT;

volatile uint16_t adc_v[6];

volatile uint16_t adcA = 0;
volatile uint16_t adcB = 0;
volatile uint16_t adcC = 0;

volatile uint16_t adcAPrint = 0;
volatile uint16_t adcBPrint = 0;
volatile uint16_t adcCPrint = 0;

uint16_t antAdcA = 0;
uint16_t antAdcB = 0;
uint16_t antAdcC = 0;

volatile uint8_t state = 10;
volatile uint8_t prvState = 0;

uint16_t upper_limit = 300;//875
//uint16_t lower_limit = 53;//525
//uint16_t lower_limit = (ARR_VALUE/7);
uint16_t lower_limit = 190;

char comma[2] = {',', 0};
char endLine[3] = {'\r', '\n', 0};
uint32_t uartDataDelay = 0;
//uint8_t dbugInt = 5;
int main(void){
	if((ARR_VALUE - 682) == 0){
		upper_limit = 150;//300
	}
	else if((ARR_VALUE - 1365) == 0){
		upper_limit = 500;//200
	}
	else if((ARR_VALUE - 2048) == 0){
		upper_limit = 300;
	}

	HAL_Init();
//	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
//	HAL_MspInit();
//	HAL_NVIC_SetPriority(TIM14_IRQn, 15, 1);
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 3, 1);

//	HAL_NVIC_EnableIRQ(TIM14_IRQn);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

	clkInit();
	pwmInit();
	gpioInit();
	uartInit();
	adcInit();
	initDebugPot();
	ramp_up_timer_init();
	dmaInit();
	uartDMAInit();
	pwm_input_init();
	dmaTransferConfig((uint32_t)(adc_v), 3);
	motor_high_z();
	local_delay(200000);

	motor_high_z();
	local_delay(300000);
	motor_init();
//	prepositioning(ARR_VALUE);
//	ramp_up();
//	motor_break();

	local_delay(300000);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_TC2(DMA1);
	uartDMATransfer((uint32_t)"@", 1);
	dmaTransferConfig((uint32_t)(adc_v), 3);
	LL_ADC_REG_StartConversion(ADC1);

	while(1){
//		LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
		if(LL_TIM_IsActiveFlag_CC2(TIM3)){
			prev_timer_duty = timer_duty;
			timer_duty = pwm_input_read();
		}

//*********************DEBUG Print

//		GPIOB->BSRR |= LL_GPIO_PIN_5;
//		GPIOB->BRR |= LL_GPIO_PIN_5;

//		uartDataDelay++;
//		if(uartDataDelay>10){
//			uint32_t a[] = {dty, LL_TIM_GetCounter(TIM14),state};
//			char string_debug[50];
//			debug_uart_print(a, 3, string_debug);
//			uartDataDelay = 0;
//		}


//*********************END DEBUG Print
//		GPIOB->BSRR |= LL_GPIO_PIN_5;

		//*************************
		fdty = (float)(prev_timer_duty + timer_duty)/2;
		fdty = ARR_VALUE*((float)fdty/2000);
//		dty = (uint16_t)fdty + 200;
		dty = (uint16_t)fdty + (lower_limit - 20);
		if(dty > ARR_VALUE) dty = (ARR_VALUE - 1);

		//*************************
//		dty = ARR_VALUE*pot_avrg;
//		dty = dty>>10;
//		dty = dty + pot_offset;
//		if(dty > ARR_VALUE) dty = (ARR_VALUE - 1);

//		GPIOB->BSRR |= LL_GPIO_PIN_5;
//		GPIOB->BRR |= LL_GPIO_PIN_5;


//		if( !LL_USART_IsActiveFlag_BUSY(USART1)){
//			adcAPrint = adc_v[0];
//			adcBPrint = adc_v[1];
//			adcCPrint = adc_v[2];
//		}
		//proto Z44N
		adcA =  adc_v[0];
		adcB =  adc_v[1];
		adcC =  adc_v[2];

//		GPIOB->BRR |= LL_GPIO_PIN_5;

		state = motor_zc_detection(adcA, adcB, adcC, state, controlState);

		if((controlState == OPENLOOP) && (prevControlState == INIT)){
			state = ramp_up(lower_limit);

			controlState = CLOSELOOP;
			ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2;
			DMA1_Channel1->CCR &= ~DMA_CCR_EN;
			DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR);
			DMA1_Channel1->CMAR = (uint32_t)(adc_v);
			DMA1_Channel1->CNDTR = 3;
			DMA1_Channel1->CCR |= DMA_CCR_EN;
			LL_TIM_EnableCounter(TIM14);
		}

		if(state > 60) state = 10;

		prevControlState = controlState;

		if(controlState == INIT){
			if((dty < lower_limit) || (dty > upper_limit)){
				controlState = INIT;
			}
			else if((dty > lower_limit) && (dty < upper_limit)){
				controlState = OPENLOOP;
			}
		}
		else if(controlState == OPENLOOP){
			if(dty < (lower_limit - 10)){
				controlState = INIT;
			}
		}
		else if(controlState == CLOSELOOP){
			if(dty < (lower_limit - 10)){
				controlState = INIT;
			}
			else{
				controlState = CLOSELOOP;
			}
		}

		if((state != prvState) && (controlState != INIT)){
			prvState = state;
			motor_update(dty, state);
		}//END if state != prvState

		if((controlState == INIT) && (prevControlState != controlState)){
			motor_high_z();
			littleDelay();
//			motor_break();
		}

	}//END while loop
}//END main()

void TIM1_Handler(void){
	ADC1->CR |= ADC_CR_ADSTART;
//	GPIOB->BSRR |= LL_GPIO_PIN_5;
//	GPIOB->BRR |= LL_GPIO_PIN_5;
//	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);

	if((TIM1->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF_Msk) LL_TIM_ClearFlag_CC1(TIM1);
	if((TIM1->SR & TIM_SR_CC2IF) == TIM_SR_CC2IF_Msk) LL_TIM_ClearFlag_CC2(TIM1);
	if((TIM1->SR & TIM_SR_CC3IF) == TIM_SR_CC3IF_Msk) LL_TIM_ClearFlag_CC3(TIM1);
}


void DMA1CH1_Handler(void){
	LL_DMA_ClearFlag_TC1(DMA1);
}

void TIM14_Handler(void){
	LL_TIM_ClearFlag_UPDATE(TIM14);
}

void local_delay(uint32_t dly){
	for(uint32_t d = 0; d<dly; d++){
		__asm__("nop");
	}
}

void littleDelay(void){
	for(int i = 0; i<2; i++){
		__asm("nop");
	}
}

void initDebugPot(void){
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
}

