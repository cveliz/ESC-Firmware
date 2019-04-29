/*
 * motor_control.c
 *
 *  Created on: 3/02/2018
 *      Author: chepe
 */

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "uart_utils.h"
#include "motor_control.h"

void tim1CH1Init(void);
void tim1CH2Init(void);
void tim1CH3Init(void);


/* ___________________________________
 *| TIMER | CHANNEL | PIN  | H-BRIDGE |
 *|-------|---------|------|----------|
 *|   1   |   3     | PA10 |    A     |
 *|   1   |   2     | PA9  |    B     |
 *|   1   |   1     | PA8  |    C     |
 *|_______|_________|______|__________|
 * */

void outAHighSetDuty(uint16_t dutyA){
//	LL_TIM_SetCounter(TIM1,0);
	if(dutyA==0){
		TIM1->CCER &= ~TIM_CCER_CC1E;
		TIM1->DIER &= ~TIM_DIER_CC1IE; //Disable Capture Compare 3 Interrupt
//		TIM1->DIER &= ~TIM_DIER_UIE;
	}else{
		TIM1->CCER |= TIM_CCER_CC1E;
		TIM1->DIER |= TIM_DIER_CC1IE;  //Enable Capture Compare 3 Interrupt
//		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
//		TIM1->DIER |= TIM_DIER_UIE;
	}
	LL_TIM_OC_SetCompareCH1(TIM1,dutyA);
}

void outBHighSetDuty(uint16_t dutyB){
//	LL_TIM_SetCounter(TIM8,0);
	if(dutyB==0){
		TIM1->CCER &= ~TIM_CCER_CC2E;
		TIM1->DIER &= ~TIM_DIER_CC2IE;  //Disable Capture Compare 2 Interrupt
//		TIM1->DIER &= ~TIM_DIER_UIE;
	}else{
		TIM1->CCER |= TIM_CCER_CC2E;
		TIM1->DIER |= TIM_DIER_CC2IE;  //Enable Capture Compare 2 Interrupt
//		TIM1->DIER |= TIM_DIER_UIE;
	}
	LL_TIM_OC_SetCompareCH2(TIM1,dutyB);
}

void outCHighSetDuty(uint16_t dutyC){
//	LL_TIM_SetCounter(TIM8,0);
	if(dutyC==0){
		TIM1->CCER &= ~TIM_CCER_CC3E;
		TIM1->DIER &= ~TIM_DIER_CC3IE;  //Disable Capture Compare 1 Interrupt
//		TIM1->DIER &= ~TIM_DIER_UIE;
	}else{
		TIM1->CCER |= TIM_CCER_CC3E;
		TIM1->DIER |= TIM_DIER_CC3IE;  //Enable Capture Compare 1 Interrupt
//		TIM1->DIER |= TIM_DIER_UIE;
	}
	LL_TIM_OC_SetCompareCH3(TIM1,dutyC);
}

void outALowSetState(uint8_t outA){
	if(outA == HIGH){
		GPIOB->BSRR |= LL_GPIO_PIN_0;
	}else if(outA == LOW){
		GPIOB->BRR |= LL_GPIO_PIN_0;
	}
}

void outBLowSetState(uint8_t outB){
	if(outB == HIGH){
		GPIOB->BSRR |= LL_GPIO_PIN_1;
	}else if(outB == LOW){
		GPIOB->BRR |= LL_GPIO_PIN_1;
	}
}

void outCLowSetState(uint8_t outC){
	if(outC == HIGH){
		GPIOB->BSRR |= LL_GPIO_PIN_3;
	}else if(outC == LOW){
		GPIOB->BRR |= LL_GPIO_PIN_3;
	}
}

void clkInit(void){
	LL_SYSCFG_SetRemapDMA_ADC(LL_SYSCFG_ADC1_RMP_DMA1_CH1);
	LL_SYSCFG_SetRemapDMA_USART(LL_SYSCFG_USART1TX_RMP_DMA1CH2);
	LL_RCC_HSI14_Enable();
	while(!LL_RCC_HSI14_IsReady());
	LL_ADC_SetClock(ADC1,LL_ADC_CLOCK_ASYNC );	//HSI14 14MHz
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
	LL_RCC_HSE_Disable();
	while(LL_RCC_HSE_IsReady());
	LL_RCC_HSE_DisableBypass();
	LL_RCC_HSI_Enable();
	while(!LL_RCC_HSI_IsReady());
	LL_RCC_PLL_Disable();
	while(LL_RCC_PLL_IsReady());
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	LL_RCC_PLL_Enable();
	while(!LL_RCC_PLL_IsReady());
}

void gpioInit(void){
	/*Enable clock for GPIOA, GPIOB, GPIOC*/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/*Setting PA8 as PWM output  TIM1 CH1 HIGH C*/
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA,LL_GPIO_PIN_8,LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetAFPin_8_15(GPIOA,LL_GPIO_PIN_8,LL_GPIO_AF_2);

	/*Setting PA9 as PWM output  TIM1  CH2 HIGH B*/
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA,LL_GPIO_PIN_9,LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetAFPin_8_15(GPIOA,LL_GPIO_PIN_9,LL_GPIO_AF_2);

	/*Setting PA10 as PWM output  TIM1  CH3 HIGH A*/
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA,LL_GPIO_PIN_10,LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetAFPin_8_15(GPIOA,LL_GPIO_PIN_10,LL_GPIO_AF_2);

	/*Setting PB0 as output LOW C*/
	LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_0,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB,LL_GPIO_PIN_0,LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB,LL_GPIO_PIN_0,LL_GPIO_SPEED_FREQ_HIGH);

	/*Setting PB1 as output LOW B*/
	LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_1,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB,LL_GPIO_PIN_1,LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB,LL_GPIO_PIN_1,LL_GPIO_SPEED_FREQ_HIGH);

	/*Setting PB3 as output LOW A*/
	LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_3,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB,LL_GPIO_PIN_3,LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB,LL_GPIO_PIN_3,LL_GPIO_SPEED_FREQ_HIGH);

	/*Setting PB5 as Debug output*/
	LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_5,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB,LL_GPIO_PIN_5,LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB,LL_GPIO_PIN_5,LL_GPIO_SPEED_FREQ_HIGH);
}

void ramp_up_timer_init(void){
	LL_TIM_SetClockSource(TIM14, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
	LL_TIM_SetPrescaler(TIM14,47);
	LL_TIM_SetCounterMode(TIM14,LL_TIM_COUNTERMODE_UP);// Interrupt at the end of the PWM?
	LL_TIM_SetAutoReload(TIM14,0xFFFF);
	LL_TIM_EnableARRPreload(TIM14);
	LL_TIM_DisableCounter(TIM14);
	LL_TIM_EnableUpdateEvent(TIM14);
}

void pwmInit(void){
	tim1CH1Init();
	tim1CH2Init();
	tim1CH3Init();
}

void tim1CH1Init(void){
	LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
	/*Enable clock TIM1*/
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
	/*Set TIM1 pre-scaler to 0, total pre-scaler 0 + 1*/
	LL_TIM_SetPrescaler(TIM1,0);
	/*Set ARR to ARR_VALUe, 20KHz PWM*/
	LL_TIM_SetAutoReload(TIM1,ARR_VALUE);
	/*Enable ARR pre-load*/
	LL_TIM_EnableARRPreload(TIM1);
	/*Set PWM1 on TIM1 CH2*/
	LL_TIM_DisableCounter(TIM1);
	LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_EnablePreload(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnableFast(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_SetCounterMode(TIM1,LL_TIM_COUNTERMODE_DOWN);
	LL_TIM_EnableAllOutputs(TIM1);
	/*Enable Update Event*/
	LL_TIM_EnableUpdateEvent(TIM1);
	/*Generate an update event on TIM1*/
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_TIM_OC_SetCompareCH1(TIM1,0);
	LL_TIM_EnableCounter(TIM1);
	LL_ADC_REG_IsConversionOngoing(ADC1);
}

void tim1CH2Init(void){
	/*Set PWM1 on TIM1 CH2*/
	LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH2,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_EnablePreload(TIM1,LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableUpdateEvent(TIM1);
	/*Generate an update event on TIM1*/
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_TIM_OC_SetCompareCH2(TIM1,0);
	LL_TIM_EnableCounter(TIM1);
}

void tim1CH3Init(void){
	/*Set PWM1 on TIM1 CH3*/
	LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH3,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_EnablePreload(TIM1,LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableUpdateEvent(TIM1);
	/*Generate an update event on TIM1*/
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_TIM_OC_SetCompareCH3(TIM1,0);
	LL_TIM_EnableCounter(TIM1);
}

void pwm_input_init(void){
	LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_1);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_1);

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_TIM_SetPrescaler(TIM3, 23);
	LL_TIM_SetAutoReload(TIM3, 40000);

	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;
	TIM3->SMCR &= ~(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2);
	TIM3->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0;
	TIM3->SMCR |= TIM_SMCR_SMS_2;
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
	TIM3->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
	TIM3->CR1 |= TIM_CR1_CEN;

//	LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1,LL_TIM_ACTIVEINPUT_DIRECTTI);
//	LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
//	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_TI1FP1);
//	LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_RESET);
//	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
//	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
//	LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
//	LL_TIM_EnableIT_CC1(TIM3);
//	LL_TIM_EnableIT_CC2(TIM3);
//	LL_TIM_EnableCounter(TIM3);
}

uint32_t pwm_input_read(void){
	uint32_t timer_value;
	timer_value = LL_TIM_IC_GetCaptureCH2(TIM3) + 7;

	if(timer_value < 2000){
		timer_value = 2000;
	}

	if(timer_value > 4000){
		timer_value = 4000;
	}

	timer_value -= 2000;
	if(LL_TIM_IsActiveFlag_CC2(TIM3)){
	}

	return timer_value;
}

void motor_init(void){
	motor_high_z();
	local_delay(300000);
	uint16_t initDuty = ARR_VALUE/8;
	uint16_t seqDelay = 300;

	for(int i = 0; i < seqDelay; i++){
		outAHighSetDuty(initDuty);
		outALowSetState(LOW);
		outBHighSetDuty(0);
		outBLowSetState(HIGH);
		outCHighSetDuty(0);
		outCLowSetState(LOW);

		local_delay(1000);

		motor_high_z();

		local_delay(1000);
	}

	for(int i = 0; i < seqDelay; i++){
		outAHighSetDuty(0);
		outALowSetState(HIGH);
		outBHighSetDuty(0);
		outBLowSetState(LOW);
		outCHighSetDuty(initDuty);
		outCLowSetState(LOW);

		local_delay(1000);

		motor_high_z();

		local_delay(1000);
	}

	for(int i = 0; i < seqDelay; i++){
		outAHighSetDuty(0);
		outALowSetState(LOW);
		outBHighSetDuty(initDuty);
		outBLowSetState(LOW);
		outCHighSetDuty(0);
		outCLowSetState(HIGH);

		local_delay(1000);

		motor_high_z();

		local_delay(1000);
	}
}

void motor_break(void){
	outAHighSetDuty(0);
	outALowSetState(HIGH);
	outBHighSetDuty(0);
	outBLowSetState(HIGH);
	outCHighSetDuty(0);
	outCLowSetState(HIGH);
}

void motor_high_z(void){
	outAHighSetDuty(0);
	outALowSetState(LOW);
	outBHighSetDuty(0);
	outBLowSetState(LOW);
	outCHighSetDuty(0);
	outCLowSetState(LOW);
}

void motor_prepositioning(uint16_t arrValue){
	uint16_t maxDuty = arrValue/4;
	uint16_t vDuty[25];

	for(uint8_t i = 0; i < 25; i++){
		float tmpd = (float)i/(float)24;
		tmpd = maxDuty*tmpd;
		vDuty[i] = (uint16_t)tmpd;
	}

	for(uint8_t i = 0; i < 20; i++){
//		TIM14->CNT = 0;
		outAHighSetDuty(0);
		outALowSetState(HIGH);
		outBHighSetDuty(vDuty[i]);
		outBLowSetState(LOW);
		outCHighSetDuty(0);
		outCLowSetState(HIGH);
//		while(TIM14->CNT < 5000);
		local_delay(100000); //33.9 ms
//		local_delay(150000); //50.9 ms
//		local_delay(250000); //50.9 ms
		LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
//		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
//		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
	}
//	LL_TIM_DisableCounter(TIM14);
//	HAL_NVIC_EnableIRQ(TIM14_IRQn);
}

uint8_t ramp_up(uint16_t rampup_duty){
	uint8_t ruSteps = 40;
	uint32_t uart_delay = 0;

	uint32_t timerDelay = 1250; //us
	uint32_t arrTimerDelay[ruSteps];
	uint8_t ruState = 10;
	//uint8_t ruStatIndex = 10;
	uint16_t ruDuty = 0;

	ruDuty = rampup_duty;

	uint8_t startupBEMFCounter = 0;
	uint8_t stPrevDetection = 0;

	uint16_t ruPhaseA = 0;
	uint16_t ruPhaseB = 0;
	uint16_t ruPhaseC = 0;
	uint16_t arrADC[3] = {0,0,0};

	HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);

	ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2;
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(arrADC);
	DMA1_Channel1->CNDTR = 3;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

	for (int t = 0; t < ruSteps; t++){
		float fDelay = ((float)t)/((float)(ruSteps - 1));
		fDelay = (float)timerDelay*fDelay;

		arrTimerDelay[t] = (uint32_t)fDelay;
		arrTimerDelay[t] = timerDelay - arrTimerDelay[t];
		arrTimerDelay[t] *= arrTimerDelay[t];
		arrTimerDelay[t] = arrTimerDelay[t] >> 9;
		arrTimerDelay[t] += 30;
	}
//	char string_debug[255];
//	debug_uart_print(arrTimerDelay, ruSteps, string_debug);
//	uartDMATransfer((uint32_t)"********\r\n", 10);
//	while(LL_USART_IsActiveFlag_BUSY(USART1));

	LL_TIM_EnableCounter(TIM14);
	for(int i = 0; i < ruSteps; i++){
		for(int s = 0; s < 2; s++){

			uint32_t cntr = LL_TIM_GetCounter(TIM14);
			while((LL_TIM_GetCounter(TIM14) - cntr) < arrTimerDelay[i]){
				uart_delay++;
				if(uart_delay>1){
					uint32_t a[] = { i, startupBEMFCounter};
					char string_debug[50];
					debug_uart_print(a, 2, string_debug);
					uart_delay = 0;
				}

//				if(LL_TIM_IsActiveFlag_CC2(TIM3)){
//					ruDuty = pwm_input_read();
//				}
				ruPhaseA = arrADC[0];
				ruPhaseB = arrADC[1];
				ruPhaseC = arrADC[2];

				startupBEMFCounter = motor_rampup_zc(ruState, ruPhaseA, ruPhaseB, ruPhaseC, &stPrevDetection, startupBEMFCounter);
				if(startupBEMFCounter >= 2) {
					i = ruSteps + 5;
					break;
				}
				motor_update(ruDuty, ruState);
			}
			ruState += 10;
			if(ruState > 60) ruState = 10;
		}
	}
	LL_TIM_DisableCounter(TIM14);
	LL_TIM_DisableCounter(TIM14);
	return ruState;
}

uint8_t motor_zc_detection(uint16_t phaseA, uint16_t phaseB, uint16_t phaseC, uint8_t step, controlstate_t control_state){
	uint16_t ths = 0;
	uint16_t ths_offset = 0;
	switch (step) {
	case 10:
		ths = (phaseA - phaseC)/2;
		ths += ths_offset;
		if(phaseB < ths){
			if(control_state == CLOSELOOP){
				step += 10;
			}
		}
		break;
	case 20:
		ths = (phaseA - phaseB)/2;
		ths -= ths_offset;
		if(phaseC > ths){
			if(control_state == CLOSELOOP){
				step += 10;
			}
		}
		break;
	case 30:
		ths = (phaseC - phaseB)/2;
		ths += ths_offset;
		if(phaseA < ths){
			if(control_state == CLOSELOOP){
				step += 10;
			}
		}
		break;
	case 40:
		ths = (phaseC - phaseA)/2;
		ths -= ths_offset;
		if(phaseB > ths){
			if(control_state == CLOSELOOP){
				step += 10;
			}
		}
		break;
	case 50:
		ths = (phaseB - phaseA)/2;
		ths += ths_offset;
		if(phaseC < ths){
			if(control_state == CLOSELOOP){
				step += 10;
			}
		}
		break;
	case 60:
		ths = (phaseB - phaseC)/2;
		ths -= ths_offset;
		if(phaseA > ths){
			if(control_state == CLOSELOOP){
				step += 10;
			}
		}
		break;
	default:
		break;
	}

	return step;
}

void motor_update(uint16_t duty, uint8_t step){
	switch(step){
	case 10:
		motor_high_z();
		outAHighSetDuty(duty);
		outALowSetState(LOW);
		outBHighSetDuty(0);
		outBLowSetState(LOW);
		outCHighSetDuty(0);
		outCLowSetState(HIGH);
		break;
	case 20:
		motor_high_z();
		outAHighSetDuty(duty);
		outALowSetState(LOW);
		outBHighSetDuty(0);
		outBLowSetState(HIGH);
		outCHighSetDuty(0);
		outCLowSetState(LOW);
		break;
	case 30:
		motor_high_z();
		outCHighSetDuty(duty);
		outCLowSetState(LOW);
		outAHighSetDuty(0);
		outALowSetState(LOW);
		outBHighSetDuty(0);
		outBLowSetState(HIGH);
		break;
	case 40:
		motor_high_z();
		outAHighSetDuty(0);
		outALowSetState(HIGH);
		outBHighSetDuty(0);
		outBLowSetState(LOW);
		outCHighSetDuty(duty);
		outCLowSetState(LOW);
		break;
	case 50:
		motor_high_z();
		outAHighSetDuty(0);
		outALowSetState(HIGH);
		outBHighSetDuty(duty);
		outBLowSetState(LOW);
		outCHighSetDuty(0);
		outCLowSetState(LOW);
		break;
	case 60:
		motor_high_z();
		outAHighSetDuty(0);
		outALowSetState(LOW);
		outBHighSetDuty(duty);
		outBLowSetState(LOW);
		outCHighSetDuty(0);
		outCLowSetState(HIGH);
		break;
	default:
		break;
	}
}

uint8_t motor_rampup_zc(uint8_t state, uint16_t phaseA, uint16_t phaseB, uint16_t phaseC, uint8_t* prevDetection, uint8_t bemfCounter){
	uint16_t threshold;

	switch(state){
	case 10://10
		threshold = (phaseA - phaseC)/2;
		if(phaseB<threshold){
			if(*prevDetection == 60){
				*prevDetection = 10;
				bemfCounter += 1;
			}else{
				*prevDetection = 10;
				bemfCounter = 0;
			}
		}
		break;
	case 20://20
		threshold = (phaseA - phaseB)/2;
		if(phaseC>threshold){
			if(*prevDetection == 10){
				*prevDetection = 20;
				bemfCounter += 1;
			}else{
				*prevDetection = 20;
				bemfCounter = 0;
			}
		}
		break;
	case 30://30
		threshold = (phaseC - phaseB)/2;
		if(phaseA<threshold){
			if(*prevDetection == 20){
				*prevDetection = 30;
				bemfCounter += 1;
			}else{
				*prevDetection = 30;
				bemfCounter = 0;
			}
		}
		break;
	case 40://40
		threshold = (phaseC - phaseA)/2;
		if(phaseB>threshold){
			if(*prevDetection == 30){
				*prevDetection = 40;
				bemfCounter += 1;
			}else{
				*prevDetection = 40;
				bemfCounter = 0;
			}
		}
		break;
	case 50://50
		threshold = (phaseB - phaseA)/2;
		if(phaseC<threshold){
			if(*prevDetection == 40){
				*prevDetection = 50;
				bemfCounter += 1;
			}else{
				*prevDetection = 50;
				bemfCounter = 0;
			}
		}
		break;
	case 60://60
		threshold = (phaseB - phaseC)/2;
		if(phaseA>threshold){
			if(*prevDetection == 50){
				*prevDetection = 60;
				bemfCounter += 1;
			}else{
				*prevDetection = 60;
				bemfCounter = 0;
			}
		}
		break;
	default:
		break;
	}
	return bemfCounter;
}
