/*
 * motor_control.h
 *
 *  Created on: 2/02/2018
 *      Author: chepe
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

//#define ARR_VALUE	682		//70.31 KHz 2048 at 144MHz
#define ARR_VALUE	1365	//35.15 KHz 4096 at 144MHz
//#define ARR_VALUE	2048	//23.44 KHz 6144 at 144MHz	Standard Frequency, MCU @ 48MHz with 2048 ARR_VALUE
//#define	ARR_VALUE	7200	//20 kHz
//#define ARR_VALUE	14400
#define	HIGH	10
#define	LOW		20

typedef enum {INIT, CLOSELOOP, OPENLOOP} controlstate_t;

void clkInit(void);
void pwmInit(void);
void gpioInit(void);
void ramp_up_timer_init(void);
void pwm_input_init(void);
uint32_t pwm_input_read(void);

void outAHighSetDuty(uint16_t dutyA);
void outBHighSetDuty(uint16_t dutyB);
void outCHighSetDuty(uint16_t dutyC);

void outALowSetState(uint8_t outA);
void outBLowSetState(uint8_t outB);
void outCLowSetState(uint8_t outC);
void motor_init(void);
void motor_high_z(void);
void motor_break(void);
void motor_prepositioning(uint16_t arrValue);
void motor_update(uint16_t duty, uint8_t step);
uint8_t ramp_up(uint16_t rampup_duty);
uint8_t motor_rampup_zc(uint8_t state, uint16_t phaseA, uint16_t phaseB, uint16_t phaseC, uint8_t* prevDetection, uint8_t bemfCounter);
uint8_t motor_zc_detection(uint16_t phaseA, uint16_t phaseB, uint16_t phaseC, uint8_t step, controlstate_t control_state);

#endif /* MOTOR_CONTROL_H_ */
