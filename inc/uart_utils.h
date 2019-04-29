/*
 * uart_utils.h
 *
 *  Created on: 10/02/2018
 *      Author: chepe
 */

#ifndef UART_UTILS_H_
#define UART_UTILS_H_

#include <stdio.h>

#define INT_DIGITS 10		/* enough for 32 bit integer */

void uartInit(void);
void uartSendCharBlocking(uint8_t character);
void uartSendStringBlocking(char *strToSend);
void uartSendIntBlocking(int16_t number, uint8_t maxSize);
void uartSendFloatBlocking(float number, uint8_t maxSize);
void strConcat(char *strOne, char *strTwo, char *outStr);
void intToStr(uint16_t number, uint8_t nDigits, char *returnedArray);
void uartDMAInit(void);
void uartDMATransfer(uint32_t  msg, uint8_t trLen);
uint8_t getStrLen(char *str);
char* my_itoa(uint32_t i);
void debug_uart_print(uint32_t array_values[], uint8_t array_len, char data_string[]);

#endif /* UART_UTILS_H_ */
