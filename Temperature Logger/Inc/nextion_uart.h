/*
 * nextion_uart.h
 *
 *  Created on: Apr 29, 2023
 *      Author: Kazım Tulgaroğlu
 */

#ifndef INC_NEXTION_UART_H_
#define INC_NEXTION_UART_H_

#include "stm32l476xx.h"

// Prototypes
void initUart4();
void sendMessage(char data[],int len);
void config_pins_as_uart();

#endif /* INC_NEXTION_UART_H_ */
