/*
 * nextion_uart.c
 *
 *  Created on: Apr 29, 2023
 *      Author: Kazım Tulgaroğlu
 */


#include "nextion_uart.h"



void initUart4() {
	  RCC->APB1ENR1 |= (1 << 19);  // Enable UART4
	  // APB1 Clock frequency is 48 MHZ => 48 MHZ/9600 = 5000d = 1388h
	  UART4->BRR = 0x1388;         // NEXTION operates at 9600 BaudRate
	  UART4->CR1 = 0x35 ;          // Enable UART, Enable RX , OverSampling = 16 , Enable Idle Line Detection And Receive Interrupt
	  UART4->CR1 |= (1 << 3);      // Enable TX
	  NVIC_EnableIRQ(UART4_IRQn);  // We will receive the messages by interrupt method
}




void config_pins_as_uart() {
	  RCC->AHB2ENR |= 1;           // Enable GPIOA
	  GPIOA->OSPEEDR |= (3 << 0);  // High Speed for PA0
	  GPIOA->MODER &= ~(1 << 0);   // Alternate Function For PA0
	  GPIOA->AFR[0] |= (8 << 0);   // AF8 For PA0
	  GPIOA->OSPEEDR |= (3 << 2);  // High Speed for PA1
	  GPIOA->MODER &= ~(1 << 2);   // Alternate Function For PA1
	  GPIOA->AFR[0] |= (8 << 4);   // AF8 For PA1
}

void sendMessage(char data[],int len) {

	for(unsigned int i = 0 ; i < len ; i++) {
		while(!(UART4->ISR & (1 << 7)));   // Wait until transmit data register gets empty.
		UART4->TDR = data[i];
		while(!(UART4->ISR & (1 << 6)));   // Wait until transfer gets completed.
	}

}
