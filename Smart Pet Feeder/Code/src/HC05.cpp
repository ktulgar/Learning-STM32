/*
 * HC05.cpp
 *
 *  Created on: 26 Ara 2021
 *      Author: ktulgar
 */

#include "HC05.h"

HC05::HC05() {
	initUart4();
	initUart5();
}

void HC05::initUart4() {
	  RCC->APB1ENR1 |= (1 << 19);  // Enable UART4
	  RCC->AHB2ENR |= 1;           // Enable GPIOA
	  GPIOA->OSPEEDR |= (3 << 2);  // High Speed for PA1
	  GPIOA->MODER &= ~(1 << 2);   // Alternate Function For PA1
	  GPIOA->AFR[0] |= (8 << 4);   // AF8 For PA1
	  UART4->BRR = 0x208d;         // HC05 operates at 9600 BaudRate
	  UART4->CR1 = 0x35 ;   // Enable Uart, Enable Receiver , OverSampling = 16 , Enable Idle Line Detection And Receive Interrupt
	  NVIC_EnableIRQ(UART4_IRQn);
}

// This is for testing purposes.I send received message to computer to see  whether received message is correct or not.
void HC05::initUart5() {
	RCC->APB1ENR1 |= (1 << 20);  // Enable UART5
	RCC->AHB2ENR |= (1 << 2);    // Enable GPIOC
	GPIOC->MODER &= ~(1 << 24);  // Alternate Function For PC12
    GPIOC->AFR[1] |= (8 << 16);  //  AF8 For PC12
    UART5->BRR = 0x56C;          // BaudRate = 115200
    UART5->CR1 = 0x8009;         // Enable Uart, Enable Transmission , OverSampling = 8
}
// This is for testing purposes.I send received message to computer to see  whether received message is correct or not.
void HC05::sendMessage(string message) {

	for(unsigned int i = 0 ; i < message.size() ; i++) {
		while(!(UART5->ISR & (1 << 7)));
		UART5->TDR = message[i];
		while(!(UART5->ISR & (1 << 6)));
	}

}
