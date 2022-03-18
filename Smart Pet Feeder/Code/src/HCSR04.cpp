/*
 * HCSR04.cpp
 *
 *  Created on: Dec 31, 2021
 *      Author: ktulgar
 */

#include "HCSR04.h"

HCSR04::HCSR04() {
	initGPIO();
	TIM2Config();
}

void HCSR04::TIM2Config() {
	RCC->APB1ENR1 |= (1<<0);
	TIM2->PSC = 99;
	TIM2->CR1 |= (1<<0);
}

void HCSR04::initGPIO() {

	RCC->AHB1ENR |= 1 << 1; // Enable GPIOB

	//GPIO Port B Pin 12
	GPIOB->MODER &= ~(1 << 25);    // General purpose output mode
	GPIOB->OTYPER &= ~(1 << 12);   // Output push-pull
	GPIOB->OSPEEDR |= (3 << 24);   // Very high speed
	GPIOB->PUPDR &= ~(3 << 24);    // No Pull Up-Down


	//GPIO Port B Pin 11
	GPIOB->MODER &= ~(3 << 22);    // Input mode
	GPIOB->PUPDR &= ~(3 << 22);    // No Pull Up-Down

}

void HCSR04::delayUS(volatile uint32_t us) {
	TIM2->CNT = 0;
	while (TIM2->CNT < us);
}


int HCSR04::getDistance() {
	int time = 0;
	GPIOB->ODR &=  ~(1 << 12);
	delayUS(10);
	GPIOB->ODR |=  (1 << 12);
	delayUS(15);
	GPIOB->ODR &=  ~(1 << 12);
	delayUS(15);

	while(!(GPIOB->IDR & (1 << 11)));
	while((GPIOB->IDR & (1 << 11)))
	{
		time++;
		delayUS(1);
	}

	return time/55;

}


