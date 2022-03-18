/*
 * HX711.cpp
 *
 *  Created on: Dec 31, 2021
 *      Author: ktulgar
 */

#include "HX711.h"

HX711::HX711() {
	initGPIO();
	TIM6Config();
}

void HX711::initGPIO(){

	RCC->AHB1ENR |= (3 << 0); // Enable GPIOA-GPIOB

	//GPIO Port A Pin 7
	GPIOA->MODER &= ~(1 << 15);  // General purpose output mode
	GPIOA->OTYPER &= ~(1 << 7);  // Output push-pull
	GPIOA->OSPEEDR |= (3 << 14); // Very high speed
	GPIOA->PUPDR &= ~(3 << 14);  // No Pull Up-Down


	//GPIO Port B Pin 6
	GPIOB->MODER &= ~(3 << 12); // Input mode
	GPIOB->PUPDR &= ~(3 << 12); // No Pull Up-Down

}

void HX711::TIM6Config() {
	RCC->APB1ENR1 |= (1<<4);
	TIM6->PSC = 79;
	TIM6->ARR = 0xffff;
	TIM6->CR1 |= (1<<0);
	while (!(TIM6->SR & (1<<0)));
}

void HX711::delayUS(volatile uint32_t us) {
	TIM6->CNT = 0;
	while (TIM6->CNT < us);
}

int HX711::rawValue(){
	int count = 0;
	GPIOA->ODR &= ~(1 << 7);
	delayUS(10);
    TIM6->CNT = 0;
	while(GPIOB->IDR & (1 << 6));
	for(int i = 0 ; i < 24 ; i++) {
		GPIOA->ODR |= (1 << 7);
		delayUS(10);
		count = count << 1;
		GPIOA->ODR &= ~(1 << 7);
		delayUS(10);
		if(GPIOB->IDR & (1 << 6))
			count = count + 1;

	}
	GPIOA->ODR |= (1 << 7);
	delayUS(10);
	count = count ^ 0x800000;
	GPIOA->ODR &= ~(1 << 7);
	delayUS(10);
	return count;
}

int HX711::getWeight() {
	return (((rawValue() - this->tare)*this->ratio));
}
