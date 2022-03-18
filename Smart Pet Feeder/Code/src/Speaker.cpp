/*
 * Speaker.cpp
 *
 *  Created on: 29 Ara 2021
 *      Author: ktulgar
 */

#include "Speaker.h"

Speaker::Speaker() {
	initGPIO();
	initTimer();
}

void Speaker::initGPIO() {

	//GPIO Port B Pin 0
	RCC->AHB2ENR |= (1 << 1);    // Enable GPIO B
	GPIOB->MODER &= ~(1 << 0);  // Alternate Function
	GPIOB->OTYPER &= ~(1 << 0); // Push-Pull
	GPIOB->OSPEEDR |= (3 << 0); // High Speed
	GPIOB->PUPDR &= ~(3 << 0);  // No Pull Up-Down
	GPIOB->AFR[0] |= (2 << 0);  // Alternate Function 2

}

void Speaker::initTimer() {

	RCC->APB1ENR1 |= 0x2;  //  Enable The Timer3
	TIM3->CCER |= 0x100;   //  Capture/Compare Channel 3 output enable.
	TIM3->CCMR2 = 0x60;    //  PWM Mode 1 <-> In upcounting, channel 3 is active as long as TIMx_CNT<TIMx_CCR1
	TIM3->PSC = 79;        //  Prescaler Value is 79.So that frequency of the timer becomes 1 MHZ.

	for(int i = 0 ; i<203 ; i++) {
		if(this->notesPOTC[i] == 0) {
			arrValues[i] = 0;
			ccrValues[i] = 0;
	 }
		else {
			this->arrValues[i] = round((float)1/(this->notesPOTC[i]) * 1000000); // Calculate the ARR values
			this->ccrValues[i] = round((float)arrValues[i]/2); // Duty Cycle is %50.
		}
	}
}

void Speaker::play() {
	for(int i=0 ; i<203 ; i++) {
		if(this->arrValues[i] != 0) {
			TIM3->ARR = this->arrValues[i];
			TIM3->CCR3 = this->ccrValues[i];
			TIM3->CR1 |= 0x1; // Enable Counter
			delayMS(this->durations[i]); // Wait
			TIM3->CR1 &= ~(0x1); // Disable Counter
		}
		else {
			delayMS(this->durations[i]);
		}
	}
}

void Speaker::delayMS(int ms) {
	ms *= 6667;
	while(ms--);
}
