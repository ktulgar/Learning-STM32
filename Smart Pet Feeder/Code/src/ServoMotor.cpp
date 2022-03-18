/*
 * ServoMotor.cpp
 *
 *  Created on: Jan 3, 2022
 *      Author: ktulgar
 */

#include "ServoMotor.h"

ServoMotor::ServoMotor() {
	initGPIO();
	initTimer();
}

void ServoMotor::initGPIO(){
	//GPIO Port A Pin 0
	RCC->AHB2ENR |= (1 << 0);    // Enable GPIO A
	GPIOA->MODER &= ~(1 << 0);  // Alternate Function
	GPIOA->OTYPER &= ~(1 << 0); // Push-Pull
	GPIOA->OSPEEDR &= ~(3 << 0); // High Speed
	GPIOA->PUPDR &= ~(3 << 0);  // No Pull Up-Down
	GPIOA->AFR[0] |= 1 << 0;  // Alternate Function 2
}

void ServoMotor::initTimer() {

	RCC->APB1ENR1 |= 1; // Enable Timer 2
	TIM2->PSC = 79;     // Prescaler value is 79. So that frequency of the timer will become 1 MHZ;
	TIM2->ARR = 20000;  // Frequency of Servo Motor should be 50 hz. That's why ARR value is 20000;
	TIM2->CCER |= 1;    // Activate channel 1
	TIM2->CCMR1 |= (6 << 4); // PWM Mode
	TIM2->CR1 |= 1; // Counter Enable
	TIM2->CCR1 = 700;

}

void ServoMotor::startPouring() {

	TIM2->CCR1 = 1770;
}

void ServoMotor::stopPouring() {
	TIM2->CCR1 = 1400;
    for(int i=0 ; i< 10000 ; i++) ;
    TIM2->CCR1 = 700;
}

