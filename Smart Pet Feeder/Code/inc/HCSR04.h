/*
 * HCSR04.h
 *
 *  Created on: Dec 31, 2021
 *      Author: ktulgar
 */


#include "stm32l4xx.h"

class HCSR04 {
public:
	HCSR04();
	int getDistance();
private:
	void TIM2Config();
	void initGPIO();
	void delayUS(volatile uint32_t us);


};

