/*
 * HX711.h
 *
 *  Created on: Dec 31, 2021
 *      Author: ktulgar
 */

#ifndef SRC_HX711_H_
#define SRC_HX711_H_

#include "stm32l4xx.h"


class HX711 {
public:
	HX711();
	int getWeight();
private:
	int rawValue();
	void TIM6Config();
	void initGPIO();
	void delayUS(volatile uint32_t us);
	const int tare = 8520962;      // Initial value that comes from sensor without putting any weight on it
	const double ratio = 0.00261;  // Assume that you put something that weights 50 grams on sensor.
	                               // Sensor gave a value.Ratio is 50/(value - tare) .

};

#endif /* SRC_HX711_H_ */
