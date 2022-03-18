/*
 * ServoMotor.h
 *
 *  Created on: Jan 3, 2022
 *      Author: ktulgar
 */

#ifndef SRC_SERVOMOTOR_H_
#define SRC_SERVOMOTOR_H_

#include "stm32l4xx.h"

class ServoMotor {
public:
	ServoMotor();
	void startPouring();
	void stopPouring();
private:
	void initTimer();
	void initGPIO();
};

#endif /* SRC_SERVOMOTOR_H_ */
