/*
 * HC05.h
 *
 *  Created on: 26 Ara 2021
 *      Author: ktulgar
 */

#ifndef HC05_H_
#define HC05_H_


#include "stm32l4xx.h"
#include <string>

using namespace std;

class HC05 {
public:
	HC05();
	void sendMessage(string);
	string message;
private:
	void initUart4();
	void initUart5();
};

#endif /* HC05_H_ */
