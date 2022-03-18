 /* DS3231.h
 *
 *  Created on: 26 Ara 2021
 *      Author: ktulgar
 */

#ifndef SRC_DS3231_H_
#define SRC_DS3231_H_

#include "stm32l4xx.h"
#include <list>


struct DateTime{
	int second;
	int minute;
	int hour;
	int day;
	int month;
	int year;
	int gram;
};

class DS3231 {
public:
	DS3231();
	void setCurrentDateTime(struct DateTime);
	void deleteAlarms();
	int bcdToDecimal(uint8_t);
	std::list<struct DateTime> alarms;
	uint8_t decimalToBcd(int);
	void sendAlarm(struct DateTime);
	void getDateTime(struct DateTime *);
	void disableAlarmMode();

private:
	void initGPIO();
	void initI2C3();
	void readFromRegister(uint8_t,uint8_t *,int);
	void writeToRegister(uint8_t,uint8_t *,int);
	void initExternalInterrupt();


};

#endif /* SRC_DS3231_H_ */
