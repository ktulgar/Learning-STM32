/*
 * rtc.h
 *
 *  Created on: Apr 30, 2023
 *      Author: Kazım Tulgaroğlu
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "stdbool.h"


#define HOUR_TEN_POS 20
#define HOUR_TEN  3

#define HOUR_UNIT_POS 16
#define HOUR_UNIT 15

#define MINUTE_TEN_POS 12
#define MINUTE_TEN 7

#define MINUTE_UNIT_POS  8
#define MINUTE_UNIT 15

#define SECOND_TEN_POS  4
#define SECOND_TEN 7

#define SECOND_UNIT_POS 0
#define SECOND_UNIT 15

#define DATE_UNIT_POS 0
#define DATE_UNIT 15

#define DATE_TEN_POS 4
#define DATE_TEN 3

#define MONTH_UNIT_POS 8
#define MONTH_UNIT 15

#define MONTH_TEN_POS 12
#define MONTH_TEN 1

#define YEAR_TEN_POS 20
#define YEAR_TEN 15

#define YEAR_UNIT_POS 16
#define YEAR_UNIT 15

struct TimeDate{
	uint8_t hour;
	uint8_t minute;
	uint8_t day;
	uint8_t month;
	uint8_t year;
};

// Prototypes
void init_RTC();
void set_Time_Date(struct TimeDate *);
void set_Alarm(int minute,struct TimeDate *);
void set_Alarm_Time(int minute,struct TimeDate *);
void get_Time_Date(struct TimeDate *);
void disable_Alarm();
void enable_Alarm();

// void RTC_Alarm_IRQHandler();

#endif /* INC_RTC_H_ */
