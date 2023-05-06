/*
 * rtc.c
 *
 *  Created on: Apr 30, 2023
 *      Author: Kazım Tulgaroğlu
 */

#include "rtc.h"

void init_RTC() {

	  PWR->CR1 |= (1 << 8);                 // Access to RTC and Backup registers enabled
	  RCC->BDCR |= (1 << 0);                // LSE oscillator ON
	  while(!(RCC->BDCR & (1 << 1)));       // Wait until LSE oscillator gets ready
	  RCC->BDCR |= (1 << 15);               // RTC clock enabled
	  RCC->BDCR |= (1 << 8);                // LSE oscillator clock used as RTC clock

	  // Unlock the write protection
	  RTC->WPR = 0xCA;
	  RTC->WPR = 0x53;

	  RTC->ISR |= (1 << 7);                 // Enter the initialization mode
	  while(!(RTC->ISR & (1 << 6)));        // Wait until it enters the initialization mode

	  // To generate a 1 Hz clock for the calendar counter, program both the prescaler factors.
	  RTC->PRER |= (127 << 16);
	  RTC->PRER |= (255 << 0);

	  RTC->CR &= ~(1 << 6);                  // 24 hour/day format

	  RTC->ISR &= ~(1 << 7);                 // Exit from initialization mode
}

void set_Time_Date(struct TimeDate *initial_Time_Date) {

	  RTC->ISR |= (1 << 7);                  // Enter the initialization mode
	  while(!(RTC->ISR & (1 << 6)));         // Wait until it enters the initialization mode

	  uint32_t timeRegister = 0;
	  uint32_t dateRegister = 0;

	  timeRegister = ((initial_Time_Date->hour/10) << HOUR_TEN_POS) | ((initial_Time_Date->hour%10) << HOUR_UNIT_POS) | ((initial_Time_Date->minute/10) << MINUTE_TEN_POS) | ((initial_Time_Date->minute%10) << MINUTE_UNIT_POS);
	  dateRegister = ((initial_Time_Date->year/10) << YEAR_TEN_POS) | ((initial_Time_Date->year%10) << YEAR_UNIT_POS) | ((initial_Time_Date->month/10) << MONTH_TEN_POS) | ((initial_Time_Date->month%10) << MONTH_UNIT_POS) | ((initial_Time_Date->day/10) << DATE_TEN_POS) | ((initial_Time_Date->day%10) << DATE_UNIT_POS);

	  // Unlock the write protection
	  RTC->WPR = 0xCA;
	  RTC->WPR = 0x53;

	  RTC->TR = timeRegister;
	  RTC->DR = dateRegister;

	  RTC->ISR &= ~(1 << 7);              // Exit from initialization mode


}

void get_Time_Date(struct TimeDate *timeDate) {

      // The software must wait until RSF is set before reading the RTC_SSR, RTC_TR and RTC_DR registers.
	  while(!(RTC->ISR &= (1 << 5)));

	  /*
	     To read the RTC calendar registers (RTC_SSR, RTC_TR and RTC_DR) properly, the APB
         clock frequency (fPCLK) must be equal to or greater than seven times the RTC clock
         frequency (fRTCCLK). It is enough to read registers only once.
      */

	  uint32_t dummySSR     = RTC->SSR;
	  uint32_t timeRegister = RTC->TR;
	  uint32_t dateRegister = RTC->DR;

	  timeDate->day = 10 * ((dateRegister & (DATE_TEN << DATE_TEN_POS)) >> DATE_TEN_POS) + ((dateRegister & (DATE_UNIT << DATE_UNIT_POS)) >> DATE_UNIT_POS);
	  timeDate->month = 10 * ((dateRegister & (MONTH_TEN << MONTH_TEN_POS)) >> MONTH_TEN_POS) + ((dateRegister & (MONTH_UNIT << MONTH_UNIT_POS)) >> MONTH_UNIT_POS);
	  timeDate->year = 10 * ((dateRegister & (YEAR_TEN << YEAR_TEN_POS)) >> YEAR_TEN_POS) + ((dateRegister & (YEAR_UNIT << YEAR_UNIT_POS)) >> YEAR_UNIT_POS);
	  timeDate->hour = 10 * ((timeRegister & (HOUR_TEN << HOUR_TEN_POS)) >> HOUR_TEN_POS) + ((timeRegister & (HOUR_UNIT << HOUR_UNIT_POS)) >> HOUR_UNIT_POS);
	  timeDate->minute = 10 * ((timeRegister & (MINUTE_TEN << MINUTE_TEN_POS)) >> MINUTE_TEN_POS) + ((timeRegister & (MINUTE_UNIT << MINUTE_UNIT_POS)) >> MINUTE_UNIT_POS);

	  RTC->ISR &=  ~(1 << 5);

}


void set_Alarm(int minute,struct TimeDate *currentTimeDate) {

	disable_Alarm();
	set_Alarm_Time(minute,currentTimeDate);
	enable_Alarm();

}

void set_Alarm_Time(int minute,struct TimeDate *currentTimeDate) {

   	get_Time_Date(currentTimeDate);
   	uint8_t current_minute = currentTimeDate->minute;
      if((current_minute + minute ) >= 60) {

    	  current_minute = (current_minute + minute) - 60;

      }
      else {
    	  current_minute += minute;
    	  }

	  uint32_t alarmRegister = 0;
	  alarmRegister |= (1 << 31);
	  alarmRegister |= (1 << 30);
	  alarmRegister |= (1 << 23);
	  alarmRegister |= ((current_minute/10) << 12) | ((current_minute%10) << 8);

	  RTC->WPR = 0xCA;
	  RTC->WPR = 0x53;
	  RTC->ALRMAR = alarmRegister;

 }


// I did the inverse of what i did in enable_Alarm()
void disable_Alarm() {

	  EXTI->IMR1 &= ~(1 << 18);
	  EXTI->RTSR1 &= ~(1 << 18);

	  RTC->WPR = 0xCA;
	  RTC->WPR = 0x53;

	  RTC->CR &= ~(1 << 8);
	  RTC->CR &= ~(1 << 12);
	  NVIC_DisableIRQ(RTC_Alarm_IRQn);
}

void enable_Alarm() {

	  // RTC alarms are connected to EXTI line 18
	  EXTI->IMR1 |= (1 << 18);     // Interrupt request from Line 18 is not masked
	  EXTI->RTSR1 |= (1 << 18);    // Rising trigger enabled (for Event and Interrupt) for input line 18

	  // Unlock the write protection
	  RTC->WPR = 0xCA;
	  RTC->WPR = 0x53;

	  RTC->CR |=  (1 << 8)  |  (1 << 12);  // Alarm A enabled and Alarm A interrupt enabled
	  NVIC_EnableIRQ(RTC_Alarm_IRQn);
}


