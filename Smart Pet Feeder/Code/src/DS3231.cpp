/*
 * DS3231.cpp
 *
 *  Created on: 26 Ara 2021
 *      Author: ktulgar
 */

#include "DS3231.h"



DS3231::DS3231() {
	initGPIO();
	initI2C3();
	initExternalInterrupt();
}


void DS3231::initGPIO() {
	RCC->AHB2ENR |= (1 << 2) | (1 << 3); // Enable GPIOC - GPIOD

	GPIOC->MODER &= ~(1 << 0);   // GPIOC 0-1 Alternate Function
	GPIOC->MODER &= ~(1 << 2);   // GPIOC 0-1 Alternate Function
	GPIOC->OTYPER |= (3 << 0);   // GPIOC 0-1 Open-Drain
	GPIOC->OSPEEDR |= (15 << 0); // GPIOC 0-1 Very High Speed
	GPIOC->PUPDR &= ~(15 << 0);  // GPIOC 0-1 No PP
	GPIOC->AFR[0] |= (1 << 6) | (1 << 2);  // GPIOC 0-1 AF4

	GPIOD->MODER &=  ~(3 << 4);  // GPIOD 2 Input
	GPIOD->PUPDR &= ~(3 << 4);   // GPIOD 2 No Pull-up or Pull-down
}


void DS3231::initI2C3() {

	  RCC->APB1ENR1 |= (1 << 23);   // Enable I2C3
	  I2C3->TIMINGR = 0x00702991;   // I got this from CubeMX, i did not calculate it myself.
	  I2C3->CR2 |= (0x68 << 1);     // DS3231's slave address is 0x68;
	  I2C3->CR1 |= (1 << 0);        // Peripheral Enable

}

void DS3231::initExternalInterrupt() {
    RCC->APB2ENR |= (1 << 0);       // Enable SysConfig
	SYSCFG->EXTICR[0] |= (3 << 8) ; // GPIOD 2 External Interrupt
	EXTI->IMR1 |=  (1 << 2);        // Interrupt Mask Register
	EXTI->FTSR1 |= (1 << 2);
}

void DS3231::readFromRegister(uint8_t registerAddress,uint8_t data[],int len) {

	I2C3->CR2 &= ~(255 << 16);         // Clear NBytes bits
	I2C3->CR2 |= (1 << 16);            // Just 1 byte for write operation
	I2C3->CR2 &= ~(1 << 10);           // Write operation

	I2C3->CR2 |= (1 << 13);           // Start generation
	while(!(I2C3->ISR & (1 << 0)));   // Wait until TX register empty
	I2C3->TXDR = registerAddress;     // Write register address where you want to read data from
	while(!(I2C3->ISR & (1 << 6)));   // Wait until transfer gets completed

	I2C3->CR2 |= (1 << 10);           // Read Operation
	I2C3->CR2 &= ~(255 << 16);        // Clear NBytes bits
	I2C3->CR2 |= (len << 16);         // Length of data that will be received.

	I2C3->CR2 |= (1 << 13);              // Start Generation

	for(int i=0 ; i < len ; i++) {
		while(!(I2C3->ISR & (1 << 2)));  // Wait until RX register is not empty
		data[i] = I2C3->RXDR;            // Read Data
	}

	while(!(I2C3->ISR & (1 << 6))); // Wait until transfer gets completed
	I2C3->CR2 |= (1 << 14);   // Stop generation

}

void DS3231::writeToRegister(uint8_t registerAddress,uint8_t data[],int len) {

	I2C3->CR2 &= ~(255 << 16);        // Clear NBytes bits
	I2C3->CR2 &= ~(1 << 10);          // Write Operation
	I2C3->CR2 |= ((len + 1) << 16);   // Length of data that will be transmitted.

	I2C3->CR2 |= (1 << 13);          // Start generation
	while(!(I2C3->ISR & (1 << 0)));  // Wait until TX register empty
	I2C3->TXDR = registerAddress;    // Write register address where you want to write data

	for(int i = 0 ; i < len ; i++) {
		while(!(I2C3->ISR & (1 << 0)));  // Wait until TX register empty
		I2C3->TXDR = data[i];            // Write data
	}

    while(!(I2C3->ISR & (1 << 6)));  // Wait until transfer gets completed
	I2C3->CR2 |= (1 << 14) ;         // Stop generation
}

void DS3231::DS3231::sendAlarm(struct DateTime alarm) {
	uint8_t alarmTime[3];
	alarmTime[0] = decimalToBcd(alarm.second);
	alarmTime[1] = decimalToBcd(alarm.minute);
	alarmTime[2] = decimalToBcd(alarm.hour);
	uint8_t alarmDate[3];
	alarmDate[0] = decimalToBcd(alarm.day);
	alarmDate[1] = decimalToBcd(alarm.month);
	alarmDate[2] = decimalToBcd(alarm.year);
	uint8_t command[] = {0x05};
	writeToRegister(0x0E,command,1);
	writeToRegister(0x07,alarmTime,3);
	writeToRegister(0x0A,alarmDate,3);
}
void DS3231::setCurrentDateTime(struct DateTime currentDateTime) {

	uint8_t command[] = {0x08};
	writeToRegister(0x0f,command,1);

	uint8_t timeValues[3];
	timeValues[0] = decimalToBcd(currentDateTime.second);
	timeValues[1] = decimalToBcd(currentDateTime.minute);
	timeValues[2] = decimalToBcd(currentDateTime.hour);
	writeToRegister(0x00,timeValues,3);

	uint8_t dateValues[3];
	dateValues[0] = decimalToBcd(currentDateTime.day);
	dateValues[1] = decimalToBcd(currentDateTime.month);
	dateValues[2] = decimalToBcd(currentDateTime.year);
	writeToRegister(0x04,dateValues,3);

}


void DS3231::getDateTime(struct DateTime *timeDate) {
	uint8_t timeValues[3];
	readFromRegister(0x00,timeValues,3);
	timeDate->second = bcdToDecimal(timeValues[0]);
	timeDate->minute = bcdToDecimal(timeValues[1]);
	timeDate->hour = bcdToDecimal(timeValues[2]);
	uint8_t dateValues[3];
	readFromRegister(0x04,dateValues,3);
	timeDate->day = bcdToDecimal(dateValues[0]);
	timeDate->month = bcdToDecimal(dateValues[1]);
	timeDate->year = bcdToDecimal(dateValues[2]);
}

void DS3231::disableAlarmMode(){
	uint8_t command[] = {0x00};
	writeToRegister(0x0E,command,1);  // Disable alarm mode.
}

void DS3231::deleteAlarms() {
	while(this->alarms.size() > 0) {
	    auto first = this->alarms.begin();
	    this->alarms.erase(first);
	}
}

int DS3231::bcdToDecimal(uint8_t bcd){
	int bcdM = bcd >> 4;
	int bcdL = bcd & (0x0F);
	return bcdM*10 + bcdL;
}

uint8_t DS3231::decimalToBcd(int decimal){
	uint8_t decimalL = decimal%10;
	uint8_t decimalM = decimal/10;
	return decimalL | (decimalM << 4);
}
