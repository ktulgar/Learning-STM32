/*
 *
 *
 *  Created on: Apr 30, 2023
 *      Author: Kazım Tulgaroğlu
 */

#include "bmp180.h"

void config_I2C(I2C_TypeDef *i2c,uint8_t slave_address,uint8_t PRESC,uint8_t SCLL,uint8_t SCLH,uint8_t SDADEL,uint8_t SCLDEL) {
	if(i2c == I2C1) {
		RCC->APB1ENR1 |= (1 << 21);       // I2C1 clock enable
	}
	else if(i2c == I2C2){
		RCC->APB1ENR1 |= (1 << 22);       // I2C2 clock enable
	}
	else if(i2c == I2C3) {
		RCC->APB1ENR1 |= (1 << 23);       // I2C3 clock enable
	}
	i2c->TIMINGR |= (PRESC << 28);
	i2c->TIMINGR |= (SCLL << 20);
	i2c->TIMINGR |= (SCLH << 16);
	i2c->TIMINGR |= (SDADEL << 8);
	i2c->TIMINGR |= (SCLDEL << 0);
	i2c->CR1 |= 1 << 0;
	i2c->CR2 |= (slave_address << 0);

}

void I2C_Read_From_Register(I2C_TypeDef *i2c,uint8_t register_Address,uint8_t data[],uint8_t len) {
	i2c->CR2 &= ~(255 << 16);                    // Clear the Number of bytes
 	i2c->CR2 &= ~(1 << 10);                      // Write Operation
	i2c->CR2 |= (1 << 16);                       // 1 byte for writing register address that is read from
	i2c->CR2 |= (1 << 13);                       // START
	while(!(i2c->ISR & (1 << 0)));               // Wait until transmit data register is empty
	i2c->TXDR = register_Address;                // Write the register address
    while(!(i2c->ISR & (1 << 6)));               // Wait until transfer is completed
	i2c->CR2 &= ~(255 << 16);                    // Clear the Number of bytes
	i2c->CR2 |= (len << 16);                     // how many bytes will be read
	i2c->CR2 |= (1 << 10);                       // Read Operation
	i2c->CR2 |= (1 << 13);                       // Repeated start
	for(int i=0 ; i < len ; i++) {
		while(!(i2c->ISR & (1 << 2)));           // Wait until receive data register is not empty
		data[i] = i2c->RXDR;                     // Get data
 }
	while(!(i2c->ISR & (1 << 6)));               // Wait until last transfer is completed
	i2c->CR2 |= (1 << 14);                       // STOP
}


void I2C_Write_To_Register(I2C_TypeDef *i2c,uint8_t register_Address,uint8_t data[],uint8_t len) {
	i2c->CR2 &= ~(255 << 16);                    // Clear the Number of bytes
	i2c->CR2 |= ((1+len) << 16);                 // How many bytes will be written + register address
	i2c->CR2 &= ~(1 << 10);                      // Write operation
	i2c->CR2 |= (1 << 13);                       // START
	while(!(i2c->ISR & (1 << 0)));               // Wait until transmit data register is empty
	i2c->TXDR = register_Address;                // Write the register address
	for(int i=0 ; i < len ; i++) {
		while(!(i2c->ISR & (1 << 0)));           // Wait until transmit data register is empty
		i2c->TXDR = data[i];                     // Write the data
 }
	while(!(i2c->ISR & (1 << 6)));               // Wait until last transfer is completed
	i2c->CR2 |= (1 << 14);                       // STOP
}


void config_pins_as_i2c() {

	RCC->AHB2ENR |= (1 << 2);
	GPIOC->MODER &= ~(1 << 0);
	GPIOC->MODER &= ~(1 << 2);
	GPIOC->OTYPER |= (1 << 0) | (1 << 1);
	GPIOC->PUPDR &= ~(3 << 0);
	GPIOC->PUPDR &= ~(3 << 2);
	GPIOC->OSPEEDR |= (3 << 0) | (3 << 2);
	GPIOC->AFR[0] |= ((4 << 0) | (4 << 4));

}


float get_Temperature() {

	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
	long UT;
	long X1;
	long X2;
	long B5;
	long T;
	float real_T;

	uint8_t message[22];
	I2C_Read_From_Register(I2C3,0xAA,message,22);
	AC1 = message[0] << 8 | message[1];
    AC2 = message[2] << 8 | message[3];
	AC3 = message[4] << 8 | message[5];
	AC4 = message[6] << 8 | message[7];
	AC5 = message[8] << 8 | message[9];
	AC6 = message[10] << 8 | message[11];
	B1 = message[12] << 8 | message[13];
	B2 = message[14] << 8 | message[15];
    MB = message[16] << 8 | message[17];
	MC = message[18] << 8 | message[19];
	MD = message[20] << 8 | message[21];
	message[0] = 0x2e;
	I2C_Write_To_Register(I2C3,0xF4,message,1);
	HAL_Delay(5);
	I2C_Read_From_Register(I2C3,0xf6,message,2);
	UT = message[0] << 8 | message[1];
	X1 = (UT - AC6)*AC5/32768;
	X2 = MC * 2048 / (X1 + MD);
	B5 = X1 + X2;
	T = (B5 + 8)/16;
    real_T = T*0.1;
    return real_T;

}
