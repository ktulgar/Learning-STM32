/*
 *
 *
 *  Created on: Apr 30, 2023
 *      Author: Kazım Tulgaroğlu
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32l476xx.h"


// Prototypes
void config_pins_as_i2c();
void config_I2C(I2C_TypeDef *i2c,uint8_t slave_address,uint8_t PRESC,uint8_t SCLL,uint8_t SCLH,uint8_t SDADEL,uint8_t SCLDEL);
void I2C_Read_From_Register(I2C_TypeDef *i2c,uint8_t register_Address,uint8_t data[],uint8_t len);
void I2C_Write_To_Register(I2C_TypeDef *i2c,uint8_t register_Address,uint8_t data[],uint8_t len);
float get_Temperature();

#endif /* INC_BMP180_H_ */
