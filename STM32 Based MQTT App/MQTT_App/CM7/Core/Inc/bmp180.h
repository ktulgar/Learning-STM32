/*
 * bmp180.h
 *
 *  Created on: Dec 9, 2024
 *      Author: Kazım Tulgaroğlu
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32h7xx_hal.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;

typedef struct {
    float temperature;
    long  pressure;
    float altitude;
}BMP180_Data;

void BMP180_GetCoefficients();
void BMP180_ReadData(BMP180_Data *bmp180);


#endif /* INC_BMP180_H_ */
